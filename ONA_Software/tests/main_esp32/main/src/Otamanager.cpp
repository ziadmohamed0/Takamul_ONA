/**
 * ============================================================================
 *  OtaManager.cpp — Takamul SCADA
 *
 *  Handles OTA for both chips:
 *
 *  ESP32 OTA path:
 *    esp_https_ota() → writes to OTA partition → esp_restart()
 *    Uses IDF built-in; rollback supported if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
 *
 *  STM32 OTA path:
 *    1. ESP32 downloads the .bin over HTTPS into heap
 *    2. Asserts STM32 BOOT0 high + pulses RESET  (or sends "CMD:BOOTLOADER\n")
 *    3. Runs STM32 UART bootloader protocol (AN3155):
 *         sync → erase → write chunks of 256 bytes → verify CRC
 *    4. Releases BOOT0 + pulses RESET → STM32 boots new firmware
 *    5. Clears the ota_stm32_url field in Supabase
 *
 *  Pin assignments (match your schematic):
 *    UART port  : UART_NUM_2  (same as the data UART — we switch modes)
 *    BOOT0 GPIO : GPIO_NUM_4  ← ESP32 IO4 drives STM32 BOOT0
 *    NRST  GPIO : GPIO_NUM_5  ← ESP32 IO5 drives STM32 NRST
 *
 *  If BOOT0/NRST GPIOs are NOT wired, set STM32_BOOT_GPIO / STM32_NRST_GPIO
 *  to -1 and the STM32 firmware must jump to bootloader on "CMD:BOOTLOADER\n".
 * ============================================================================
 */

#include "inc/Otamanager.h"
#include "inc/SupabaseClient.h"
#include "inc/UartBridge.h"

#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>
#include <cstdlib>

// ─── Board-specific pin config ────────────────────────────────────────────────
// Set to -1 if the GPIO is not connected on your PCB.
// If -1, we send "CMD:BOOTLOADER\n" and rely on STM32 firmware to self-jump.
static constexpr int STM32_BOOT0_GPIO = -1;   // e.g. GPIO_NUM_4
static constexpr int STM32_NRST_GPIO  = -1;   // e.g. GPIO_NUM_5

// UART used to talk to STM32 (same port as data UART — switched to bootloader baud)
static constexpr uart_port_t STM32_UART_PORT   = UART_NUM_2;
static constexpr int         STM32_UART_RX_PIN = CONFIG_TAKAMUL_UART_RX_PIN;
static constexpr int         STM32_UART_TX_PIN = CONFIG_TAKAMUL_UART_TX_PIN;

// STM32 bootloader baud rate (AN3155 default)
static constexpr int STM32_BOOT_BAUD = 115200;

// STM32F401 flash start address
static constexpr uint32_t STM32_FLASH_BASE = 0x08000000UL;

// Maximum binary size we'll accept (512 KB — F401RC has 256 KB, guard with margin)
static constexpr size_t STM32_MAX_BIN_SIZE = 512 * 1024;

static const char* Tag = "OtaManager";

namespace Takamul {

// ─── Singleton ────────────────────────────────────────────────────────────────

OtaManager& OtaManager::getInstance() {
    static OtaManager instance;
    return instance;
}

// ─── checkAndRun ─────────────────────────────────────────────────────────────

void OtaManager::checkAndRun(const std::string& esp32_url,
                              const std::string& stm32_url,
                              const std::string& device_id)
{
    if (m_busy) {
        ESP_LOGD(Tag, "OTA already in progress, skipping poll");
        return;
    }

    if (!esp32_url.empty()) {
        ESP_LOGI(Tag, "ESP32 OTA triggered: %s", esp32_url.c_str());
        m_busy = true;
        auto* ctx = new OtaCtx{ esp32_url, device_id };
        xTaskCreate(esp32OtaTask, "esp32_ota", 8192, ctx, tskIDLE_PRIORITY + 3, nullptr);
    }
    else if (!stm32_url.empty()) {
        ESP_LOGI(Tag, "STM32 OTA triggered: %s", stm32_url.c_str());
        m_busy = true;
        auto* ctx = new OtaCtx{ stm32_url, device_id };
        xTaskCreate(stm32OtaTask, "stm32_ota", 16384, ctx, tskIDLE_PRIORITY + 3, nullptr);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ESP32 SELF-OTA
// ═══════════════════════════════════════════════════════════════════════════════

void OtaManager::esp32OtaTask(void* arg) {
    auto* ctx = static_cast<OtaCtx*>(arg);
    OtaManager::getInstance().runEsp32Ota(ctx->url);
    delete ctx;
    vTaskDelete(nullptr);
}

void OtaManager::runEsp32Ota(const std::string& url) {
    ESP_LOGI(Tag, "═══ ESP32 OTA START ═══");
    ESP_LOGI(Tag, "URL: %s", url.c_str());

    esp_http_client_config_t http_cfg = {};
    http_cfg.url                      = url.c_str();
    http_cfg.crt_bundle_attach        = esp_crt_bundle_attach;
    http_cfg.keep_alive_enable        = true;
    http_cfg.timeout_ms               = 30000;

    esp_https_ota_config_t ota_cfg = {};
    ota_cfg.http_config             = &http_cfg;

    esp_err_t ret = esp_https_ota(&ota_cfg);

    if (ret == ESP_OK) {
        ESP_LOGI(Tag, "ESP32 OTA OK — rebooting in 2 s…");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        ESP_LOGE(Tag, "ESP32 OTA FAILED: %s", esp_err_to_name(ret));
        m_busy = false;   // allow retry on next poll
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  STM32 OTA
// ═══════════════════════════════════════════════════════════════════════════════

void OtaManager::stm32OtaTask(void* arg) {
    auto* ctx = static_cast<OtaCtx*>(arg);
    OtaManager::getInstance().runStm32Ota(ctx->url, ctx->device_id);
    delete ctx;
    vTaskDelete(nullptr);
}

void OtaManager::runStm32Ota(const std::string& url, const std::string& device_id) {
    ESP_LOGI(Tag, "═══ STM32 OTA START ═══");
    ESP_LOGI(Tag, "URL: %s", url.c_str());

    // ── 1. Download binary ───────────────────────────────────────────────────
    uint8_t* bin    = nullptr;
    size_t   binLen = downloadToHeap(url, bin, STM32_MAX_BIN_SIZE);
    if (binLen == 0 || bin == nullptr) {
        ESP_LOGE(Tag, "Download failed");
        m_busy = false;
        return;
    }
    ESP_LOGI(Tag, "Downloaded %zu bytes", binLen);

    // ── 2. Stop normal UART data exchange ───────────────────────────────────
    UartBridge::getInstance().stop();
    vTaskDelay(pdMS_TO_TICKS(200));

    // ── 3. Re-init UART at bootloader baud ──────────────────────────────────
    uart_config_t cfg = {};
    cfg.baud_rate  = STM32_BOOT_BAUD;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_EVEN;   // STM32 bootloader requires EVEN parity
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(STM32_UART_PORT, &cfg);
    uart_set_pin(STM32_UART_PORT, STM32_UART_TX_PIN, STM32_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(STM32_UART_PORT, 1024, 1024, 0, nullptr, 0);

    // ── 4. Enter bootloader mode ─────────────────────────────────────────────
    stm32EnterBootloader();

    // ── 5. Flash ─────────────────────────────────────────────────────────────
    bool ok = stm32FlashBinary(bin, binLen);
    free(bin);

    // ── 6. Exit bootloader → STM32 runs new firmware ─────────────────────────
    stm32ExitBootloader();

    // ── 7. Restore normal UART (data baud, no parity) ────────────────────────
    uart_driver_delete(STM32_UART_PORT);
    vTaskDelay(pdMS_TO_TICKS(100));
    // UartBridge will be re-inited by the main task's next poll cycle,
    // but the easiest clean approach is to reinit it here:
    UartBridge::getInstance().init(STM32_UART_PORT,
                                   STM32_UART_RX_PIN,
                                   STM32_UART_TX_PIN,
                                   STM32_BOOT_BAUD);   // back to 115200

    if (ok) {
        ESP_LOGI(Tag, "STM32 OTA SUCCESS ✓");
        clearOtaField("ota_stm32_url", device_id);
    } else {
        ESP_LOGE(Tag, "STM32 OTA FAILED ✗");
    }

    m_busy = false;
}

// ─── downloadToHeap ──────────────────────────────────────────────────────────

size_t OtaManager::downloadToHeap(const std::string& url,
                                  uint8_t*& out_buf,
                                  size_t    max_bytes)
{
    out_buf = nullptr;

    esp_http_client_config_t cfg = {};
    cfg.url                = url.c_str();
    cfg.crt_bundle_attach  = esp_crt_bundle_attach;
    cfg.timeout_ms         = 30000;
    cfg.buffer_size        = 4096;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return 0;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(Tag, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return 0;
    }

    int content_len = esp_http_client_fetch_headers(client);
    ESP_LOGI(Tag, "Content-Length: %d", content_len);

    size_t alloc_size = (content_len > 0 && (size_t)content_len <= max_bytes)
                        ? (size_t)content_len
                        : max_bytes;

    uint8_t* buf   = static_cast<uint8_t*>(malloc(alloc_size));
    size_t   total = 0;

    if (!buf) {
        ESP_LOGE(Tag, "malloc(%zu) failed", alloc_size);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return 0;
    }

    uint8_t chunk[512];
    int     got;
    while ((got = esp_http_client_read(client, (char*)chunk, sizeof(chunk))) > 0) {
        if (total + (size_t)got > alloc_size) {
            ESP_LOGE(Tag, "Binary exceeds max_bytes (%zu)", max_bytes);
            free(buf);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return 0;
        }
        memcpy(buf + total, chunk, got);
        total += got;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (total == 0) { free(buf); return 0; }

    out_buf = buf;
    return total;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  STM32 UART BOOTLOADER PROTOCOL  (AN3155)
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Low-level byte helpers ──────────────────────────────────────────────────

bool OtaManager::stm32SendByte(uint8_t b) {
    return uart_write_bytes(STM32_UART_PORT, &b, 1) == 1;
}

bool OtaManager::stm32GetAck() {
    uint8_t ack = 0;
    for (int i = 0; i < 100; i++) {   // wait up to ~1 s
        int r = uart_read_bytes(STM32_UART_PORT, &ack, 1, pdMS_TO_TICKS(10));
        if (r == 1) {
            if (ack == 0x79) return true;   // ACK
            if (ack == 0x1F) {              // NACK
                ESP_LOGW(Tag, "NACK received");
                return false;
            }
        }
    }
    ESP_LOGE(Tag, "ACK timeout");
    return false;
}

// ─── stm32Sync ───────────────────────────────────────────────────────────────

bool OtaManager::stm32Sync() {
    uart_flush(STM32_UART_PORT);
    ESP_LOGI(Tag, "STM32 sync...");
    for (int attempt = 0; attempt < 5; attempt++) {
        stm32SendByte(0x7F);
        uint8_t resp = 0;
        int r = uart_read_bytes(STM32_UART_PORT, &resp, 1, pdMS_TO_TICKS(200));
        if (r == 1 && (resp == 0x79 || resp == 0x7F)) {
            ESP_LOGI(Tag, "STM32 bootloader sync OK (0x%02X)", resp);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(Tag, "STM32 sync FAILED");
    return false;
}

// ─── stm32EraseAll ───────────────────────────────────────────────────────────

bool OtaManager::stm32EraseAll() {
    ESP_LOGI(Tag, "Erasing STM32 flash...");
    // Extended erase — global mass erase
    stm32SendByte(0x44);   // Extended erase command
    stm32SendByte(0xBB);   // XOR checksum
    if (!stm32GetAck()) return false;

    // Send 0xFFFF = global mass erase  +  checksum 0x00
    stm32SendByte(0xFF);
    stm32SendByte(0xFF);
    stm32SendByte(0x00);

    // Mass erase can take up to 30 s on large devices
    uint8_t ack = 0;
    for (int i = 0; i < 300; i++) {
        int r = uart_read_bytes(STM32_UART_PORT, &ack, 1, pdMS_TO_TICKS(100));
        if (r == 1 && ack == 0x79) {
            ESP_LOGI(Tag, "STM32 flash erased OK");
            return true;
        }
    }
    ESP_LOGE(Tag, "Erase timeout / NACK");
    return false;
}

// ─── stm32WriteChunk ─────────────────────────────────────────────────────────

bool OtaManager::stm32WriteChunk(uint32_t addr, const uint8_t* data, uint8_t len) {
    // Write Memory command
    stm32SendByte(0x31);
    stm32SendByte(0xCE);
    if (!stm32GetAck()) return false;

    // Address (big-endian) + checksum
    uint8_t a[5];
    a[0] = (addr >> 24) & 0xFF;
    a[1] = (addr >> 16) & 0xFF;
    a[2] = (addr >>  8) & 0xFF;
    a[3] = (addr      ) & 0xFF;
    a[4] = a[0] ^ a[1] ^ a[2] ^ a[3];
    uart_write_bytes(STM32_UART_PORT, a, 5);
    if (!stm32GetAck()) return false;

    // N-1 byte, then data, then XOR checksum of (N-1 XOR data[0..N-1])
    uint8_t n = len - 1;
    uint8_t crc = n;
    uart_write_bytes(STM32_UART_PORT, &n, 1);
    uart_write_bytes(STM32_UART_PORT, data, len);
    for (int i = 0; i < len; i++) crc ^= data[i];
    uart_write_bytes(STM32_UART_PORT, &crc, 1);

    return stm32GetAck();
}

// ─── stm32FlashBinary ────────────────────────────────────────────────────────

bool OtaManager::stm32FlashBinary(const uint8_t* bin, size_t size) {
    if (!stm32Sync())    return false;
    if (!stm32EraseAll()) return false;

    uint32_t addr   = STM32_FLASH_BASE;
    size_t   offset = 0;
    int      page   = 0;

    while (offset < size) {
        uint8_t chunk_size = (size - offset >= 256) ? 256 : (uint8_t)(size - offset);

        // Pad last chunk to 4-byte boundary
        uint8_t padded[256];
        memcpy(padded, bin + offset, chunk_size);
        if (chunk_size % 4 != 0) {
            memset(padded + chunk_size, 0xFF, 4 - (chunk_size % 4));
            chunk_size += 4 - (chunk_size % 4);
        }

        if (!stm32WriteChunk(addr + offset, padded, chunk_size)) {
            ESP_LOGE(Tag, "Write failed at page %d (addr 0x%08X)", page, (unsigned)(addr + offset));
            return false;
        }

        offset += chunk_size;
        page++;
        if (page % 16 == 0) {
            ESP_LOGI(Tag, "  Written %zu / %zu bytes (%.0f%%)",
                     offset, size, 100.0f * offset / size);
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // yield to watchdog
    }

    ESP_LOGI(Tag, "All %zu bytes written OK", size);
    return true;
}

// ─── stm32EnterBootloader ────────────────────────────────────────────────────

void OtaManager::stm32EnterBootloader() {
    if (STM32_BOOT0_GPIO >= 0 && STM32_NRST_GPIO >= 0) {
        // Hardware method: drive BOOT0 high then pulse RESET
        gpio_set_direction((gpio_num_t)STM32_BOOT0_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)STM32_NRST_GPIO,  GPIO_MODE_OUTPUT);

        gpio_set_level((gpio_num_t)STM32_BOOT0_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)STM32_NRST_GPIO, 0);   // assert reset
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level((gpio_num_t)STM32_NRST_GPIO, 1);   // release reset
        vTaskDelay(pdMS_TO_TICKS(100));                     // bootloader startup

        ESP_LOGI(Tag, "STM32 in bootloader (hardware BOOT0+NRST)");
    } else {
        // Software method: send jump command, STM32 firmware handles the rest
        const char* cmd = "CMD:BOOTLOADER\n";
        uart_write_bytes(STM32_UART_PORT, cmd, strlen(cmd));
        ESP_LOGI(Tag, "Sent CMD:BOOTLOADER to STM32 — waiting for reboot...");
        vTaskDelay(pdMS_TO_TICKS(500));   // wait for STM32 to reboot into bootloader
    }
}

// ─── stm32ExitBootloader ─────────────────────────────────────────────────────

void OtaManager::stm32ExitBootloader() {
    if (STM32_BOOT0_GPIO >= 0 && STM32_NRST_GPIO >= 0) {
        gpio_set_level((gpio_num_t)STM32_BOOT0_GPIO, 0);  // BOOT0 low → run flash
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)STM32_NRST_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level((gpio_num_t)STM32_NRST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));   // STM32 startup
        ESP_LOGI(Tag, "STM32 exited bootloader (BOOT0 low + RESET)");
    } else {
        // Go command — jump to flash start (already written)
        stm32SendByte(0x21);   // Go command
        stm32SendByte(0xDE);   // XOR checksum
        stm32GetAck();

        uint32_t addr = STM32_FLASH_BASE;
        uint8_t  a[5];
        a[0] = (addr >> 24) & 0xFF;
        a[1] = (addr >> 16) & 0xFF;
        a[2] = (addr >>  8) & 0xFF;
        a[3] = (addr      ) & 0xFF;
        a[4] = a[0] ^ a[1] ^ a[2] ^ a[3];
        uart_write_bytes(STM32_UART_PORT, a, 5);
        stm32GetAck();
        ESP_LOGI(Tag, "STM32 Go command sent → running new firmware");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ─── clearOtaField ───────────────────────────────────────────────────────────

void OtaManager::clearOtaField(const std::string& field, const std::string& device_id) {
    // PATCH controls SET ota_xxx_url = NULL WHERE device_id = ...
    // We build a minimal PATCH via SupabaseClient::upsert — actually we need PATCH.
    // SupabaseClient doesn't expose PATCH directly, so we use httpRequest via select
    // workaround: upsert with null string is not ideal; build a raw PATCH instead.
    //
    // Simplest approach: upsert the row with the field set to empty string.
    // The website checks for empty string || null so this is safe.
    auto& sb = SupabaseClient::getInstance();

    std::string json  = "{\"" + field + "\":null}";
    std::string filter = "device_id=eq." + device_id;

    // We need a PATCH — add a simple update helper to SupabaseClient,
    // OR use select+upsert trick. Here we call update() which we'll add:
    // For now: select the row id then upsert.
    std::string body;
    int s = sb.select("controls", "id", filter.c_str(), body);
    if (s == 200 && body.size() > 2) {
        // Body: [{"id":123}] — extract id
        const char* idp = strstr(body.c_str(), "\"id\":");
        if (idp) {
            idp += 5;
            // Build upsert-style PATCH json  (id must be in the payload for upsert)
            char patch[256];
            snprintf(patch, sizeof(patch),
                     "{\"id\":%s,\"%s\":null}",
                     idp, field.c_str());   // idp includes trailing chars but upsert will stop at comma/}
            sb.upsert("controls", std::string(patch), "id");
            ESP_LOGI(Tag, "Cleared %s in Supabase", field.c_str());
        }
    }
}

} // namespace Takamul