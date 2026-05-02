/**
 * ============================================================================
 *  Takamul SCADA — ESP32 Firmware
 *  Target  : ESP32 + ESP-IDF v5.x  (C++17)
 *
 *  Boot flow:
 *   1. NVS init
 *   2. WiFi init
 *   3. If no WiFi credentials → AP mode + WebServer (captive portal)
 *      User connects phone, opens browser, enters SSID/password → saved to NVS
 *   4. STA mode → wait for IP
 *   5. UART init (STM32 bridge)
 *   6. Supabase init → TelemetryManager start
 *   7. Loop: UART RX feeds TelemetryManager; upload + poll tasks handle the rest
 *
 *  Data flow:
 *   STM32 → UART → UartBridge → TelemetryManager → Supabase (telemetry)
 *   Supabase (controls) → TelemetryManager → UartBridge → STM32
 * ============================================================================
 */

#include "main.h"
#include "esp_log.h"
#include "esp_system.h"

static const char* TAG = "main";

// ─── Global manager pointers ─────────────────────────────────────────────────
Takamul::NVSManager*       NVS_OBJ      = nullptr;
Takamul::WifiManager*      WiFi_OBJ     = nullptr;
Takamul::SupabaseClient*   Supabase_OBJ = nullptr;
Takamul::UartBridge*       Uart_OBJ     = nullptr;
Takamul::TelemetryManager* Telem_OBJ    = nullptr;

// ─── Helper: read MAC as "AA:BB:CC:DD:EE:FF" ─────────────────────────────────
static std::string readMacString() {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(buf);
}

// ─── app_main ─────────────────────────────────────────────────────────────────

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Takamul SCADA — ESP32 Firmware v1.0  ");
    ESP_LOGI(TAG, "========================================");

    // ── 1. NVS ───────────────────────────────────────────────────────────────
    NVS_OBJ = &Takamul::NVSManager::getInstance();
    NVS_OBJ->init();

    // ── 2. WiFi init ─────────────────────────────────────────────────────────
    WiFi_OBJ = &Takamul::WifiManager::getInstance();
    WiFi_OBJ->init();

    // ── 3. Check for saved credentials ───────────────────────────────────────
    std::string nvs_ssid, nvs_pass;
    {
        Takamul::NVSHandle nvs("wifi_config", NVS_READONLY);
        bool has_creds = nvs.isValid()
                      && nvs.getString("ssid",     nvs_ssid) == ESP_OK
                      && nvs.getString("password", nvs_pass) == ESP_OK
                      && !nvs_ssid.empty();

        if (!has_creds) {
            // ── No credentials → AP + captive portal ─────────────────────────
            ESP_LOGI(TAG, "No WiFi credentials found → provisioning mode");
            WiFi_OBJ->startAP();
            Takamul::WebServer::getInstance().start();

            // Block until STA gets an IP (user enters credentials via portal)
            ESP_LOGI(TAG, "Waiting for user to provision WiFi...");
            while (!WiFi_OBJ->waitForStaIp(pdMS_TO_TICKS(1000))) {
                ESP_LOGD(TAG, "  ...waiting for IP");
            }

            // Tear down provisioning services
            Takamul::WebServer::getInstance().stop();
            ESP_LOGI(TAG, "Provisioning complete — AP and WebServer stopped");
        } else {
            // ── Saved credentials → connect directly ─────────────────────────
            ESP_LOGI(TAG, "Saved credentials found, connecting to: %s", nvs_ssid.c_str());
            WiFi_OBJ->startSTA(nvs_ssid, nvs_pass);

            // Wait up to 15 s for connection
            if (!WiFi_OBJ->waitForStaIp(pdMS_TO_TICKS(15000))) {
                // Connection failed → fall back to AP provisioning
                ESP_LOGW(TAG, "Failed to connect to '%s' — falling back to provisioning", nvs_ssid.c_str());
                WiFi_OBJ->startAP();
                Takamul::WebServer::getInstance().start();

                while (!WiFi_OBJ->waitForStaIp(pdMS_TO_TICKS(1000))) {
                    ESP_LOGD(TAG, "  ...waiting for IP after fallback");
                }

                Takamul::WebServer::getInstance().stop();
                ESP_LOGI(TAG, "Provisioning complete after fallback");
            }
        }
    }

    ESP_LOGI(TAG, "WiFi connected — IP obtained ✓");

    // ── 4. Read device identity ───────────────────────────────────────────────
    std::string device_id = readMacString();
    ESP_LOGI(TAG, "Device ID (MAC): %s", device_id.c_str());

    // ── 5. Supabase init ──────────────────────────────────────────────────────
    Supabase_OBJ = &Takamul::SupabaseClient::getInstance();
    Supabase_OBJ->init(CONFIG_TAKAMUL_SUPABASE_URL, CONFIG_TAKAMUL_SUPABASE_KEY);

    // ── 6. UART bridge (ESP32 ↔ STM32) ───────────────────────────────────────
    Uart_OBJ = &Takamul::UartBridge::getInstance();
    Uart_OBJ->init(UART_STM32_PORT, UART_STM32_RX_PIN, UART_STM32_TX_PIN, UART_STM32_BAUD);

    // ── 7. Telemetry manager ──────────────────────────────────────────────────
    Telem_OBJ = &Takamul::TelemetryManager::getInstance();
    Telem_OBJ->init(device_id);

    // Wire UART callback into TelemetryManager
    Uart_OBJ->start([](const Takamul::SensorFrame& frame) {
        Takamul::TelemetryManager::getInstance().onSensorFrame(frame);
    });

    // Start upload + poll tasks (they run forever in the background)
    Telem_OBJ->start();

    ESP_LOGI(TAG, "All subsystems started — entering idle loop");

    // ── Main task idle loop ───────────────────────────────────────────────────
    // The actual work is done by:
    //   • UartBridge::rxTask      — receives STM32 frames
    //   • TelemetryManager::uploadTask — uploads to Supabase
    //   • TelemetryManager::pollTask   — fetches controls from Supabase → STM32
    //
    // This task just keeps the stack alive and logs a heartbeat.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(30000)); // 30 s heartbeat
        ESP_LOGI(TAG, "♥ Heartbeat — system running");
    }
}
