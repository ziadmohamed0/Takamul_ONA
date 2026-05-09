/**
 * ============================================================================
 *  Takamul SCADA — ESP32 Firmware
 *  Target  : ESP32-S3 N16R8 + ESP-IDF v5.x  (C++17)
 *
 *  Boot flow (every cycle after deep sleep):
 *   1. SleepManager::init() + checkWakeupReason()
 *   2. NVS init
 *   3. WiFi connect (from credentials saved in NVS)
 *      first boot only -> AP mode + captive portal
 *   4. UART init (STM32 bridge)
 *   5. Supabase init
 *   6. Get sensor frame from STM32
 *   7. Check thresholds -> if anomaly, shorten next sleep
 *   8. Upload telemetry to Supabase
 *   9. Get controls (pump/speed) + send to STM32
 *  10. Check OTA
 *  11. SleepManager::prepareAndSleep() -> deep sleep (5 minutes)
 *      [SLEEP DISABLED] -> continuous loop mode
 *
 *  Wakeup sources:
 *   - Timer (5 min)         -> Normal state
 *   - EXT1 GPIO IO21 HIGH   -> STM32 raised alert (sensor fault / pump fault)
 *   - (force_wakeup flag)   -> Website requested urgent wakeup -> sleep 30s only
 *
 *  Data flow:
 *   STM32 -> UART -> UartBridge -> TelemetryManager -> Supabase (telemetry)
 *   Supabase (controls) -> TelemetryManager -> UartBridge -> STM32
 * ============================================================================
 */

#include "main.h"
#include "esp_log.h"
#include "esp_system.h"

static const char* TAG = "main";

// ─── Helper: read MAC as "AA:BB:CC:DD:EE:FF" ─────────────────────────────────
static std::string readMacString() {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(buf);
}

// ─── Helper: connect WiFi (STA or AP fallback) ───────────────────────────────
static bool connectWifi(Takamul::WifiManager& wifi, Takamul::NVSManager& nvs) {
    std::string nvs_ssid, nvs_pass;

    Takamul::NVSHandle handle("wifi_config", NVS_READONLY);
    bool has_creds = handle.isValid()
                  && handle.getString("ssid",     nvs_ssid) == ESP_OK
                  && handle.getString("password", nvs_pass) == ESP_OK
                  && !nvs_ssid.empty();

    if (!has_creds) {
        // first boot or credentials cleared -> provisioning
        ESP_LOGI(TAG, "No WiFi credentials -> provisioning mode");

        // do not sleep in provisioning mode
        Takamul::SleepManager::getInstance().disableSleep();

        wifi.startAP();
        Takamul::WebServer::getInstance().start();

        while (!wifi.waitForStaIp(pdMS_TO_TICKS(1000))) {
            ESP_LOGD(TAG, "  ...waiting for IP");
        }

        Takamul::WebServer::getInstance().stop();
        ESP_LOGI(TAG, "Provisioning complete");

        // keep sleep disabled — app_main will enter loop mode
        return true;
    }

    // STA mode
    ESP_LOGI(TAG, "Connecting to: %s", nvs_ssid.c_str());
    wifi.startSTA(nvs_ssid, nvs_pass);

    if (wifi.waitForStaIp(pdMS_TO_TICKS(15000))) {
        return true;
    }

    // connection failed -> AP fallback
    ESP_LOGW(TAG, "Failed to connect '%s' — AP fallback", nvs_ssid.c_str());
    Takamul::SleepManager::getInstance().disableSleep();
    wifi.startAP();
    Takamul::WebServer::getInstance().start();

    while (!wifi.waitForStaIp(pdMS_TO_TICKS(1000))) {
        ESP_LOGD(TAG, "  ...waiting for IP (fallback)");
    }

    Takamul::WebServer::getInstance().stop();
    return true;
}

// ─── app_main ─────────────────────────────────────────────────────────────────

extern "C" void app_main(void) {

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 1 — Sleep Manager
    // ══════════════════════════════════════════════════════════════════════════

    auto& sleepMgr = Takamul::SleepManager::getInstance();
    sleepMgr.init();
    sleepMgr.disableSleep();   // ← SLEEP DISABLED: device stays awake permanently
    Takamul::WakeupReason reason = sleepMgr.checkWakeupReason();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Takamul SCADA — ESP32-S3 Firmware    ");
    ESP_LOGI(TAG, "  Wakeup: %s",
             reason == Takamul::WakeupReason::Timer     ? "TIMER" :
             reason == Takamul::WakeupReason::GpioAlert ? "GPIO ALERT" :
             reason == Takamul::WakeupReason::FirstBoot ? "FIRST BOOT" : "UNKNOWN");
    ESP_LOGI(TAG, "========================================");

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 2 — NVS
    // ══════════════════════════════════════════════════════════════════════════

    auto& nvsMgr = Takamul::NVSManager::getInstance();
    nvsMgr.init();

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 3 — WiFi
    // ══════════════════════════════════════════════════════════════════════════

    auto& wifiMgr = Takamul::WifiManager::getInstance();
    wifiMgr.init();
    connectWifi(wifiMgr, nvsMgr);
    ESP_LOGI(TAG, "WiFi connected ✓");

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 4 — Device Identity
    // ══════════════════════════════════════════════════════════════════════════

    std::string device_id = readMacString();
    ESP_LOGI(TAG, "Device ID: %s", device_id.c_str());

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 5 — Supabase
    // ══════════════════════════════════════════════════════════════════════════

    auto& supabase = Takamul::SupabaseClient::getInstance();
    supabase.init(CONFIG_TAKAMUL_SUPABASE_URL, CONFIG_TAKAMUL_SUPABASE_KEY);

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 6 — UART Bridge (ESP32 <-> STM32)
    // ══════════════════════════════════════════════════════════════════════════

    auto& uart = Takamul::UartBridge::getInstance();
    uart.init(UART_STM32_PORT, UART_STM32_RX_PIN, UART_STM32_TX_PIN, UART_STM32_BAUD);

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 7 — Telemetry Manager (continuous mode)
    // ══════════════════════════════════════════════════════════════════════════

    auto& telem = Takamul::TelemetryManager::getInstance();
    telem.init(device_id);

    // Start UART — frames will be forwarded to TelemetryManager continuously
    uart.start([&telem](const Takamul::SensorFrame& f) {
        telem.onSensorFrame(f);
    });

    // Start telemetry tasks (upload + poll loops)
    telem.start();

    ESP_LOGI(TAG, "Sleep disabled — running in continuous mode");
    ESP_LOGI(TAG, "Running in continuous mode — tasks started");
    // ── Heartbeat loop ────────────────────────────────────────────────────────
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "♥ Heartbeat — continuous mode");
    }
}