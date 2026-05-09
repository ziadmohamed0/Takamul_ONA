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
 *
 *  Wakeup sources:
 *   - Timer (5 min)         -> Normal state
 *   - EXT1 GPIO IO38 HIGH   -> STM32 raised alert (sensor fault / pump fault)
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

        // resume sleeping after provisioning
        Takamul::SleepManager::getInstance().enableSleep();
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
    Takamul::SleepManager::getInstance().enableSleep();
    return true;
}

// ─── app_main ─────────────────────────────────────────────────────────────────
// called at the beginning of each wakeup from deep sleep (like reset but RTC memory persists)

extern "C" void app_main(void) {

    // ══════════════════════════════════════════════════════════════════════════
    //  STEP 1 — Sleep Manager: determine wakeup reason
    // ══════════════════════════════════════════════════════════════════════════

    auto& sleepMgr = Takamul::SleepManager::getInstance();
    sleepMgr.init();
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
    //  STEP 7 — Telemetry Manager
    //           runs in one-shot mode rather than loop in deep sleep mode
    // ══════════════════════════════════════════════════════════════════════════

    auto& telem = Takamul::TelemetryManager::getInstance();
    telem.init(device_id);

    // Get sensor frame from STM32 (3 seconds timeout)
    // UartBridge collects one frame and returns
    Takamul::SensorFrame frame;
    bool got_frame = false;

    uart.start([&](const Takamul::SensorFrame& f) {
        if (f.valid && !got_frame) {
            frame     = f;
            got_frame = true;
        }
    });

    // wait for frame for 3 seconds
    for (int i = 0; i < 30 && !got_frame; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (got_frame) {
        ESP_LOGI(TAG, "Frame: TDS=%.1f TEMP=%.1f FLOW=%.1f PRES=%.2f DIFF=%.2f",
                 frame.tds, frame.temperature, frame.flow,
                 frame.pressure, frame.diff_pressure);

        // ── STEP 8: check thresholds ─────────────────────────────────────────
        bool anomaly = sleepMgr.checkThresholds(
            frame.tds, frame.pressure, frame.flow,
            frame.temperature, frame.diff_pressure
        );
        if (anomaly) {
            ESP_LOGW(TAG, "Anomaly detected — will upload immediately + shorten sleep");
        }

        // Save reading to RTC memory for comparison in the next cycle
        sleepMgr.saveLastFrame(
            frame.tds, frame.pressure, frame.flow,
            frame.temperature, frame.diff_pressure
        );

        // ── STEP 9: upload telemetry ──────────────────────────────────────────
        telem.onSensorFrame(frame);

        // uploadTelemetry immediately (not via task - one-shot mode)
        telem.uploadOnce();

    } else {
        ESP_LOGW(TAG, "No sensor frame received from STM32 in 3s");
    }

    // ── STEP 10: get controls from Supabase and send to STM32 ──────────────────
    // this also fetches OTA URLs and invokes OtaManager
    telem.pollOnce();

    // if OTA exists -> OtaManager runs and we won't sleep before it finishes
    // (OtaManager triggers esp_restart at the end for ESP32 OTA)
    // (STM32 OTA finishes, URL gets cleared, then sleep)

    // ── STEP 11: Sleep ─────────────────────────────────────────────────────────
    // prepareAndSleep stops WiFi + UART and enters deep sleep
    // if sleep disabled (provisioning) -> returns without sleeping
    sleepMgr.prepareAndSleep();

    // ─── This block runs only if sleep disabled ───────────────────────────────
    // In provisioning or debug mode — keep running
    ESP_LOGI(TAG, "Sleep disabled — entering normal loop mode");

    // restart tasks normally
    uart.start([](const Takamul::SensorFrame& f) {
        Takamul::TelemetryManager::getInstance().onSensorFrame(f);
    });
    telem.start();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "♥ Heartbeat — sleep disabled mode");
    }
}