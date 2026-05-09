#include "inc/TelemetryManager.h"
#include "inc/SupabaseClient.h"
#include "inc/UartBridge.h"
#include "inc/Otamanager.h"      // ← OTA integration
#include "inc/Sleepmanager.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>


// Tiny JSON builder — avoids pulling in a full JSON lib for simple payloads
// Format: {"key":val,...} where val is either a number or a string
namespace {

void jsonAppendFloat(std::string& out, const char* key, float val) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.4f", val);
    out += '"';
    out += key;
    out += "\":";
    out += buf;
}

void jsonAppendStr(std::string& out, const char* key, const char* val) {
    out += '"';
    out += key;
    out += "\":\"";
    out += val;
    out += '"';
}

// ─── Extract a quoted string value from a JSON body ──────────────────────────
// Looks for "key":"value" and returns the value, or empty string if not found.
// Handles null values (returns empty string).
std::string extractJsonString(const char* body, const char* key) {
    const char* pos = strstr(body, key);
    if (!pos) return "";

    pos += strlen(key);
    // skip : and whitespace
    while (*pos && (*pos == ':' || *pos == ' ')) pos++;

    if (strncmp(pos, "null", 4) == 0) return "";   // SQL NULL → empty

    if (*pos != '"') return "";                     // not a string value
    pos++;                                          // skip opening quote

    const char* end = strchr(pos, '"');
    if (!end) return "";

    return std::string(pos, end);
}

} // anonymous namespace

namespace Takamul {

static const char* Tag = "TelemetryManager";

// ─── Singleton ────────────────────────────────────────────────────────────────

TelemetryManager& TelemetryManager::getInstance() {
    static TelemetryManager instance;
    return instance;
}

// ─── init ─────────────────────────────────────────────────────────────────────

void TelemetryManager::init(const std::string& device_id) {
    m_device_id   = device_id;
    m_frame_mutex = xSemaphoreCreateMutex();
    m_initialized = true;
    ESP_LOGI(Tag, "Initialized. Device: %s", device_id.c_str());
}

// ─── start / stop ─────────────────────────────────────────────────────────────

void TelemetryManager::start() {
    if (!m_initialized) { ESP_LOGE(Tag, "init() not called"); return; }
    m_running = true;
    registerDevice();
    xTaskCreate(uploadTask, "telem_upload", 8192, this, tskIDLE_PRIORITY + 4, nullptr);
    xTaskCreate(pollTask,   "ctrl_poll",    6144, this, tskIDLE_PRIORITY + 5, nullptr);
    ESP_LOGI(Tag, "Tasks started");
}

void TelemetryManager::stop() {
    m_running = false;
}

// ─── onSensorFrame ────────────────────────────────────────────────────────────

void TelemetryManager::onSensorFrame(const SensorFrame& frame) {
    if (m_frame_mutex && xSemaphoreTake(m_frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_latest_frame = frame;
        m_has_frame    = true;
        xSemaphoreGive(m_frame_mutex);
    }
}

// ─── uploadTask ───────────────────────────────────────────────────────────────

void TelemetryManager::uploadTask(void* arg) {
    auto* self = static_cast<TelemetryManager*>(arg);
    ESP_LOGI(Tag, "Upload task running — interval %lu ms", (unsigned long)kUploadInterval * portTICK_PERIOD_MS);

    while (self->m_running) {
        vTaskDelay(kUploadInterval);

        SensorFrame frame;
        bool valid = false;

        if (xSemaphoreTake(self->m_frame_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (self->m_has_frame) {
                frame = self->m_latest_frame;
                valid = true;
            }
            xSemaphoreGive(self->m_frame_mutex);
        }

        if (valid) {
            self->uploadTelemetry(frame);
        } else {
            ESP_LOGD(Tag, "No sensor frame yet, skipping upload");
        }
    }

    ESP_LOGI(Tag, "Upload task exiting");
    vTaskDelete(nullptr);
}

// ─── pollTask ─────────────────────────────────────────────────────────────────

void TelemetryManager::pollTask(void* arg) {
    auto* self = static_cast<TelemetryManager*>(arg);
    ESP_LOGI(Tag, "Poll task running — interval %lu ms", (unsigned long)kPollInterval * portTICK_PERIOD_MS);

    while (self->m_running) {
        self->pollControls();
        vTaskDelay(kPollInterval);
    }

    ESP_LOGI(Tag, "Poll task exiting");
    vTaskDelete(nullptr);
}

// ─── uploadTelemetry ──────────────────────────────────────────────────────────

void TelemetryManager::uploadTelemetry(const SensorFrame& frame) {
    struct Row { const char* sensor_type; float value; const char* unit; };
    const Row rows[] = {
        { "TDS",           frame.tds,           "ppm"   },
        { "TEMPERATURE",   frame.temperature,   "C"     },
        { "FLOW",          frame.flow,          "L/min" },
        { "PRESSURE",      frame.pressure,      "bar"   },
        { "DIFF_PRESSURE", frame.diff_pressure, "bar"   },
    };

    auto& sb = SupabaseClient::getInstance();
    ESP_LOGI(Tag, "Uploading batch | device: %s", m_device_id.c_str());

    for (const auto& row : rows) {
        // Build JSON: {"device_id":"...","sensor_type":"...","value":0.0000,"unit":"..."}
        std::string json;
        json.reserve(128);
        json += '{';
        jsonAppendStr(json,   "device_id",   m_device_id.c_str()); json += ',';
        jsonAppendStr(json,   "sensor_type", row.sensor_type);      json += ',';
        jsonAppendFloat(json, "value",       row.value);            json += ',';
        jsonAppendStr(json,   "unit",        row.unit);
        json += '}';

        int status = sb.insert("telemetry", json);
        if (status == 201) {
            ESP_LOGI(Tag, "  ✓ %-14s = %.4f %s", row.sensor_type, row.value, row.unit);
        } else {
            ESP_LOGW(Tag, "  ✗ %-14s  HTTP %d", row.sensor_type, status);
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // small gap to avoid burst
    }
}

// ─── pollControls ─────────────────────────────────────────────────────────────
//
// Fetches the latest controls row for this device from Supabase.
//
// Expected Supabase response shape:
//   [{
//     "pump_speed": 35.0,
//     "status": true,
//     "target_pressure": 3.5,
//     "updated_at": "2025-01-01T00:00:00Z",
//     "ota_esp32_url": "https://github.com/.../firmware.bin",   ← or null
//     "ota_stm32_url": "https://github.com/.../firmware.bin"    ← or null
//   }]
//
// FIX: The previous implementation used instance variables (m_last_pump_on,
// m_last_speed, m_last_target_pres) to detect changes and only send to STM32
// when something changed. These variables are reset to zero on every wakeup
// from deep sleep (the device does a full reboot each cycle), so a command
// that was already in Supabase before the device woke up would never look
// "changed" — it would always compare against false/0.0, causing commands
// sent while the device was sleeping to be silently dropped.
//
// The fix has two parts:
//   1. Always send the current Supabase controls to STM32 every wakeup cycle,
//      regardless of what the "previous" values were. This guarantees that
//      whatever the website writes is always forwarded on the next wakeup.
//   2. Use RTC memory (via SleepManager::RtcData) to store the last-sent
//      values across deep sleep, so the optional change-detection in
//      continuous (non-sleep) mode still works correctly.

void TelemetryManager::pollControls() {
    auto& sb = SupabaseClient::getInstance();

    // DEBUG: log exact device_id being used
    ESP_LOGI(Tag, "Polling controls for device_id: [%s]", m_device_id.c_str());

    // Filter: fetch only our device row, latest first
    // TEMP: removed device_id filter to diagnose empty response
    std::string filter = "order=updated_at.desc&limit=1";
    // std::string filter = "device_id=eq." + m_device_id + "&order=updated_at.desc&limit=1";

    std::string body;
    int status = sb.select("controls",
                     "pump_speed,status,target_pressure,updated_at,ota_esp32_url,ota_stm32_url,force_wakeup",
                           filter.c_str(),
                           body);

    if (status != 200 || body.empty() || body == "[]") {
        ESP_LOGI(Tag, "Controls poll FAILED: HTTP=%d body=[%s]", status, body.c_str());        return;
    }

    // ─── Minimal JSON parser ─────────────────────────────────────────────────
    // Body: [{...}]  — we work directly on the C string to avoid heap churn.

    const char* p = body.c_str();

    auto findVal = [&](const char* key) -> const char* {
        const char* pos = strstr(p, key);
        if (!pos) return nullptr;
        pos += strlen(key);
        while (*pos && (*pos == ':' || *pos == ' ')) pos++;
        return pos;
    };

    const char* v;

    // ── Pump / speed / pressure ───────────────────────────────────────────────
    bool  pump_on     = false;
    float speed_hz    = 0.0f;
    float target_pres = 3.5f;

    v = findVal("\"status\"");
    if (v) pump_on = (strncmp(v, "true", 4) == 0);

    v = findVal("\"pump_speed\"");
    if (v) speed_hz = strtof(v, nullptr);

    v = findVal("\"target_pressure\"");
    if (v) target_pres = strtof(v, nullptr);

    ESP_LOGD(Tag, "Controls: pump=%s speed=%.1f Hz pres_sp=%.2f bar",
             pump_on ? "ON" : "OFF", speed_hz, target_pres);

    // ── FIX: Read last-sent values from RTC memory ────────────────────────────
    // RTC memory persists across deep sleep, unlike instance variables which
    // reset to zero on every boot. Using RTC memory means we can correctly
    // detect what was last sent to STM32 even after sleeping.
    auto& sleepMgr = SleepManager::getInstance();
    const RtcData* rtc = sleepMgr.getRtcData();

    // Determine if values have actually changed compared to last sent values.
    // controls_valid guards the first boot where RTC holds no previous values.
    bool changed;
    if (rtc == nullptr || !rtc->controls_valid) {
        // First boot or RTC memory was reset — always send
        changed = true;
        ESP_LOGI(Tag, "Controls: first cycle, sending unconditionally");
    } else {
        changed = (pump_on     != rtc->last_pump_on)
               || (speed_hz    != rtc->last_speed_hz)
               || (target_pres != rtc->last_target_pres);
    }

    // ── Always send to STM32 every wakeup cycle ───────────────────────────────
    // Even if `changed` is false we still forward the command. This is safe
    // because the STM32 is stateless after reset and needs to re-receive its
    // operating parameters on every wakeup. The `changed` flag is only used
    // for logging clarity.
    {
        ControlCmd cmd;
        cmd.pump_on         = pump_on;
        cmd.speed_hz        = speed_hz;
        cmd.target_pressure = target_pres;

        UartBridge::getInstance().sendControl(cmd);

        if (changed) {
            ESP_LOGI(Tag, "→ STM32: pump=%s speed=%.1f Hz pres_sp=%.2f bar  [CHANGED]",
                     pump_on ? "ON" : "OFF", speed_hz, target_pres);
        } else {
            ESP_LOGI(Tag, "→ STM32: pump=%s speed=%.1f Hz pres_sp=%.2f bar  [resend]",
                     pump_on ? "ON" : "OFF", speed_hz, target_pres);
        }
    }

    // ── Save current controls to RTC memory ───────────────────────────────────
    // These will be read on the next wakeup cycle (after deep sleep) to
    // correctly detect future changes from the website.
    sleepMgr.saveLastControls(pump_on, speed_hz, target_pres);

    // ── OTA URLs ──────────────────────────────────────────────────────────────
    // extractJsonString handles both "value" and null gracefully (returns "").
    std::string esp32_ota_url = extractJsonString(p, "\"ota_esp32_url\"");
    std::string stm32_ota_url = extractJsonString(p, "\"ota_stm32_url\"");

    if (!esp32_ota_url.empty()) {
        ESP_LOGI(Tag, "OTA ESP32 URL detected: %s", esp32_ota_url.c_str());
    }
    if (!stm32_ota_url.empty()) {
        ESP_LOGI(Tag, "OTA STM32 URL detected: %s", stm32_ota_url.c_str());
    }

    // ── Force wakeup flag ─────────────────────────────────────────────────────
    v = findVal("\"force_wakeup\"");
    if (v && strncmp(v, "true", 4) == 0) {
        ESP_LOGI(Tag, "force_wakeup flag detected → shortening next sleep");
        sleepMgr.setForceWakeup(true);
    }
    ESP_LOGI(Tag, "Controls poll HTTP=%d body=%s", status, body.c_str());
    // Delegate to OtaManager — internally throttled, safe to call every poll.
    OtaManager::getInstance().checkAndRun(esp32_ota_url, stm32_ota_url, m_device_id);
}

// ─── registerDevice ───────────────────────────────────────────────────────────

void TelemetryManager::registerDevice() {
    auto& sb = SupabaseClient::getInstance();

    // UPSERT into devices table — safe to call every boot.
    // devices table schema: device_id (text PK), last_seen (timestamptz), firmware_ver (text)
    std::string json;
    json.reserve(128);
    json += '{';
    jsonAppendStr(json, "device_id",    m_device_id.c_str()); json += ',';
    jsonAppendStr(json, "firmware_ver", "1.0.0");
    json += '}';

    int status = sb.upsert("devices", json, "device_id");
    if (status == 200 || status == 201) {
        ESP_LOGI(Tag, "Device registered: %s", m_device_id.c_str());
    } else {
        ESP_LOGW(Tag, "Device register failed HTTP %d (table may not exist yet)", status);
    }
}

void TelemetryManager::uploadOnce() {
    if (m_has_frame) uploadTelemetry(m_latest_frame);
}

void TelemetryManager::pollOnce() {
    pollControls();
}

} // namespace Takamul