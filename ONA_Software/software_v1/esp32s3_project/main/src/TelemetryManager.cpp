#include "inc/TelemetryManager.h"
#include "inc/SupabaseClient.h"
#include "inc/UartBridge.h"
#include "inc/Otamanager.h"
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
    while (*pos && (*pos == ':' || *pos == ' ')) pos++;

    if (strncmp(pos, "null", 4) == 0) return "";

    if (*pos != '"') return "";
    pos++;

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
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ─── pollControls ─────────────────────────────────────────────────────────────
//
// FIX v1.5 — Resend Throttle:
//
// PROBLEM:
//   pollControls() was calling sendControl() on every poll cycle (every 1s),
//   even when nothing changed. With force_wakeup=true stuck in Supabase, the
//   ESP32 was sending CMD:PUMP_ON to STM32 every single second non-stop.
//   This flooded UART2, causing STM32's HAL_UART_RxCpltCallback to fire
//   constantly and interfere with the telemetry TX window → sensor readings
//   appeared frozen on the dashboard.
//
// FIX:
//   - If controls CHANGED  → send to STM32 immediately (no delay).
//   - If controls UNCHANGED → throttle resend to once per kResendInterval (10s).
//   - The 9 intermediate poll cycles do nothing on UART2, leaving it free
//     for STM32 to send telemetry frames uninterrupted.

void TelemetryManager::pollControls() {
    auto& sb = SupabaseClient::getInstance();

    ESP_LOGI(Tag, "Polling controls for device_id: [%s]", m_device_id.c_str());

    std::string filter = "device_id=eq." + m_device_id + "&order=updated_at.desc&limit=1";

    std::string body;
    int status = sb.select("controls",
                     "pump_speed,status,target_pressure,updated_at,ota_esp32_url,ota_stm32_url,force_wakeup",
                           filter.c_str(),
                           body);

    if (status != 200 || body.empty() || body == "[]") {
        ESP_LOGI(Tag, "Controls poll FAILED: HTTP=%d body=[%s]", status, body.c_str());
        return;
    }

    const char* p = body.c_str();

    auto findVal = [&](const char* key) -> const char* {
        const char* pos = strstr(p, key);
        if (!pos) return nullptr;
        pos += strlen(key);
        while (*pos && (*pos == ':' || *pos == ' ')) pos++;
        return pos;
    };

    const char* v;

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

    // ── Read last-sent values from RTC memory ─────────────────────────────────
    auto& sleepMgr = SleepManager::getInstance();
    const RtcData* rtc = sleepMgr.getRtcData();

    bool changed;
    if (rtc == nullptr || !rtc->controls_valid) {
        changed = true;
        ESP_LOGI(Tag, "Controls: first cycle, sending unconditionally");
    } else {
        changed = (pump_on     != rtc->last_pump_on)
               || (speed_hz    != rtc->last_speed_hz)
               || (target_pres != rtc->last_target_pres);
    }

    // ── Resend throttle ───────────────────────────────────────────────────────
    // Changed  → send immediately.
    // Unchanged → send only once per kResendInterval (10s), skip the rest.
    {
        TickType_t now           = xTaskGetTickCount();
        bool due_for_resend      = (now - m_last_resend_tick) >= kResendInterval;

        if (changed || due_for_resend) {
            ControlCmd cmd;
            cmd.pump_on         = pump_on;
            cmd.speed_hz        = speed_hz;
            cmd.target_pressure = target_pres;

            UartBridge::getInstance().sendControl(cmd);
            m_last_resend_tick = now;

            if (changed) {
                ESP_LOGI(Tag, "→ STM32: pump=%s speed=%.1f Hz pres_sp=%.2f bar  [CHANGED]",
                         pump_on ? "ON" : "OFF", speed_hz, target_pres);
            } else {
                ESP_LOGI(Tag, "→ STM32: pump=%s speed=%.1f Hz pres_sp=%.2f bar  [resend]",
                         pump_on ? "ON" : "OFF", speed_hz, target_pres);
            }
        } else {
            ESP_LOGD(Tag, "→ STM32: no change, throttled (next resend in %lu ms)",
                     (unsigned long)((kResendInterval - (now - m_last_resend_tick)) * portTICK_PERIOD_MS));
        }
    }

    // ── Save current controls to RTC memory ───────────────────────────────────
    sleepMgr.saveLastControls(pump_on, speed_hz, target_pres);

    // ── OTA URLs ──────────────────────────────────────────────────────────────
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

    OtaManager::getInstance().checkAndRun(esp32_ota_url, stm32_ota_url, m_device_id);
}

// ─── registerDevice ───────────────────────────────────────────────────────────

void TelemetryManager::registerDevice() {
    auto& sb = SupabaseClient::getInstance();

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