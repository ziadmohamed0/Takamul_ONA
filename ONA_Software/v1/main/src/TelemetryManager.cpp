#include "inc/TelemetryManager.h"
#include "inc/SupabaseClient.h"
#include "inc/UartBridge.h"
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
// Expected Supabase response (array with one row):
//   [{"pump_speed":35.0,"status":true,"target_pressure":3.5,"updated_at":"..."}]
//

void TelemetryManager::pollControls() {
    auto& sb = SupabaseClient::getInstance();

    // Filter: device_id=eq.<mac>&order=updated_at.desc&limit=1
    std::string filter = "device_id=eq." + m_device_id + "&order=updated_at.desc&limit=1";

    std::string body;
    int status = sb.select("controls",
                           "pump_speed,status,target_pressure,updated_at",
                           filter.c_str(),
                           body);

    if (status != 200 || body.empty() || body == "[]") {
        ESP_LOGD(Tag, "Controls poll: no row or error (HTTP %d)", status);
        return;
    }

    // --- Minimal JSON parser for the known response shape ---
    // Body looks like: [{"pump_speed":35.0,"status":true,"target_pressure":3.50,...}]
    // We parse key:value pairs manually to avoid a JSON library dependency.

    bool  pump_on      = false;
    float speed_hz     = 0.0f;
    float target_pres  = 3.5f;

    const char* p = body.c_str();

    auto findVal = [&](const char* key) -> const char* {
        const char* pos = strstr(p, key);
        if (!pos) return nullptr;
        pos += strlen(key);
        while (*pos && (*pos == ':' || *pos == ' ')) pos++;
        return pos;
    };

    const char* v;

    v = findVal("\"status\"");
    if (v) pump_on = (strncmp(v, "true", 4) == 0);

    v = findVal("\"pump_speed\"");
    if (v) speed_hz = strtof(v, nullptr);

    v = findVal("\"target_pressure\"");
    if (v) target_pres = strtof(v, nullptr);

    ESP_LOGD(Tag, "Controls: pump=%s speed=%.1f Hz pres_sp=%.2f bar",
             pump_on ? "ON" : "OFF", speed_hz, target_pres);

    // Only forward to STM32 if something changed (debounce)
    bool changed = (pump_on      != m_last_pump_on)
                || (speed_hz     != m_last_speed)
                || (target_pres  != m_last_target_pres);

    if (changed) {
        ControlCmd cmd;
        cmd.pump_on        = pump_on;
        cmd.speed_hz       = speed_hz;
        cmd.target_pressure = target_pres;

        UartBridge::getInstance().sendControl(cmd);

        m_last_pump_on     = pump_on;
        m_last_speed       = speed_hz;
        m_last_target_pres = target_pres;

        ESP_LOGI(Tag, "→ STM32: pump=%s speed=%.1f Hz pres_sp=%.2f bar",
                 pump_on ? "ON" : "OFF", speed_hz, target_pres);
    }
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

} // namespace Takamul
