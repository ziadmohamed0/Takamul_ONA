#include "inc/UartBridge.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>

namespace Takamul {

static const char* Tag = "UartBridge";

// RX ring-buffer size (bytes)
static constexpr int kRxBufSize = 1024;
// Maximum line length from STM32
static constexpr int kMaxLine   = 256;

// ─── Singleton ────────────────────────────────────────────────────────────────

UartBridge& UartBridge::getInstance() {
    static UartBridge instance;
    return instance;
}

// ─── init ─────────────────────────────────────────────────────────────────────

void UartBridge::init(uart_port_t port, int rx_pin, int tx_pin, int baud) {
    m_port = port;

    uart_config_t cfg = {};
    cfg.baud_rate           = baud;
    cfg.data_bits           = UART_DATA_8_BITS;
    cfg.parity              = UART_PARITY_DISABLE;
    cfg.stop_bits           = UART_STOP_BITS_1;
    cfg.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    cfg.rx_flow_ctrl_thresh = 0;
    cfg.source_clk          = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port, kRxBufSize * 2, kRxBufSize * 2, 0, nullptr, 0));

    m_initialized = true;
    ESP_LOGI(Tag, "UART%d init OK — baud=%d RX=%d TX=%d", (int)port, baud, rx_pin, tx_pin);
}

// ─── start ────────────────────────────────────────────────────────────────────

void UartBridge::start(SensorCallback cb) {
    if (!m_initialized) {
        ESP_LOGE(Tag, "Call init() before start()");
        return;
    }
    m_callback = std::move(cb);
    m_running  = true;
    // FIX: store task handle so stop() can wait for the task to finish
    xTaskCreate(rxTask, "uart_rx", 4096, this, tskIDLE_PRIORITY + 6, &m_task_handle);
    ESP_LOGI(Tag, "RX task started");
}

// ─── stop ─────────────────────────────────────────────────────────────────────
//
// FIX: Race condition fix.
// Old code called uart_driver_delete() directly from SleepManager while the
// rxTask was still running and blocked inside uart_read_bytes().
// This caused a spinlock_acquire assert crash because the UART driver's
// internal ring-buffer was accessed after being freed.
//
// Fix: set m_running = false so the task exits its loop on the next 100ms
// timeout, then wait for the task to fully exit (via task handle notification),
// and only THEN delete the UART driver.

void UartBridge::stop() {
    if (!m_initialized) return;

    // 1. Signal the task to stop
    m_running = false;

    // 2. Wait for the task to exit (max 500ms — task polls every 100ms)
    if (m_task_handle != nullptr) {
        // Give the task time to notice m_running=false and call vTaskDelete
        // We poll the handle until it becomes invalid
        for (int i = 0; i < 10; i++) {
            if (eTaskGetState(m_task_handle) == eDeleted) break;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        m_task_handle = nullptr;
    }

    // 3. Now safe to delete the driver — no task is touching it anymore
    uart_driver_delete(m_port);
    m_initialized = false;
    ESP_LOGI(Tag, "UART driver stopped");
}

// ─── sendControl ──────────────────────────────────────────────────────────────

void UartBridge::sendControl(const ControlCmd& cmd) {
    if (!m_initialized) return;

    char buf[128];
    int len;
    if (cmd.pump_on) {
        len = snprintf(buf, sizeof(buf),
                       "CMD:PUMP_ON,SPEED:%.1f,PRES_SP:%.2f\n",
                       cmd.speed_hz, cmd.target_pressure);
    } else {
        len = snprintf(buf, sizeof(buf), "CMD:PUMP_OFF\n");
    }

    uart_write_bytes(m_port, buf, len);
    ESP_LOGI(Tag, "TX -> %.*s", len - 1, buf); // -1 to skip the \n in log
}

// ─── rxTask ───────────────────────────────────────────────────────────────────

void UartBridge::rxTask(void* arg) {
    auto* self = static_cast<UartBridge*>(arg);
    char  line[kMaxLine];
    int   line_pos = 0;

    ESP_LOGI(Tag, "RX task running on UART%d", (int)self->m_port);

    while (self->m_running) {
        uint8_t byte;
        int r = uart_read_bytes(self->m_port, &byte, 1, pdMS_TO_TICKS(100));
        if (r != 1) continue;

        if (byte == '\0') continue;   
        if (byte == '\n' || byte == '\r') {
            if (line_pos > 0) {
                line[line_pos] = '\0';
                ESP_LOGD(Tag, "RX <- %s", line);
                SensorFrame frame = parseLine(line);
                if (frame.valid && self->m_callback) {
                    self->m_callback(frame);
                } else if (!frame.valid) {
                    ESP_LOGW(Tag, "Failed to parse frame [%d bytes]: ", line_pos);
                    for(int i = 0; i < line_pos && i < 20; i++) {
                        ESP_LOGW(Tag, "  [%d] = 0x%02X '%c'", i, (uint8_t)line[i],
                                (line[i] >= 32 && line[i] < 127) ? line[i] : '.');
                    }
                }
                line_pos = 0;
            }
        } else {
            if (line_pos < kMaxLine - 1) {
                line[line_pos++] = static_cast<char>(byte);
            } else {
                // Line too long — discard
                ESP_LOGW(Tag, "Line overflow, discarding");
                line_pos = 0;
            }
        }
    }

    ESP_LOGI(Tag, "RX task exiting");
    vTaskDelete(nullptr);
}

// ─── parseLine ────────────────────────────────────────────────────────────────
//
// Expected format (comma-separated key:value pairs, case-insensitive keys):
//   TDS:290.50,TEMP:22.43,FLOW:41.08,PRES:3.79,DIFF:0.40
//
// Fields may appear in any order. Unknown fields are silently ignored.
//

SensorFrame UartBridge::parseLine(const char* line) {
    SensorFrame frame;

    // Work on a mutable copy
    char buf[kMaxLine];
    strncpy(buf, line, kMaxLine - 1);
    buf[kMaxLine - 1] = '\0';

    bool got_tds  = false;
    bool got_temp = false;
    bool got_flow = false;
    bool got_pres = false;
    bool got_diff = false;

    char* token = strtok(buf, ",");
    while (token != nullptr) {
        char* colon = strchr(token, ':');
        if (colon) {
            *colon = '\0';
            const char* key = token;
            const char* val = colon + 1;

            if (strcasecmp(key, "TDS") == 0)  { frame.tds           = strtof(val, nullptr); got_tds  = true; }
            else if (strcasecmp(key, "TEMP") == 0) { frame.temperature   = strtof(val, nullptr); got_temp = true; }
            else if (strcasecmp(key, "FLOW") == 0) { frame.flow          = strtof(val, nullptr); got_flow = true; }
            else if (strcasecmp(key, "PRES") == 0) { frame.pressure      = strtof(val, nullptr); got_pres = true; }
            else if (strcasecmp(key, "DIFF") == 0) { frame.diff_pressure = strtof(val, nullptr); got_diff = true; }
        }
        token = strtok(nullptr, ",");
    }

    // Require all 5 fields to consider the frame valid
    frame.valid = got_tds && got_temp && got_flow && got_pres && got_diff;
    return frame;
}

} // namespace Takamul