/**
 * ============================================================================
 *  SleepManager.cpp — Takamul SCADA
 *
 *  Deep sleep cycle:
 *
 *  app_main()
 *    ↓ SleepManager::init()
 *    ↓ SleepManager::checkWakeupReason()   <- Determine wakeup reason
 *    ↓ ... WiFi + UART + Supabase ...
 *    ↓ SleepManager::saveLastFrame()       <- Save reading to RTC
 *    ↓ SleepManager::prepareAndSleep()     <- Sleep (does not return)
 *    ↓↓↓  After SLEEP_DURATION_US  ↓↓↓
 *    ↓ app_main() from start (like reset but RTC memory persists)
 *
 *  Wakeup sources:
 *    - Timer     -> Normal state every 5 minutes
 *    - EXT1 GPIO -> STM32 pulled IO38 HIGH (alert/fault)
 * ============================================================================
 */

#include "inc/Sleepmanager.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>

// RTC_DATA_ATTR -> These variables persist in RTC SRAM during deep sleep
RTC_DATA_ATTR static Takamul::RtcData g_rtc = {};

static const char* Tag = "SleepManager";

namespace Takamul {

// ─── Singleton ────────────────────────────────────────────────────────────────

SleepManager& SleepManager::getInstance() {
    static SleepManager instance;
    return instance;
}

// ─── init ─────────────────────────────────────────────────────────────────────

void SleepManager::init() {
    ESP_LOGI(Tag, "SleepManager init");

    // Initialize GPIO for STM32 wakeup
    if (STM32_ALERT_GPIO >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask  = (1ULL << STM32_ALERT_GPIO);
        io.mode          = GPIO_MODE_INPUT;
        io.pull_down_en  = GPIO_PULLDOWN_ENABLE;  // Default LOW, STM32 pulls HIGH on alert
        io.pull_up_en    = GPIO_PULLUP_DISABLE;
        io.intr_type     = GPIO_INTR_DISABLE;
        gpio_config(&io);
        ESP_LOGI(Tag, "Alert GPIO IO%d configured (active HIGH)", STM32_ALERT_GPIO);
    }
}

// ─── checkWakeupReason ────────────────────────────────────────────────────────

WakeupReason SleepManager::checkWakeupReason() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    switch (cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            m_reason = WakeupReason::Timer;
            g_rtc.wakeup_count++;
            ESP_LOGI(Tag, "⏰ Wakeup: TIMER (cycle #%lu)", (unsigned long)g_rtc.wakeup_count);
            break;

        case ESP_SLEEP_WAKEUP_EXT1: {
            m_reason = WakeupReason::GpioAlert;
            g_rtc.wakeup_count++;
            g_rtc.alert_pending = true;
            uint64_t pins = esp_sleep_get_ext1_wakeup_status();
            ESP_LOGW(Tag, "🚨 Wakeup: GPIO ALERT — pins=0x%llX (cycle #%lu)",
                     (unsigned long long)pins, (unsigned long)g_rtc.wakeup_count);
            break;
        }

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            // First boot or reset — RTC memory might be garbage
            m_reason = WakeupReason::FirstBoot;
            if (g_rtc.magic != RTC_MAGIC) {
                // RTC memory not initialized — clear it
                memset(&g_rtc, 0, sizeof(g_rtc));
                g_rtc.magic = RTC_MAGIC;
                ESP_LOGI(Tag, "🔌 First boot — RTC memory initialized");
            } else {
                ESP_LOGI(Tag, "🔄 Reset (RTC memory preserved, count=%lu)",
                         (unsigned long)g_rtc.wakeup_count);
            }
            break;
    }

    return m_reason;
}

// ─── getRtcData ───────────────────────────────────────────────────────────────

const RtcData* SleepManager::getRtcData() const {
    if (g_rtc.magic != RTC_MAGIC) return nullptr;
    return &g_rtc;
}

// ─── saveLastFrame ────────────────────────────────────────────────────────────

void SleepManager::saveLastFrame(float tds, float pressure, float flow,
                                  float temperature, float diff_pressure)
{
    g_rtc.magic           = RTC_MAGIC;
    g_rtc.last_tds        = tds;
    g_rtc.last_pressure   = pressure;
    g_rtc.last_flow       = flow;
    g_rtc.last_temperature    = temperature;
    g_rtc.last_diff_pressure  = diff_pressure;
    ESP_LOGD(Tag, "RTC saved: TDS=%.1f P=%.2f F=%.1f T=%.1f DP=%.2f",
             tds, pressure, flow, temperature, diff_pressure);
}

// ─── setOtaPending ────────────────────────────────────────────────────────────

void SleepManager::setOtaPending(bool pending) {
    g_rtc.ota_pending = pending;
    if (pending) {
        ESP_LOGI(Tag, "OTA pending flag set in RTC");
    }
}

// ─── setForceWakeup ───────────────────────────────────────────────────────────

void SleepManager::setForceWakeup(bool force) {
    g_rtc.force_wakeup = force;
    if (force) {
        ESP_LOGI(Tag, "Force wakeup flag set — next cycle will be shorter");
        // Shorten next sleep to 30 seconds if there's an urgent command
        m_override_sleep_us = 30ULL * 1000000ULL;
    }
}

// ─── checkThresholds ─────────────────────────────────────────────────────────

bool SleepManager::checkThresholds(float tds, float pressure, float flow,
                                    float temperature, float diff_pressure)
{
    bool anomaly = false;
    const RtcData* prev = getRtcData();

    // ── Check absolute values ─────────────────────────────────────────────────────

    if (tds > THRESHOLD_TDS_MAX) {
        ESP_LOGW(Tag, "⚠ TDS high: %.1f ppm (limit %.0f)", tds, THRESHOLD_TDS_MAX);
        anomaly = true;
    }
    if (pressure > THRESHOLD_PRES_MAX) {
        ESP_LOGW(Tag, "⚠ Pressure high: %.2f bar (limit %.1f)", pressure, THRESHOLD_PRES_MAX);
        anomaly = true;
    }
    if (pressure < THRESHOLD_PRES_MIN && pressure > 0.01f) {
        ESP_LOGW(Tag, "⚠ Pressure low: %.2f bar (limit %.1f)", pressure, THRESHOLD_PRES_MIN);
        anomaly = true;
    }
    if (temperature > THRESHOLD_TEMP_MAX) {
        ESP_LOGW(Tag, "⚠ Temperature high: %.1f°C (limit %.0f)", temperature, THRESHOLD_TEMP_MAX);
        anomaly = true;
    }

    // ── Check sudden changes compared to previous cycle ────────────────────────
    // (only if there is previous data — not first boot)

    if (prev != nullptr) {
        // If TDS changed by more than 20% -> anomaly
        if (prev->last_tds > 1.0f) {
            float tds_change = fabsf(tds - prev->last_tds) / prev->last_tds;
            if (tds_change > 0.20f) {
                ESP_LOGW(Tag, "⚠ TDS sudden change: %.1f→%.1f (%.0f%%)",
                         prev->last_tds, tds, tds_change * 100.0f);
                anomaly = true;
            }
        }

        // If pressure changed by more than 1 bar at once -> anomaly
        if (fabsf(pressure - prev->last_pressure) > 1.0f) {
            ESP_LOGW(Tag, "⚠ Pressure sudden change: %.2f→%.2f bar",
                     prev->last_pressure, pressure);
            anomaly = true;
        }

        // If flow stopped suddenly (was > 5 L/min and became < 0.5) -> anomaly
        if (prev->last_flow > 5.0f && flow < 0.5f) {
            ESP_LOGW(Tag, "⚠ Flow stopped suddenly: %.1f→%.1f L/min",
                     prev->last_flow, flow);
            anomaly = true;
        }
    }

    m_anomaly_detected = anomaly;

    if (anomaly) {
        // If anomaly -> shorten next sleep to 60 seconds
        m_override_sleep_us = 60ULL * 1000000ULL;
        ESP_LOGW(Tag, "Anomaly detected → next sleep shortened to 60s");
    }

    return anomaly;
}

// ─── getSleepDuration ─────────────────────────────────────────────────────────

uint64_t SleepManager::getSleepDuration() const {
    if (m_override_sleep_us > 0) {
        return m_override_sleep_us;
    }
    return SLEEP_DURATION_US;
}

// ─── configureWakeupSources ───────────────────────────────────────────────────

void SleepManager::configureWakeupSources() {
    // ── Wakeup 1: Timer ───────────────────────────────────────────────────────
    uint64_t sleep_us = getSleepDuration();
    esp_sleep_enable_timer_wakeup(sleep_us);
    ESP_LOGI(Tag, "Timer wakeup: %llu sec", (unsigned long long)(sleep_us / 1000000ULL));

    // ── Wakeup 2: EXT1 GPIO from STM32 ─────────────────────────────────────────
    if (STM32_ALERT_GPIO >= 0) {
        // HIGH level wakeup — STM32 pulls pin HIGH on alert
        uint64_t alert_mask = (1ULL << STM32_ALERT_GPIO);
        esp_sleep_enable_ext1_wakeup(alert_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
        ESP_LOGI(Tag, "EXT1 wakeup: GPIO IO%d (HIGH)", STM32_ALERT_GPIO);
    }
}

// ─── shutdownPeripherals ─────────────────────────────────────────────────────

void SleepManager::shutdownPeripherals() {
    // WiFi
    esp_wifi_stop();
    esp_wifi_deinit();

    // UART — stops automatically in deep sleep but clean it up
    uart_driver_delete(UART_NUM_2);

    // Short delay to ensure log is printed
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ─── prepareAndSleep ──────────────────────────────────────────────────────────

void SleepManager::prepareAndSleep() {
    if (m_sleep_disabled) {
        ESP_LOGI(Tag, "Sleep disabled — staying awake");
        // Clear overrides so next cycle starts clean
        m_override_sleep_us = 0;
        m_anomaly_detected  = false;
        g_rtc.force_wakeup  = false;
        return;
    }

    // Ensure RTC data is preserved
    g_rtc.magic = RTC_MAGIC;

    // Clear flags that have been handled
    g_rtc.alert_pending = false;
    g_rtc.force_wakeup  = false;
    // Note: ota_pending is cleared by OtaManager after OTA finishes

    ESP_LOGI(Tag, "─────────────────────────────────────────");
    ESP_LOGI(Tag, "Preparing to sleep...");
    ESP_LOGI(Tag, "  Wakeup count : %lu", (unsigned long)g_rtc.wakeup_count);
    ESP_LOGI(Tag, "  Sleep for    : %llu sec",
             (unsigned long long)(getSleepDuration() / 1000000ULL));
    ESP_LOGI(Tag, "─────────────────────────────────────────");

    // Configure wakeup sources
    configureWakeupSources();

    // Stop peripherals cleanly
    shutdownPeripherals();

    ESP_LOGI(Tag, "💤 Entering deep sleep... good night.");

    // This function never returns
    esp_deep_sleep_start();
}

} // namespace Takamul