#ifndef SLEEP_MANAGER_H_
#define SLEEP_MANAGER_H_

#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <string>

namespace Takamul {

/**
 * ============================================================================
 *  SleepManager — Takamul SCADA
 *
 *  Controls the sleep/wakeup cycle for the ESP32-S3 based on:
 *
 *  WAKEUP SOURCES:
 *  ─────────────────────────────────────────────────────────────────────────
 *  1. Timer         → every SLEEP_DURATION_MS (default 5 mins)
 *                     Normal state — wakes up, sends data, and sleeps
 *
 *  2. EXT1 GPIO     → STM32_ALERT_GPIO (IO38 from schematic)
 *                     STM32 pulls the pin HIGH when there is an alarm
 *                     (sensor out of range / pump fault / etc.)
 *
 *  3. Flag in RTC Memory → if the website sends OTA or an urgent command
 *                           TelemetryManager sets a flag before sleeping
 *                           and checks it upon wakeup
 *
 *  RTC MEMORY (persists during sleep):
 *  ─────────────────────────────────────────────────────────────────────────
 *  - wakeup_count       counter for how many times it woke up (for logging)
 *  - last_tds           last TDS reading (for threshold check)
 *  - last_pressure      last pressure reading
 *  - last_flow          last flow reading
 *  - alert_pending      flag if STM32 sent an alert
 *  - ota_pending        flag if there is an OTA URL in Supabase
 *  - force_wakeup       flag if the website requested urgent wakeup
 *
 *  THRESHOLDS (limits that trigger immediate wakeup):
 *  ─────────────────────────────────────────────────────────────────────────
 *  All thresholds are modifiable from main.h
 *
 *  Usage:
 *    SleepManager::getInstance().init();
 *    SleepManager::getInstance().checkWakeupReason();   // on first boot
 *    // ... run everything ...
 *    SleepManager::getInstance().prepareAndSleep();     // at the end of the cycle
 * ============================================================================
 */

// ─── RTC Memory struct (saved during sleep) ─────────────────────────────────────
// The struct must be fixed size and contain no pointers or strings
struct RtcData {
    uint32_t magic;             // 0xCAFEBABE — to verify data is valid
    uint32_t wakeup_count;      // how many times it woke up since first boot
    float    last_tds;          // last TDS reading (ppm)
    float    last_pressure;     // last pressure reading (bar)
    float    last_flow;         // last flow reading (L/min)
    float    last_temperature;  // last temperature reading (°C)
    float    last_diff_pressure;// last diff pressure reading (bar)
    bool     alert_pending;     // STM32 raised GPIO alert
    bool     ota_pending;       // OTA URL exists in Supabase
    bool     force_wakeup;      // website requested urgent wakeup
    bool     provisioning_mode; // if in provisioning → do not sleep
};

static constexpr uint32_t RTC_MAGIC = 0xCAFEBABEu;

// ─── Wakeup reason enum ───────────────────────────────────────────────────────
enum class WakeupReason {
    FirstBoot,      // first time running (or after power loss)
    Timer,          // woke up from the 5 min timer
    GpioAlert,      // STM32 raised GPIO (alarm/fault)
    ULP,            // (for future use)
    Unknown,
};

class SleepManager {
public:
    static SleepManager& getInstance();

    // ─── Config (change these in main.h or here directly) ─────────────────────

    // Normal sleep duration (5 mins)
    static constexpr uint64_t SLEEP_DURATION_US = 5ULL * 60ULL * 1000000ULL;

    // The GPIO that STM32 pulls high on alert
    // From schematic: IO38 on ESP32-S3
    // Change it if connected to a different pin
    static constexpr gpio_num_t STM32_ALERT_GPIO = GPIO_NUM_38;

    // Thresholds — if reading exceeds this, the next sleep cycle is shorter
    // or can implement logic to wake up earlier
    static constexpr float THRESHOLD_TDS_MAX     = 1000.0f;   // ppm
    static constexpr float THRESHOLD_PRES_MAX    = 6.0f;       // bar
    static constexpr float THRESHOLD_PRES_MIN    = 0.5f;       // bar
    static constexpr float THRESHOLD_FLOW_MIN    = 1.0f;       // L/min (if pump is on)
    static constexpr float THRESHOLD_TEMP_MAX    = 45.0f;      // °C

    // ─── Public API ──────────────────────────────────────────────────────────

    /**
     * @brief Initialize wakeup GPIO and read RTC memory.
     *        Call this first thing in app_main.
     */
    void init();

    /**
     * @brief Determine wakeup reason and log it.
     *        Call this right after init().
     * @return wakeup reason
     */
    WakeupReason checkWakeupReason();

    /**
     * @brief Is this wakeup caused by GPIO alert from STM32?
     */
    bool isAlertWakeup() const { return m_reason == WakeupReason::GpioAlert; }

    /**
     * @brief Is this wakeup caused by the normal timer?
     */
    bool isTimerWakeup() const { return m_reason == WakeupReason::Timer; }

    /**
     * @brief Is it the first boot (or after power loss)?
     */
    bool isFirstBoot() const { return m_reason == WakeupReason::FirstBoot; }

    /**
     * @brief Read the saved RTC data from the previous cycle.
     *        Returns nullptr if data is invalid (first boot).
     */
    const RtcData* getRtcData() const;

    /**
     * @brief Save last sensor readings to RTC memory before sleeping.
     *        TelemetryManager calls this after uploading.
     */
    void saveLastFrame(float tds, float pressure, float flow,
                       float temperature, float diff_pressure);

    /**
     * @brief Save flag that OTA URL exists in Supabase.
     *        OtaManager calls this if a URL is found.
     */
    void setOtaPending(bool pending);

    /**
     * @brief Save flag that website requested urgent wakeup.
     *        TelemetryManager calls this if force_wakeup=true in controls.
     */
    void setForceWakeup(bool force);

    /**
     * @brief Is there a reason to shorten the sleep cycle?
     *        (e.g., if reading is near threshold)
     * @return sleep duration in microseconds
     */
    uint64_t getSleepDuration() const;

    /**
     * @brief Check new readings against thresholds.
     *        If sudden change → shorten next sleep duration.
     *        TelemetryManager calls this after getting sensor frame.
     * @return true if an anomaly needs attention
     */
    bool checkThresholds(float tds, float pressure, float flow,
                         float temperature, float diff_pressure);

    /**
     * @brief Prevent device from sleeping (e.g., in provisioning mode).
     */
    void disableSleep() { m_sleep_disabled = true; }
    void enableSleep()  { m_sleep_disabled = false; }

    /**
     * @brief Final step in every cycle:
     *        - Save RTC data
     *        - Stop WiFi and all peripherals
     *        - Enter deep sleep
     *        This function never returns (calls esp_deep_sleep_start).
     *        If sleep disabled → returns without sleeping.
     */
    void prepareAndSleep();

private:
    SleepManager() = default;
    ~SleepManager() = default;
    SleepManager(const SleepManager&) = delete;
    SleepManager& operator=(const SleepManager&) = delete;

    void configureWakeupSources();
    void shutdownPeripherals();

    WakeupReason m_reason         = WakeupReason::Unknown;
    bool         m_sleep_disabled = false;
    bool         m_anomaly_detected = false;
    uint64_t     m_override_sleep_us = 0;  // 0 = use default
};

} // namespace Takamul

#endif // SLEEP_MANAGER_H_