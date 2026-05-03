#ifndef TELEMETRY_MANAGER_H_
#define TELEMETRY_MANAGER_H_

#include "inc/UartBridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string>
#include <atomic>

namespace Takamul {

    /**
     * @brief Orchestrates the full data flow:
     *
     *   STM32 → UART → TelemetryManager → Supabase (telemetry table)
     *                                   ← Supabase (controls table)
     *                                   → STM32 (via UartBridge::sendControl)
     *
     * Responsibilities:
     *  - Receives SensorFrames from UartBridge callback (ISR-safe: just stores latest).
     *  - Uploads batch telemetry to Supabase every UPLOAD_INTERVAL_MS.
     *  - Polls Supabase controls table every POLL_INTERVAL_MS and sends ControlCmd to STM32.
     *  - Registers the device (MAC address) in the Supabase `devices` table on first boot.
     *
     * Usage:
     *   TelemetryManager::getInstance().init(device_id);
     *   TelemetryManager::getInstance().start();
     */
    class TelemetryManager {
    public:
        static TelemetryManager& getInstance();

        /**
         * @brief Set device identity. Call after reading MAC.
         * @param device_id  MAC-derived string e.g. "AA:BB:CC:DD:EE:FF".
         */
        void init(const std::string& device_id);

        /**
         * @brief Spawn upload + poll tasks. Call after WiFi is connected.
         */
        void start();

        /**
         * @brief Stop tasks (graceful).
         */
        void stop();

        /**
         * @brief Called by UartBridge when a new SensorFrame arrives.
         *        Stores the frame atomically; never blocks.
         */
        void onSensorFrame(const SensorFrame& frame);
        void uploadOnce();
        void pollOnce();
    private:
        TelemetryManager() = default;
        ~TelemetryManager() { stop(); }
        TelemetryManager(const TelemetryManager&) = delete;
        TelemetryManager& operator=(const TelemetryManager&) = delete;

        static void uploadTask(void* arg);
        static void pollTask(void* arg);

        /**
         * @brief Build and POST a 5-row telemetry batch.
         */
        void uploadTelemetry(const SensorFrame& frame);

        /**
         * @brief Fetch latest row from controls table and act on it.
         */
        void pollControls();

        /**
         * @brief Register/update device record in `devices` table.
         */
        void registerDevice();

        // Latest sensor snapshot — written by UART callback, read by upload task
        SensorFrame     m_latest_frame;
        SemaphoreHandle_t m_frame_mutex = nullptr;
        bool            m_has_frame     = false;

        std::string     m_device_id;
        bool            m_initialized   = false;
        std::atomic<bool> m_running     {false};

        // Track last control state sent to STM32 to avoid redundant TX
        bool  m_last_pump_on      = false;
        float m_last_speed        = -1.0f;
        float m_last_target_pres  = -1.0f;

        // Configurable intervals (compile-time constants)
        static constexpr TickType_t kUploadInterval = pdMS_TO_TICKS(5000);   // 5 s
        static constexpr TickType_t kPollInterval   = pdMS_TO_TICKS(2000);   // 2 s
    };

} // namespace Takamul

#endif // TELEMETRY_MANAGER_H_
