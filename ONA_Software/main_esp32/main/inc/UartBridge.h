#ifndef UART_BRIDGE_H_
#define UART_BRIDGE_H_

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string>
#include <functional>

namespace Takamul {

    /**
     * @brief Parsed sensor frame received from STM32 over UART.
     *
     * STM32 sends newline-terminated ASCII frames:
     *   TDS:290.50,TEMP:22.43,FLOW:41.08,PRES:3.79,DIFF:0.40\n
     *
     * Fields:
     *   tds           — Total Dissolved Solids (ppm)
     *   temperature   — Water temperature (°C)
     *   flow          — Flow rate (L/min)
     *   pressure      — Line pressure (bar)
     *   diff_pressure — Differential pressure (bar)
     *   valid         — true if parsing succeeded
     */
    struct SensorFrame {
        float tds           = 0.0f;
        float temperature   = 0.0f;
        float flow          = 0.0f;
        float pressure      = 0.0f;
        float diff_pressure = 0.0f;
        bool  valid         = false;
    };

    /**
     * @brief Control command to send to STM32.
     *
     * ESP32 sends newline-terminated ASCII commands:
     *   CMD:PUMP_ON,SPEED:35.0,PRES_SP:3.50\n
     *   CMD:PUMP_OFF\n
     */
    struct ControlCmd {
        bool  pump_on        = false;
        float speed_hz       = 0.0f;   // VFD frequency Hz (0–50)
        float target_pressure = 3.5f;  // bar
    };

    /**
     * @brief UART bridge between ESP32 and STM32.
     *
     * Configures a UART port, spawns a RX task that reads frames
     * and calls a user-provided callback on each valid SensorFrame.
     *
     * TX side: sendControl() serialises a ControlCmd and writes it.
     *
     * Example:
     *   UartBridge::getInstance().init(UART_NUM_2, GPIO_NUM_16, GPIO_NUM_17);
     *   UartBridge::getInstance().start([](const SensorFrame& f){
     *       // handle sensor data
     *   });
     */
    class UartBridge {
    public:
        using SensorCallback = std::function<void(const SensorFrame&)>;

        static UartBridge& getInstance();

        /**
         * @brief Configure UART peripheral. Call before start().
         * @param port     UART port number (e.g. UART_NUM_2).
         * @param rx_pin   GPIO for RX.
         * @param tx_pin   GPIO for TX.
         * @param baud     Baud rate (default 115200).
         */
        void init(uart_port_t port, int rx_pin, int tx_pin, int baud = 115200);

        /**
         * @brief Start the RX listener task.
         * @param cb Callback invoked on every valid SensorFrame received.
         */
        void start(SensorCallback cb);

        /**
         * @brief Send a control command to STM32.
         * @param cmd The command to serialise and transmit.
         */
        void sendControl(const ControlCmd& cmd);

        /**
         * @brief Stop the RX task and release UART resources.
         */
        void stop();

    private:
        UartBridge() = default;
        ~UartBridge() { stop(); }
        UartBridge(const UartBridge&) = delete;
        UartBridge& operator=(const UartBridge&) = delete;

        /**
         * @brief FreeRTOS task function.
         */
        static void rxTask(void* arg);

        /**
         * @brief Parse a raw ASCII line into a SensorFrame.
         * @param line  Null-terminated string (without the trailing '\n').
         * @return      Populated SensorFrame; .valid=false on parse error.
         */
        static SensorFrame parseLine(const char* line);

        uart_port_t    m_port        = UART_NUM_2;
        SensorCallback m_callback;
        bool           m_running     = false;
        bool           m_initialized = false;
    };

} // namespace Takamul

#endif // UART_BRIDGE_H_
