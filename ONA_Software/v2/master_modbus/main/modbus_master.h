#ifndef MODBUS_MASTER_H_
#define MODBUS_MASTER_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

constexpr gpio_num_t TX_PIN    = GPIO_NUM_17;
constexpr gpio_num_t RX_PIN    = GPIO_NUM_16;
constexpr gpio_num_t RTS_PIN   = GPIO_NUM_19;
constexpr uart_port_t UART_PORT = UART_NUM_1;
constexpr uint8_t SLAVE_ADDR = 1;

uint16_t crc16_modbus(const uint8_t* data, size_t len);

class VfdMaster{
    public:
        VfdMaster(uart_port_t uart_num = UART_PORT, 
            gpio_num_t tx_pin_num = TX_PIN, 
            gpio_num_t rx_pin_num = RX_PIN,
            gpio_num_t rts_pin_num = RTS_PIN
        ): 
        m_port(uart_num),
        m_tx(tx_pin_num),
        m_rx(rx_pin_num),
        m_rts(rts_pin_num)
        {}
        void init();
        bool send_frame(const uint8_t* frame, size_t len);
        bool receive_response(uint8_t* response, size_t max_len, size_t* received_len);
        bool set_run_stop(bool run);
        bool set_frequency(uint16_t hz);
        bool read_current(uint16_t* current);
        bool read_voltage(uint16_t* voltage);
        ~VfdMaster() = default;
    private:
        uart_port_t m_port;
        gpio_num_t m_tx;
        gpio_num_t m_rx;
        gpio_num_t m_rts;
};

#endif // !MODBUS_MASTER_H_