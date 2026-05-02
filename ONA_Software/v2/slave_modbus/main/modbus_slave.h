#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

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

class VfdSlave{
    public:
        VfdSlave(uart_port_t uart_num = UART_PORT, 
            gpio_num_t tx_pin_num = TX_PIN, 
            gpio_num_t rx_pin_num = RX_PIN,
            gpio_num_t rts_pin_num = RTS_PIN
        ): 
        vfd_port(uart_num),
        vfd_tx(tx_pin_num),
        vfd_rx(rx_pin_num),
        vfd_rts(rts_pin_num)
        {}
        void init();
        void send_response(const uint8_t* response, size_t len);
        void vfd_logic_task();
        ~VfdSlave() = default;
    private:
        bool is_running{false};
        uint16_t freq_hz{0};
        uint16_t current_ma{1000}; // dummy
        uint16_t voltage_mv{2400}; // dummy
        uart_port_t vfd_port;
        gpio_num_t vfd_tx;
        gpio_num_t vfd_rx;
        gpio_num_t vfd_rts;
};

#endif // !MODBUS_SLAVE_H_