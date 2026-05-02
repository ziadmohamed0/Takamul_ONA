#include "modbus_slave.h"

uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static const char* TAG = "VFD_SLAVE";

void VfdSlave::init() {
    uart_config_t cfg = {};
    cfg.baud_rate = 9600;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT; 
    uart_driver_install(vfd_port, 1024 * 2, 1024 * 2, 0, NULL, 0); 
    uart_param_config(vfd_port, &cfg);
    uart_set_pin(vfd_port, vfd_tx, vfd_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_reset_pin(vfd_rts);
    gpio_set_direction(vfd_rts, GPIO_MODE_OUTPUT);
    gpio_set_level(vfd_rts, 0);
}

void VfdSlave::send_response(const uint8_t* response, size_t len) {    ESP_LOGI(TAG, "Sending response:");
    ESP_LOG_BUFFER_HEX(TAG, response, len);    gpio_set_level(vfd_rts, 1);
    esp_rom_delay_us(100); 
    uart_write_bytes(vfd_port, (const char*)response, len);
    uart_wait_tx_done(vfd_port, pdMS_TO_TICKS(200));
    esp_rom_delay_us(5000); 
    gpio_set_level(vfd_rts, 0); 
    vTaskDelay(pdMS_TO_TICKS(20));
}

void VfdSlave::vfd_logic_task() {
    ESP_LOGI(TAG, "Logic task started");
    uint8_t recv_buffer[256];
    size_t recv_len = 0;
    uint8_t temp[128];

    while (true) {
        int len = uart_read_bytes(vfd_port, temp, sizeof(temp), pdMS_TO_TICKS(100));
        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes", len);
            ESP_LOG_BUFFER_HEX(TAG, temp, len);

            if (recv_len + (size_t)len > sizeof(recv_buffer)) {
                recv_len = 0;
                ESP_LOGW(TAG, "Receive buffer overflow, clearing buffer");
            }

            memcpy(recv_buffer + recv_len, temp, len);
            recv_len += len;

            size_t offset = 0;
            while (recv_len - offset >= 8) {
                if (recv_buffer[offset] != SLAVE_ADDR) {
                    offset++;
                    continue;
                }

                uint16_t frame_len = 8;
                if (recv_len - offset < frame_len) break;

                uint16_t received_crc = (recv_buffer[offset + frame_len - 1] << 8) | recv_buffer[offset + frame_len - 2];
                uint16_t calculated_crc = crc16_modbus(recv_buffer + offset, frame_len - 2);
                if (received_crc != calculated_crc) {
                    ESP_LOGW(TAG, "CRC error at offset %d", offset);
                    offset++;
                    continue;
                }

                uint8_t func = recv_buffer[offset + 1];
                uint8_t response[256];
                size_t resp_len = 0;

                if (func == 5) {
                    uint16_t addr = (recv_buffer[offset + 2] << 8) | recv_buffer[offset + 3];
                    uint16_t value = (recv_buffer[offset + 4] << 8) | recv_buffer[offset + 5];
                    if (addr == 0) {
                        is_running = (value == 0xFF00);
                    }
                    memcpy(response, recv_buffer + offset, frame_len - 2);
                    resp_len = frame_len - 2;
                } else if (func == 6) {
                    uint16_t addr = (recv_buffer[offset + 2] << 8) | recv_buffer[offset + 3];
                    uint16_t value = (recv_buffer[offset + 4] << 8) | recv_buffer[offset + 5];
                    if (addr == 1) {
                        freq_hz = value / 100;
                    }
                    memcpy(response, recv_buffer + offset, frame_len - 2);
                    resp_len = frame_len - 2;
                } else if (func == 4) {
                    uint16_t addr = (recv_buffer[offset + 2] << 8) | recv_buffer[offset + 3];
                    uint16_t count = (recv_buffer[offset + 4] << 8) | recv_buffer[offset + 5];
                    response[0] = SLAVE_ADDR;
                    response[1] = 4;
                    response[2] = count * 2;
                    for (uint16_t i = 0; i < count; i++) {
                        uint16_t val = 0;
                        if (addr + i == 2) val = current_ma;
                        else if (addr + i == 3) val = voltage_mv;
                        response[3 + i*2] = val >> 8;
                        response[4 + i*2] = val & 0xFF;
                    }
                    resp_len = 3 + count * 2;
                } else {
                    response[0] = SLAVE_ADDR;
                    response[1] = func | 0x80;
                    response[2] = 1;
                    resp_len = 3;
                }

                uint16_t crc = crc16_modbus(response, resp_len);
                response[resp_len++] = crc & 0xFF;
                response[resp_len++] = crc >> 8;

                ESP_LOGI(TAG, "Status: %s | Freq: %d Hz | Current: %d mA | Voltage: %d mV", 
                         is_running ? "RUN" : "STOP", freq_hz, current_ma, voltage_mv);
                send_response(response, resp_len);
                offset += frame_len;
            }

            if (offset > 0) {
                memmove(recv_buffer, recv_buffer + offset, recv_len - offset);
                recv_len -= offset;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
