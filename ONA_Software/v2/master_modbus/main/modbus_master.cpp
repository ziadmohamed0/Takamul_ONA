#include "modbus_master.h"

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

static const char* TAG = "MASTER_GATEWAY";

void VfdMaster::init() {
    ESP_LOGI(TAG, "Initializing VfdMaster: UART_PORT=%d, TX=%d, RX=%d, RTS=%d", m_port, m_tx, m_rx, m_rts);
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(m_port, 1024 * 2, 1024 * 2, 0, NULL, 0);
    uart_param_config(m_port, &cfg);
    uart_set_pin(m_port, m_tx, m_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_reset_pin(m_rts);
    gpio_set_direction(m_rts, GPIO_MODE_OUTPUT);
    gpio_set_level(m_rts, 1); // Start in receive mode (active-low)
    ESP_LOGI(TAG, "GPIO18 configured as output, level set to 1 (receive mode)");
}

bool VfdMaster::send_frame(const uint8_t* frame, size_t len) {
    ESP_LOGI(TAG, "Sending frame:");
    ESP_LOG_BUFFER_HEX(TAG, frame, len);
    uart_flush_input(m_port);
    gpio_set_level(m_rts, 0); // Active-low: 0 for transmit
    esp_rom_delay_us(500);
    uart_write_bytes(m_port, (const char*)frame, len);
    uart_wait_tx_done(m_port, pdMS_TO_TICKS(200));
    esp_rom_delay_us(5000);
    gpio_set_level(m_rts, 1); // Back to receive mode
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Frame sent, RTS set to receive mode");
    return true;
}

bool VfdMaster::receive_response(uint8_t* response, size_t max_len, size_t* received_len) {
    uint8_t buffer[64] = {0};
    size_t total = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);

    ESP_LOGI(TAG, "Starting to receive response...");
    while (xTaskGetTickCount() < deadline && total < max_len) {
        int len = uart_read_bytes(m_port, buffer + total, max_len - total, pdMS_TO_TICKS(50));
        if (len > 0) {
            ESP_LOGI(TAG, "Master received %d bytes", len);
            ESP_LOG_BUFFER_HEX(TAG, buffer + total, len);
            total += len;

            size_t offset = 0;
            while (total - offset >= 5) {
                if (buffer[offset] != SLAVE_ADDR) {
                    offset++;
                    continue;
                }

                uint8_t func = buffer[offset + 1];
                size_t expected = 0;
                if (func == 5 || func == 6) {
                    expected = 8;
                } else if (func == 4) {
                    if (total - offset < 5) break;
                    expected = 5 + buffer[offset + 2];
                } else {
                    expected = 5;
                }

                if (total - offset < expected) break;

                uint16_t received_crc = (buffer[offset + expected - 1] << 8) | buffer[offset + expected - 2];
                uint16_t calculated_crc = crc16_modbus(buffer + offset, expected - 2);
                if (received_crc != calculated_crc) {
                    ESP_LOGW(TAG, "Master CRC error at offset %zu: %04X vs %04X", offset, received_crc, calculated_crc);
                    offset++;
                    continue;
                }

                memcpy(response, buffer + offset, expected);
                *received_len = expected;

                if (offset + expected < total) {
                    memmove(buffer, buffer + offset + expected, total - offset - expected);
                    total -= offset + expected;
                } else {
                    total = 0;
                }
                ESP_LOGI(TAG, "Valid response received, length: %zu", *received_len);
                return true;
            }

            if (offset > 0) {
                memmove(buffer, buffer + offset, total - offset);
                total -= offset;
            }
        }
    }
    ESP_LOGW(TAG, "No valid response received within timeout");
    return false;
}

bool VfdMaster::set_run_stop(bool run) {
    ESP_LOGI(TAG, "Sending set_run_stop command: %s", run ? "RUN" : "STOP");
    uint8_t frame[8];
    frame[0] = SLAVE_ADDR;
    frame[1] = 5; // write single coil
    frame[2] = 0; // addr hi
    frame[3] = 0; // addr lo
    frame[4] = run ? 0xFF : 0x00;
    frame[5] = 0x00;
    uint16_t crc = crc16_modbus(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!send_frame(frame, 8)) return false;

    uint8_t response[8];
    size_t resp_len;
    if (receive_response(response, sizeof(response), &resp_len) && resp_len == 8 && response[1] == 5) {
        ESP_LOGI(TAG, "Run/Stop set to %s", run ? "RUN" : "STOP");
        return true;
    }
    ESP_LOGE(TAG, "Failed to set Run/Stop");
    return false;
}

bool VfdMaster::set_frequency(uint16_t hz) {
    uint8_t frame[8];
    frame[0] = SLAVE_ADDR;
    frame[1] = 6; // write single holding register
    frame[2] = 0; // addr hi
    frame[3] = 1; // addr lo
    uint16_t value = hz * 100; // scaled
    frame[4] = value >> 8;
    frame[5] = value & 0xFF;
    uint16_t crc = crc16_modbus(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!send_frame(frame, 8)) return false;

    uint8_t response[8];
    size_t resp_len;
    if (receive_response(response, sizeof(response), &resp_len) && resp_len == 8 && response[1] == 6) {
        ESP_LOGI(TAG, "Frequency set to %d Hz", hz);
        return true;
    }
    ESP_LOGE(TAG, "Failed to set Frequency");
    return false;
}

bool VfdMaster::read_current(uint16_t* current) {
    uint8_t frame[8];
    frame[0] = SLAVE_ADDR;
    frame[1] = 4; // read input registers
    frame[2] = 0; // addr hi
    frame[3] = 2; // addr lo
    frame[4] = 0; // count hi
    frame[5] = 1; // count lo
    uint16_t crc = crc16_modbus(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!send_frame(frame, 8)) return false;

    uint8_t response[7];
    size_t resp_len;
    if (receive_response(response, sizeof(response), &resp_len) && resp_len == 7 && response[1] == 4) {
        *current = (response[3] << 8) | response[4];
        ESP_LOGI(TAG, "Current: %d mA", *current);
        return true;
    }
    ESP_LOGE(TAG, "Failed to read Current");
    return false;
}

bool VfdMaster::read_voltage(uint16_t* voltage) {
    uint8_t frame[8];
    frame[0] = SLAVE_ADDR;
    frame[1] = 4; // read input registers
    frame[2] = 0; // addr hi
    frame[3] = 3; // addr lo
    frame[4] = 0; // count hi
    frame[5] = 1; // count lo
    uint16_t crc = crc16_modbus(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!send_frame(frame, 8)) return false;

    uint8_t response[7];
    size_t resp_len;
    if (receive_response(response, sizeof(response), &resp_len) && resp_len == 7 && response[1] == 4) {
        *voltage = (response[3] << 8) | response[4];
        ESP_LOGI(TAG, "Voltage: %d mV", *voltage);
        return true;
    }
    ESP_LOGE(TAG, "Failed to read Voltage");
    return false;
}
