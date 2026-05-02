/**
 * ============================================================
 *  Promag P300 Modbus RTU Simulator - ESP32 (ESP-IDF + C++)
 * ============================================================
 *
 *  الوصف:
 *    يسيمولات سينسور Endress+Hauser Proline Promag 300
 *    عن طريق Modbus RTU على RS485 (MAX485)
 *
 *  الاتصال:
 *    ESP32 UART2  <-->  MAX485 (DE/RE = GPIO4, DI = GPIO17, RO = GPIO16)
 *    MAX485 A/B   <-->  الكابل التوصيل للماكس التاني على STM32
 *
 *  الريجيسترات المدعومة (من الـ Modbus Map الرسمي):
 *    - Volume flow        Reg 3874  (Float, 2 registers)
 *    - Mass flow          Reg 3876  (Float, 2 registers)
 *    - Conductivity       Reg 2013  (Float, 2 registers)
 *    - Corr. vol. flow    Reg 2011  (Float, 2 registers)
 *    - Temperature        Reg 2015  (Float, 2 registers)
 *    - Corr. conductivity Reg 3977  (Float, 2 registers)
 *    - Totalizer 1        Reg 2610  (Float, 2 registers)
 *    - Totalizer 2        Reg 2810  (Float, 2 registers)
 *    - Totalizer 3        Reg 3010  (Float, 2 registers)
 *    - Output current 1   Reg 5931  (Float, 2 registers)
 *    - Locking status     Reg 4918  (Integer, 1 register)
 *    - Access status      Reg 2178  (Integer, 1 register)
 *
 *  Function Codes المدعومة:
 *    - FC03 (Read Holding Registers)
 *    - FC04 (Read Input Registers)
 *
 * ============================================================
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "PROMAG_SIM";

// ===== UART / RS485 Config =====
#define UART_PORT         UART_NUM_2
#define UART_TX_PIN       GPIO_NUM_17
#define UART_RX_PIN       GPIO_NUM_16
#define RS485_DE_RE_PIN   GPIO_NUM_4      // MAX485: DE و RE متوصلين مع بعض

#define BAUD_RATE         19200
#define UART_BUF_SIZE     256

// ===== Modbus Config =====
#define SLAVE_ADDRESS     1              // عنوان الـ Slave (يعديله لو السينسور الأصلي مختلف)

// ===== Register Addresses (من الـ Modbus Map الرسمي) =====
#define REG_VOLUME_FLOW         3874
#define REG_MASS_FLOW           3876
#define REG_CONDUCTIVITY        2013
#define REG_CORR_VOL_FLOW       2011
#define REG_TEMPERATURE         2015
#define REG_CORR_CONDUCTIVITY   3977
#define REG_TOTALIZER_1         2610
#define REG_TOTALIZER_2         2810
#define REG_TOTALIZER_3         3010
#define REG_OUTPUT_CURRENT_1    5931
#define REG_LOCKING_STATUS      4918
#define REG_ACCESS_STATUS       2178

// ===== Simulated Sensor Values =====
typedef struct {
    float volume_flow;          // m³/h
    float mass_flow;            // kg/h
    float conductivity;         // µS/cm
    float corr_vol_flow;        // Nl/h (Corrected volume flow)
    float temperature;          // °C
    float corr_conductivity;    // µS/cm
    float totalizer_1;          // m³
    float totalizer_2;          // m³
    float totalizer_3;          // kg
    float output_current_1;     // mA (4–20 mA)
    uint16_t locking_status;    // 0 = unlocked
    uint16_t access_status;     // 1 = Maintenance
} SensorData_t;

static SensorData_t sensor_data = {
    .volume_flow        = 25.5f,
    .mass_flow          = 25500.0f,
    .conductivity       = 500.0f,
    .corr_vol_flow      = 24.8f,
    .temperature        = 23.5f,
    .corr_conductivity  = 485.0f,
    .totalizer_1        = 1234.56f,
    .totalizer_2        = 5678.90f,
    .totalizer_3        = 25500.0f,
    .output_current_1   = 12.5f,
    .locking_status     = 0,
    .access_status      = 1
};

// ===== CRC16 Modbus =====
static uint16_t crc16_modbus(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ===== Float → 2 Registers (Big-Endian IEEE 754) =====
static void float_to_regs(float val, uint16_t *reg_hi, uint16_t *reg_lo) {
    uint32_t raw;
    memcpy(&raw, &val, 4);
    *reg_hi = (uint16_t)(raw >> 16);
    *reg_lo = (uint16_t)(raw & 0xFFFF);
}

// ===== قراية قيمة ريجيستر بناءً على العنوان =====
// بترجع عدد الريجيسترات المملوءة (1 أو 2)، وبتملي buf
static int read_register(uint16_t address, uint16_t *buf, uint16_t count) {
    uint16_t hi, lo;

    // Float registers (2 registers each)
    auto fill_float = [&](float val) -> int {
        float_to_regs(val, &hi, &lo);
        if (count >= 1) buf[0] = hi;
        if (count >= 2) buf[1] = lo;
        return 2;
    };

    switch (address) {
        case REG_VOLUME_FLOW:       return fill_float(sensor_data.volume_flow);
        case REG_MASS_FLOW:         return fill_float(sensor_data.mass_flow);
        case REG_CONDUCTIVITY:      return fill_float(sensor_data.conductivity);
        case REG_CORR_VOL_FLOW:     return fill_float(sensor_data.corr_vol_flow);
        case REG_TEMPERATURE:       return fill_float(sensor_data.temperature);
        case REG_CORR_CONDUCTIVITY: return fill_float(sensor_data.corr_conductivity);
        case REG_TOTALIZER_1:       return fill_float(sensor_data.totalizer_1);
        case REG_TOTALIZER_2:       return fill_float(sensor_data.totalizer_2);
        case REG_TOTALIZER_3:       return fill_float(sensor_data.totalizer_3);
        case REG_OUTPUT_CURRENT_1:  return fill_float(sensor_data.output_current_1);
        case REG_LOCKING_STATUS:    buf[0] = sensor_data.locking_status; return 1;
        case REG_ACCESS_STATUS:     buf[0] = sensor_data.access_status;  return 1;
        default:
            // الريجيستر مش معروف → رجّع صفر
            buf[0] = 0x0000;
            return 1;
    }
}

// ===== بناء Response لـ FC03 / FC04 =====
static int build_read_response(uint8_t *resp_buf,
                               uint8_t slave_id,
                               uint8_t func_code,
                               uint16_t start_addr,
                               uint16_t reg_count) {
    uint16_t data[128] = {0};
    int data_idx = 0;

    for (uint16_t i = 0; i < reg_count; ) {
        uint16_t temp[2] = {0};
        int filled = read_register(start_addr + i, temp, reg_count - i);
        for (int j = 0; j < filled && data_idx < 128; j++, data_idx++) {
            data[data_idx] = temp[j];
        }
        i += filled;
    }

    // بناء الـ PDU
    resp_buf[0] = slave_id;
    resp_buf[1] = func_code;
    resp_buf[2] = (uint8_t)(reg_count * 2);   // Byte count

    for (uint16_t i = 0; i < reg_count; i++) {
        resp_buf[3 + i * 2]     = (uint8_t)(data[i] >> 8);
        resp_buf[3 + i * 2 + 1] = (uint8_t)(data[i] & 0xFF);
    }

    uint16_t frame_len = 3 + reg_count * 2;
    uint16_t crc = crc16_modbus(resp_buf, frame_len);
    resp_buf[frame_len]     = (uint8_t)(crc & 0xFF);
    resp_buf[frame_len + 1] = (uint8_t)(crc >> 8);

    return frame_len + 2;
}

// ===== بناء Exception Response =====
static int build_exception_response(uint8_t *resp_buf,
                                    uint8_t slave_id,
                                    uint8_t func_code,
                                    uint8_t exception_code) {
    resp_buf[0] = slave_id;
    resp_buf[1] = func_code | 0x80;
    resp_buf[2] = exception_code;
    uint16_t crc = crc16_modbus(resp_buf, 3);
    resp_buf[3] = (uint8_t)(crc & 0xFF);
    resp_buf[4] = (uint8_t)(crc >> 8);
    return 5;
}

// ===== معالجة الريكوست الواردة =====
static void process_modbus_request(const uint8_t *req, int len) {
    if (len < 8) return;  // أقل من frame صح

    uint8_t slave_id  = req[0];
    uint8_t func_code = req[1];

    // تحقق من الـ CRC
    uint16_t recv_crc = (uint16_t)(req[len - 1] << 8) | req[len - 2];
    uint16_t calc_crc = crc16_modbus(req, len - 2);
    if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC Error! recv=0x%04X calc=0x%04X", recv_crc, calc_crc);
        return;
    }

    // مش عنواننا؟ إهمال (broadcast address=0 ممكن)
    if (slave_id != SLAVE_ADDRESS && slave_id != 0) {
        return;
    }

    uint16_t start_addr = (uint16_t)(req[2] << 8) | req[3];
    uint16_t reg_count  = (uint16_t)(req[4] << 8) | req[5];

    ESP_LOGI(TAG, "FC=%02X | StartAddr=%u | Count=%u", func_code, start_addr, reg_count);

    uint8_t resp[256] = {0};
    int resp_len = 0;

    switch (func_code) {
        case 0x03:  // Read Holding Registers
        case 0x04:  // Read Input Registers
            if (reg_count == 0 || reg_count > 125) {
                resp_len = build_exception_response(resp, slave_id, func_code, 0x03); // Illegal Data Value
            } else {
                resp_len = build_read_response(resp, slave_id, func_code, start_addr, reg_count);
            }
            break;

        default:
            resp_len = build_exception_response(resp, slave_id, func_code, 0x01); // Illegal Function
            break;
    }

    if (resp_len > 0 && slave_id != 0) {
        // تفعيل TX على MAX485
        gpio_set_level(RS485_DE_RE_PIN, 1);
        uart_write_bytes(UART_PORT, (const char *)resp, resp_len);
        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(20));
        // رجوع لـ RX
        gpio_set_level(RS485_DE_RE_PIN, 0);

        ESP_LOGI(TAG, "Response sent (%d bytes)", resp_len);
    }
}

// ===== Task تحديث قيم السينسور (Simulation) =====
static void sensor_update_task(void *pvParam) {
    float time_s = 0.0f;
    while (1) {
        time_s += 0.5f;

        // Volume flow: موجة جيبية حول 25 m³/h
        sensor_data.volume_flow = 25.0f + 5.0f * sinf(time_s * 0.1f);

        // Mass flow: نسبة للـ volume flow × density (~1000 kg/m³)
        sensor_data.mass_flow = sensor_data.volume_flow * 1000.0f;

        // Conductivity: قيمة ثابتة مع ضوضاء بسيطة
        sensor_data.conductivity = 500.0f + 10.0f * sinf(time_s * 0.3f);

        // Corrected volume flow: قريب من volume flow
        sensor_data.corr_vol_flow = sensor_data.volume_flow * 0.99f;

        // Temperature: بتتغير ببطء
        sensor_data.temperature = 23.0f + 2.0f * sinf(time_s * 0.05f);

        // Corrected conductivity
        sensor_data.corr_conductivity = sensor_data.conductivity * 0.97f;

        // Totalizers: بتزيد مع الوقت
        sensor_data.totalizer_1 += (sensor_data.volume_flow / 3600.0f) * 0.5f;
        sensor_data.totalizer_2 += (sensor_data.volume_flow / 3600.0f) * 0.5f;
        sensor_data.totalizer_3 += (sensor_data.mass_flow / 3600.0f) * 0.5f;

        // Output current: خطي مع volume flow (4–20 mA يقابل 0–50 m³/h)
        sensor_data.output_current_1 = 4.0f + (sensor_data.volume_flow / 50.0f) * 16.0f;
        if (sensor_data.output_current_1 > 20.0f) sensor_data.output_current_1 = 20.0f;
        if (sensor_data.output_current_1 < 4.0f)  sensor_data.output_current_1 = 4.0f;

        ESP_LOGD(TAG, "VolFlow=%.2f | MassFlow=%.1f | Temp=%.2f | Cond=%.1f | I=%.2fmA",
                 sensor_data.volume_flow,
                 sensor_data.mass_flow,
                 sensor_data.temperature,
                 sensor_data.conductivity,
                 sensor_data.output_current_1);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ===== Task استقبال ومعالجة Modbus =====
static void modbus_rx_task(void *pvParam) {
    uint8_t rx_buf[UART_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(50));
        if (len > 0) {
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_buf, len, ESP_LOG_DEBUG);
            process_modbus_request(rx_buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ===== Initialization =====
static void uart_rs485_init(void) {
    // Config MAX485 DE/RE pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RS485_DE_RE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(RS485_DE_RE_PIN, 0);  // RX mode by default

    // UART Config
    uart_config_t uart_cfg = {
        .baud_rate           = BAUD_RATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_EVEN,   // Modbus RTU standard = Even parity
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
        .flags               = 0,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    // RS485 Half-Duplex mode
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "UART2 RS485 initialized | Baud=%d | TX=%d | RX=%d | DE/RE=%d",
             BAUD_RATE, UART_TX_PIN, UART_RX_PIN, RS485_DE_RE_PIN);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== Promag P300 Modbus RTU Simulator ===");
    ESP_LOGI(TAG, "Slave Address: %d", SLAVE_ADDRESS);

    uart_rs485_init();

    xTaskCreate(sensor_update_task, "sensor_update", 4096, NULL, 5, NULL);
    xTaskCreate(modbus_rx_task,     "modbus_rx",     4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Simulator running...");
}