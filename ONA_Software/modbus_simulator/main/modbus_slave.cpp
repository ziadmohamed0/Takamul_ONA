/**
 * @file  modbus_slave.cpp
 */

#include "modbus_slave.hpp"

static const char* TAG = "MB_SIM";

/* ═══════════════════════════════════════════════════════════════════════════
 * Constructor
 * ═══════════════════════════════════════════════════════════════════════════*/
ModbusSlave::ModbusSlave()
{
    m_in.fill(0u);
    m_hr.fill(0u);

    /* Holding register defaults */
    m_hr[Reg::HR_SLAVE_ID]  = Cfg::SLAVE_ID;
    m_hr[Reg::HR_TEMP_SP]   = 300u;    /* 30.0 °C  setpoint  */
    m_hr[Reg::HR_PRESS_ALM] = 10500u;  /* 1050.0 hPa alarm   */
    m_hr[Reg::HR_FLOW_ALM]  = 4000u;   /* 40.00 L/min alarm  */
    m_hr[Reg::HR_SIM_MODE]  = 0u;      /* SINE WAVE default  */

    /* Input register defaults */
    m_in[Reg::IN_TEMP]   = 250u;
    m_in[Reg::IN_PRESS]  = 10132u;
    m_in[Reg::IN_FLOW]   = 2000u;
    m_in[Reg::IN_STATUS] = Reg::ST_SIM;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * begin — UART init + task launch
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::begin()
{
    m_mtx = xSemaphoreCreateMutex();
    configASSERT(m_mtx);

    /* ── UART2 @ 19200 8N1 ── */
    const uart_config_t cfg = {
        .baud_rate           = Cfg::BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(Cfg::UART, &cfg));

    /* Pins: TX=17, RX=16, RTS(DE)=4, CTS=none */
    ESP_ERROR_CHECK(uart_set_pin(Cfg::UART,
                                 Cfg::TX, Cfg::RX,
                                 Cfg::DE,            /* RTS → DE/RE pin */
                                 UART_PIN_NO_CHANGE));

    /* RS-485 half-duplex: ESP-IDF toggles DE automatically via RTS */
    ESP_ERROR_CHECK(uart_driver_install(Cfg::UART, 512, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_set_mode(Cfg::UART, UART_MODE_RS485_HALF_DUPLEX));

    /*
     * RX timeout = inter-frame silence detector.
     * At 19200 baud: 1 char ≈ 0.52 ms → 3.5 chars ≈ 1.8 ms
     * uart_set_rx_timeout unit = number of symbol periods.
     * 16 symbols @ 19200 ≈ 8.3 ms — safe margin above spec minimum.
     */
    ESP_ERROR_CHECK(uart_set_rx_timeout(Cfg::UART, 16));

    ESP_LOGI(TAG, "UART2 ready — Slave ID=%u  Baud=%d", Cfg::SLAVE_ID, Cfg::BAUD);

    xTaskCreatePinnedToCore(mbTaskEntry,  "mb_task",  4096, this, 10, nullptr, 1);
    xTaskCreatePinnedToCore(simTaskEntry, "sim_task", 2048, this,  5, nullptr, 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * crc16 — Modbus CRC-16/ANSI (poly=0xA001, seed=0xFFFF)
 * ═══════════════════════════════════════════════════════════════════════════*/
uint16_t ModbusSlave::crc16(const uint8_t* d, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; ++i) {
        crc ^= static_cast<uint16_t>(d[i]);
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 1u) ? ((crc >> 1u) ^ 0xA001u) : (crc >> 1u);
        }
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * send — transmit a complete RTU frame
 * DE/RE toggling is automatic via UART_MODE_RS485_HALF_DUPLEX
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::send(const uint8_t* buf, size_t len)
{
    uart_write_bytes(Cfg::UART, reinterpret_cast<const char*>(buf), len);
    uart_wait_tx_done(Cfg::UART, pdMS_TO_TICKS(50));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * sendEx — build and send a Modbus exception response
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::sendEx(uint8_t fc, uint8_t code)
{
    uint8_t frame[5];
    frame[0] = static_cast<uint8_t>(m_hr[Reg::HR_SLAVE_ID]);
    frame[1] = fc | FC::EX_BIT;
    frame[2] = code;
    const uint16_t c = crc16(frame, 3u);
    frame[3] = static_cast<uint8_t>(c & 0xFFu);
    frame[4] = static_cast<uint8_t>(c >> 8u);
    send(frame, 5u);
    ESP_LOGW(TAG, "Exception FC=0x%02X code=0x%02X", fc, code);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * onReadHolding — FC03
 *
 * Request  : [slave][0x03][addr_hi][addr_lo][qty_hi][qty_lo][crc_lo][crc_hi]
 * Response : [slave][0x03][byte_cnt][data…][crc_lo][crc_hi]
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::onReadHolding(const uint8_t* f, int len)
{
    if (len < 8) return;

    const uint16_t addr = (static_cast<uint16_t>(f[2]) << 8u) | f[3];
    const uint16_t qty  = (static_cast<uint16_t>(f[4]) << 8u) | f[5];

    if (qty == 0u || qty > 125u) { sendEx(FC::READ_HOLDING, Ex::ILLEGAL_VALUE); return; }
    if (addr >= Reg::HR_COUNT || (addr + qty) > Reg::HR_COUNT) {
        sendEx(FC::READ_HOLDING, Ex::ILLEGAL_ADDR); return;
    }

    const uint16_t data_bytes = qty * 2u;
    const uint16_t total      = 5u + data_bytes;
    uint8_t resp[255];

    resp[0] = f[0];               /* slave ID  */
    resp[1] = FC::READ_HOLDING;
    resp[2] = static_cast<uint8_t>(data_bytes);

    xSemaphoreTake(m_mtx, portMAX_DELAY);
    for (uint16_t i = 0u; i < qty; ++i) {
        resp[3u + i * 2u] = static_cast<uint8_t>(m_hr[addr + i] >> 8u);
        resp[4u + i * 2u] = static_cast<uint8_t>(m_hr[addr + i] & 0xFFu);
    }
    xSemaphoreGive(m_mtx);

    const uint16_t c = crc16(resp, total - 2u);
    resp[total - 2u] = static_cast<uint8_t>(c & 0xFFu);
    resp[total - 1u] = static_cast<uint8_t>(c >> 8u);

    ESP_LOGI(TAG, "FC03 → addr=0x%04X qty=%u", addr, qty);
    send(resp, total);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * onReadInput — FC04  (identical structure to FC03 but reads m_in[])
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::onReadInput(const uint8_t* f, int len)
{
    if (len < 8) return;

    const uint16_t addr = (static_cast<uint16_t>(f[2]) << 8u) | f[3];
    const uint16_t qty  = (static_cast<uint16_t>(f[4]) << 8u) | f[5];

    if (qty == 0u || qty > 125u) { sendEx(FC::READ_INPUT, Ex::ILLEGAL_VALUE); return; }
    if (addr >= Reg::IN_COUNT || (addr + qty) > Reg::IN_COUNT) {
        sendEx(FC::READ_INPUT, Ex::ILLEGAL_ADDR); return;
    }

    const uint16_t data_bytes = qty * 2u;
    const uint16_t total      = 5u + data_bytes;
    uint8_t resp[255];

    resp[0] = f[0];
    resp[1] = FC::READ_INPUT;
    resp[2] = static_cast<uint8_t>(data_bytes);

    xSemaphoreTake(m_mtx, portMAX_DELAY);
    for (uint16_t i = 0u; i < qty; ++i) {
        resp[3u + i * 2u] = static_cast<uint8_t>(m_in[addr + i] >> 8u);
        resp[4u + i * 2u] = static_cast<uint8_t>(m_in[addr + i] & 0xFFu);
    }
    xSemaphoreGive(m_mtx);

    const uint16_t c = crc16(resp, total - 2u);
    resp[total - 2u] = static_cast<uint8_t>(c & 0xFFu);
    resp[total - 1u] = static_cast<uint8_t>(c >> 8u);

    ESP_LOGI(TAG, "FC04 → addr=0x%04X qty=%u", addr, qty);
    send(resp, total);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * onWriteSingle — FC06
 *
 * Request/echo : [slave][0x06][addr_hi][addr_lo][val_hi][val_lo][crc×2]
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::onWriteSingle(const uint8_t* f, int len)
{
    if (len < 8) return;

    const uint16_t addr = (static_cast<uint16_t>(f[2]) << 8u) | f[3];
    const uint16_t val  = (static_cast<uint16_t>(f[4]) << 8u) | f[5];

    if (addr >= Reg::HR_COUNT) { sendEx(FC::WRITE_SINGLE, Ex::ILLEGAL_ADDR); return; }

    xSemaphoreTake(m_mtx, portMAX_DELAY);
    m_hr[addr] = val;
    xSemaphoreGive(m_mtx);

    ESP_LOGI(TAG, "FC06 WRITE HR[0x%04X] = %u", addr, val);

    /* Echo: same bytes as request, recalculate CRC */
    uint8_t echo[8];
    std::memcpy(echo, f, 6u);
    const uint16_t c = crc16(echo, 6u);
    echo[6] = static_cast<uint8_t>(c & 0xFFu);
    echo[7] = static_cast<uint8_t>(c >> 8u);
    send(echo, 8u);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * onWriteMulti — FC16 (0x10)
 *
 * Request:
 *   [slave][0x10][addr_hi][addr_lo][qty_hi][qty_lo][byte_cnt][data…][crc×2]
 * Response:
 *   [slave][0x10][addr_hi][addr_lo][qty_hi][qty_lo][crc×2]
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::onWriteMulti(const uint8_t* f, int len)
{
    if (len < 9) return;

    const uint16_t addr      = (static_cast<uint16_t>(f[2]) << 8u) | f[3];
    const uint16_t qty       = (static_cast<uint16_t>(f[4]) << 8u) | f[5];
    const uint8_t  byte_cnt  = f[6];

    if (qty == 0u || qty > 123u)                  { sendEx(FC::WRITE_MULTI, Ex::ILLEGAL_VALUE); return; }
    if (byte_cnt != static_cast<uint8_t>(qty*2u)) { sendEx(FC::WRITE_MULTI, Ex::ILLEGAL_VALUE); return; }
    if (addr >= Reg::HR_COUNT || (addr+qty) > Reg::HR_COUNT) {
        sendEx(FC::WRITE_MULTI, Ex::ILLEGAL_ADDR); return;
    }

    xSemaphoreTake(m_mtx, portMAX_DELAY);
    for (uint16_t i = 0u; i < qty; ++i) {
        m_hr[addr + i] =
            (static_cast<uint16_t>(f[7u + i*2u]) << 8u) |
             static_cast<uint16_t>(f[8u + i*2u]);
    }
    xSemaphoreGive(m_mtx);

    ESP_LOGI(TAG, "FC16 WRITE %u regs from HR[0x%04X]", qty, addr);

    uint8_t resp[8];
    resp[0] = f[0];
    resp[1] = FC::WRITE_MULTI;
    resp[2] = f[2]; resp[3] = f[3];  /* echo addr */
    resp[4] = f[4]; resp[5] = f[5];  /* echo qty  */
    const uint16_t c = crc16(resp, 6u);
    resp[6] = static_cast<uint8_t>(c & 0xFFu);
    resp[7] = static_cast<uint8_t>(c >> 8u);
    send(resp, 8u);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * nextLfsr — 16-bit Galois LFSR for noise mode
 * ═══════════════════════════════════════════════════════════════════════════*/
uint16_t ModbusSlave::nextLfsr()
{
    const uint16_t lsb = m_lfsr & 1u;
    m_lfsr >>= 1u;
    if (lsb) m_lfsr ^= 0xB400u;
    return m_lfsr;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * updateSensors — called every SIM_TICK_MS from simTask
 *
 *  Mode 0 SINE  : smooth sine wave variation
 *  Mode 1 FIXED : constant at setpoint value
 *  Mode 2 RAMP  : linear ramp up then reset
 *  Mode 3 NOISE : pseudo-random LFSR noise
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::updateSensors()
{
    xSemaphoreTake(m_mtx, portMAX_DELAY);
    const uint16_t mode    = m_hr[Reg::HR_SIM_MODE];
    const uint16_t temp_sp = m_hr[Reg::HR_TEMP_SP];
    xSemaphoreGive(m_mtx);

    float temp  = 0.0f;
    float press = 0.0f;
    float flow  = 0.0f;

    switch (mode) {
        case 0u: /* SINE */
            m_phase += 0.05f;
            if (m_phase > 6.2832f) m_phase -= 6.2832f;
            temp  = 250.0f + 50.0f  * sinf(m_phase);
            press = 10132.0f + 80.0f  * sinf(m_phase * 0.7f);
            flow  = 2000.0f + 500.0f * sinf(m_phase * 1.3f);
            break;

        case 1u: /* FIXED */
            temp  = static_cast<float>(temp_sp);
            press = 10132.0f;
            flow  = 2000.0f;
            break;

        case 2u: /* RAMP */
            m_ramp += 1.0f;
            if (m_ramp > 100.0f) m_ramp = 0.0f;
            temp  = 200.0f + m_ramp;
            press = 10000.0f + m_ramp * 13.0f;
            flow  = 1500.0f + m_ramp * 10.0f;
            break;

        case 3u: /* NOISE */
        default:
            temp  = 250.0f + static_cast<float>(nextLfsr() % 101u) - 50.0f;
            press = 10132.0f + static_cast<float>(nextLfsr() % 201u) - 100.0f;
            flow  = 2000.0f + static_cast<float>(nextLfsr() % 1001u) - 500.0f;
            break;
    }

    /* Clamp */
    if (temp  < 0.0f)     temp  = 0.0f;
    if (temp  > 8500.0f)  temp  = 8500.0f;
    if (press < 0.0f)     press = 0.0f;
    if (flow  < 0.0f)     flow  = 0.0f;

    const uint64_t uptime_s = esp_timer_get_time() / 1000000ULL;

    uint16_t status = Reg::ST_SIM;

    xSemaphoreTake(m_mtx, portMAX_DELAY);
    m_in[Reg::IN_TEMP]      = static_cast<uint16_t>(temp);
    m_in[Reg::IN_PRESS]     = static_cast<uint16_t>(press);
    m_in[Reg::IN_FLOW]      = static_cast<uint16_t>(flow);
    m_in[Reg::IN_UPTIME_HI] = static_cast<uint16_t>(uptime_s >> 16u);
    m_in[Reg::IN_UPTIME_LO] = static_cast<uint16_t>(uptime_s & 0xFFFFu);

    if (m_in[Reg::IN_TEMP]  > m_hr[Reg::HR_TEMP_SP])   status |= Reg::ST_TEMP_ALM;
    if (m_in[Reg::IN_PRESS] > m_hr[Reg::HR_PRESS_ALM])  status |= Reg::ST_PRESS_ALM;
    if (m_in[Reg::IN_FLOW]  > m_hr[Reg::HR_FLOW_ALM])   status |= Reg::ST_FLOW_ALM;
    m_in[Reg::IN_STATUS] = status;
    xSemaphoreGive(m_mtx);

    /* ── Print to UART0 → FTDI232 on STM32 side (not used here, but useful
     *    if you also hook a USB-serial to the ESP32 for debug)            ── */
    printf("\n[SIM] Temp=%5.1f C  Press=%7.1f hPa  Flow=%6.2f L/min  "
           "Status=0x%04X  Uptime=%llus\n",
           static_cast<double>(temp)  / 10.0,
           static_cast<double>(press) / 10.0,
           static_cast<double>(flow)  / 100.0,
           status,
           static_cast<unsigned long long>(uptime_s));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * mbTask — receive loop: read → validate CRC → dispatch
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::mbTask()
{
    static uint8_t buf[256];
    ESP_LOGI(TAG, "Modbus task running — slave ID=%u  baud=%d",
             Cfg::SLAVE_ID, Cfg::BAUD);

    for (;;) {
        /*
         * uart_read_bytes returns when either:
         *   (a) the RX timeout fires (inter-frame silence → end of frame), or
         *   (b) the task timeout of 100 ms expires (no data).
         * The RX-timeout (16 symbol periods ≈ 8.3 ms @ 19200) is what
         * delimits Modbus frames reliably.
         */
        const int n = uart_read_bytes(Cfg::UART, buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
        if (n < 4) continue;   /* too short to be a valid frame */

        /* Is this frame addressed to us? */
        const uint8_t my_id = static_cast<uint8_t>(m_hr[Reg::HR_SLAVE_ID]);
        if (buf[0] != my_id) continue;

        /* CRC check */
        const uint16_t calc = crc16(buf, static_cast<uint16_t>(n - 2));
        const uint16_t recv = static_cast<uint16_t>(buf[n-2])
                            | (static_cast<uint16_t>(buf[n-1]) << 8u);
        if (calc != recv) {
            ESP_LOGW(TAG, "CRC fail: calc=0x%04X recv=0x%04X", calc, recv);
            continue;  /* silent discard per Modbus spec */
        }

        const uint8_t fc = buf[1];
        switch (fc) {
            case FC::READ_HOLDING: onReadHolding(buf, n); break;
            case FC::READ_INPUT:   onReadInput  (buf, n); break;
            case FC::WRITE_SINGLE: onWriteSingle(buf, n); break;
            case FC::WRITE_MULTI:  onWriteMulti (buf, n); break;
            default:
                ESP_LOGW(TAG, "Unknown FC=0x%02X", fc);
                sendEx(fc, Ex::ILLEGAL_FUNC);
                break;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * simTask — update sensor values every SIM_TICK_MS
 * ═══════════════════════════════════════════════════════════════════════════*/
void ModbusSlave::simTask()
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(Cfg::SIM_TICK_MS));
        updateSensors();
    }
}