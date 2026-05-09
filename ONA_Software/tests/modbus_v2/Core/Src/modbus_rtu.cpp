/*
 * modbus_rtu.cpp
 *
 *  Created on: Apr 22, 2026
 *      Author: ziad
 *
 *  Revision history:
 *    v1.0  — Initial implementation
 *    v1.1  — uint8_t→uint16_t for register addresses/values; CRC byte-order
 *             corrected; writeMulti response validation; count high-byte fix
 *    v1.2  — Mutex race in read functions fixed; FC16 limit enforced (≤123);
 *             null pointer guard; byte-count field validated in parseResponse
 *    v1.3  — [FIX-A] writeSingle TX timeout unified: frame_len+10 (was 20 ms)
 *             [FIX-B] ID=0 broadcast guard on all read/write functions
 *             [FIX-D] UART RX FIFO flushed before every HAL_UART_Receive call
 *             [FIX-F] Result::registers uses MAX_READ_REGS=125; stack note added
 *             [CLN-G] Removed redundant 'static' from namespace-scope constexpr
 *    v1.4  — [FIX-H] osMutexNew failure now caught with configASSERT in ctor
 *             [FIX-I] flushRx: DR vs RDR selected via STM32 family #ifdef;
 *                     previously would not compile on STM32F7 / H7 series
 *             [FIX-J] Post-TX turnaround: osDelay(2) replaced with polling on
 *                     UART TC flag in writeSingle and writeMulti
 *    v1.5  — [FIX-K] Syntax error (dangling comment prefix) in writeMulti
 *                     removed — file would not compile previously
 *             [FIX-L] transact() now uses waitTxComplete() (TC-flag polling)
 *                     instead of osDelay(2) — same fix applied to read path
 *             [FIX-M] tx_status now checked in writeSingle and writeMulti;
 *                     TX failure no longer silently falls through to RX phase
 *             [FIX-N] Echo validation in writeSingle extended to verify echoed
 *                     register address and value, not just slave ID + FC;
 *                     writeMulti verifies echoed address and quantity fields
 *             [FIX-O] frame_len overflow check moved inside mutex in writeMulti
 *                     to eliminate the window between check and buffer use
 *             [CLN-P] parseResponse returns PROTOCOL_ERROR (not RX_OVERFLOW)
 *                     on byte-count field mismatch — name now matches meaning
 *             [CLN-Q] setInterFrameDelay clamps values below 1 ms (header)
 *             [CLN-R] MAX_READ_REGS / MAX_WRITE_REGS typed as uint16_t
 *             [CLN-S] waitTxComplete() extracted as a private helper to
 *                     de-duplicate TC-polling code across all three TX paths
 */

#include "modbus_rtu.h"

namespace ModBus {

/* ═══════════════════════════════════════════════════════════════════════════
 * Constructor
 * ═══════════════════════════════════════════════════════════════════════════*/
RTU::RTU(UART_HandleTypeDef* huart,
         GPIO_TypeDef*       de_port,
         uint16_t            de_pin,
         uint32_t            timeout_ms)
    : m_huart(huart),
      m_de_port(de_port),
      m_de_pin(de_pin),
      m_timeout_ms(timeout_ms)
{
    m_tx_buf.fill(0u);
    m_rx_buf.fill(0u);

    m_mutex = osMutexNew(nullptr);

    /* [FIX-H] A null mutex handle causes silent undefined behaviour in every
     * osMutexAcquire call.  configASSERT halts in debug builds and triggers
     * the fault handler in release builds — both are safer than continuing
     * with an invalid handle.                                               */
    configASSERT(m_mutex != nullptr);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FC03 — Read Holding Registers
 * ═══════════════════════════════════════════════════════════════════════════*/
Result RTU::readHolding(uint8_t ID, uint16_t start_reg, uint8_t count)
{
    /* [FIX-B] Reject broadcast address — reads require a unicast slave ID */
    if (ID < SLAVE_ADDR_MIN || ID > SLAVE_ADDR_MAX) {
        return {Error::INVALID_ARG};
    }
    if (count == 0u || count > static_cast<uint8_t>(MAX_READ_REGS)) {
        return {Error::INVALID_ARG};
    }

    /* Acquire mutex before buildReadFrame so m_tx_buf and the subsequent
     * transact() call are atomic — no other task may touch m_tx_buf between
     * frame construction and transmission.                                  */
    MutexGuard lock{m_mutex};
    const uint8_t len = buildReadFrame(ID, FC_READ_HOLDING, start_reg, count);
    return transact(len, ID, FC_READ_HOLDING, count);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FC04 — Read Input Registers
 * ═══════════════════════════════════════════════════════════════════════════*/
Result RTU::readInput(uint8_t ID, uint16_t start_reg, uint8_t count)
{
    if (ID < SLAVE_ADDR_MIN || ID > SLAVE_ADDR_MAX) {
        return {Error::INVALID_ARG};
    }
    if (count == 0u || count > static_cast<uint8_t>(MAX_READ_REGS)) {
        return {Error::INVALID_ARG};
    }

    MutexGuard lock{m_mutex};
    const uint8_t len = buildReadFrame(ID, FC_READ_INPUT, start_reg, count);
    return transact(len, ID, FC_READ_INPUT, count);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FC06 — Write Single Register
 * ═══════════════════════════════════════════════════════════════════════════*/
Result RTU::writeSingle(uint8_t ID, uint16_t start_reg, uint16_t value)
{
    /* [FIX-B] Validate slave address */
    if (ID < SLAVE_ADDR_MIN || ID > SLAVE_ADDR_MAX) {
        return {Error::INVALID_ARG};
    }

    MutexGuard lock{m_mutex};

    /* Build 8-byte FC06 request ─────────────────────────────────────────
     * [0]   slave ID
     * [1]   0x06
     * [2]   register address hi
     * [3]   register address lo
     * [4]   value hi
     * [5]   value lo
     * [6]   CRC lo
     * [7]   CRC hi
     * ──────────────────────────────────────────────────────────────────── */
    m_tx_buf[0u] = ID;
    m_tx_buf[1u] = FC_WRITE_SINGLE;
    m_tx_buf[2u] = static_cast<uint8_t>(start_reg >> 8u);
    m_tx_buf[3u] = static_cast<uint8_t>(start_reg & 0xFFu);
    m_tx_buf[4u] = static_cast<uint8_t>(value >> 8u);
    m_tx_buf[5u] = static_cast<uint8_t>(value & 0xFFu);

    const uint16_t crc = crc16(m_tx_buf.data(), 6u);
    m_tx_buf[6u] = static_cast<uint8_t>(crc & 0xFFu);   /* CRC lo */
    m_tx_buf[7u] = static_cast<uint8_t>(crc >> 8u);     /* CRC hi */

    /* Transmit ──────────────────────────────────────────────────────────── */
    osDelay(m_inter_frame_ms);
    driveEnable();

    /* [FIX-A] Timeout = frame_bytes + 10 ms (was hardcoded 20 ms) */
    const HAL_StatusTypeDef tx_status =
        HAL_UART_Transmit(m_huart, m_tx_buf.data(), 8u, 8u + 10u);

    /* [FIX-J / CLN-S] Poll TC flag via shared helper */
    waitTxComplete();
    driveDisable();

    /* [FIX-M] Check TX result before attempting to receive */
    if (tx_status != HAL_OK) {
        return {Error::TX_FAIL};
    }

    /* Receive echo ─────────────────────────────────────────────────────── */
    std::array<uint8_t, 8u> echo{};
    flushRx();   /* [FIX-D] Clear stale bytes / overrun flag */
    const HAL_StatusTypeDef rx_status =
        HAL_UART_Receive(m_huart, echo.data(), 8u, m_timeout_ms);

    if (rx_status != HAL_OK) {
        return {Error::TIMEOUT};
    }

    /* Validate slave ID and function code */
    if (echo[0u] != ID || echo[1u] != FC_WRITE_SINGLE) {
        return {Error::WRONG_SLAVE};
    }

    /* [FIX-N] Validate echoed register address and written value */
    if (echo[2u] != m_tx_buf[2u] || echo[3u] != m_tx_buf[3u] ||
        echo[4u] != m_tx_buf[4u] || echo[5u] != m_tx_buf[5u])
    {
        return {Error::PROTOCOL_ERROR};
    }

    /* Validate CRC over first 6 bytes */
    const uint16_t calc_crc = crc16(echo.data(), 6u);
    if (echo[6u] != static_cast<uint8_t>(calc_crc & 0xFFu) ||
        echo[7u] != static_cast<uint8_t>(calc_crc >> 8u))
    {
        return {Error::CRC_MISMATCH};
    }

    return {Error::OK};
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FC16 — Write Multiple Registers
 * ═══════════════════════════════════════════════════════════════════════════*/
Result RTU::writeMulti(uint8_t         ID,
                       uint16_t        start_reg,
                       const uint16_t* values,
                       uint8_t         count)
{
    /* [FIX-B] Validate slave address */
    if (ID < SLAVE_ADDR_MIN || ID > SLAVE_ADDR_MAX) {
        return {Error::INVALID_ARG};
    }
    /* Enforce Modbus FC16 spec limit */
    if (count == 0u || count > static_cast<uint8_t>(MAX_WRITE_REGS)) {
        return {Error::INVALID_ARG};
    }
    /* Guard null pointer */
    if (values == nullptr) {
        return {Error::INVALID_ARG};
    }

    MutexGuard lock{m_mutex};

    /* [FIX-O] Overflow check moved inside mutex to eliminate the window
     * between the check and the actual buffer use.                         */
    const size_t frame_len = 9u + (static_cast<size_t>(count) * 2u);
    if (frame_len > m_tx_buf.size()) {
        /* Should never trigger given MAX_WRITE_REGS=123 → frame=255 bytes */
        return {Error::TX_FAIL};
    }

    /* Build FC16 request ─────────────────────────────────────────────────
     * [0]       slave ID
     * [1]       0x10
     * [2–3]     starting address hi / lo
     * [4–5]     quantity hi / lo  (hi always 0; max 123 fits in lo byte)
     * [6]       byte count = count × 2  (max 246, fits in uint8_t)
     * [7…N-3]   register data: hi byte then lo byte per register
     * [N-2]     CRC lo
     * [N-1]     CRC hi
     * ──────────────────────────────────────────────────────────────────── */
    uint16_t idx = 0u;
    m_tx_buf[idx++] = ID;
    m_tx_buf[idx++] = FC_WRITE_MULTI;
    m_tx_buf[idx++] = static_cast<uint8_t>(start_reg >> 8u);
    m_tx_buf[idx++] = static_cast<uint8_t>(start_reg & 0xFFu);
    m_tx_buf[idx++] = 0u;                                        /* qty hi — always 0 (max 123) */
    m_tx_buf[idx++] = count;                                     /* qty lo */
    m_tx_buf[idx++] = static_cast<uint8_t>(count * 2u);         /* byte count: 123×2=246 < 255 */

    for (uint8_t i = 0u; i < count; ++i) {
        m_tx_buf[idx++] = static_cast<uint8_t>(values[i] >> 8u);
        m_tx_buf[idx++] = static_cast<uint8_t>(values[i] & 0xFFu);
    }

    const uint16_t crc = crc16(m_tx_buf.data(), idx);
    m_tx_buf[idx++] = static_cast<uint8_t>(crc & 0xFFu);  /* CRC lo */
    m_tx_buf[idx++] = static_cast<uint8_t>(crc >> 8u);    /* CRC hi */

    /* Transmit ──────────────────────────────────────────────────────────── */
    osDelay(m_inter_frame_ms);
    driveEnable();

    const HAL_StatusTypeDef tx_status =
        HAL_UART_Transmit(m_huart, m_tx_buf.data(), idx,
                          static_cast<uint32_t>(idx) + 10u);

    /* [FIX-J / CLN-S] Poll TC flag via shared helper */
    waitTxComplete();
    driveDisable();

    /* [FIX-M] Check TX result before attempting to receive */
    if (tx_status != HAL_OK) {
        return {Error::TX_FAIL};
    }

    /* Receive response ──────────────────────────────────────────────────
     * FC16 normal response layout:
     *   [0]   slave ID
     *   [1]   0x10
     *   [2–3] starting address echo (hi / lo)
     *   [4–5] quantity written echo (hi / lo)
     *   [6]   CRC lo
     *   [7]   CRC hi
     * ──────────────────────────────────────────────────────────────────── */
    std::array<uint8_t, 8u> resp{};
    flushRx();   /* [FIX-D] Clear stale bytes / overrun flag */
    const HAL_StatusTypeDef rx_status =
        HAL_UART_Receive(m_huart, resp.data(), 8u, m_timeout_ms);

    if (rx_status != HAL_OK) {
        return {Error::TIMEOUT};
    }

    /* Validate slave ID and function code */
    if (resp[0u] != ID || resp[1u] != FC_WRITE_MULTI) {
        return {Error::WRONG_SLAVE};
    }

    /* [FIX-N] Validate echoed starting address and quantity */
    if (resp[2u] != m_tx_buf[2u] || resp[3u] != m_tx_buf[3u] ||
        resp[4u] != m_tx_buf[4u] || resp[5u] != m_tx_buf[5u])
    {
        return {Error::PROTOCOL_ERROR};
    }

    /* Validate CRC over first 6 bytes */
    const uint16_t resp_crc = crc16(resp.data(), 6u);
    if (resp[6u] != static_cast<uint8_t>(resp_crc & 0xFFu) ||
        resp[7u] != static_cast<uint8_t>(resp_crc >> 8u))
    {
        return {Error::CRC_MISMATCH};
    }

    return {Error::OK};
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DE/RE line control  (ADM2587E: DE and RE are tied to the same pin)
 * ═══════════════════════════════════════════════════════════════════════════*/
void RTU::driveEnable()
{
    HAL_GPIO_WritePin(m_de_port, m_de_pin, GPIO_PIN_SET);
}

void RTU::driveDisable()
{
    HAL_GPIO_WritePin(m_de_port, m_de_pin, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * waitTxComplete — poll UART TC flag before de-asserting DE
 *
 * HAL_UART_Transmit() returns as soon as the last byte is loaded into the
 * shift register (TXE set), but the stop bit of that byte may still be on
 * the wire.  De-asserting DE before TC is set corrupts the last stop bit.
 *
 * osDelay(2) was the previous workaround but it is tick-phase-dependent:
 * at a 1 ms RTOS tick rate osDelay(2) can return after only ~1 ms (if the
 * tick fires immediately), which is insufficient for slow baud rates.
 *
 * This helper busy-polls TC with a 10 ms safety timeout to prevent an
 * infinite loop on hardware faults (stuck TC).
 *
 * [CLN-S] Extracted from writeSingle, writeMulti, and transact to avoid
 *         code duplication across three TX paths.
 * ═══════════════════════════════════════════════════════════════════════════*/
void RTU::waitTxComplete()
{
    const uint32_t t0 = osKernelGetTickCount();
    while (!__HAL_UART_GET_FLAG(m_huart, UART_FLAG_TC)) {
        if ((osKernelGetTickCount() - t0) >= 10u) {
            break;  /* Safety timeout — hardware may be faulty */
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * flushRx — clear the UART overrun-error flag and drain stale bytes
 *
 * On STM32 HAL, a previous TIMEOUT or noise event can leave the UART with
 * an overrun flag set; the next HAL_UART_Receive then reads stale/garbage
 * data instead of the actual slave response.  Clearing the flag and draining
 * any residual byte from the data register before every receive call prevents
 * this class of error.
 *
 * Register name differs by STM32 peripheral IP version:
 *   STM32F0/F1/F2/F3/F4/L0/L1  →  USART_TypeDef::DR   (USART v1 IP)
 *   STM32F7/H7/G0/G4/L4/WB/WL  →  USART_TypeDef::RDR  (USART v2 IP)
 *
 * [FIX-I] The correct register is selected at compile time via #ifdef on
 *         the peripheral bit-field macro defined by the device header.  A
 *         hard #error is emitted for unrecognised families so the problem
 *         surfaces at compile time rather than silently producing wrong code.
 * ═══════════════════════════════════════════════════════════════════════════*/
void RTU::flushRx()
{
    /* Clear the overrun-error flag unconditionally */
    __HAL_UART_CLEAR_OREFLAG(m_huart);

    /* Drain one residual byte if the RX-not-empty flag is set.
     * The volatile read is cast to void so the compiler cannot optimise
     * it away even under aggressive optimisation levels.                    */
#if defined(USART_DR_DR)        /* F0/F1/F2/F3/F4/L0/L1 — register named DR  */
    if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_RXNE)) {
        (void)m_huart->Instance->DR;
    }
#elif defined(USART_RDR_RDR)    /* F7/H7/G0/G4/L4/WB/WL — register named RDR */
    if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_RXNE)) {
        (void)m_huart->Instance->RDR;
    }
#else
    #error "flushRx: cannot determine UART data register name for this STM32 family."
    #error "Add a matching #elif for USART_TypeDef::DR or ::RDR above."
#endif
}

/* ═══════════════════════════════════════════════════════════════════════════
 * crc16 — Modbus CRC-16/ANSI  (poly 0xA001, init 0xFFFF)
 * ═══════════════════════════════════════════════════════════════════════════*/
uint16_t RTU::crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (uint8_t b = 0u; b < 8u; ++b) {
            if (crc & 0x0001u) {
                crc = (crc >> 1u) ^ 0xA001u;
            } else {
                crc >>= 1u;
            }
        }
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * buildReadFrame — write FC03/FC04 request bytes into m_tx_buf
 *
 * Frame layout (8 bytes):
 *   [0]   slave ID
 *   [1]   function code (0x03 or 0x04)
 *   [2]   starting address hi
 *   [3]   starting address lo
 *   [4]   0x00  (quantity hi — max 125 always fits in one byte)
 *   [5]   count (quantity lo)
 *   [6]   CRC lo
 *   [7]   CRC hi
 *
 * Caller MUST hold m_mutex.
 * ═══════════════════════════════════════════════════════════════════════════*/
uint8_t RTU::buildReadFrame(uint8_t  slave,
                            uint8_t  fc,
                            uint16_t reg,
                            uint8_t  count)
{
    m_tx_buf[0u] = slave;
    m_tx_buf[1u] = fc;
    m_tx_buf[2u] = static_cast<uint8_t>(reg >> 8u);
    m_tx_buf[3u] = static_cast<uint8_t>(reg & 0xFFu);
    m_tx_buf[4u] = 0u;     /* quantity hi — always 0 (max 125 < 256) */
    m_tx_buf[5u] = count;  /* quantity lo */

    const uint16_t crc = crc16(m_tx_buf.data(), 6u);
    m_tx_buf[6u] = static_cast<uint8_t>(crc & 0xFFu);  /* CRC lo */
    m_tx_buf[7u] = static_cast<uint8_t>(crc >> 8u);    /* CRC hi */

    return 8u;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * parseResponse — validate and decode a FC03/FC04 response from m_rx_buf
 *
 * Expected frame layout  (total_len = 5 + expected_count × 2):
 *   [0]           slave ID
 *   [1]           function code
 *   [2]           byte count  (MUST equal expected_count × 2)
 *   [3 … N-3]     register data: hi byte then lo byte per register
 *   [N-2]         CRC lo
 *   [N-1]         CRC hi
 *
 * Validation order (fail-fast):
 *   1. Slave ID
 *   2. Exception flag  (FC | 0x80)
 *   3. Function code
 *   4. Byte-count field  → PROTOCOL_ERROR on mismatch  [CLN-P]
 *   5. CRC
 *   6. Data extraction
 *
 * Caller MUST hold m_mutex.
 * ═══════════════════════════════════════════════════════════════════════════*/
Result RTU::parseResponse(uint8_t slave, uint8_t fc, uint8_t expected_count)
{
    Result r{};
    const uint16_t total_len =
        5u + static_cast<uint16_t>(expected_count) * 2u;

    /* 1. Slave ID ─────────────────────────────────────────────────────── */
    if (m_rx_buf[0u] != slave) {
        r.error = Error::WRONG_SLAVE;
        return r;
    }

    /* 2. Exception response — function code with bit 7 set ────────────── */
    if (m_rx_buf[1u] == static_cast<uint8_t>(fc | 0x80u)) {
        r.error          = Error::EXCEPTION;
        r.exception_code = m_rx_buf[2u];
        return r;
    }

    /* 3. Function code ─────────────────────────────────────────────────── */
    if (m_rx_buf[1u] != fc) {
        r.error = Error::WRONG_SLAVE;
        return r;
    }

    /* 4. Byte-count field ──────────────────────────────────────────────── */
    /* [CLN-P] Byte-count mismatch is a protocol-level error, not an overflow */
    if (m_rx_buf[2u] != static_cast<uint8_t>(expected_count * 2u)) {
        r.error = Error::PROTOCOL_ERROR;
        return r;
    }

    /* 5. CRC — RTU convention: lo byte at [N-2], hi byte at [N-1] ─────── */
    const uint16_t calc_crc = crc16(m_rx_buf.data(), total_len - 2u);
    const uint16_t recv_crc =
          static_cast<uint16_t>(m_rx_buf[total_len - 2u])             /* lo */
        | (static_cast<uint16_t>(m_rx_buf[total_len - 1u]) << 8u);   /* hi */

    if (calc_crc != recv_crc) {
        r.error = Error::CRC_MISMATCH;
        return r;
    }

    /* 6. Extract register values (big-endian: hi byte first) ──────────── */
    r.count = expected_count;
    for (uint8_t i = 0u; i < expected_count; ++i) {
        r.registers[i] =
              (static_cast<uint16_t>(m_rx_buf[3u + (i * 2u)]) << 8u)
            |  static_cast<uint16_t>(m_rx_buf[4u + (i * 2u)]);
    }

    r.error = Error::OK;
    return r;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * transact — inter-frame silence → TX → TC-poll → RX → parse
 *
 * Implements the complete read-side transaction for FC03 and FC04.
 * m_tx_buf must already contain the complete request frame.
 * Caller MUST already hold m_mutex.
 * ═══════════════════════════════════════════════════════════════════════════*/
Result RTU::transact(uint8_t frame_len,
                     uint8_t slave,
                     uint8_t fc,
                     uint8_t expected_regs)
{
    /* Inter-frame silence gap (≥ 3.5 character times per Modbus spec) */
    osDelay(m_inter_frame_ms);

    /* TX phase ──────────────────────────────────────────────────────────── */
    driveEnable();

    const HAL_StatusTypeDef tx_status =
        HAL_UART_Transmit(m_huart,
                          m_tx_buf.data(),
                          frame_len,
                          static_cast<uint32_t>(frame_len) + 10u);

    /* [FIX-L / CLN-S] Poll TC flag — replaces the old osDelay(2) */
    waitTxComplete();
    driveDisable();

    if (tx_status != HAL_OK) {
        return {Error::TX_FAIL};
    }

    /* RX phase ──────────────────────────────────────────────────────────── */
    const uint16_t rx_len =
        5u + static_cast<uint16_t>(expected_regs) * 2u;

    m_rx_buf.fill(0u);
    flushRx();   /* [FIX-D] Clear stale bytes / overrun flag before receive */

    const HAL_StatusTypeDef rx_status =
        HAL_UART_Receive(m_huart, m_rx_buf.data(), rx_len, m_timeout_ms);

    if (rx_status != HAL_OK) {
        return {Error::TIMEOUT};
    }

    return parseResponse(slave, fc, expected_regs);
}

} /* namespace ModBus */
