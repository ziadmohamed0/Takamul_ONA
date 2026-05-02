/*
 * modbus_rtu.h
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
 *                     UART TC flag to guarantee last bit has left the wire
 *                     before DE is de-asserted
 *    v1.5  — [FIX-K] Syntax error in writeMulti comment block removed
 *             [FIX-L] transact() now uses TC-flag polling (was still osDelay(2))
 *             [FIX-M] tx_status checked in writeSingle and writeMulti
 *             [FIX-N] Echo validation extended to address+value in writeSingle
 *                     and address+quantity in writeMulti
 *             [FIX-O] frame_len overflow check moved inside mutex in writeMulti
 *             [CLN-P] RX_OVERFLOW replaced with PROTOCOL_ERROR for byte-count
 *                     mismatch; new Error::PROTOCOL_ERROR added
 *             [CLN-Q] setInterFrameDelay enforces minimum 1 ms guard
 *             [CLN-R] MAX_READ_REGS / MAX_WRITE_REGS typed as uint16_t to
 *                     avoid implicit-conversion compiler warnings
 */

#ifndef INC_MODBUS_RTU_H_
#define INC_MODBUS_RTU_H_

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include <cstdint>
#include <cstring>
#include <array>
#include <cassert>

namespace ModBus {

    /* ── Function codes ──────────────────────────────────────────────────── */
    constexpr uint8_t FC_READ_HOLDING = 0x03u;
    constexpr uint8_t FC_READ_INPUT   = 0x04u;
    constexpr uint8_t FC_WRITE_SINGLE = 0x06u;
    constexpr uint8_t FC_WRITE_MULTI  = 0x10u;

    /* ── Modbus spec limits ───────────────────────────────────────────────── */
    /* [CLN-R] Typed as uint16_t to avoid implicit-conversion warnings when
     *         used in arithmetic expressions with uint16_t operands.        */
    constexpr uint16_t MAX_READ_REGS  = 125u;  /* FC03 / FC04 maximum */
    constexpr uint16_t MAX_WRITE_REGS = 123u;  /* FC16 maximum        */

    /* ── Valid slave address range ───────────────────────────────────────── */
    constexpr uint8_t SLAVE_ADDR_MIN = 1u;
    constexpr uint8_t SLAVE_ADDR_MAX = 247u;

    /* ── Inter-frame delay guard ─────────────────────────────────────────── */
    /* [CLN-Q] Minimum enforced in setInterFrameDelay() to satisfy Modbus spec
     *         requirement of ≥ 3.5 character times silence between frames.  */
    constexpr uint32_t INTER_FRAME_DELAY_MIN_MS = 1u;

    /* ── RAII mutex guard ────────────────────────────────────────────────── */
    struct MutexGuard {
        osMutexId_t handle;
        explicit MutexGuard(osMutexId_t h) : handle(h) {
            osMutexAcquire(handle, osWaitForever);
        }
        ~MutexGuard() { osMutexRelease(handle); }
        MutexGuard(const MutexGuard&)            = delete;
        MutexGuard& operator=(const MutexGuard&) = delete;
    };

    /* ── Error codes ──────────────────────────────────────────────────────── */
    enum class Error : uint8_t {
        OK             = 0u,
        TIMEOUT        = 1u,
        CRC_MISMATCH   = 2u,
        WRONG_SLAVE    = 3u,
        EXCEPTION      = 4u,
        TX_FAIL        = 5u,
        RX_OVERFLOW    = 6u,
        INVALID_ARG    = 7u,  /* null pointer, ID out of range, count out of range */
        PROTOCOL_ERROR = 8u   /* [CLN-P] Byte-count field mismatch in response     */
    };

    /* ── Transaction result ───────────────────────────────────────────────── */
    /*
     * STACK NOTE: sizeof(Result) ≈ 256 bytes (250 for registers + overhead).
     * Functions return Result by value.  The calling RTOS task stack must be
     * large enough — recommend allocating ≥ 1 KB for any task that calls RTU.
     */
    struct Result {
        Error    error          = Error::OK;
        uint8_t  exception_code = 0u;
        uint8_t  count          = 0u;
        std::array<uint16_t, MAX_READ_REGS> registers{};
    };

    /* ── RTU master class ─────────────────────────────────────────────────── */
    class RTU {
    public:
        /**
         * Construct and initialise the Modbus RTU master.
         *
         * @param huart      Pointer to the HAL UART handle (must not be null)
         * @param de_port    GPIO port for the RS-485 DE/RE pin
         * @param de_pin     GPIO pin mask for DE/RE
         * @param timeout_ms RX timeout in milliseconds (default 100 ms)
         */
        RTU(UART_HandleTypeDef* huart,
            GPIO_TypeDef*       de_port,
            uint16_t            de_pin,
            uint32_t            timeout_ms = 100u);

        RTU(const RTU&)            = delete;
        RTU& operator=(const RTU&) = delete;

        /**
         * Read holding registers (FC03).
         *
         * @param ID        Slave address 1–247.
         *                  0 (broadcast) is invalid for reads → INVALID_ARG.
         * @param start_reg Starting register address (0x0000–0xFFFF)
         * @param count     Number of registers to read (1–125)
         * @return          Result with Error::OK and decoded registers on success
         */
        Result readHolding(uint8_t ID, uint16_t start_reg, uint8_t count);

        /**
         * Read input registers (FC04).
         * Same constraints as readHolding().
         */
        Result readInput(uint8_t ID, uint16_t start_reg, uint8_t count);

        /**
         * Write single register (FC06).
         *
         * @param ID        Slave address 1–247
         * @param start_reg Register address (0x0000–0xFFFF)
         * @param value     16-bit value to write
         * @return          Result with Error::OK on success
         */
        Result writeSingle(uint8_t ID, uint16_t start_reg, uint16_t value);

        /**
         * Write multiple registers (FC16).
         *
         * @param ID        Slave address 1–247
         * @param start_reg Starting register address (0x0000–0xFFFF)
         * @param values    Non-null pointer to array of count uint16_t values
         * @param count     Number of registers to write (1–123)
         * @return          Result with Error::OK on success
         */
        Result writeMulti(uint8_t         ID,
                          uint16_t        start_reg,
                          const uint16_t* values,
                          uint8_t         count);

        /**
         * Override the minimum inter-frame silence (default 4 ms).
         *
         * Must satisfy ≥ 3.5 character times at the configured baud rate:
         *     9600 baud   → 4 ms
         *     19200 baud  → 2 ms
         *     115200 baud → 1 ms
         *
         * [CLN-Q] Values below INTER_FRAME_DELAY_MIN_MS (1 ms) are clamped.
         */
        void setInterFrameDelay(uint32_t ms) {
            m_inter_frame_ms = (ms < INTER_FRAME_DELAY_MIN_MS)
                                ? INTER_FRAME_DELAY_MIN_MS
                                : ms;
        }

    private:
        /* ── Hardware handles ─────────────────────────────────────────── */
        UART_HandleTypeDef* m_huart;
        GPIO_TypeDef*       m_de_port;
        uint16_t            m_de_pin;

        /* ── Timing ───────────────────────────────────────────────────── */
        uint32_t m_timeout_ms;
        uint32_t m_inter_frame_ms = 4u;

        /* ── Tx/Rx buffers — max legal RTU frame = 256 bytes ─────────── */
        std::array<uint8_t, 256u> m_tx_buf{};
        std::array<uint8_t, 256u> m_rx_buf{};

        /* ── RTOS mutex ───────────────────────────────────────────────── */
        osMutexId_t m_mutex = nullptr;

        /* ── DE/RE line control ───────────────────────────────────────── */
        void driveEnable();
        void driveDisable();

        /**
         * Wait for the UART Transmission Complete (TC) flag with a safety
         * timeout of 10 ms.  Must be called immediately after
         * HAL_UART_Transmit() returns and before driveDisable(), to guarantee
         * the last stop bit has left the wire before DE is de-asserted.
         *
         * [FIX-J / FIX-L] Replaces osDelay(2) which is tick-phase-dependent
         * and can fire 0–1 ms too early at high RTOS tick rates.
         */
        void waitTxComplete();

        /**
         * Modbus CRC-16/ANSI  (poly 0xA001, seed 0xFFFF).
         */
        uint16_t crc16(const uint8_t* data, uint16_t len);

        /**
         * Flush stale bytes and clear the overrun-error flag from the UART
         * RX FIFO.  Must be called before every HAL_UART_Receive().
         *
         * [FIX-D / FIX-I] Clears ORE flag and drains the data register using
         * the correct register name (DR on F0-F4/L0/L1; RDR on F7/H7/G0/G4/
         * L4/WB/WL) selected at compile time via #ifdef.
         */
        void flushRx();

        /**
         * Build an 8-byte FC03/FC04 request frame into m_tx_buf.
         * Caller MUST hold m_mutex.
         */
        uint8_t buildReadFrame(uint8_t  slave,
                               uint8_t  fc,
                               uint16_t reg,
                               uint8_t  count);

        /**
         * Validate and decode a FC03/FC04 response from m_rx_buf.
         * Caller MUST hold m_mutex.
         */
        Result parseResponse(uint8_t slave,
                             uint8_t fc,
                             uint8_t expected_count);

        /**
         * Execute one full Modbus transaction:
         *   inter-frame silence → TX → TC-poll → RX → parse
         *
         * m_tx_buf must already contain the complete request frame.
         * Caller MUST hold m_mutex.
         */
        Result transact(uint8_t frame_len,
                        uint8_t slave,
                        uint8_t fc,
                        uint8_t expected_regs);
    };

} /* namespace ModBus */

/* ── Utility: two uint16 registers → IEEE-754 float ─────────────────────── */
/*
 * Most Modbus devices store a float as two consecutive 16-bit registers with
 * the high word first (big-endian word order).  If your device uses the
 * opposite order, swap the hi / lo arguments at the call site.
 *
 * Example:
 *   float temperature = regsToFloat(result.registers[0], result.registers[1]);
 */
inline float regsToFloat(uint16_t hi, uint16_t lo)
{
    uint32_t raw = (static_cast<uint32_t>(hi) << 16u)
                 |  static_cast<uint32_t>(lo);
    float f;
    std::memcpy(&f, &raw, sizeof(f));
    return f;
}

#endif /* INC_MODBUS_RTU_H_ */
