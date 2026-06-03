/*
 * modbus_crc.cpp
 *
 *  Created on: Sep 28, 2026
 *      Author: ziad
 *
 *  ── FIX LOG ──────────────────────────────────────────────────────────────────
 *  v1.1 — ROOT CAUSE & FIX:
 *
 *  PROBLEM:
 *    sendFrame() was using HAL_Delay(2) after HAL_UART_Transmit() before
 *    pulling DE (Driver Enable) LOW on the MAX485.
 *
 *    HAL_UART_Transmit() returns as soon as TXE (TX Empty) flag is set,
 *    meaning the TX data register is empty — but the UART SHIFT REGISTER
 *    is STILL physically clocking out the last byte onto the RS485 wire.
 *
 *    Result: DE goes LOW while the last byte is mid-transmission.
 *    The MAX485 stops driving the line → the slave receives a truncated
 *    frame with a corrupted/missing stop bit → CheckSum Error in ModSim.
 *
 *  FIX:
 *    Wait for TC (Transmission Complete) flag using __HAL_UART_GET_FLAG()
 *    with a safe timeout loop BEFORE de-asserting DE.
 *    TC is only set after the shift register has fully serialized AND the
 *    stop bit has been transmitted — the line is truly idle at that point.
 *    A 1ms post-TC guard is kept for MAX485 line-turnaround settling.
 *
 *  v1.2 — WRITE TIMEOUT FIX:
 *
 *  PROBLEM:
 *    writeSingleCoil() and writeSingleRegister() used 1000ms HAL_UART_Receive
 *    timeout. OpenModSim is configured for INPUT REGISTERS only (FC04).
 *    FC05 (Write Coil) and FC06 (Write Register) either return a Modbus
 *    Exception frame (5 bytes, not 8) or no response at all.
 *    Result: each write call blocks for the full 1000ms timeout → 2+ seconds
 *    of UART blocking per control command → readInputRegisters() runs on a
 *    partially corrupted UART state → all sensor values read as 0.
 *
 *  FIX:
 *    Reduced write timeouts from 1000ms → 100ms.
 *    If ModSim responds (even with exception) it arrives in <5ms.
 *    If it doesn't respond, we only lose 100ms instead of 1000ms.
 *    readInputRegisters() timeout kept at 1000ms (it must succeed).
 *  ─────────────────────────────────────────────────────────────────────────────
 */

#include "modbus_crc.h"
#include <cstdint>
#include <array>

/* ── CRC Tables ──────────────────────────────────────────────────────────── */

static constexpr std::array<uint8_t, 256> table_crc_hi = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static constexpr std::array<uint8_t, 256> table_crc_lo = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* ── CRC16 Calculation ───────────────────────────────────────────────────── */

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    unsigned int i;

    while (buffer_length--) {
        i = crc_lo ^ *buffer++;
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

/* ── Constructor ─────────────────────────────────────────────────────────── */

ModbusMaster::ModbusMaster(UART_HandleTypeDef* huart,
                           GPIO_TypeDef*       txEnPort,
                           uint16_t            txEnPin)
{
    _huart     = huart;
    _txEnPort  = txEnPort;
    _txEnPin   = txEnPin;
}

/* ── sendFrame() ─────────────────────────────────────────────────────────────
 *
 *  CRITICAL TIMING:
 *
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │  DE HIGH  │◄──── HAL_UART_Transmit (blocking) ────►│  TC wait  │DE LOW│
 *  └─────────────────────────────────────────────────────────────────────┘
 *
 *  Step 1: Assert DE HIGH (MAX485 TX mode) + 1ms settle
 *  Step 2: HAL_UART_Transmit — blocks until TXE (last byte in DR)
 *           ⚠ At this point the SHIFT REGISTER is still sending!
 *  Step 3: Poll TC flag (Transmission Complete) with 10ms timeout
 *           TC = shift register finished + stop bit on wire
 *  Step 4: 1ms line-turnaround guard for MAX485 propagation
 *  Step 5: De-assert DE LOW (MAX485 RX mode) — line is truly idle now
 *
 * ─────────────────────────────────────────────────────────────────────── */

void ModbusMaster::sendFrame(uint16_t length)
{
    /* ── Step 1: Assert DE (TX mode) with settle time ── */
    HAL_GPIO_WritePin(_txEnPort, _txEnPin, GPIO_PIN_SET);
    HAL_Delay(1);

    /* ── Step 2: Transmit frame (returns after TXE, NOT TC) ── */
    HAL_UART_Transmit(_huart, txBuffer, length, 1000);

    /* ── Step 3: Wait for TC — shift register truly idle ── */
    uint32_t tc_timeout = HAL_GetTick() + 10;   // 10ms max safety timeout
    while (!__HAL_UART_GET_FLAG(_huart, UART_FLAG_TC)) {
        if (HAL_GetTick() >= tc_timeout) {
            break;   // timeout guard — never hang in production
        }
    }
    __HAL_UART_CLEAR_FLAG(_huart, UART_FLAG_TC);

    /* ── Step 4: Line-turnaround guard (MAX485 propagation) ── */
    HAL_Delay(1);

    /* ── Step 5: De-assert DE (RX mode) — line is fully idle ── */
    HAL_GPIO_WritePin(_txEnPort, _txEnPin, GPIO_PIN_RESET);
}

/* ── Read Functions ──────────────────────────────────────────────────────── */

bool ModbusMaster::readCoils(uint8_t slaveId, uint16_t startAddress, uint16_t quantity)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_READ_COILS;
    txBuffer[2] = (startAddress >> 8) & 0xFF;
    txBuffer[3] =  startAddress       & 0xFF;
    txBuffer[4] = (quantity    >> 8)  & 0xFF;
    txBuffer[5] =  quantity           & 0xFF;

    uint16_t crc = crc16(txBuffer, 6);
    txBuffer[6] =  crc        & 0xFF;
    txBuffer[7] = (crc >> 8)  & 0xFF;

    sendFrame(8);

    uint8_t byteCount    = (quantity + 7) / 8;
    uint8_t expectedBytes = 3 + byteCount + 2;

    return (HAL_UART_Receive(_huart, rxBuffer, expectedBytes, 1000) == HAL_OK);
}

bool ModbusMaster::readDiscreteInputs(uint8_t slaveId, uint16_t startAddress, uint16_t quantity)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_READ_DISCRETE;
    txBuffer[2] = (startAddress >> 8) & 0xFF;
    txBuffer[3] =  startAddress       & 0xFF;
    txBuffer[4] = (quantity    >> 8)  & 0xFF;
    txBuffer[5] =  quantity           & 0xFF;

    uint16_t crc = crc16(txBuffer, 6);
    txBuffer[6] =  crc        & 0xFF;
    txBuffer[7] = (crc >> 8)  & 0xFF;

    sendFrame(8);

    uint8_t byteCount     = (quantity + 7) / 8;
    uint8_t expectedBytes = 3 + byteCount + 2;

    return (HAL_UART_Receive(_huart, rxBuffer, expectedBytes, 1000) == HAL_OK);
}

bool ModbusMaster::readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t quantity)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_READ_HOLDING;
    txBuffer[2] = (startAddress >> 8) & 0xFF;
    txBuffer[3] =  startAddress       & 0xFF;
    txBuffer[4] = (quantity    >> 8)  & 0xFF;
    txBuffer[5] =  quantity           & 0xFF;

    uint16_t crc = crc16(txBuffer, 6);
    txBuffer[6] =  crc        & 0xFF;
    txBuffer[7] = (crc >> 8)  & 0xFF;

    sendFrame(8);

    uint8_t expectedBytes = 3 + (quantity * 2) + 2;

    return (HAL_UART_Receive(_huart, rxBuffer, expectedBytes, 1000) == HAL_OK);
}

bool ModbusMaster::readInputRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t quantity)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_READ_INPUT;
    txBuffer[2] = (startAddress >> 8) & 0xFF;
    txBuffer[3] =  startAddress       & 0xFF;
    txBuffer[4] = (quantity    >> 8)  & 0xFF;
    txBuffer[5] =  quantity           & 0xFF;

    uint16_t crc = crc16(txBuffer, 6);
    txBuffer[6] =  crc        & 0xFF;
    txBuffer[7] = (crc >> 8)  & 0xFF;

    sendFrame(8);

    // ── Drain any garbage bytes left in RX register before reading response
    {
        uint8_t dummy;
        while (HAL_UART_Receive(_huart, &dummy, 1, 0) == HAL_OK) {}
        __HAL_UART_CLEAR_FLAG(_huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
    }

    uint8_t expectedBytes = 3 + (quantity * 2) + 2;
    return (HAL_UART_Receive(_huart, rxBuffer, expectedBytes, 1000) == HAL_OK);
}

/* ── Write Functions ─────────────────────────────────────────────────────── */

bool ModbusMaster::writeSingleCoil(uint8_t slaveId, uint16_t address, bool state)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_WRITE_SINGLE_COILS;
    txBuffer[2] = (address >> 8) & 0xFF;
    txBuffer[3] =  address       & 0xFF;
    txBuffer[4] = state ? 0xFF : 0x00;
    txBuffer[5] = 0x00;

    uint16_t crc = crc16(txBuffer, 6);
    txBuffer[6] =  crc        & 0xFF;
    txBuffer[7] = (crc >> 8)  & 0xFF;

    sendFrame(8);

    /* ── WRITE timeout: 100ms — if ModSim doesn't support FC05, don't block ── */
    return (HAL_UART_Receive(_huart, rxBuffer, 8, 100) == HAL_OK);
}

bool ModbusMaster::writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_WRITE_SINGLE_HOLDING;
    txBuffer[2] = (address >> 8) & 0xFF;
    txBuffer[3] =  address       & 0xFF;
    txBuffer[4] = (value   >> 8) & 0xFF;
    txBuffer[5] =  value         & 0xFF;

    uint16_t crc = crc16(txBuffer, 6);
    txBuffer[6] =  crc        & 0xFF;
    txBuffer[7] = (crc >> 8)  & 0xFF;

    sendFrame(8);

    /* ── WRITE timeout: 100ms — if ModSim doesn't support FC06, don't block ── */
    return (HAL_UART_Receive(_huart, rxBuffer, 8, 100) == HAL_OK);
}

bool ModbusMaster::writeMultipleRegisters(uint8_t  slaveId,
                                          uint16_t startAddress,
                                          uint16_t quantity,
                                          uint16_t *values)
{
    txBuffer[0] = slaveId;
    txBuffer[1] = FC_WRITE_MULTI_HOLDING;
    txBuffer[2] = (startAddress >> 8) & 0xFF;
    txBuffer[3] =  startAddress       & 0xFF;
    txBuffer[4] = (quantity     >> 8) & 0xFF;
    txBuffer[5] =  quantity           & 0xFF;

    uint8_t  byteCount    = quantity * 2;
    txBuffer[6]           = byteCount;

    uint16_t bufferIndex  = 7;
    for (uint16_t i = 0; i < quantity; i++) {
        txBuffer[bufferIndex++] = (values[i] >> 8) & 0xFF;
        txBuffer[bufferIndex++] =  values[i]        & 0xFF;
    }

    uint16_t totalLen = 7 + byteCount;
    uint16_t crc      = crc16(txBuffer, totalLen);
    txBuffer[bufferIndex++] =  crc        & 0xFF;
    txBuffer[bufferIndex++] = (crc >> 8)  & 0xFF;

    sendFrame(bufferIndex);

    return (HAL_UART_Receive(_huart, rxBuffer, 8, 100) == HAL_OK);
}
