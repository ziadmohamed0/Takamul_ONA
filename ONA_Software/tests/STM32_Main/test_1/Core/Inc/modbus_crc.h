/*
 * modbus_crc.h
 *
 *  Created on: Sep 28, 2026
 *      Author: ziad
 *
 *  ── FIX LOG ──────────────────────────────────────────────────────────────────
 *  v1.1 - Fixed RS485 DE/RE de-assertion timing:
 *         Wait for TC (Transmission Complete) flag before pulling DE LOW.
 *         Previously HAL_Delay(2) was used which de-asserted DE while the
 *         UART shift register was still transmitting the last byte, causing
 *         truncated frames → CheckSum errors on the slave (ModSim).
 *  ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ── Modbus Function Codes ───────────────────────────────────────────────── */
#define FC_READ_COILS           0x01
#define FC_READ_DISCRETE        0x02
#define FC_READ_HOLDING         0x03
#define FC_READ_INPUT           0x04
#define FC_WRITE_SINGLE_COILS   0x05
#define FC_WRITE_SINGLE_HOLDING 0x06
#define FC_WRITE_MULTI_COILS    0x0F
#define FC_WRITE_MULTI_HOLDING  0x10

/* ── CRC Function ────────────────────────────────────────────────────────── */
uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);

/* ── ModbusMaster Class ──────────────────────────────────────────────────── */
class ModbusMaster {
private:
    UART_HandleTypeDef* _huart;
    GPIO_TypeDef*       _txEnPort;
    uint16_t            _txEnPin;

    uint8_t txBuffer[64];
    uint8_t rxBuffer[64];

    void sendFrame(uint16_t length);   // ← TC-safe version inside .cpp

public:
    ModbusMaster(UART_HandleTypeDef* huart,
                 GPIO_TypeDef*       txEnPort,
                 uint16_t            txEnPin);

    bool readCoils            (uint8_t slaveId, uint16_t startAddress, uint16_t quantity);
    bool readDiscreteInputs   (uint8_t slaveId, uint16_t startAddress, uint16_t quantity);
    bool readHoldingRegisters (uint8_t slaveId, uint16_t startAddress, uint16_t quantity);
    bool readInputRegisters   (uint8_t slaveId, uint16_t startAddress, uint16_t quantity);

    bool writeSingleCoil      (uint8_t slaveId, uint16_t address, bool value);
    bool writeSingleRegister  (uint8_t slaveId, uint16_t address, uint16_t value);
    bool writeMultipleRegisters(uint8_t slaveId, uint16_t startAddress,
                                uint16_t quantity, uint16_t *values);

    uint8_t* getRxData() { return rxBuffer; }
};

#endif /* INC_MODBUS_CRC_H_ */
