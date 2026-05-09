/*
 * modbus_crc.h
 *
 *  Created on: Sep 28, 2026
 *      Author: ziad
 */

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_

#include "main.h"

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);


#endif /* INC_MODBUS_CRC_H_ */
