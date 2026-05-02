/**
 * @file    modbus_slave.hpp
 * @brief   Modbus RTU Slave Simulator
 *          ESP32-WROOM + MAX485
 *
 *  Wiring:
 *    ESP32 GPIO17 (UART2 TX) ──► MAX485 DI
 *    ESP32 GPIO16 (UART2 RX) ◄── MAX485 RO
 *    ESP32 GPIO4             ──► MAX485 DE + RE (tied)
 *    MAX485 A / B            ◄──► RS-485 bus → STM32 side
 *
 *  Baud rate : 19200
 *  Slave ID  : 1
 *  Parity    : None
 *  Stop bits : 1
 *
 *  Register Map
 *  ─────────────────────────────────────────────────────────
 *  INPUT REGISTERS  FC04  (read-only — simulated sensor data)
 *    0x0000  Temperature   [°C × 10]      e.g. 253 → 25.3 °C
 *    0x0001  Pressure      [hPa × 10]     e.g. 10132 → 1013.2 hPa
 *    0x0002  Flow rate     [L/min × 100]  e.g. 2000 → 20.00 L/min
 *    0x0003  Status        bitfield
 *    0x0004  Uptime HI     upper 16 bits of uptime in seconds
 *    0x0005  Uptime LO     lower 16 bits of uptime in seconds
 *
 *  HOLDING REGISTERS  FC03  (read/write — setpoints & config)
 *    0x0000  Slave address  (1–247)
 *    0x0001  Temp setpoint  [°C × 10]
 *    0x0002  Press alarm    [hPa × 10]
 *    0x0003  Flow alarm     [L/min × 100]
 *    0x0004  Sim mode       0=SINE  1=FIXED  2=RAMP  3=NOISE
 */

#pragma once

#include <cstdint>
#include <array>
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

/* ── Hardware config ─────────────────────────────────────────────────────── */
namespace Cfg {
    constexpr uart_port_t  UART        = UART_NUM_2;
    constexpr gpio_num_t   TX          = GPIO_NUM_17;
    constexpr gpio_num_t   RX          = GPIO_NUM_16;
    constexpr gpio_num_t   DE          = GPIO_NUM_4;
    constexpr int          BAUD        = 19200;
    constexpr uint8_t      SLAVE_ID    = 1u;
    constexpr uint32_t     SIM_TICK_MS = 200u;   /* sensor update period */
}

/* ── FC codes ────────────────────────────────────────────────────────────── */
namespace FC {
    constexpr uint8_t READ_HOLDING = 0x03u;
    constexpr uint8_t READ_INPUT   = 0x04u;
    constexpr uint8_t WRITE_SINGLE = 0x06u;
    constexpr uint8_t WRITE_MULTI  = 0x10u;
    constexpr uint8_t EX_BIT       = 0x80u;
}

/* ── Exception codes ─────────────────────────────────────────────────────── */
namespace Ex {
    constexpr uint8_t ILLEGAL_FUNC  = 0x01u;
    constexpr uint8_t ILLEGAL_ADDR  = 0x02u;
    constexpr uint8_t ILLEGAL_VALUE = 0x03u;
}

/* ── Register indices ────────────────────────────────────────────────────── */
namespace Reg {
    /* Input */
    constexpr uint8_t IN_TEMP      = 0u;
    constexpr uint8_t IN_PRESS     = 1u;
    constexpr uint8_t IN_FLOW      = 2u;
    constexpr uint8_t IN_STATUS    = 3u;
    constexpr uint8_t IN_UPTIME_HI = 4u;
    constexpr uint8_t IN_UPTIME_LO = 5u;
    constexpr uint8_t IN_COUNT     = 6u;

    /* Holding */
    constexpr uint8_t HR_SLAVE_ID  = 0u;
    constexpr uint8_t HR_TEMP_SP   = 1u;
    constexpr uint8_t HR_PRESS_ALM = 2u;
    constexpr uint8_t HR_FLOW_ALM  = 3u;
    constexpr uint8_t HR_SIM_MODE  = 4u;
    constexpr uint8_t HR_COUNT     = 5u;

    /* Status bits */
    constexpr uint16_t ST_OK        = 0x0000u;
    constexpr uint16_t ST_TEMP_ALM  = 0x0001u;
    constexpr uint16_t ST_PRESS_ALM = 0x0002u;
    constexpr uint16_t ST_FLOW_ALM  = 0x0004u;
    constexpr uint16_t ST_SIM       = 0x8000u;  /* always set: I am a simulator */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ModbusSlave
 * ═══════════════════════════════════════════════════════════════════════════*/
class ModbusSlave {
public:
    ModbusSlave();
    ModbusSlave(const ModbusSlave&)            = delete;
    ModbusSlave& operator=(const ModbusSlave&) = delete;

    /* Call once from app_main() */
    void begin();

private:
    std::array<uint16_t, Reg::IN_COUNT>  m_in{};
    std::array<uint16_t, Reg::HR_COUNT>  m_hr{};
    SemaphoreHandle_t                    m_mtx = nullptr;

    /* Simulation state */
    float    m_phase = 0.0f;
    float    m_ramp  = 0.0f;
    uint16_t m_lfsr  = 0xACE1u;

    /* ── CRC ─────────────────────────────────────────────────────────── */
    static uint16_t crc16(const uint8_t* d, uint16_t len);

    /* ── TX ──────────────────────────────────────────────────────────── */
    void send(const uint8_t* buf, size_t len);
    void sendEx(uint8_t fc, uint8_t code);

    /* ── Frame handlers ──────────────────────────────────────────────── */
    void onReadHolding(const uint8_t* f, int len);
    void onReadInput  (const uint8_t* f, int len);
    void onWriteSingle(const uint8_t* f, int len);
    void onWriteMulti (const uint8_t* f, int len);

    /* ── Simulation ──────────────────────────────────────────────────── */
    uint16_t nextLfsr();
    void     updateSensors();

    /* ── Tasks ───────────────────────────────────────────────────────── */
    static void mbTaskEntry (void* a) { static_cast<ModbusSlave*>(a)->mbTask();  }
    static void simTaskEntry(void* a) { static_cast<ModbusSlave*>(a)->simTask(); }
    void mbTask();
    void simTask();
};