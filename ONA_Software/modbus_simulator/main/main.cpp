/**
 * @file  main.cpp
 * @brief ESP32 Modbus RTU Slave Simulator — entry point
 */

#include "modbus_slave.hpp"

extern "C" void app_main()
{
    static ModbusSlave slave;
    slave.begin();
    /* app_main() can return — FreeRTOS keeps tasks alive */
}