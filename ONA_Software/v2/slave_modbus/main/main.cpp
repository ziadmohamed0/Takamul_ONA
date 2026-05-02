#include "modbus_slave.h"

static void slave_task(void* arg) {
    ESP_LOGI("SLAVE", "Slave task started");
    VfdSlave* slave = (VfdSlave*)arg;
    slave->vfd_logic_task();
}

extern "C" void app_main(void) {

    static VfdSlave vfd_slave(UART_PORT, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_18);    
    vfd_slave.init();
    if (xTaskCreate(slave_task, "vfd_task", 4096, &vfd_slave, 10, NULL) != pdPASS) {
        ESP_LOGE("SLAVE", "Failed to create slave task");
    }
}


