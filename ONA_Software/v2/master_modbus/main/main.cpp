
#include "modbus_master.h"

extern "C" void app_main(void) {
    static VfdMaster gateway;
    gateway.init();

    while (1) {
        // 1. Run the VFD
        gateway.set_run_stop(true);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 2. Set frequency to 60 Hz
        gateway.set_frequency(60);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 3. Read current
        uint16_t current;
        gateway.read_current(&current);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 4. Read voltage
        uint16_t voltage;
        gateway.read_voltage(&voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 5. Stop the VFD
        gateway.set_run_stop(false);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
