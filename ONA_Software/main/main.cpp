#include "main.h"
#include "esp_log.h"
#include <time.h>

// Topic for receiving commands from Node-RED
const char* topic_pump_sub = "/ONA/ACTUATOR/PUMP";

extern "C" void app_main(void) {
    // Definition objects
    NVS_OBJ  = &Takamul::NVSManager::getInstance(); 
    WiFi_OBJ = &Takamul::WifiManager::getInstance();
    MQTT_OBJ = &Takamul::MqttManager::getInstance();

    // Initialization modules
    NVS_OBJ->init();
    WiFi_OBJ->init();

    // Start station mode
    WiFi_OBJ->startSTA(ssid, pass);
    if (!WiFi_OBJ->waitForStaIp(pdMS_TO_TICKS(10000))) {
        ESP_LOGE("main", "WiFi Failed; Resetting...");
        esp_restart();
    }

    // Start MQTT and Subscribe to Pump Control
    MQTT_OBJ->start(broker);
    MQTT_OBJ->subscribe(topic_pump_sub); 

    // Seed random for dummy data
    srand(time(NULL));

    float dummy_tds = 450.0;
    float dummy_flow = 15.5;
    float dummy_pressure = 3.2;

    while(true) {   
        // --- CASE 1: Normal Operation ---
        dummy_tds = 400.0 + (rand() % 100); 
        dummy_flow = 10.0 + (rand() % 5);
        dummy_pressure = 2.0 + (rand() % 2);

        // --- CASE 2: Simulate High TDS Alert (Every 5th cycle) ---
        static int cycle_count = 0;
        if (++cycle_count % 5 == 0) {
            dummy_tds = 1600.0; // Trigger "ALERT: TDS exceeded 1500"
            ESP_LOGW("main", "Triggering ALERT Case: High TDS");
        }

        // --- CASE 3: Simulate Low Pressure Request (Every 8th cycle) ---
        if (cycle_count % 8 == 0) {
            dummy_pressure = 0.2; // Trigger "Irrigation Request" Question
            dummy_flow = 0.05;
            ESP_LOGW("main", "Triggering QUESTION Case: Low Pressure");
        }

        char str_tds[10], str_flow[10], str_pressure[10];
        sprintf(str_tds, "%.2f", dummy_tds);
        sprintf(str_flow, "%.2f", dummy_flow);
        sprintf(str_pressure, "%.2f", dummy_pressure);

        // Publish to Node-RED
        MQTT_OBJ->publish(topic_tds, str_tds);
        MQTT_OBJ->publish(topic_flow, str_flow);
        MQTT_OBJ->publish(topic_pressure, str_pressure);

        ESP_LOGI("main", "Data Sent - TDS:%s, Flow:%s, Pres:%s", str_tds, str_flow, str_pressure);

        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 10 seconds per cycle
    }
}