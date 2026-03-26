#ifndef MAIN_H_
#define MAIN_H_

#include "sdkconfig.h"

// include custome drivers
#include "MqttManager.h"
#include "NVSManager.h"
#include "WifiManager.h"

// Wifi configuratian Defines User, Password.
#define ssid            "Mohamed Fathy"
#define pass            "341978341978"

// MQTT configuratian Defines Topic, Broker (see idf.py menuconfig → ONA MQTT).
#define broker          "mqtt://192.168.100.25:1883"
#define topic_tds       "/ONA/SENSOR/TDS"
#define topic_pump      "/ONA/ACTUATOR/PUMP"
#define topic_flow      "/ONA/SENSOR/FLOW"
#define topic_diff      "/ONA/SENSOR/DIFF"
#define topic_pressure  "/ONA/SENSOR/PRESSURE"

// Global variables 
Takamul::NVSManager*  NVS_OBJ;  // NVS Object
Takamul::WifiManager* WiFi_OBJ; // WiFi Object
Takamul::MqttManager* MQTT_OBJ; // MQTT Object

#endif