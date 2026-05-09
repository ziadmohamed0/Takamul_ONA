#ifndef MAIN_H_
#define MAIN_H_

#include "sdkconfig.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// ─── Custom drivers ───────────────────────────────────────────────────────────
#include "inc/NVSManager.h"
#include "inc/WifiManager.h"
#include "inc/WebServer.h"
#include "inc/SupabaseClient.h"
#include "inc/UartBridge.h"
#include "inc/TelemetryManager.h"
#include "inc/Sleepmanager.h"       

// ─── Supabase config ──────────────────────────────────────────────────────────
#ifndef CONFIG_TAKAMUL_SUPABASE_URL
#define CONFIG_TAKAMUL_SUPABASE_URL \
    "https://xfcicrtmyvpgirwvnqfh.supabase.co"
#endif

#ifndef CONFIG_TAKAMUL_SUPABASE_KEY
#define CONFIG_TAKAMUL_SUPABASE_KEY \
    "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9." \
    "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InhmY2ljcnRteXZwZ2lyd3ZucWZoIiwicm9sZSI6ImFub24iLCJpYXQiOjE3Nzc0MjIzNzYsImV4cCI6MjA5Mjk5ODM3Nn0." \
    "W6NkexIqULpwRa-3G4x_C6bAmaZXgRm7xXxRypibJ7A"
#endif

// ─── UART config (ESP32 <-> STM32) ─────────────────────────────────────────────
// From the schematic:
//   STM_TX (STM32 TX) -> ESP32 IO16 (RX)
//   STM_RX (STM32 RX) -> ESP32 IO17 (TX)
#define UART_STM32_PORT    UART_NUM_2
#define UART_STM32_RX_PIN  GPIO_NUM_16   // ESP32 RX <- STM32 TX
#define UART_STM32_TX_PIN  GPIO_NUM_17   // ESP32 TX -> STM32 RX
#define UART_STM32_BAUD    115200

// ─── Deep Sleep Config ────────────────────────────────────────────────────────
// Change these values according to your needs

// Normal sleep duration - 5 minutes
// To change it: change the 5 to the desired number of minutes
#define SLEEP_NORMAL_MINUTES    5

// Sleep duration when there is an anomaly - 60 seconds
#define SLEEP_ANOMALY_SECONDS   60

// Sleep duration when force_wakeup from website - 30 seconds
#define SLEEP_FORCE_SECONDS     30

// GPIO that STM32 pulls high when there is an alert
// From the schematic: IO38 on ESP32-S3
// If not connected -> set to -1
#define STM32_ALERT_GPIO_PIN    GPIO_NUM_38

// ─── Sensor Thresholds ────────────────────────────────────────────────────────
// These thresholds are checked every cycle
// If reading exceeds limit -> anomaly -> next sleep is shorter + immediate upload

#define THRESHOLD_TDS_MAX       1000.0f   // ppm  - above this -> polluted water
#define THRESHOLD_PRESSURE_MAX  6.0f      // bar  - above this -> high pressure
#define THRESHOLD_PRESSURE_MIN  0.5f      // bar  - below this -> low pressure / leak
#define THRESHOLD_FLOW_MIN      1.0f      // L/min - below this with pump on -> issue
#define THRESHOLD_TEMP_MAX      45.0f     // °C   - above this -> high temperature

// Sudden change is considered an anomaly
#define THRESHOLD_TDS_CHANGE_PCT  0.20f   // 20% change in TDS
#define THRESHOLD_PRESSURE_JUMP   1.0f    // bar  - sudden jump in pressure

#endif // MAIN_H_