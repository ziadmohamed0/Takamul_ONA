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
//   STM_TX (STM32 TX) -> ESP32 IO18 (RX)
//   STM_RX (STM32 RX) -> ESP32 IO17 (TX)
#define UART_STM32_PORT    UART_NUM_2
#define UART_STM32_RX_PIN  GPIO_NUM_18   // ESP32 RX <- STM32 TX
#define UART_STM32_TX_PIN  GPIO_NUM_17   // ESP32 TX -> STM32 RX
#define UART_STM32_BAUD    115200

// ─── Deep Sleep Config ────────────────────────────────────────────────────────

// Normal sleep duration - 5 minutes
#define SLEEP_NORMAL_MINUTES    5

// Sleep duration when there is an anomaly - 60 seconds
#define SLEEP_ANOMALY_SECONDS   60

// Sleep duration when force_wakeup from website - 30 seconds
#define SLEEP_FORCE_SECONDS     30

// FIX: Changed from GPIO_NUM_38 to GPIO_NUM_21.
// ESP32-S3 EXT1 wakeup only works with RTC GPIOs (IO0–IO21).
// IO38 is NOT an RTC GPIO — log was showing: "E sleep: Not an RTC IO: GPIO38"
// Reconnect the STM32 alert wire from IO38 to IO21 on the PCB.
#define STM32_ALERT_GPIO_PIN    GPIO_NUM_21

// ─── Sensor Thresholds ────────────────────────────────────────────────────────

#define THRESHOLD_TDS_MAX       1000.0f   // ppm  - above this -> polluted water
#define THRESHOLD_PRESSURE_MAX  6.0f      // bar  - above this -> high pressure
#define THRESHOLD_PRESSURE_MIN  0.5f      // bar  - below this -> low pressure / leak
#define THRESHOLD_FLOW_MIN      1.0f      // L/min - below this with pump on -> issue
#define THRESHOLD_TEMP_MAX      45.0f     // °C   - above this -> high temperature

// Sudden change is considered an anomaly
#define THRESHOLD_TDS_CHANGE_PCT  0.20f   // 20% change in TDS
#define THRESHOLD_PRESSURE_JUMP   1.0f    // bar  - sudden jump in pressure

// ─── Stack Size ───────────────────────────────────────────────────────────────
// FIX: The main task stack was overflowing (8KB needed for WiFi + HTTP + strings).
// Set CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192 in sdkconfig or menuconfig:
//   idf.py menuconfig -> Component config -> ESP System Settings
//                     -> Main task stack size -> 8192
// The default (3584) is too small for this application.

#endif // MAIN_H_
