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

// ─── Supabase config (override via idf.py menuconfig → Takamul Config) ───────
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

// ─── UART config (ESP32 ↔ STM32) ─────────────────────────────────────────────
// Adjust GPIO pins to match your PCB layout
#define UART_STM32_PORT   UART_NUM_2
#define UART_STM32_RX_PIN GPIO_NUM_16   // ESP32 RX ← STM32 TX
#define UART_STM32_TX_PIN GPIO_NUM_17   // ESP32 TX → STM32 RX
#define UART_STM32_BAUD   115200

// ─── Global manager pointers (extern, defined in main.cpp) ────────────────────
extern Takamul::NVSManager*       NVS_OBJ;
extern Takamul::WifiManager*      WiFi_OBJ;
extern Takamul::SupabaseClient*   Supabase_OBJ;
extern Takamul::UartBridge*       Uart_OBJ;
extern Takamul::TelemetryManager* Telem_OBJ;

#endif // MAIN_H_
