idf_component_register(
    SRCS
        "main.cpp"
        "src/NVSManager.cpp"
        "src/WifiManager.cpp"
        "src/WebServer.cpp"
        "src/MqttManager.cpp"
        "src/SupabaseClient.cpp"
        "src/UartBridge.cpp"
        "src/TelemetryManager.cpp"
        "src/Otamanager.cpp"           # OTA manager (ESP32 self + STM32 UART flash)
        "src/Sleepmanager.cpp"         # Deep sleep controller
    INCLUDE_DIRS
        "."
        "inc"
    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        esp_http_client
        esp_https_ota                  # esp_https_ota()
        app_update                     # esp_ota_ops.h
        esp_driver_gpio                # gpio_config() for wakeup pin
        mqtt
        driver
        freertos
        mbedtls
        lwip
        esp_pm                         # power management
)