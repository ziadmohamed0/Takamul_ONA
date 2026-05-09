#ifndef WIFI_MANAGER_H_
#define WIFI_MANAGER_H_

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <vector>
#include <string>

namespace Takamul {
    class WifiManager {
        public:
            /**
             * @brief Get the singleton instance of WifiManager.
             * @return Reference to the WifiManager instance.
             */
            static WifiManager& getInstance();

            /**
             * @brief Initialize the WiFi stack and event handlers.
             */
            void init();

            /**
             * @brief Start the WiFi in Station mode and connect to the specified network.
             * @param ssid The SSID of the WiFi network.
             * @param password The password for the WiFi network.
             */
            void startSTA(const std::string &ssid, const std::string& password);

            /**
             * @brief Block until STA has an IPv4 address or timeout (use before MQTT).
             */
            bool waitForStaIp(TickType_t timeout_ticks);

            /**
             * @brief Start the WiFi in Access Point mode.
             */
            void startAP();

            /**
             * @brief Stop the WiFi and clean up resources.
             */
            void stop();
        private:
            /**
             * @brief Default constructor for WifiManager (private for singleton).
             */
            WifiManager() = default;

            /**
             * @brief Destructor for WifiManager.
             */
            ~WifiManager() = default;
        
            // Delete copy/move
            WifiManager(const WifiManager&) = default;
            WifiManager &operator=(const WifiManager&) = default;

            /**
             * @brief Static event handler for WiFi events.
             * @param arg Pointer to the WifiManager instance.
             * @param event_base The event base.
             * @param event_id The event ID.
             * @param event_data Pointer to event data.
             */
            static void eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

            static constexpr EventBits_t kIpReadyBit = static_cast<EventBits_t>(1u << 0);
            EventGroupHandle_t m_ip_ready = nullptr;
            esp_netif_t* m_netif_ap = nullptr;
            esp_netif_t* m_netif_sta = nullptr;
    };
}

#endif