#include "inc/WifiManager.h"
#include "esp_log.h"
#include <cstring>

namespace Takamul {
    static const char *Tag = "WifiManager"; 

    WifiManager& WifiManager::getInstance() {
        static WifiManager instance;
        return instance;
    }
    void WifiManager::init() {
        if (m_ip_ready == nullptr) {
            m_ip_ready = xEventGroupCreate();
        }
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &WifiManager::eventHandler,
            this,
            nullptr
        ));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &WifiManager::eventHandler,
            this,
            nullptr
        ));
    }

    void WifiManager::startAP() {
        m_netif_ap = esp_netif_create_default_wifi_ap();
        wifi_config_t cfg = {};
        strcpy((char*)cfg.ap.ssid, "Takamul_Prov");
        strcpy((char*)cfg.ap.password, "Takamul_2026");
        cfg.ap.ssid_len = strlen("Takamul_Prov");
        cfg.ap.channel = 1;
        cfg.ap.max_connection = 4;
        cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(Tag, "AP Started. SSID: Takamul_Prov");
    }

    void WifiManager::startSTA(const std::string &ssid, const std::string& password){
        if(m_netif_sta == nullptr) {
            m_netif_sta = esp_netif_create_default_wifi_sta();
        }
        wifi_config_t cfg = {};
        strncpy((char*)cfg.sta.ssid, ssid.c_str(), sizeof(cfg.sta.ssid));
        strncpy((char*)cfg.sta.password, password.c_str(), sizeof(cfg.sta.password));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(Tag, "Connecting to SSID: %s", ssid.c_str());
    }

    bool WifiManager::waitForStaIp(TickType_t timeout_ticks) {
        if (m_ip_ready == nullptr) {
            return false;
        }
        EventBits_t bits = xEventGroupWaitBits(
            m_ip_ready, kIpReadyBit, pdFALSE, pdTRUE, timeout_ticks);
        return (bits & kIpReadyBit) != 0;
    }

    void WifiManager::stop() {
        esp_wifi_stop();
        if(m_netif_ap) {
            esp_netif_destroy(m_netif_ap);
            m_netif_ap = nullptr;
        }
        if(m_netif_sta) {
            esp_netif_destroy(m_netif_sta);
            m_netif_sta = nullptr;
        }
    }

    void WifiManager::eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        WifiManager* self = static_cast<WifiManager*>(arg);

        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        }

        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (self->m_ip_ready) {
                xEventGroupClearBits(self->m_ip_ready, WifiManager::kIpReadyBit);
            }
            ESP_LOGI(Tag, "Disconnected from AP, retrying...");
            esp_wifi_connect();
        }

        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(Tag, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            if (self->m_ip_ready) {
                xEventGroupSetBits(self->m_ip_ready, WifiManager::kIpReadyBit);
            }
        }
    }
    
    
}