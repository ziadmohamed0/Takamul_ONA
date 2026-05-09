#include "inc/MqttManager.h"
#include "esp_log.h"
#include <cstring>

namespace Takamul {
    static const char* Tag = "MqttManager";

    MqttManager& MqttManager::getInstance() {
        static MqttManager instance;
        return instance;
    }

    MqttManager::MqttManager() {}
    MqttManager::~MqttManager() {
        stop();
    }
    
    void MqttManager::stop() {
        if(m_client) {
            esp_mqtt_client_stop(m_client);
            esp_mqtt_client_destroy(m_client);
            m_client= nullptr;
        }
    }

    void MqttManager::start(const char* broker_uri, const char* username, const char* password) {
        if (m_client != nullptr) return;
        esp_mqtt_client_config_t cfg = {};
        memset(&cfg, 0, sizeof(cfg)); 

        if (username && password) {
            std::string uri(broker_uri);
            size_t pos = uri.find("://");
            if (pos != std::string::npos) {
                pos += 3;
                std::string creds = std::string(username) + ":" + std::string(password) + "@";
                uri.insert(pos, creds);
            }
            m_broker_uri_storage = std::move(uri);
            cfg.broker.address.uri = m_broker_uri_storage.c_str();
        } else {
            cfg.broker.address.uri = broker_uri;
        }

        m_client = esp_mqtt_client_init(&cfg);
        if (m_client) {
            esp_mqtt_client_register_event(m_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, eventHandler, this);
            esp_mqtt_client_start(m_client);
        } else {
            ESP_LOGE(Tag, "Failed to init MQTT Client");
        }
    }
    void MqttManager::publish(const char* topic, const std::string& data) {
        if(m_client) {
            esp_mqtt_client_publish(m_client, topic, data.c_str(), 0, 1, 0);
        }
    }

    int MqttManager::subscribe(const char* topic, int qos) {
        if(!m_client) return -1;
        return esp_mqtt_client_subscribe(m_client, topic, qos);
    }

    void MqttManager::eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        (void)arg;
        esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
        switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(Tag, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(Tag, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(Tag, "MQTT_EVENT_ERROR");
            if (event->error_handle && event->error_handle->esp_transport_sock_errno != 0) {
                ESP_LOGE(Tag, "Socket errno %d: %s", event->error_handle->esp_transport_sock_errno,
                    strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        case MQTT_EVENT_DATA: {
            // Single-chunk message: total_data_len is 0 or equals data_len
            int data_len = event->data_len;
            int total = event->total_data_len;
            bool single_chunk = (total == 0) || (total == data_len);
            if (event->topic != nullptr && single_chunk) {
                ESP_LOGI(Tag, "MQTT data received on topic: %s, data: %.*s", event->topic, data_len, event->data);
            }
            break;
        }
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(Tag, "MQTT_EVENT_PUBLISHED, message sent successfully");
            break;
        default:
            break;
        
        }
    }
    
}