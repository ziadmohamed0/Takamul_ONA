#ifndef MQTT_MANAGER_H_
#define MQTT_MANAGER_H_

#include "mqtt_client.h"
#include <string>
#include <vector>

namespace Takamul {
    class MqttManager {
        public:
            /**
             * @brief Get the singleton instance of MqttManager.
             * @return Reference to the MqttManager instance.
             */
            static MqttManager& getInstance();

            /**
             * @brief Start the MQTT client with the given broker URI and optional credentials.
             * @param broker_uri The URI of the MQTT broker (e.g., "mqtt://broker.example.com:1883").
             * @param username Optional username for authentication (nullptr if not used).
             * @param password Optional password for authentication (nullptr if not used).
             */
            void start(const char* broker_uri, const char* username = nullptr, const char* password = nullptr);

            /**
             * @brief Stop the MQTT client and clean up resources.
             */
            void stop();

            /**
             * @brief Publish a message to a specific MQTT topic.
             * @param topic The topic to publish to.
             * @param data The message data as a string.
             */
            void publish(const char* topic, const std::string& data);

            /**
             * @brief Subscribe to a specific MQTT topic with a given QoS level.
             * @param topic The topic to subscribe to.
             * @param qos The Quality of Service level (default 0).
             * @return The message ID of the subscribe request, or -1 if failed.
             */
            int subscribe(const char* topic, int qos = 0);
        private:
            /**
             * @brief Constructor for MqttManager (private for singleton).
             */
            MqttManager();

            /**
             * @brief Destructor for MqttManager.
             */
            ~MqttManager();

            // Delete copy/move
            MqttManager(const MqttManager&) = default;
            MqttManager &operator=(const MqttManager&) = default;

            /**
             * @brief Static event handler for MQTT events.
             * @param arg Pointer to the MqttManager instance.
             * @param event_base The event base.
             * @param event_id The event ID.
             * @param event_data Pointer to event data.
             */
            static void eventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
            esp_mqtt_client_handle_t m_client = nullptr;
            std::string m_broker_uri_storage;
    };
}

#endif