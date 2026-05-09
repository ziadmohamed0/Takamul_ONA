#ifndef NVS_MANAGER_H_
#define NVS_MANAGER_H_

#include "nvs_flash.h"
#include "nvs.h"
#include <string>
#include <memory>
#include <vector>

namespace Takamul {
    class NVSHandle {
        public:
            /**
             * @brief Constructor for NVSHandle, opens an NVS namespace.
             * @param namespace_name The name of the NVS namespace.
             * @param mode The open mode (e.g., NVS_READWRITE).
             */
            NVSHandle(const char *namespace_name, nvs_open_mode_t mode);

            /**
             * @brief Destructor for NVSHandle, closes the handle.
             */
            ~NVSHandle();

            // Delete copy constructor and assignment operator to prevent double-free
            NVSHandle(const NVSHandle&) = delete;
            NVSHandle& operator=(const NVSHandle&) = delete;
            
            /**
             * @brief Move constructor for NVSHandle.
             * @param other The other NVSHandle to move from.
             */
            NVSHandle(NVSHandle&& other) noexcept;

            /**
             * @brief Move assignment operator for NVSHandle.
             * @param other The other NVSHandle to move from.
             * @return Reference to this NVSHandle.
             */
            NVSHandle& operator=(NVSHandle&& other) noexcept;   
            
            /**
             * @brief Set a string value in NVS.
             * @param key The key for the value.
             * @param value The string value to set.
             * @return ESP_OK on success, or an error code.
             */
            esp_err_t setString(const char *key, const std::string &value);

            /**
             * @brief Get a string value from NVS.
             * @param key The key for the value.
             * @param value Reference to store the retrieved string.
             * @return ESP_OK on success, or an error code.
             */
            esp_err_t getString(const char *key, std::string &value);

            /**
             * @brief Commit changes to NVS.
             * @return ESP_OK on success, or an error code.
             */
            esp_err_t commit();

            /**
             * @brief Check if the handle is valid.
             * @return True if valid, false otherwise.
             */
            bool isValid() const { return m_handle != 0; }
        private:
            nvs_handle_t m_handle; 
    };

    class NVSManager {
        public:
            /**
             * @brief Get the singleton instance of NVSManager.
             * @return Reference to the NVSManager instance.
             */
            static NVSManager& getInstance();

            /**
             * @brief Initialize the NVS flash.
             */
            void init();
        private:
            /**
             * @brief Default constructor for NVSManager (private for singleton).
             */
            NVSManager() = default; 
    };
}

#endif
