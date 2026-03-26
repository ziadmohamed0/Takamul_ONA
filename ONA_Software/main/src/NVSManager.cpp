#include "inc/NVSManager.h"
#include "esp_log.h"
#include <cstring>

namespace Takamul{
    static const char *Tag = "NVSManager"; 
}

Takamul::NVSHandle::NVSHandle(const char *namespace_name, nvs_open_mode_t mode) {
    esp_err_t err = nvs_open(namespace_name, mode, &this->m_handle);
    if(err != ESP_OK) {
        ESP_LOGI(Tag, "Error (%s) opening NVS Handle (%s)", esp_err_to_name(err), namespace_name);
        this->m_handle = 0;
    }
}

Takamul::NVSHandle::~NVSHandle() {
    if(this->m_handle != 0){
        nvs_close(this->m_handle);
        this->m_handle = 0;
    }
}

Takamul::NVSHandle::NVSHandle(NVSHandle&& other) noexcept : m_handle(other.m_handle) {
    other.m_handle = 0;
}

Takamul::NVSHandle& Takamul::NVSHandle::operator=(NVSHandle&& other) noexcept {
    if(this != &other) {
        if(m_handle != 0) {
            nvs_close(m_handle);
        }
        m_handle = other.m_handle;
        other.m_handle = 0;
    }
    return *this;
}  


esp_err_t Takamul::NVSHandle::setString(const char *key, const std::string &value) {
    if(this->m_handle == 0) {
        return ESP_FAIL;
    }

    else {
        return nvs_set_str(this->m_handle, key, value.c_str());
    }
}

esp_err_t Takamul::NVSHandle::getString(const char *key, std::string &value) {
    if(this->m_handle == 0) {
        return ESP_FAIL;
    }
    size_t requird_size;
    esp_err_t err = nvs_get_str(this->m_handle, key, NULL, &requird_size);
    if(err != ESP_OK) return err;

    std::vector<char> buf(requird_size);
    err = nvs_get_str(this->m_handle, key, buf.data(), &requird_size);
    if(err == ESP_OK) {
        value.assign(buf.data());
    }
    return err;
}

esp_err_t Takamul::NVSHandle::commit() {
    if(this->m_handle == 0) {
        return ESP_FAIL;
    }

    else {
        return nvs_commit(this->m_handle);
    }
}

Takamul::NVSManager& Takamul::NVSManager::getInstance() {
    static NVSManager instance;
    return instance;
}

void Takamul::NVSManager::init() {
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(Tag, "NVS Flash Initialized");
}
