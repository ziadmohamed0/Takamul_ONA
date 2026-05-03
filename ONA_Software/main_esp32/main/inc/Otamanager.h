#ifndef OTA_MANAGER_H_
#define OTA_MANAGER_H_

#include <string>

namespace Takamul {

/**
 * @brief Handles OTA firmware updates for ESP32 and STM32.
 *
 * ESP32 OTA  → uses esp_https_ota() to download + flash directly.
 * STM32 OTA  → downloads .bin into PSRAM/heap, then programs STM32
 *              over UART using the built-in STM32 bootloader protocol.
 *
 * Both are triggered by writing a URL into the Supabase `controls` table:
 *   ota_esp32_url  → triggers ESP32 self-update  (then esp_restart())
 *   ota_stm32_url  → triggers STM32 programming  (no ESP32 reboot)
 *
 * After a successful STM32 flash the OTA URL is cleared in Supabase
 * so the command is not re-executed on the next poll.
 *
 * Usage (called from TelemetryManager::pollControls):
 *   OtaManager::getInstance().checkAndRun(esp32_url, stm32_url, device_id);
 */
class OtaManager {
public:
    static OtaManager& getInstance();

    /**
     * @brief Check both OTA fields and run whichever is set.
     *        Call from pollControls() every poll cycle.
     *        Internally throttled — won't re-trigger if already running.
     *
     * @param esp32_url   Value of ota_esp32_url column (empty = no update)
     * @param stm32_url   Value of ota_stm32_url column (empty = no update)
     * @param device_id   MAC string used to clear the Supabase field after flash
     */
    void checkAndRun(const std::string& esp32_url,
                     const std::string& stm32_url,
                     const std::string& device_id);

private:
    OtaManager()  = default;
    ~OtaManager() = default;
    OtaManager(const OtaManager&)            = delete;
    OtaManager& operator=(const OtaManager&) = delete;

    // ── ESP32 self-OTA ───────────────────────────────────────────────────────
    static void esp32OtaTask(void* arg);        // FreeRTOS task wrapper
    void        runEsp32Ota(const std::string& url);

    // ── STM32 UART bootloader ────────────────────────────────────────────────
    static void stm32OtaTask(void* arg);        // FreeRTOS task wrapper
    void        runStm32Ota(const std::string& url, const std::string& device_id);

    // Download a URL into a heap buffer; caller must free().
    // Returns byte count, 0 on error. max_bytes = guard against huge files.
    static size_t downloadToHeap(const std::string& url,
                                 uint8_t*& out_buf,
                                 size_t    max_bytes = 1024 * 1024); // 1 MB guard

    // STM32 UART bootloader helpers (AN3155 protocol)
    bool stm32Sync();
    bool stm32GetAck();
    bool stm32SendByte(uint8_t b);
    bool stm32EraseAll();
    bool stm32WriteChunk(uint32_t addr, const uint8_t* data, uint8_t len);
    bool stm32FlashBinary(const uint8_t* bin, size_t size);
    void stm32EnterBootloader();
    void stm32ExitBootloader();

    // Clear OTA URL in Supabase so the command is not repeated
    void clearOtaField(const std::string& field, const std::string& device_id);

    bool m_busy = false;    // prevent concurrent OTA attempts

    // Context passed to FreeRTOS OTA tasks (heap-allocated, task frees)
    struct OtaCtx {
        std::string url;
        std::string device_id;
    };
};

} // namespace Takamul

#endif // OTA_MANAGER_H_