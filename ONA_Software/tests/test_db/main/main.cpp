/**
 * ============================================================
 *  Takamul SCADA — ESP32 Fake Telemetry Sender
 *  Target : ESP32 + ESP-IDF v5.x  (C++)
 *  Sends  : TDS, TEMPERATURE, FLOW, PRESSURE, DIFF_PRESSURE
 *  device_id = MAC address  (e.g. "AA:BB:CC:DD:EE:FF")
 * ============================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_crt_bundle.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_mac.h"

#include "nvs_flash.h"

// ─── User config ──────────────────────────────────────────────
#define WIFI_SSID          "Mohamed Fathy"
#define WIFI_PASS          "341978341978"
#define SUPABASE_URL       "https://xfcicrtmyvpgirwvnqfh.supabase.co"
#define SUPABASE_ANON_KEY  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InhmY2ljcnRteXZwZ2lyd3ZucWZoIiwicm9sZSI6ImFub24iLCJpYXQiOjE3Nzc0MjIzNzYsImV4cCI6MjA5Mjk5ODM3Nn0.W6NkexIqULpwRa-3G4x_C6bAmaZXgRm7xXxRypibJ7A"

#define SEND_INTERVAL_MS   5000
#define WIFI_MAX_RETRY     10
// ─────────────────────────────────────────────────────────────

static const char *TAG = "TAKAMUL";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// MAC address stored as string — filled in app_main before tasks start
static char g_device_id[18];  // "AA:BB:CC:DD:EE:FF\0"

// ─── Sensor state ─────────────────────────────────────────────
typedef struct {
    float tds;
    float temperature;
    float flow;
    float pressure;
    float diff_pressure;
} SensorState;

static SensorState g_state;

static void init_sensor_state(void)
{
    g_state.tds           = 285.0f;
    g_state.temperature   = 22.4f;
    g_state.flow          = 42.1f;
    g_state.pressure      = 3.8f;
    g_state.diff_pressure = 0.42f;
}

static float frand_range(float lo, float hi)
{
    float t = (float)(esp_random() % 10001) / 10000.0f;
    return lo + t * (hi - lo);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void update_sensor_state(void)
{
    g_state.tds           = clampf(g_state.tds           + frand_range(-4.0f,   4.0f),   100.0f, 900.0f);
    g_state.temperature   = clampf(g_state.temperature   + frand_range(-0.3f,   0.3f),     5.0f,  45.0f);
    g_state.flow          = clampf(g_state.flow           + frand_range(-2.0f,   2.0f),     5.0f, 120.0f);
    g_state.pressure      = clampf(g_state.pressure       + frand_range(-0.15f,  0.15f),    0.5f,   8.0f);
    g_state.diff_pressure = clampf(g_state.diff_pressure  + frand_range(-0.02f,  0.02f),   0.05f,   1.2f);
}

// ─── HTTP ─────────────────────────────────────────────────────
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (!esp_http_client_is_chunked_response(evt->client)) {
            ESP_LOGD(TAG, "Response: %.*s", evt->data_len, (char *)evt->data);
        }
    }
    return ESP_OK;
}

typedef struct {
    const char *sensor_type;
    float       value;
    const char *unit;
} TelemetryRow;

static esp_err_t post_telemetry(const TelemetryRow *row)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/rest/v1/telemetry", SUPABASE_URL);

    // Include device_id (MAC address) in every row
    char body[320];
    snprintf(body, sizeof(body),
        "{\"device_id\":\"%s\",\"sensor_type\":\"%s\",\"value\":%.4f,\"unit\":\"%s\"}",
        g_device_id, row->sensor_type, row->value, row->unit);

    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));
    config.url               = url;
    config.event_handler     = http_event_handler;
    config.method            = HTTP_METHOD_POST;
    config.timeout_ms        = 10000;
    config.buffer_size       = 2048;
    config.buffer_size_tx    = 2048;
    config.crt_bundle_attach = esp_crt_bundle_attach;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "HTTP client init failed");
        return ESP_FAIL;
    }

    char auth_header[512];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", SUPABASE_ANON_KEY);

    esp_http_client_set_header(client, "Content-Type",  "application/json");
    esp_http_client_set_header(client, "apikey",         SUPABASE_ANON_KEY);
    esp_http_client_set_header(client, "Authorization",  auth_header);
    esp_http_client_set_header(client, "Prefer",         "return=minimal");
    esp_http_client_set_post_field(client, body, (int)strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status == 201) {
            ESP_LOGI(TAG, "OK  %-14s = %.4f %s  [%s]",
                     row->sensor_type, row->value, row->unit, g_device_id);
        } else {
            ESP_LOGW(TAG, "ERR %-14s  HTTP %d  body=%s",
                     row->sensor_type, status, body);
        }
    } else {
        ESP_LOGE(TAG, "ERR %-14s  %s", row->sensor_type, esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

static void send_all_telemetry(void)
{
    update_sensor_state();

    TelemetryRow rows[5];
    rows[0].sensor_type = "TDS";           rows[0].value = g_state.tds;           rows[0].unit = "ppm";
    rows[1].sensor_type = "TEMPERATURE";   rows[1].value = g_state.temperature;   rows[1].unit = "C";
    rows[2].sensor_type = "FLOW";          rows[2].value = g_state.flow;          rows[2].unit = "L/min";
    rows[3].sensor_type = "PRESSURE";      rows[3].value = g_state.pressure;      rows[3].unit = "bar";
    rows[4].sensor_type = "DIFF_PRESSURE"; rows[4].value = g_state.diff_pressure; rows[4].unit = "bar";

    ESP_LOGI(TAG, "---- Sending batch | device: %s ----", g_device_id);
    for (int i = 0; i < 5; i++) {
        post_telemetry(&rows[i]);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGI(TAG, "--------------------------------------");
}

// ─── WiFi ─────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "WiFi retry %d/%d", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,     &wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,   IP_EVENT_STA_GOT_IP,  &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_cfg;
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));
    strncpy((char *)wifi_cfg.sta.ssid,     WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASS,  sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);

    bool connected = (bits & WIFI_CONNECTED_BIT) != 0;

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        IP_EVENT,   IP_EVENT_STA_GOT_IP, inst_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID,    inst_any));
    vEventGroupDelete(s_wifi_event_group);
    return connected;
}

// ─── Telemetry task ───────────────────────────────────────────
static void telemetry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Telemetry task started — interval %d ms", SEND_INTERVAL_MS);
    while (true) {
        send_all_telemetry();
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

// ─── Entry point ──────────────────────────────────────────────
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Takamul SCADA - ESP32 Telemetry Node  ");
    ESP_LOGI(TAG, "========================================");

    // Read MAC address and format as "AA:BB:CC:DD:EE:FF"
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(g_device_id, sizeof(g_device_id),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device ID (MAC): %s", g_device_id);

    init_sensor_state();

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Connecting to WiFi: %s", WIFI_SSID);
    if (!wifi_init_sta()) {
        ESP_LOGE(TAG, "WiFi failed — halting.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "WiFi OK");

    xTaskCreate(telemetry_task, "telemetry", 8192, NULL, 5, NULL);
}
