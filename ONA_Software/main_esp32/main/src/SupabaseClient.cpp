#include "inc/SupabaseClient.h"
#include "esp_log.h"
#include <cstring>
#include <cassert>

namespace Takamul {

static const char* Tag = "SupabaseClient";

// ─── Response body accumulator ────────────────────────────────────────────────

/**
 * We pass a pointer to a std::string as the user_data of the HTTP client.
 * The event handler appends every HTTP_EVENT_ON_DATA chunk into it.
 */
esp_err_t SupabaseClient::httpEventHandler(esp_http_client_event_t* evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (!esp_http_client_is_chunked_response(evt->client) && evt->user_data) {
            auto* body = static_cast<std::string*>(evt->user_data);
            body->append(static_cast<char*>(evt->data), evt->data_len);
        }
    }
    return ESP_OK;
}

// ─── Singleton ────────────────────────────────────────────────────────────────

SupabaseClient& SupabaseClient::getInstance() {
    static SupabaseClient instance;
    return instance;
}

// ─── Public API ───────────────────────────────────────────────────────────────

void SupabaseClient::init(const char* url, const char* anon_key) {
    assert(url && anon_key);
    m_url      = url;
    m_anon_key = anon_key;
    m_initialized = true;
    ESP_LOGI(Tag, "Initialized. Project: %s", url);
}

int SupabaseClient::insert(const char* table, const std::string& json, const char* prefer) {
    if (!m_initialized) { ESP_LOGE(Tag, "Not initialized"); return -1; }

    std::string url = m_url + "/rest/v1/" + table;
    std::string body_out;

    std::vector<std::pair<std::string,std::string>> headers = {
        {"Prefer", prefer ? prefer : "return=minimal"}
    };

    int status = httpRequest("POST", url, json, headers, body_out);
    if (status != 200 && status != 201) {
        ESP_LOGW(Tag, "INSERT %s -> HTTP %d  body=%s", table, status, body_out.c_str());
    }
    return status;
}

int SupabaseClient::select(const char* table,
                           const char* select_cols,
                           const char* filter,
                           std::string& out_body) {
    if (!m_initialized) { ESP_LOGE(Tag, "Not initialized"); return -1; }

    std::string url = m_url + "/rest/v1/" + table + "?select=" + select_cols;
    if (filter && filter[0] != '\0') {
        url += "&";
        url += filter;
    }

    int status = httpRequest("GET", url, "", {}, out_body);
    if (status != 200) {
        ESP_LOGW(Tag, "SELECT %s -> HTTP %d  body=%s", table, status, out_body.c_str());
    }
    return status;
}

int SupabaseClient::upsert(const char* table, const std::string& json, const char* on_conflict) {
    if (!m_initialized) { ESP_LOGE(Tag, "Not initialized"); return -1; }

    std::string url = m_url + "/rest/v1/" + table;
    std::string body_out;

    std::string prefer_val = "resolution=merge-duplicates,return=minimal";
    std::vector<std::pair<std::string,std::string>> headers = {
        {"Prefer", prefer_val}
    };
    if (on_conflict && on_conflict[0] != '\0') {
        url += "?on_conflict=" + std::string(on_conflict);
    }

    int status = httpRequest("POST", url, json, headers, body_out);
    if (status != 200 && status != 201) {
        ESP_LOGW(Tag, "UPSERT %s -> HTTP %d  body=%s", table, status, body_out.c_str());
    }
    return status;
}

// ─── Internal HTTP helper ─────────────────────────────────────────────────────

int SupabaseClient::httpRequest(const char* method,
                                const std::string& url,
                                const std::string& body,
                                const std::vector<std::pair<std::string,std::string>>& extra_headers,
                                std::string& out_body) {
    out_body.clear();

    esp_http_client_config_t cfg = {};
    cfg.url               = url.c_str();
    cfg.event_handler     = httpEventHandler;
    cfg.user_data         = &out_body;
    cfg.timeout_ms        = 15000;
    cfg.buffer_size       = 4096;
    cfg.buffer_size_tx    = 4096;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;

    if (strcmp(method, "POST") == 0)       cfg.method = HTTP_METHOD_POST;
    else if (strcmp(method, "PATCH") == 0) cfg.method = HTTP_METHOD_PATCH;
    else                                   cfg.method = HTTP_METHOD_GET;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(Tag, "Failed to init HTTP client");
        return -1;
    }

    // Standard Supabase headers
    std::string auth = "Bearer " + m_anon_key;
    esp_http_client_set_header(client, "Content-Type",  "application/json");
    esp_http_client_set_header(client, "apikey",         m_anon_key.c_str());
    esp_http_client_set_header(client, "Authorization",  auth.c_str());

    // Extra headers
    for (const auto& [k, v] : extra_headers) {
        esp_http_client_set_header(client, k.c_str(), v.c_str());
    }

    if (!body.empty()) {
        esp_http_client_set_post_field(client, body.c_str(), static_cast<int>(body.size()));
    }

    esp_err_t err = esp_http_client_perform(client);
    int status = -1;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
    } else {
        ESP_LOGE(Tag, "HTTP perform error: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return status;
}

} // namespace Takamul
