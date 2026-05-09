#ifndef SUPABASE_CLIENT_H_
#define SUPABASE_CLIENT_H_

#include "esp_http_client.h"
// esp_crt_bundle is provided by the mbedtls component in IDF v5.x
#include "esp_crt_bundle.h"
#include <string>
#include <functional>
#include <vector>

namespace Takamul {

    /**
     * @brief Lightweight Supabase REST API client (PostgREST over HTTPS).
     *
     * Usage:
     *   auto& sb = SupabaseClient::getInstance();
     *   sb.init(SUPABASE_URL, SUPABASE_ANON_KEY);
     *   sb.insert("telemetry", R"({"sensor_type":"TDS","value":290.5,"unit":"ppm","device_id":"AA:BB:CC"})");
     *
     * Thread safety: Each call creates/destroys its own HTTP client handle,
     * so concurrent calls from different tasks are safe.
     */
    class SupabaseClient {
    public:
        static SupabaseClient& getInstance();

        /**
         * @brief Configure the client. Call once after NVS init.
         * @param url      e.g. "https://xfcicrtmyvpgirwvnqfh.supabase.co"
         * @param anon_key JWT anon key from Supabase project settings.
         */
        void init(const char* url, const char* anon_key);

        /**
         * @brief INSERT a JSON row into a table.
         * @param table   Table name (e.g. "telemetry").
         * @param json    JSON object string.
         * @param prefer  PostgREST Prefer header value. "return=minimal" for no body back.
         * @return HTTP status code, or -1 on transport error.
         */
        int insert(const char* table, const std::string& json, const char* prefer = "return=minimal");

        /**
         * @brief SELECT rows from a table with optional filter and ordering.
         * @param table    Table name.
         * @param select   Columns to return (e.g. "*" or "pump_speed,status,target_pressure,updated_at").
         * @param filter   PostgREST filter query string appended to URL (e.g. "device_id=eq.AA:BB&order=updated_at.desc&limit=1").
         * @param out_body Receives the response JSON body.
         * @return HTTP status code, or -1 on transport error.
         */
        int select(const char* table,
                   const char* select_cols,
                   const char* filter,
                   std::string& out_body);

        /**
         * @brief UPSERT a row (INSERT ... ON CONFLICT DO UPDATE).
         * @param table   Table name.
         * @param json    JSON object string.
         * @param on_conflict Comma-separated column(s) forming the unique key (e.g. "device_id").
         * @return HTTP status code, or -1 on transport error.
         */
        int upsert(const char* table, const std::string& json, const char* on_conflict = nullptr);

    private:
        SupabaseClient() = default;
        ~SupabaseClient() = default;
        SupabaseClient(const SupabaseClient&) = delete;
        SupabaseClient& operator=(const SupabaseClient&) = delete;

        /**
         * @brief Internal HTTP request helper.
         * @param method   HTTP method string: "GET", "POST", "PATCH".
         * @param url      Full URL.
         * @param body     Request body (empty for GET).
         * @param extra_headers  Additional headers as key-value pairs.
         * @param out_body Receives response body.
         * @return HTTP status code, or -1.
         */
        int httpRequest(const char* method,
                        const std::string& url,
                        const std::string& body,
                        const std::vector<std::pair<std::string,std::string>>& extra_headers,
                        std::string& out_body);

        // Accumulate response body in the event handler
        static esp_err_t httpEventHandler(esp_http_client_event_t* evt);

        std::string m_url;
        std::string m_anon_key;
        bool        m_initialized = false;
    };

} // namespace Takamul

#endif // SUPABASE_CLIENT_H_