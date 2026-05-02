#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include "esp_http_server.h"

namespace Takamul {
    class WebServer {
        public:
            static WebServer& getInstance();

            void init();
            void start();
            void stop();
        private:
            WebServer() = default;
            ~WebServer() = default;

            WebServer(const WebServer&) = delete;
            WebServer& operator=(const WebServer&) = delete;

            httpd_handle_t m_server = nullptr;

            static esp_err_t handleRoot(httpd_req_t *req);
            static esp_err_t handleWifiConnect(httpd_req_t *req);
            static esp_err_t handleStatus(httpd_req_t *req);
    };
}

#endif
