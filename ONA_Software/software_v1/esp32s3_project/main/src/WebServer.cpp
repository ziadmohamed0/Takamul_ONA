#include "inc/WebServer.h"
#include "inc/WifiManager.h"
#include "inc/NVSManager.h"
#include "esp_log.h"
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <strings.h>

namespace Takamul {
    static const char *Tag = "WebServer";

        static const char *HTML_PAGE = R"rawliteral(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; }
.container { max-width: 400px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
input[type=text], input[type=password] { width: 100%; padding: 12px; margin: 8px 0; display: inline-block; border: 1px solid #ccc; box-sizing: border-box; }
button { background-color: #4CAF50; color: white; padding: 14px 20px; margin: 8px 0; border: none; cursor: pointer; width: 100%; }
button:hover { opacity: 0.8; }
h2 { text-align: center; color: #333; }
</style>
</head>
<body>
<div class="container">
    <h2>Takamul Provisioning</h2>
    <form action="/wifi" method="post">
        <label for="ssid"><b>SSID</b></label>
        <input type="text" placeholder="Enter WiFi Name" name="ssid" required>

        <label for="psw"><b>Password</b></label>
        <input type="password" placeholder="Enter Password" name="password" required>

        <button type="submit">Connect</button>
    </form>
</div>
</body>
</html>)rawliteral";

    WebServer& WebServer::getInstance() {
        static WebServer instance;
        return instance;
    }

    esp_err_t WebServer::handleRoot(httpd_req_t *req) {
        ESP_LOGI(Tag, "GET / request received");
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        return httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
    }

    static void url_decode_inplace(char *src) {
        char *dst = src;
        while (*src) {
            if (*src == '+') { *dst++ = ' '; src++; }
            else if (*src == '%' && isxdigit((unsigned char)src[1]) && isxdigit((unsigned char)src[2])) {
                char hex[3] = { src[1], src[2], '\0' };
                *dst++ = (char) strtol(hex, nullptr, 16);
                src += 3;
            } else { *dst++ = *src++; }
        }
        *dst = '\0';
    }

    esp_err_t WebServer::handleWifiConnect(httpd_req_t *req) {
        const int MAX_POST = 2048; // accept up to 2KB form data
        int total_len = req->content_len;
        if (total_len <= 0 || total_len > MAX_POST) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        char *buf = (char*)calloc(1, total_len + 1);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        int ret = httpd_req_recv(req, buf, total_len);
        if (ret <= 0) {
            free(buf);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        buf[ret] = '\0';

        // parse urlencoded form: ssid=...&password=...
        char ssid[64] = {0};
        char password[128] = {0};
        char *pair = strtok(buf, "&");
        while (pair) {
            char *eq = strchr(pair, '=');
            if (eq) {
                *eq = '\0';
                char *k = pair;
                char *v = eq + 1;
                url_decode_inplace(v);
                if (strcmp(k, "ssid") == 0) strncpy(ssid, v, sizeof(ssid)-1);
                else if (strcmp(k, "password") == 0) strncpy(password, v, sizeof(password)-1);
            }
            pair = strtok(nullptr, "&");
        }

        ESP_LOGI(Tag, "Received credentials SSID='%s' password_len=%d", ssid, (int)strlen(password));

        NVSHandle nvs("wifi_config", NVS_READWRITE);
        if (nvs.isValid()) {
            nvs.setString("ssid", std::string(ssid));
            nvs.setString("password", std::string(password));
            nvs.commit();
            ESP_LOGI(Tag, "Credentials saved to NVS");
        } else {
            ESP_LOGE(Tag, "Failed to open NVS handle");
        }

        const char *resp = "<html><body><h1>Saved. Connecting...</h1></body></html>";
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_send(req, resp, strlen(resp));

        if (strlen(ssid) > 0) {
            WifiManager::getInstance().startSTA(std::string(ssid), std::string(password));
        }

        free(buf);
        return ESP_OK;
    }

    esp_err_t WebServer::handleStatus(httpd_req_t *req) {
        const char *resp = "{\"status\":\"ok\"}";
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, resp, strlen(resp));
    }

    // Simple socket-based HTTP server to avoid httpd header limits.
    static volatile bool s_stop = false;
    static volatile bool s_dns_stop = false;

    static void dns_task(void *arg) {
        ESP_LOGI(Tag, "Starting DNS server on port 53");
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(Tag, "Failed to create DNS socket");
            vTaskDelete(nullptr);
            return;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(53);

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            ESP_LOGE(Tag, "Failed to bind DNS socket");
            close(sock);
            vTaskDelete(nullptr);
            return;
        }

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        uint8_t buf[512];
        while (!s_dns_stop) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &client_len);
            if (len >= 12) {
                // DNS Header flag: Response, no error
                buf[2] = 0x81; 
                buf[3] = 0x80; 
                buf[6] = 0x00; // ANCOUNT
                buf[7] = 0x01; 
                buf[8] = 0x00; // NSCOUNT
                buf[9] = 0x00;
                buf[10] = 0x00; // ARCOUNT
                buf[11] = 0x00;

                // Answer Record (Pointer to Name 0xC00C, Type A (1), Class IN (1), TTL (60), Length (4), IP (192.168.4.1))
                uint8_t answer[] = {
                    0xC0, 0x0C,
                    0x00, 0x01,
                    0x00, 0x01,
                    0x00, 0x00, 0x00, 0x3C,
                    0x00, 0x04,
                    192, 168, 4, 1
                };

                if (len + sizeof(answer) <= sizeof(buf)) {
                    memcpy(buf + len, answer, sizeof(answer));
                    sendto(sock, buf, len + sizeof(answer), 0, (struct sockaddr *)&client_addr, client_len);
                }
            }
        }
        close(sock);
        ESP_LOGI(Tag, "DNS server stopped");
        vTaskDelete(nullptr);
    }

    static int start_listen_socket(int port) {
        int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) return -1;
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if (bind(listen_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
            close(listen_sock);
            return -1;
        }
        if (listen(listen_sock, 4) != 0) {
            close(listen_sock);
            return -1;
        }
        return listen_sock;
    }

    static void send_all(int sock, const char* data, size_t len) {
        size_t sent = 0;
        while (sent < len) {
            int r = send(sock, data + sent, len - sent, 0);
            if (r < 0) break;
            sent += r;
        }
    }

    static void handle_client(int client_sock) {
        const int BUF_SZ = 8192;
        char *buf = (char*)malloc(BUF_SZ+1);
        if (!buf) { close(client_sock); return; }
        int r = recv(client_sock, buf, BUF_SZ, 0);
        if (r <= 0) { free(buf); close(client_sock); return; }
        buf[r] = '\0';

        // parse request line
        char method[8] = {0};
        char uri[256] = {0};
        if (sscanf(buf, "%7s %255s", method, uri) != 2) {
            free(buf); close(client_sock); return;
        }

        // find end of headers
        char *hdr_end = strstr(buf, "\r\n\r\n");
        char *body = hdr_end ? hdr_end + 4 : nullptr;
        int content_len = 0;
        if (hdr_end) {
            // find Content-Length header
            char *cl = strcasestr(buf, "Content-Length:");
            if (cl) content_len = atoi(cl + 15);
        }

        if (strcmp(method, "GET") == 0 && strcmp(uri, "/") == 0) {
            const char *hdr = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: ";
            char lenbuf[32];
            int hlen = snprintf(lenbuf, sizeof(lenbuf), "%d", (int)strlen(HTML_PAGE));
            send_all(client_sock, hdr, strlen(hdr));
            send_all(client_sock, lenbuf, hlen);
            send_all(client_sock, "\r\n\r\n", 4);
            send_all(client_sock, HTML_PAGE, strlen(HTML_PAGE));
        } else if (strcmp(method, "POST") == 0 && strcmp(uri, "/wifi") == 0) {
            // ensure we have full body
            int received_body = body ? (r - (body - buf)) : 0;
            while (received_body < content_len && received_body < 2048) {
                int n = recv(client_sock, buf + r, BUF_SZ - r, 0);
                if (n <= 0) break;
                r += n;
                received_body = r - (body - buf);
            }
            if (body && content_len > 0) {
                // copy body to temporary buffer and parse
                char *b = (char*)malloc(content_len + 1);
                if (b) {
                    memcpy(b, body, content_len);
                    b[content_len] = '\0';
                    // parse urlencoded
                    char ssid[64] = {0};
                    char password[128] = {0};
                    char *pair = strtok(b, "&");
                    while (pair) {
                        char *eq = strchr(pair, '=');
                        if (eq) {
                            *eq = '\0';
                            char *k = pair;
                            char *v = eq + 1;
                            url_decode_inplace(v);
                            if (strcmp(k, "ssid") == 0) strncpy(ssid, v, sizeof(ssid)-1);
                            else if (strcmp(k, "password") == 0) strncpy(password, v, sizeof(password)-1);
                        }
                        pair = strtok(nullptr, "&");
                    }
                    
                    // Trim leading/trailing whitespace
                    auto trim_inplace = [](char* str) {
                        char* start = str;
                        while(*start && isspace((unsigned char)*start)) start++;
                        if(start != str) memmove(str, start, strlen(start) + 1);
                        if(*str == 0) return;
                        char* end = str + strlen(str) - 1;
                        while(end >= str && isspace((unsigned char)*end)) end--;
                        end[1] = '\0';
                    };
                    trim_inplace(ssid);
                    trim_inplace(password);

                    ESP_LOGI(Tag, "Received credentials SSID='%s' password_len=%d", ssid, (int)strlen(password));
                    NVSHandle nvs("wifi_config", NVS_READWRITE);
                    if (nvs.isValid()) {
                        nvs.setString("ssid", std::string(ssid));
                        nvs.setString("password", std::string(password));
                        nvs.commit();
                        ESP_LOGI(Tag, "Credentials saved to NVS");
                    } else {
                        ESP_LOGE(Tag, "Failed to open NVS handle");
                    }
                    free(b);
                    WifiManager::getInstance().startSTA(std::string(ssid), std::string(password));
                }
            }
            const char *resp = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: 34\r\n\r\n<html><body>Saved. Connecting...</body></html>";
            send_all(client_sock, resp, strlen(resp));
        } else {
            // Captive portal redirect to ESP IP address
            const char *resp = "HTTP/1.1 302 Found\r\nLocation: http://192.168.4.1/\r\nContent-Length: 0\r\n\r\n";
            send_all(client_sock, resp, strlen(resp));
        }

        free(buf);
        close(client_sock);
    }

    static void web_task(void *arg) {
        (void)arg;
        ESP_LOGI(Tag, "Starting socket web server on port 80");
        int listen_sock = start_listen_socket(80);
        if (listen_sock < 0) {
            ESP_LOGE(Tag, "Failed to start listen socket");
            vTaskDelete(nullptr);
            return;
        }
        while (!s_stop) {
            struct sockaddr_in6 client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int client_sock = accept(listen_sock, (struct sockaddr*)&client_addr, &addr_len);
            if (client_sock < 0) continue;
            handle_client(client_sock);
        }
        close(listen_sock);
        ESP_LOGI(Tag, "Socket web server stopped");
        vTaskDelete(nullptr);
    }

    void WebServer::init() {
        s_stop = false;
        s_dns_stop = false;
        xTaskCreate(web_task, "web_task", 8 * 1024, nullptr, tskIDLE_PRIORITY + 5, nullptr);
        xTaskCreate(dns_task, "dns_task", 4 * 1024, nullptr, tskIDLE_PRIORITY + 4, nullptr);
    }

    void WebServer::start() {
        if (!m_server) init();
    }

    void WebServer::stop() {
        s_stop = true;
        s_dns_stop = true;
        if (m_server) {
            httpd_stop(m_server);
            m_server = nullptr;
        }
        ESP_LOGI(Tag, "Web server stop requested");
    }

}
