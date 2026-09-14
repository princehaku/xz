#pragma once

#include <esp_http_server.h>

#include <functional>
#include <string>

// Start/Stop must run on the application task. Actions must queue mutations there;
// status is called on the HTTP server task and must return a thread-safe snapshot.
class SdVideoPortal {
public:
    struct Actions {
        std::function<void(const std::string&)> download;
        std::function<void(const std::string&)> control;
        std::function<std::string()> status;
    };

    explicit SdVideoPortal(Actions actions);
    ~SdVideoPortal();
    SdVideoPortal(const SdVideoPortal&) = delete;
    SdVideoPortal& operator=(const SdVideoPortal&) = delete;

    bool Start();
    void Stop();

private:
    Actions actions_;
    httpd_handle_t server_ = nullptr;
    std::string token_;

    bool Authorize(httpd_req_t* request) const;
    static esp_err_t HandleIndex(httpd_req_t* request);
    static esp_err_t HandleStatus(httpd_req_t* request);
    static esp_err_t HandleDownload(httpd_req_t* request);
    static esp_err_t HandleControl(httpd_req_t* request);
};
