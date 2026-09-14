#include "sd_video_portal.h"

#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>

#include <cJSON.h>
#include <strings.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "sd_video_url.h"

namespace {
constexpr char kTag[] = "SdVideoPortal";
constexpr size_t kMaxBodyBytes = 1536;
constexpr int64_t kReadTimeoutUs = 5000000;

constexpr char kPage[] = R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SD 卡视频</title><style>
body{font:16px system-ui,sans-serif;max-width:640px;margin:24px auto;padding:0 16px;line-height:1.6}
input,button{font:inherit;padding:10px;box-sizing:border-box}input{width:100%}
button{margin:8px 6px 0 0}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f4f4;padding:12px}
progress{width:100%}.hint{color:#555;font-size:14px}
</style></head><body>
<h1>SD 卡视频</h1>
<p>通过 Wi-Fi 下载 AVI 到板上的 SD 卡，文件保存在 video 目录。</p>
<p class="hint">支持 MJPEG 编码的 AVI，最大 320 × 240；音轨支持 16 位 PCM，单声道或双声道，8–48 kHz。视频地址需能被开发板直接访问。</p>
<form id="download-form"><label for="url">HTTP / HTTPS 视频地址</label>
<input id="url" type="url" maxlength="1024" required placeholder="http://192.168.1.2:8000/test.avi" autocomplete="off">
<button type="submit">下载到 SD 卡</button></form>
<div><button type="button" data-action="play">播放本地</button>
<button type="button" data-action="pause">暂停 / 继续</button>
<button type="button" data-action="stop">停止</button>
<button type="button" data-action="rescan">重新扫描</button></div>
<p id="result" role="status">等待操作</p>
<h2>开发板状态</h2><progress id="progress" max="100" value="0"></progress>
<pre id="status">正在读取…</pre>
<script>
const token = '__SD_TOKEN__';
const result = document.getElementById('result');
const statusText = document.getElementById('status');
const progress = document.getElementById('progress');
async function submit(path, payload) {
  try {
    const response = await fetch(path, {method:'POST', cache:'no-store',
      headers:{'Content-Type':'application/json','X-SD-Token':token}, body:JSON.stringify(payload)});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || '请求失败');
    result.textContent = data.queued ? '请求已排队，请查看开发板状态。' : '已收到响应。';
  } catch (error) { result.textContent = '操作失败：' + error.message; }
}
document.getElementById('download-form').addEventListener('submit', event => {
  event.preventDefault();
  submit('/download', {url:document.getElementById('url').value.trim()});
});
document.querySelectorAll('[data-action]').forEach(button => {
  button.addEventListener('click', () => submit('/control', {action:button.dataset.action}));
});
async function updateStatus() {
  try {
    const response = await fetch('/status', {cache:'no-store'});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || '状态读取失败');
    const percent = Math.min(100, Math.max(0, Number(data.download_percent) || 0));
    progress.value = percent;
    statusText.textContent = '状态：' + (data.state ?? '未知') + '\n文件：' + (data.title || '—')
      + '\n提示：' + (data.message || '—') + '\n下载进度：' + percent + '%'
      + '\n视频：' + (data.is_video ? '是' : '否')
      + '\n视频帧数：' + (data.frame_count ?? data.video_frames ?? 0)
      + '\n音轨：' + (data.is_video ? (data.has_audio ? 'PCM' : '无音轨') : '—')
      + '\n已提交音频采样：' + (data.audio_samples ?? 0);
  } catch (error) { statusText.textContent = '读取失败：' + error.message; }
  setTimeout(updateStatus, 1000);
}
updateStatus();
</script></body></html>)HTML";

esp_err_t SendResponse(httpd_req_t* request, const char* status, const char* content_type,
                       const char* body, size_t size) {
    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, content_type);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(request, "X-Frame-Options", "DENY");
    // Closing also prevents unread bytes from a rejected POST becoming another request.
    httpd_resp_set_hdr(request, "Connection", "close");
    return httpd_resp_send(request, body, size);
}

esp_err_t SendJson(httpd_req_t* request, const char* status, const char* json) {
    return SendResponse(request, status, "application/json; charset=utf-8", json,
                        std::strlen(json));
}

bool ReadBody(httpd_req_t* request, std::string& body) {
    if (request->content_len == 0 || request->content_len > kMaxBodyBytes) {
        SendJson(request, "413 Payload Too Large",
                 "{\"error\":\"JSON body must be 1-1536 bytes\"}");
        return false;
    }
    char content_type[64] = {};
    if (httpd_req_get_hdr_value_str(request, "Content-Type", content_type, sizeof(content_type)) !=
            ESP_OK ||
        strncasecmp(content_type, "application/json", 16) != 0 ||
        (content_type[16] != '\0' && content_type[16] != ';' && content_type[16] != ' ')) {
        SendJson(request, "415 Unsupported Media Type", "{\"error\":\"Use application/json\"}");
        return false;
    }
    body.resize(request->content_len);
    size_t received = 0;
    const int64_t deadline = esp_timer_get_time() + kReadTimeoutUs;
    while (received < body.size()) {
        if (esp_timer_get_time() >= deadline) {
            SendJson(request, "408 Request Timeout", "{\"error\":\"Request body timed out\"}");
            return false;
        }
        int count = httpd_req_recv(request, body.data() + received, body.size() - received);
        if (count == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (count <= 0) {
            SendJson(request, "400 Bad Request", "{\"error\":\"Incomplete request body\"}");
            return false;
        }
        received += static_cast<size_t>(count);
    }
    // cJSON represents strings with NUL termination, so reject embedded NULs explicitly.
    if (body.find('\0') != std::string::npos || body.find("\\u0000") != std::string::npos) {
        SendJson(request, "400 Bad Request", "{\"error\":\"NUL characters are not allowed\"}");
        return false;
    }
    return true;
}

using JsonPtr = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;

JsonPtr ParseBody(httpd_req_t* request, const std::string& body) {
    // These endpoints accept one string field. Reject nesting before cJSON so an
    // untrusted request cannot consume the small HTTP task stack with recursion.
    bool in_string = false;
    bool escaped = false;
    unsigned objects = 0;
    for (char character : body) {
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else if (character == '"') {
                in_string = false;
            }
        } else if (character == '"') {
            in_string = true;
        } else if (character == '[' || (character == '{' && ++objects > 1)) {
            SendJson(request, "400 Bad Request", "{\"error\":\"Nested JSON is not supported\"}");
            return JsonPtr(nullptr, cJSON_Delete);
        }
    }
    JsonPtr json(cJSON_ParseWithOpts(body.c_str(), nullptr, true), cJSON_Delete);
    if (!json || !cJSON_IsObject(json.get()) || cJSON_GetArraySize(json.get()) != 1) {
        SendJson(request, "400 Bad Request", "{\"error\":\"Expected one JSON string field\"}");
        return JsonPtr(nullptr, cJSON_Delete);
    }
    return json;
}

}  // namespace

SdVideoPortal::SdVideoPortal(Actions actions) : actions_(std::move(actions)) {}

SdVideoPortal::~SdVideoPortal() { Stop(); }

bool SdVideoPortal::Start() {
    if (server_) {
        return true;
    }
    uint8_t random[16];
    esp_fill_random(random, sizeof(random));
    constexpr char kHex[] = "0123456789abcdef";
    token_.resize(sizeof(random) * 2);
    for (size_t i = 0; i < sizeof(random); ++i) {
        token_[i * 2] = kHex[random[i] >> 4];
        token_[i * 2 + 1] = kHex[random[i] & 0x0f];
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;
    config.ctrl_port = 32769;
    config.max_uri_handlers = 4;
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.stack_size = 6144;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 3;
    esp_err_t error = httpd_start(&server_, &config);
    if (error != ESP_OK) {
        server_ = nullptr;
        token_.clear();
        ESP_LOGW(kTag, "HTTP server start failed: %s", esp_err_to_name(error));
        return false;
    }
    const httpd_uri_t handlers[] = {
        {.uri = "/", .method = HTTP_GET, .handler = HandleIndex, .user_ctx = this},
        {.uri = "/status", .method = HTTP_GET, .handler = HandleStatus, .user_ctx = this},
        {.uri = "/download", .method = HTTP_POST, .handler = HandleDownload, .user_ctx = this},
        {.uri = "/control", .method = HTTP_POST, .handler = HandleControl, .user_ctx = this},
    };
    for (const auto& handler : handlers) {
        error = httpd_register_uri_handler(server_, &handler);
        if (error != ESP_OK) {
            ESP_LOGW(kTag, "HTTP handler registration failed: %s", esp_err_to_name(error));
            Stop();
            return false;
        }
    }
    for (auto code : {HTTPD_404_NOT_FOUND, HTTPD_405_METHOD_NOT_ALLOWED}) {
        error = httpd_register_err_handler(
            server_, code, [](httpd_req_t* request, httpd_err_code_t error_code) -> esp_err_t {
                if (error_code == HTTPD_405_METHOD_NOT_ALLOWED) {
                    SendJson(request, "405 Method Not Allowed",
                             "{\"error\":\"Method not allowed\"}");
                } else {
                    SendJson(request, "404 Not Found", "{\"error\":\"Route not found\"}");
                }
                return ESP_FAIL;
            });
        if (error != ESP_OK) {
            ESP_LOGW(kTag, "HTTP error handler registration failed: %s", esp_err_to_name(error));
            Stop();
            return false;
        }
    }
    ESP_LOGI(kTag, "SD video portal started on port 8080");
    return true;
}

void SdVideoPortal::Stop() {
    if (server_) {
        httpd_stop(server_);
        server_ = nullptr;
    }
    token_.clear();
}

bool SdVideoPortal::Authorize(httpd_req_t* request) const {
    char supplied[33] = {};
    if (token_.size() != 32 || httpd_req_get_hdr_value_len(request, "X-SD-Token") != 32 ||
        httpd_req_get_hdr_value_str(request, "X-SD-Token", supplied, sizeof(supplied)) != ESP_OK) {
        SendJson(request, "403 Forbidden",
                 "{\"error\":\"Reload the control page before posting\"}");
        return false;
    }
    unsigned difference = 0;
    for (size_t i = 0; i < token_.size(); ++i) {
        difference |=
            static_cast<unsigned char>(supplied[i]) ^ static_cast<unsigned char>(token_[i]);
    }
    if (difference != 0) {
        SendJson(request, "403 Forbidden",
                 "{\"error\":\"Reload the control page before posting\"}");
        return false;
    }
    return true;
}

esp_err_t SdVideoPortal::HandleIndex(httpd_req_t* request) {
    auto* self = static_cast<SdVideoPortal*>(request->user_ctx);
    std::string page(kPage);
    constexpr char kPlaceholder[] = "__SD_TOKEN__";
    page.replace(page.find(kPlaceholder), sizeof(kPlaceholder) - 1, self->token_);
    return SendResponse(request, "200 OK", "text/html; charset=utf-8", page.data(), page.size());
}

esp_err_t SdVideoPortal::HandleStatus(httpd_req_t* request) {
    auto* self = static_cast<SdVideoPortal*>(request->user_ctx);
    if (!self->actions_.status) {
        return SendJson(request, "503 Service Unavailable", "{\"error\":\"Player unavailable\"}");
    }
    const std::string json = self->actions_.status();
    return SendResponse(request, "200 OK", "application/json; charset=utf-8", json.data(),
                        json.size());
}

esp_err_t SdVideoPortal::HandleDownload(httpd_req_t* request) {
    auto* self = static_cast<SdVideoPortal*>(request->user_ctx);
    std::string body;
    if (!self->Authorize(request) || !ReadBody(request, body)) {
        return ESP_FAIL;
    }
    auto json = ParseBody(request, body);
    if (!json) {
        return ESP_FAIL;
    }
    auto* url = cJSON_GetObjectItemCaseSensitive(json.get(), "url");
    if (!cJSON_IsString(url) || !url->valuestring || !IsSdVideoUrlValid(url->valuestring)) {
        return SendJson(request, "400 Bad Request",
                        "{\"error\":\"Invalid HTTP(S) URL, maximum 1024 bytes\"}");
    }
    if (!self->actions_.download) {
        return SendJson(request, "503 Service Unavailable", "{\"error\":\"Player unavailable\"}");
    }
    self->actions_.download(url->valuestring);
    return SendJson(request, "202 Accepted", "{\"queued\":true}");
}

esp_err_t SdVideoPortal::HandleControl(httpd_req_t* request) {
    auto* self = static_cast<SdVideoPortal*>(request->user_ctx);
    std::string body;
    if (!self->Authorize(request) || !ReadBody(request, body)) {
        return ESP_FAIL;
    }
    auto json = ParseBody(request, body);
    if (!json) {
        return ESP_FAIL;
    }
    auto* field = cJSON_GetObjectItemCaseSensitive(json.get(), "action");
    if (!cJSON_IsString(field) || !field->valuestring) {
        return SendJson(request, "400 Bad Request", "{\"error\":\"Missing action\"}");
    }
    const std::string action(field->valuestring);
    if (action != "play" && action != "pause" && action != "stop" && action != "rescan") {
        return SendJson(request, "400 Bad Request", "{\"error\":\"Unknown action\"}");
    }
    if (!self->actions_.control) {
        return SendJson(request, "503 Service Unavailable", "{\"error\":\"Player unavailable\"}");
    }
    self->actions_.control(action);
    return SendJson(request, "202 Accepted", "{\"queued\":true}");
}
