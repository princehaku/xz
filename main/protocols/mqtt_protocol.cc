#include "mqtt_protocol.h"
#include "board.h"
#include "application.h"
#include "settings.h"

#include <esp_log.h>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include "assets/lang_config.h"

#define TAG "MQTT"

namespace {
struct ReconnectTarget {
    MqttProtocol* protocol;
    std::weak_ptr<std::atomic<bool>> alive;
};
std::mutex reconnect_targets_mutex;
std::map<uintptr_t, ReconnectTarget> reconnect_targets;
uintptr_t next_reconnect_id = 1;
}  // namespace

MqttProtocol::MqttProtocol() {
    mbedtls_aes_init(&aes_ctx_);
    event_group_handle_ = xEventGroupCreate();

    // The timer must obtain the lifetime guard without dereferencing its target.
    // Protocol destruction and scheduled work both run on the application task.
    {
        std::lock_guard<std::mutex> lock(reconnect_targets_mutex);
        reconnect_id_ = next_reconnect_id++;
        reconnect_targets.emplace(reconnect_id_, ReconnectTarget{this, alive_});
    }
    esp_timer_create_args_t reconnect_timer_args = {
        .callback = [](void* arg) {
            MqttProtocol* protocol = nullptr;
            std::shared_ptr<std::atomic<bool>> alive;
            {
                std::lock_guard<std::mutex> lock(reconnect_targets_mutex);
                auto target = reconnect_targets.find(reinterpret_cast<uintptr_t>(arg));
                if (target == reconnect_targets.end()) {
                    return;
                }
                protocol = target->second.protocol;
                alive = target->second.alive.lock();
            }
            if (!alive || !alive->load()) {
                return;
            }
            Application::GetInstance().Schedule([protocol, alive]() {
                if (alive->load()) {
                    protocol->RetryConnection();
                }
            });
        },
        .arg = reinterpret_cast<void*>(reconnect_id_),
    };
    if (esp_timer_create(&reconnect_timer_args, &reconnect_timer_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MQTT reconnect timer");
    }
}

MqttProtocol::~MqttProtocol() {
    ESP_LOGI(TAG, "MqttProtocol deinit");
    
    // Mark as dead first to prevent any pending scheduled tasks from executing
    *alive_ = false;
    {
        std::lock_guard<std::mutex> lock(reconnect_targets_mutex);
        reconnect_targets.erase(reconnect_id_);
    }
    
    {
        std::lock_guard<std::mutex> lock(reconnect_mutex_);
        if (reconnect_timer_ != nullptr) {
            esp_timer_stop(reconnect_timer_);
            esp_timer_delete(reconnect_timer_);
            reconnect_timer_ = nullptr;
        }
    }

    udp_.reset();
    mqtt_.reset();
    mbedtls_aes_free(&aes_ctx_);
    
    if (event_group_handle_ != nullptr) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool MqttProtocol::Start() {
    return StartMqttClient(false);
}

void MqttProtocol::ScheduleReconnect() {
    std::lock_guard<std::mutex> lock(reconnect_mutex_);
    if (!alive_->load() || !reconnect_timer_ || mqtt_connected_.load() ||
        esp_timer_is_active(reconnect_timer_)) {
        return;
    }
    if (esp_timer_start_once(reconnect_timer_, uint64_t(reconnect_delay_ms_) * 1000) == ESP_OK) {
        ESP_LOGI(TAG, "MQTT reconnect scheduled in %lu seconds", (unsigned long)(reconnect_delay_ms_ / 1000));
        reconnect_delay_ms_ = std::min(reconnect_delay_ms_ * 2, uint32_t(MQTT_RECONNECT_MAX_INTERVAL_MS));
    }
}

void MqttProtocol::ResetReconnect() {
    std::lock_guard<std::mutex> lock(reconnect_mutex_);
    if (reconnect_timer_) {
        esp_timer_stop(reconnect_timer_);
    }
    reconnect_delay_ms_ = MQTT_RECONNECT_INTERVAL_MS;
}

void MqttProtocol::RetryConnection() {
    if (mqtt_connected_.load()) {
        return;
    }
    if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
        ScheduleReconnect();
        return;
    }
    StartMqttClient(false);
}

bool MqttProtocol::StartMqttClient(bool report_error) {
    {
        std::lock_guard<std::mutex> lock(reconnect_mutex_);
        if (reconnect_timer_) {
            esp_timer_stop(reconnect_timer_);
        }
    }
    mqtt_connected_.store(false);
    if (mqtt_ != nullptr) {
        ESP_LOGW(TAG, "Mqtt client already started");
        mqtt_->OnConnected(nullptr);
        mqtt_->OnDisconnected(nullptr);
        mqtt_.reset();
    }

    Settings settings("mqtt", false);
    auto endpoint = settings.GetString("endpoint");
    auto client_id = settings.GetString("client_id");
    auto username = settings.GetString("username");
    auto password = settings.GetString("password");
    int keepalive_interval = settings.GetInt("keepalive", 240);
    publish_topic_ = settings.GetString("publish_topic");

    if (endpoint.empty()) {
        ESP_LOGW(TAG, "MQTT endpoint is not specified");
        if (report_error) {
            SetError(Lang::Strings::SERVER_NOT_FOUND);
        }
        ScheduleReconnect();
        return false;
    }

    auto network = Board::GetInstance().GetNetwork();
    if (network == nullptr) {
        if (report_error) {
            SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        }
        ScheduleReconnect();
        return false;
    }
    mqtt_ = network->CreateMqtt(0);
    if (!mqtt_) {
        if (report_error) {
            SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        }
        ScheduleReconnect();
        return false;
    }
    mqtt_->SetKeepAlive(keepalive_interval);

    mqtt_->OnDisconnected([this, alive = alive_]() {
        if (!alive->load()) {
            return;
        }
        mqtt_connected_.store(false);
        if (on_disconnected_ != nullptr) {
            on_disconnected_();
        }
        ScheduleReconnect();
    });

    mqtt_->OnConnected([this, alive = alive_]() {
        if (!alive->load()) {
            return;
        }
        mqtt_connected_.store(true);
        ResetReconnect();
        if (on_connected_ != nullptr) {
            on_connected_();
        }
    });

    mqtt_->OnMessage([this](const std::string& topic, const std::string& payload) {
        cJSON* root = cJSON_ParseWithLength(payload.data(), payload.size());
        if (root == nullptr) {
            ESP_LOGE(TAG, "Failed to parse json message %s", payload.c_str());
            return;
        }
        cJSON* type = cJSON_GetObjectItem(root, "type");
        if (!cJSON_IsString(type)) {
            ESP_LOGE(TAG, "Message type is invalid");
            cJSON_Delete(root);
            return;
        }

        if (strcmp(type->valuestring, "hello") == 0) {
            ParseServerHello(root);
        } else if (strcmp(type->valuestring, "goodbye") == 0) {
            auto session_id = cJSON_GetObjectItem(root, "session_id");
            if (session_id == nullptr ||
                (cJSON_IsString(session_id) && session_id_ == session_id->valuestring)) {
                auto alive = alive_;  // Capture alive flag
                Application::GetInstance().Schedule([this, alive]() {
                    if (*alive) {
                        // Server initiated goodbye, don't send goodbye back to avoid ping-pong
                        CloseAudioChannel(false);
                    }
                });
            }
        } else if (on_incoming_json_ != nullptr) {
            on_incoming_json_(root);
        }
        cJSON_Delete(root);
        last_incoming_time_ = std::chrono::steady_clock::now();
    });

    ESP_LOGI(TAG, "Connecting to endpoint %s", endpoint.c_str());
    std::string broker_address;
    int broker_port = 8883;
    size_t pos = endpoint.find(':');
    if (pos != std::string::npos) {
        broker_address = endpoint.substr(0, pos);
        broker_port = std::stoi(endpoint.substr(pos + 1));
    } else {
        broker_address = endpoint;
    }
    if (!mqtt_->Connect(broker_address, broker_port, client_id, username, password)) {
        ESP_LOGE(TAG, "Failed to connect to endpoint, code=%d", mqtt_->GetLastError());
        mqtt_.reset();
        mqtt_connected_.store(false);
        if (report_error) {
            SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        }
        ScheduleReconnect();
        return false;
    }

    ESP_LOGI(TAG, "Connected to endpoint");
    mqtt_connected_.store(true);
    ResetReconnect();
    return true;
}

bool MqttProtocol::SendText(const std::string& text) {
    if (publish_topic_.empty() || !mqtt_) {
        return false;
    }
    if (!mqtt_->Publish(publish_topic_, text)) {
        ESP_LOGE(TAG, "Failed to publish message: %s", text.c_str());
        SetError(Lang::Strings::SERVER_ERROR);
        return false;
    }
    return true;
}

bool MqttProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    std::lock_guard<std::mutex> lock(channel_mutex_);
    if (udp_ == nullptr || aes_nonce_.size() != 16 || packet->payload.size() > UINT16_MAX) {
        return false;
    }

    std::string nonce(aes_nonce_);
    uint16_t payload_size = htons(packet->payload.size());
    uint32_t timestamp = htonl(packet->timestamp);
    uint32_t sequence = htonl(++local_sequence_);
    memcpy(nonce.data() + 2, &payload_size, sizeof(payload_size));
    memcpy(nonce.data() + 8, &timestamp, sizeof(timestamp));
    memcpy(nonce.data() + 12, &sequence, sizeof(sequence));

    std::string encrypted;
    encrypted.resize(aes_nonce_.size() + packet->payload.size());
    memcpy(encrypted.data(), nonce.data(), nonce.size());

    size_t nc_off = 0;
    uint8_t stream_block[16] = {0};
    if (mbedtls_aes_crypt_ctr(&aes_ctx_, packet->payload.size(), &nc_off, (uint8_t*)nonce.data(), stream_block,
        (uint8_t*)packet->payload.data(), (uint8_t*)&encrypted[nonce.size()]) != 0) {
        ESP_LOGE(TAG, "Failed to encrypt audio data");
        return false;
    }

    return udp_->Send(encrypted) > 0;
}

void MqttProtocol::CloseAudioChannel(bool send_goodbye) {
    if (audio_channel_closing_.exchange(true)) {
        ESP_LOGW(TAG, "Audio channel close already in progress");
        return;
    }

    bool had_udp = false;
    {
        std::lock_guard<std::mutex> lock(channel_mutex_);
        had_udp = (udp_ != nullptr);
        udp_.reset();
    }

    if (!had_udp) {
        audio_channel_closing_.store(false);
        return;
    }

    ESP_LOGI(TAG, "Closing audio channel, send_goodbye: %d", send_goodbye);

    // Only send goodbye when client initiates the close
    // Don't send if server already sent goodbye (to avoid ping-pong)
    if (send_goodbye) {
        std::string message = "{";
        message += "\"session_id\":\"" + session_id_ + "\",";
        message += "\"type\":\"goodbye\"";
        message += "}";
        SendText(message);
    }

    if (on_audio_channel_closed_ != nullptr) {
        on_audio_channel_closed_();
    }
    audio_channel_closing_.store(false);
}

bool MqttProtocol::OpenAudioChannel() {
    {
        std::lock_guard<std::mutex> lock(channel_mutex_);
        udp_.reset();
    }
    session_id_.clear();
    aes_nonce_.clear();
    server_sample_rate_ = 24000;
    server_frame_duration_ = 60;
    xEventGroupClearBits(event_group_handle_, MQTT_PROTOCOL_SERVER_HELLO_EVENT);
    if (mqtt_ == nullptr || !mqtt_->IsConnected()) {
        ESP_LOGI(TAG, "MQTT is not connected, try to connect now");
        if (!StartMqttClient(true)) {
            return false;
        }
    }

    error_occurred_ = false;

    auto message = GetHelloMessage();
    if (!SendText(message)) {
        return false;
    }

    // 等待服务器响应
    EventBits_t bits = xEventGroupWaitBits(event_group_handle_, MQTT_PROTOCOL_SERVER_HELLO_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    if (!(bits & MQTT_PROTOCOL_SERVER_HELLO_EVENT)) {
        ESP_LOGE(TAG, "Failed to receive server hello");
        SetError(Lang::Strings::SERVER_TIMEOUT);
        return false;
    }

    std::lock_guard<std::mutex> lock(channel_mutex_);
    auto network = Board::GetInstance().GetNetwork();
    if (!network) {
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }
    udp_ = network->CreateUdp(2);
    if (!udp_) {
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }
    udp_->OnMessage([this](const std::string& data) {
        /*
         * UDP Encrypted OPUS Packet Format:
         * |type 1u|flags 1u|payload_len 2u|ssrc 4u|timestamp 4u|sequence 4u|
         * |payload payload_len|
         */
        constexpr size_t kHeaderSize = 16;
        if (data.size() < kHeaderSize) {
            ESP_LOGE(TAG, "Invalid audio packet size: %u", data.size());
            return;
        }
        if (data[0] != 0x01) {
            ESP_LOGE(TAG, "Invalid audio packet type: %x", data[0]);
            return;
        }
        uint16_t payload_size;
        uint32_t timestamp;
        uint32_t sequence;
        memcpy(&payload_size, data.data() + 2, sizeof(payload_size));
        memcpy(&timestamp, data.data() + 8, sizeof(timestamp));
        memcpy(&sequence, data.data() + 12, sizeof(sequence));
        payload_size = ntohs(payload_size);
        timestamp = ntohl(timestamp);
        sequence = ntohl(sequence);
        if (payload_size == 0 || payload_size != data.size() - kHeaderSize) {
            ESP_LOGE(TAG, "Invalid audio payload length");
            return;
        }
        if (sequence <= remote_sequence_) {
            ESP_LOGW(TAG, "Received audio packet with old sequence: %lu, expected: %lu", sequence, remote_sequence_);
            return;
        }
        if (sequence != remote_sequence_ + 1) {
            ESP_LOGW(TAG, "Received audio packet with wrong sequence: %lu, expected: %lu", sequence, remote_sequence_ + 1);
        }

        size_t decrypted_size = payload_size;
        size_t nc_off = 0;
        uint8_t stream_block[16] = {0};
        uint8_t nonce[kHeaderSize];
        memcpy(nonce, data.data(), sizeof(nonce));
        auto encrypted = reinterpret_cast<const uint8_t*>(data.data() + kHeaderSize);
        auto packet = std::make_unique<AudioStreamPacket>();
        packet->sample_rate = server_sample_rate_;
        packet->frame_duration = server_frame_duration_;
        packet->timestamp = timestamp;
        packet->payload.resize(decrypted_size);
        int ret = mbedtls_aes_crypt_ctr(&aes_ctx_, decrypted_size, &nc_off, nonce, stream_block, encrypted, (uint8_t*)packet->payload.data());
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to decrypt audio data, ret: %d", ret);
            return;
        }
        if (on_incoming_audio_ != nullptr) {
            on_incoming_audio_(std::move(packet));
        }
        remote_sequence_ = sequence;
        last_incoming_time_ = std::chrono::steady_clock::now();
    });

    if (!udp_->Connect(udp_server_, udp_port_)) {
        ESP_LOGE(TAG, "Failed to connect UDP audio channel, code=%d", udp_->GetLastError());
        udp_.reset();
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }

    if (on_audio_channel_opened_ != nullptr) {
        on_audio_channel_opened_();
    }
    return true;
}

std::string MqttProtocol::GetHelloMessage() {
    // 发送 hello 消息申请 UDP 通道
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddNumberToObject(root, "version", 3);
    cJSON_AddStringToObject(root, "transport", "udp");
    cJSON* features = cJSON_CreateObject();
#if CONFIG_USE_SERVER_AEC
    cJSON_AddBoolToObject(features, "aec", true);
#endif
    cJSON_AddBoolToObject(features, "mcp", true);
    cJSON_AddItemToObject(root, "features", features);
    cJSON* audio_params = cJSON_CreateObject();
    cJSON_AddStringToObject(audio_params, "format", "opus");
    cJSON_AddNumberToObject(audio_params, "sample_rate", 16000);
    cJSON_AddNumberToObject(audio_params, "channels", 1);
    cJSON_AddNumberToObject(audio_params, "frame_duration", OPUS_FRAME_DURATION_MS);
    cJSON_AddItemToObject(root, "audio_params", audio_params);
    auto json_str = cJSON_PrintUnformatted(root);
    std::string message(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    return message;
}

void MqttProtocol::ParseServerHello(const cJSON* root) {
    auto transport = cJSON_GetObjectItem(root, "transport");
    if (!cJSON_IsString(transport) || strcmp(transport->valuestring, "udp") != 0) {
        ESP_LOGE(TAG, "Invalid UDP transport");
        return;
    }

    auto session_id = cJSON_GetObjectItem(root, "session_id");
    if (session_id && !cJSON_IsString(session_id)) {
        return;
    }

    // Get sample rate from hello message
    auto audio_params = cJSON_GetObjectItem(root, "audio_params");
    if (audio_params && !cJSON_IsObject(audio_params)) {
        return;
    }
    int sample_rate_value = server_sample_rate_;
    int frame_duration_value = server_frame_duration_;
    if (cJSON_IsObject(audio_params)) {
        auto sample_rate = cJSON_GetObjectItem(audio_params, "sample_rate");
        if (sample_rate) {
            if (!cJSON_IsNumber(sample_rate) || sample_rate->valuedouble != sample_rate->valueint ||
                (sample_rate->valueint != 8000 && sample_rate->valueint != 12000 &&
                 sample_rate->valueint != 16000 && sample_rate->valueint != 24000 &&
                 sample_rate->valueint != 48000)) {
                return;
            }
            sample_rate_value = sample_rate->valueint;
        }
        auto frame_duration = cJSON_GetObjectItem(audio_params, "frame_duration");
        if (frame_duration) {
            if (!cJSON_IsNumber(frame_duration) || frame_duration->valuedouble != frame_duration->valueint ||
                (frame_duration->valueint != 5 && frame_duration->valueint != 10 &&
                 frame_duration->valueint != 20 && frame_duration->valueint != 40 &&
                 frame_duration->valueint != 60 && frame_duration->valueint != 80 &&
                 frame_duration->valueint != 100 && frame_duration->valueint != 120)) {
                return;
            }
            frame_duration_value = frame_duration->valueint;
        }
    }

    auto udp = cJSON_GetObjectItem(root, "udp");
    if (!cJSON_IsObject(udp)) {
        ESP_LOGE(TAG, "UDP is not specified");
        return;
    }
    auto server = cJSON_GetObjectItem(udp, "server");
    auto port = cJSON_GetObjectItem(udp, "port");
    auto key = cJSON_GetObjectItem(udp, "key");
    auto nonce = cJSON_GetObjectItem(udp, "nonce");
    if (!cJSON_IsString(server) || server->valuestring[0] == '\0' ||
        !cJSON_IsNumber(port) || port->valuedouble != port->valueint ||
        port->valueint < 1 || port->valueint > UINT16_MAX ||
        !cJSON_IsString(key) || !cJSON_IsString(nonce)) {
        ESP_LOGE(TAG, "Invalid UDP connection parameters");
        return;
    }
    auto decoded_key = DecodeHexString(key->valuestring);
    auto decoded_nonce = DecodeHexString(nonce->valuestring);
    if (decoded_key.size() != 16 || decoded_nonce.size() != 16) {
        ESP_LOGE(TAG, "Invalid UDP key or nonce");
        return;
    }

    // auto encryption = cJSON_GetObjectItem(udp, "encryption")->valuestring;
    // ESP_LOGI(TAG, "UDP server: %s, port: %d, encryption: %s", udp_server_.c_str(), udp_port_, encryption);
    if (mbedtls_aes_setkey_enc(&aes_ctx_, reinterpret_cast<const unsigned char*>(decoded_key.data()), 128) != 0) {
        return;
    }
    udp_server_ = server->valuestring;
    udp_port_ = port->valueint;
    aes_nonce_ = std::move(decoded_nonce);
    session_id_ = session_id ? session_id->valuestring : "";
    server_sample_rate_ = sample_rate_value;
    server_frame_duration_ = frame_duration_value;
    local_sequence_ = 0;
    remote_sequence_ = 0;
    xEventGroupSetBits(event_group_handle_, MQTT_PROTOCOL_SERVER_HELLO_EVENT);
}

// 辅助函数，将单个十六进制字符转换为对应的数值
static inline int CharToHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

std::string MqttProtocol::DecodeHexString(const std::string& hex_string) {
    if (hex_string.size() != 32) {
        return {};
    }
    std::string decoded;
    decoded.reserve(hex_string.size() / 2);
    for (size_t i = 0; i < hex_string.size(); i += 2) {
        int high = CharToHex(hex_string[i]);
        int low = CharToHex(hex_string[i + 1]);
        if (high < 0 || low < 0) {
            return {};
        }
        char byte = (high << 4) | low;
        decoded.push_back(byte);
    }
    return decoded;
}

bool MqttProtocol::IsAudioChannelOpened() const {
    return udp_ != nullptr && !error_occurred_ && !IsTimeout();
}
