#!/usr/bin/env python3
"""Run the actual protocol C++ implementations with host transport/RTOS stubs.

Requires g++ and libcjson.so.1 (no ESP-IDF or device). On this Windows host:
  wsl -d Ubuntu-24.04 -- python3 /mnt/e/xiaozhi-esp32-s3-cam/scripts/test_protocol_boundaries.py
The JSON parser is the installed cJSON library; AES is stubbed because these
tests validate framing and buffer ownership, not the cipher implementation.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

PREAMBLE = r'''
#include <arpa/inet.h>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Public cJSON ABI; use the real shared-library parser without requiring its
// development headers in the firmware workspace.
extern "C" {
struct cJSON {
    cJSON *next, *prev, *child;
    int type;
    char *valuestring;
    int valueint;
    double valuedouble;
    char *string;
};
cJSON* cJSON_Parse(const char*);
cJSON* cJSON_ParseWithLength(const char*, size_t);
void cJSON_Delete(cJSON*);
cJSON* cJSON_GetObjectItem(const cJSON*, const char*);
int cJSON_IsString(const cJSON*);
int cJSON_IsNumber(const cJSON*);
int cJSON_IsObject(const cJSON*);
cJSON* cJSON_CreateObject();
cJSON* cJSON_AddStringToObject(cJSON*, const char*, const char*);
cJSON* cJSON_AddNumberToObject(cJSON*, const char*, double);
cJSON* cJSON_AddBoolToObject(cJSON*, const char*, int);
int cJSON_AddItemToObject(cJSON*, const char*, cJSON*);
char* cJSON_PrintUnformatted(const cJSON*);
void cJSON_free(void*);
}

#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_OK 0
#define OPUS_FRAME_DURATION_MS 60
#define pdTRUE true
#define pdFALSE false
#define pdMS_TO_TICKS(x) (x)
using EventBits_t = unsigned;
using EventGroupHandle_t = unsigned*;
EventGroupHandle_t xEventGroupCreate() { return new unsigned(0); }
void vEventGroupDelete(EventGroupHandle_t e) { delete e; }
void xEventGroupSetBits(EventGroupHandle_t e, unsigned bits) { *e |= bits; }
void xEventGroupClearBits(EventGroupHandle_t e, unsigned bits) { *e &= ~bits; }
unsigned xEventGroupWaitBits(EventGroupHandle_t e, unsigned bits, bool clear, bool, int) {
    unsigned result = *e & bits;
    if (clear) *e &= ~bits;
    return result;
}
struct esp_timer_create_args_t { void (*callback)(void*); void* arg; };
struct TestTimer { esp_timer_create_args_t args; bool active = false; uint64_t delay = 0; };
using esp_timer_handle_t = TestTimer*;
int timer_live = 0;
int esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* t) {
    *t = new TestTimer{*args}; ++timer_live; return 0;
}
int esp_timer_stop(esp_timer_handle_t timer) { timer->active = false; return 0; }
int esp_timer_delete(esp_timer_handle_t timer) { delete timer; --timer_live; return 0; }
bool esp_timer_is_active(esp_timer_handle_t timer) { return timer->active; }
int esp_timer_start_once(esp_timer_handle_t timer, uint64_t delay) {
    if (timer->active) return -1;
    timer->active = true; timer->delay = delay; return 0;
}
void fire_timer(esp_timer_handle_t timer) {
    assert(timer->active); timer->active = false; timer->args.callback(timer->args.arg);
}

struct mbedtls_aes_context { bool keyed = false; };
void mbedtls_aes_init(mbedtls_aes_context* c) { c->keyed = false; }
void mbedtls_aes_free(mbedtls_aes_context*) {}
int mbedtls_aes_setkey_enc(mbedtls_aes_context* c, const unsigned char* key, unsigned bits) {
    assert(bits == 128); unsigned char copy[16]; memcpy(copy, key, 16); c->keyed = true; return 0;
}
int mbedtls_aes_crypt_ctr(mbedtls_aes_context* c, size_t len, size_t*, unsigned char* nonce,
                         unsigned char*, const unsigned char* input, unsigned char* output) {
    assert(c->keyed); memcpy(output, input, len); ++nonce[15]; return 0;
}

const std::string ws_hello = R"({"type":"hello","transport":"websocket","version":2,"session_id":"fresh","audio_params":{"sample_rate":24000,"frame_duration":60}})";
const std::string mqtt_hello = R"({"type":"hello","transport":"udp","session_id":"fresh","udp":{"server":"host","port":9000,"key":"00112233445566778899aabbccddeeff","nonce":"01000000000000000000000000000000"}})";
bool ws_connect = true, ws_reply = true, ws_send = true, udp_connect = true, mqtt_connect = true, mqtt_reply = true;
int ws_live = 0, udp_live = 0, mqtt_connect_calls = 0;
struct WebSocket {
    bool connected = false;
    std::function<void(const char*, size_t, bool)> receive;
    std::function<void()> disconnected;
    WebSocket() { ++ws_live; }
    ~WebSocket() { --ws_live; if (connected && disconnected) disconnected(); }
    void SetHeader(const char*, const char*) {}
    bool IsConnected() const { return connected; }
    void OnData(decltype(receive) cb) { receive = std::move(cb); }
    void OnDisconnected(decltype(disconnected) cb) { disconnected = std::move(cb); }
    bool Connect(const char*) { return connected = ws_connect; }
    int GetLastError() { return -1; }
    bool Send(const std::string& text) {
        if (ws_send && ws_reply && text.find("hello") != std::string::npos) receive(ws_hello.data(), ws_hello.size(), false);
        return ws_send;
    }
    bool Send(const void*, size_t, bool) { return ws_send; }
};
struct Mqtt {
    bool connected = false;
    std::function<void()> on_connected, on_disconnected;
    std::function<void(const std::string&, const std::string&)> receive;
    void SetKeepAlive(int) {}
    void OnConnected(decltype(on_connected) cb) { on_connected = std::move(cb); }
    void OnDisconnected(decltype(on_disconnected) cb) { on_disconnected = std::move(cb); }
    void OnMessage(decltype(receive) cb) { receive = std::move(cb); }
    bool Connect(const std::string&, int, const std::string&, const std::string&, const std::string&) {
        ++mqtt_connect_calls;
        connected = mqtt_connect; if (connected && on_connected) on_connected(); return connected;
    }
    bool IsConnected() { return connected; }
    int GetLastError() { return -1; }
    bool Publish(const std::string&, const std::string& text) {
        if (mqtt_reply && text.find("hello") != std::string::npos) receive("", mqtt_hello);
        return connected;
    }
};
struct Udp {
    std::function<void(const std::string&)> receive;
    Udp() { ++udp_live; } ~Udp() { --udp_live; }
    void OnMessage(decltype(receive) cb) { receive = std::move(cb); }
    bool Connect(const std::string&, int) { return udp_connect; }
    int GetLastError() { return -1; }
    int Send(const std::string& data) { return data.size(); }
};
struct NetworkInterface {
    std::unique_ptr<WebSocket> CreateWebSocket(int) { return std::make_unique<WebSocket>(); }
    std::unique_ptr<Mqtt> CreateMqtt(int) { return std::make_unique<Mqtt>(); }
    std::unique_ptr<Udp> CreateUdp(int) { return std::make_unique<Udp>(); }
};
struct Board {
    static Board& GetInstance() { static Board b; return b; }
    NetworkInterface* GetNetwork() { static NetworkInterface n; return &n; }
    std::string GetUuid() { return "test"; }
};
struct Settings {
    Settings(const char*, bool) {}
    std::string GetString(const char* key) { return key == std::string("endpoint") ? "host:8883" : "test"; }
    int GetInt(const char*, int value = 0) { return value; }
};
struct SystemInfo { static std::string GetMacAddress() { return "test"; } };
constexpr int kDeviceStateIdle = 0;
int app_state = kDeviceStateIdle;
std::vector<std::function<void()>> scheduled_tasks;
struct Application {
    static Application& GetInstance() { static Application a; return a; }
    int GetDeviceState() { return app_state; }
    void Schedule(std::function<void()> task) { scheduled_tasks.push_back(std::move(task)); }
};
void run_scheduled_tasks() {
    auto tasks = std::move(scheduled_tasks); scheduled_tasks.clear();
    for (auto& task : tasks) task();
}
namespace Lang { namespace Strings {
const char *SERVER_ERROR = "error", *SERVER_NOT_CONNECTED = "offline", *SERVER_TIMEOUT = "timeout", *SERVER_NOT_FOUND = "missing";
} }
'''

TESTS = r'''
template<class P> void parse_hello(P& protocol, const std::string& json) {
    cJSON* root = cJSON_Parse(json.c_str()); assert(root);
    protocol.ParseServerHello(root); cJSON_Delete(root);
}
std::string changed(std::string text, const std::string& from, const std::string& to) {
    size_t pos = text.find(from); assert(pos != std::string::npos); text.replace(pos, from.size(), to); return text;
}
void test_websocket() {
    WebsocketProtocol p;
    int received = 0, opened = 0, json_received = 0;
    std::unique_ptr<AudioStreamPacket> packet;
    p.OnIncomingAudio([&](auto audio) { ++received; packet = std::move(audio); });
    p.OnIncomingJson([&](auto) { ++json_received; });
    p.OnAudioChannelOpened([&] { ++opened; });
    assert(p.OpenAudioChannel()); assert(opened == 1 && p.session_id() == "fresh");

    // Invalid or truncated binary data must never allocate or deliver audio.
    char short_data[20] = {};
    for (int version : {2, 3}) {
        p.version_ = version;
        const size_t header = version == 2 ? sizeof(BinaryProtocol2) : sizeof(BinaryProtocol3);
        for (size_t len = 0; len < header; ++len) p.websocket_->receive(short_data, len, true);
        for (size_t declared : {0u, 1u, 65535u}) {
            std::vector<char> data(header + 2);
            if (version == 2) {
                BinaryProtocol2 h{}; h.version = htons(2); h.payload_size = htonl(declared); memcpy(data.data(), &h, header);
            } else { BinaryProtocol3 h{}; h.payload_size = htons(declared); memcpy(data.data(), &h, header); }
            p.websocket_->receive(data.data(), data.size(), true);
        }
        assert(received == 0);
    }
    for (int version : {2, 3}) {
        p.version_ = version;
        size_t header = version == 2 ? sizeof(BinaryProtocol2) : sizeof(BinaryProtocol3);
        std::vector<char> unaligned(header + 4);
        if (version == 2) {
            BinaryProtocol2 h{}; h.version = htons(2); h.timestamp = htonl(1234); h.payload_size = htonl(3);
            memcpy(unaligned.data() + 1, &h, header);
        } else { BinaryProtocol3 h{}; h.payload_size = htons(3); memcpy(unaligned.data() + 1, &h, header); }
        memcpy(unaligned.data() + 1 + header, "abc", 3);
        auto original = unaligned;
        p.websocket_->receive(unaligned.data() + 1, header + 3, true);
        assert(unaligned == original && packet->payload == std::vector<uint8_t>({'a','b','c'}));
        assert(packet->timestamp == (version == 2 ? 1234u : 0u));
    }
    assert(received == 2);
    // Explicitly allocate no trailing NUL and parse an unterminated string.
    const char text[] = {'{','"','t','y','p','e','"',':','"','s','t','t','"','}'};
    p.websocket_->receive(text, sizeof(text), false); assert(json_received == 1);
    p.websocket_->receive(text, sizeof(text) - 2, false); assert(json_received == 1);

    for (const std::string& invalid : {
        std::string("{}"), changed(ws_hello, "\"websocket\"", "null"),
        changed(ws_hello, "\"version\":2", "\"version\":99"),
        changed(ws_hello, "24000", "44100"), changed(ws_hello, "60", "0"),
        changed(ws_hello, "\"session_id\":\"fresh\"", "\"session_id\":3")}) {
        xEventGroupClearBits(p.event_group_handle_, 1); p.session_id_ = "sentinel";
        parse_hello(p, invalid); assert(*p.event_group_handle_ == 0 && p.session_id_ == "sentinel");
    }
    parse_hello(p, changed(ws_hello, "60", "120")); assert(p.server_frame_duration_ == 120);
    // A stale hello bit/session cannot turn the next timed-out connection into success.
    xEventGroupSetBits(p.event_group_handle_, 1); ws_reply = false;
    assert(!p.OpenAudioChannel()); assert(!p.websocket_ && p.session_id_.empty() && ws_live == 0 && opened == 1);
    ws_reply = true; ws_connect = false;
    assert(!p.OpenAudioChannel() && ws_live == 0); ws_connect = true;
    ws_send = false; assert(!p.OpenAudioChannel() && ws_live == 0); ws_send = true;
    assert(p.OpenAudioChannel());
}
void test_mqtt() {
    MqttProtocol p;
    for (const std::string& invalid : {
        std::string("{}"), changed(mqtt_hello, "\"udp\"", "null"),
        changed(mqtt_hello, "9000", "0"), changed(mqtt_hello, "9000", "65536"),
        changed(mqtt_hello, "00112233445566778899aabbccddeeff", "0"),
        changed(mqtt_hello, "00112233445566778899aabbccddeeff", "g0112233445566778899aabbccddeeff"),
        changed(mqtt_hello, "01000000000000000000000000000000", ""),
        changed(mqtt_hello, "\"host\"", "false"),
        changed(mqtt_hello, "\"session_id\":\"fresh\"", "\"session_id\":\"fresh\",\"audio_params\":{\"sample_rate\":0}"),
        changed(mqtt_hello, "\"session_id\":\"fresh\"", "\"session_id\":\"fresh\",\"audio_params\":{\"frame_duration\":121}")}) {
        p.session_id_ = "sentinel"; parse_hello(p, invalid);
        assert(*p.event_group_handle_ == 0 && p.session_id_ == "sentinel" && !p.aes_ctx_.keyed);
    }
    assert(p.DecodeHexString("0").empty());
    int opened = 0, received = 0;
    p.OnAudioChannelOpened([&] { ++opened; });
    p.OnIncomingAudio([&](auto packet) { ++received; assert(packet->payload == std::vector<uint8_t>({'a','b','c'})); });
    udp_connect = false; assert(!p.OpenAudioChannel() && !p.udp_ && opened == 0 && udp_live == 0);
    udp_connect = true; assert(p.OpenAudioChannel() && opened == 1);
    for (size_t len = 0; len < 16; ++len) p.udp_->receive(std::string(len, 1));
    std::string data(16, '\0'); data[0] = 1; data[3] = 3; data[15] = 1; data += "abc";
    auto original = data;
    p.udp_->receive(data); assert(data == original && received == 1);
    p.udp_->receive(data); assert(received == 1); // duplicate sequence
    data[15] = 2; data[3] = 4; p.udp_->receive(data); assert(received == 1);
    data[3] = 0; p.udp_->receive(data); assert(received == 1);
    xEventGroupSetBits(p.event_group_handle_, 1); mqtt_reply = false;
    assert(!p.OpenAudioChannel() && !p.udp_ && p.session_id_.empty() && opened == 1);
    mqtt_reply = true;
    MqttProtocol failed; mqtt_connect = false;
    assert(!failed.OpenAudioChannel() && !failed.mqtt_); mqtt_connect = true;
}
void test_mqtt_reconnect() {
    MqttProtocol p;
    int errors = 0;
    p.OnNetworkError([&](auto&) { ++errors; });
    mqtt_connect = false;
    assert(!p.Start());
    auto timer = p.reconnect_timer_;
    assert(timer->active && timer->delay == 60000000 && errors == 0);
    int calls = mqtt_connect_calls;
    // Busy when the main task executes, even if idle when the timer fires.
    fire_timer(timer); app_state = 1; run_scheduled_tasks();
    assert(mqtt_connect_calls == calls && timer->active && timer->delay == 120000000);
    app_state = kDeviceStateIdle;
    for (uint64_t next_delay : {240000000ull, 480000000ull, 600000000ull, 600000000ull}) {
        fire_timer(timer); run_scheduled_tasks();
        assert(timer->active && timer->delay == next_delay && errors == 0);
    }
    // Multiple disconnect signals must not postpone or duplicate the active retry.
    uint64_t delay = timer->delay;
    p.ScheduleReconnect(); assert(timer->delay == delay);
    mqtt_connect = true;
    fire_timer(timer); run_scheduled_tasks();
    assert(p.mqtt_connected_ && !timer->active && p.reconnect_delay_ms_ == MQTT_RECONNECT_INTERVAL_MS);
    // An already queued retry must not replace a recovered connection.
    p.mqtt_->connected = false; p.mqtt_->on_disconnected();
    assert(timer->active && timer->delay == 60000000);
    fire_timer(timer);
    p.mqtt_->connected = true; p.mqtt_->on_connected();
    calls = mqtt_connect_calls; run_scheduled_tasks();
    assert(mqtt_connect_calls == calls && !timer->active);
    // The timer callback may already be dispatched when destruction cancels it.
    // It must perform no dereference of the deleted protocol; queued main work
    // must also be discarded through the shared lifetime guard.
    mqtt_connect = false;
    auto dying = std::make_unique<MqttProtocol>(); assert(!dying->Start());
    auto args = dying->reconnect_timer_->args;
    fire_timer(dying->reconnect_timer_); calls = mqtt_connect_calls;
    dying.reset();
    args.callback(args.arg);
    run_scheduled_tasks();
    assert(mqtt_connect_calls == calls);
    mqtt_connect = true;
}
int main() {
    test_websocket(); assert(ws_live == 0);
    test_mqtt(); assert(udp_live == 0);
    test_mqtt_reconnect(); assert(timer_live == 0 && reconnect_targets.empty());
    std::cout << "PASS: real WS/MQTT parsing, immutable/unaligned buffers, malformed hello, stale handshake, connection cleanup, bounded reconnect, and callback lifetime\n";
}
'''


def source(name):
    content = (ROOT / "main/protocols" / name).read_text(encoding="utf-8")
    return re.sub(r"^\s*#include[^\n]*\n|^#define TAG[^\n]*\n", "", content, flags=re.M)


def main():
    headers = "\n".join(source(name) for name in ("protocol.h", "websocket_protocol.h", "mqtt_protocol.h"))
    code = "\n".join(source(name) for name in ("protocol.cc", "websocket_protocol.cc", "mqtt_protocol.cc"))
    with tempfile.TemporaryDirectory(prefix="xiaozhi-protocol-test-") as directory:
        directory = Path(directory)
        cpp = directory / "protocol_test.cc"
        binary = directory / "protocol_test"
        cpp.write_text(PREAMBLE + "\n#define private public\n#define protected public\n" + headers
                       + "\n#undef private\n#undef protected\n" + code + TESTS, encoding="utf-8")
        subprocess.run(["g++", "-std=c++17", "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                        str(cpp), "-l:libcjson.so.1", "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
