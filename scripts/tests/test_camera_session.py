#!/usr/bin/env python3
"""Exercise production camera preparation/MCP methods and the real state machine.

Run with Python 3 and g++ (inside WSL on Windows). Network, timers, audio and
settings are host stubs; no ESP-IDF build, serial access or real credentials.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def definition(path, name):
    source = (ROOT / path).read_text(encoding="utf-8")
    hit = source.index(name + "(")
    start = source.rfind("\n", 0, hit) + 1
    brace = source.index("{", hit)
    depth = 0
    for token in re.finditer(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', source[brace:]):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[start:brace + token.end()]
    raise AssertionError(name)


HEAD = r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "esp_log.h"
#include "device_state_machine.h"
int64_t now_us = 1000000;
int64_t esp_timer_get_time() { return now_us; }
int esp_timer_stop(void*) { return 0; }
int esp_timer_start_periodic(void*, int64_t) { return 0; }
enum ListeningMode { kListeningModeManualStop, kListeningModeRealtime };
enum AbortReason { kAbortReasonNone, kAbortReasonWakeWordDetected };
enum AecMode { kAecOff, kAecOnDeviceSide };
struct cJSON {
    bool object = true;
    std::string value;
    std::map<std::string, cJSON> children;
};
cJSON* cJSON_GetObjectItem(const cJSON* j, const char* key) {
    if (!j) return nullptr;
    auto it = j->children.find(key);
    return it == j->children.end() ? nullptr : const_cast<cJSON*>(&it->second);
}
bool cJSON_IsObject(const cJSON* j) { return j && j->object; }
const char* cJSON_GetStringValue(const cJSON* j) { return j && !j->object ? j->value.c_str() : nullptr; }
struct Camera { int sets = 0; void SetExplainUrl(const std::string&, const std::string&) { ++sets; } };
struct Display {
    void SetStatus(const char*) {} void ClearChatMessages() {} void SetEmotion(const char*) {}
    void SetChatMessage(const char*, const char*) {} void UpdateStatusBar(bool) {}
};
struct Led { void OnStateChanged() {} };
struct Board {
    static Board& GetInstance() { static Board b; return b; }
    Camera camera; Display display; Led led;
    Camera* GetCamera() { return &camera; }
    Display* GetDisplay() { return &display; }
    Led* GetLed() { return &led; }
};
struct Settings { Settings(const char*, bool) {} void SetString(const char*, const std::string&) {} };
class McpServer {
public:
    static McpServer& GetInstance() { static McpServer m; return m; }
    std::atomic<uint32_t> vision_capability_revision_{0};
    uint32_t VisionCapabilityRevision() const { return vision_capability_revision_.load(); }
    void ParseCapabilities(const cJSON*);
};
namespace Lang::Strings {
const char* STANDBY="idle"; const char* CONNECTING="connecting";
const char* LISTENING="listening"; const char* SPEAKING="speaking";
}
namespace Lang::Sounds { const char* OGG_POPUP="popup"; }
struct AudioService {
    bool voice = false, wake = true, uplink = false, local = false, complete = true;
    int resets = 0, encoded_wake = 0;
    void SetKeepUplink(bool v) { uplink=v; }
    void EnableVoiceProcessing(bool v) { voice=v; }
    void EnableWakeWordDetection(bool v) { wake=v; }
    void ResetEncoder() { ++resets; } void ResetDecoder() { ++resets; }
    bool IsAudioProcessorRunning() { return voice; } bool IsAfeWakeWord() { return true; }
    bool IsLocalPlaybackActive() { return local; } bool IsPlaybackComplete() { return complete; }
    std::string GetLastWakeWord() { return "test"; } void EncodeWakeWord() { ++encoded_wake; }
    bool PopPacketFromSendQueue() { return false; } void PlaySound(const char*) {}
};
struct Protocol {
    bool opened = false, open_result = true;
    int opens=0, closes=0, aborts=0, listens=0;
    std::vector<std::string> texts;
    std::function<void()> on_open;
    bool IsAudioChannelOpened() { return opened; }
    bool OpenAudioChannel() { ++opens; if(on_open) on_open(); opened=open_result; return opened; }
    void CloseAudioChannel() { ++closes; opened=false; }
    void SendAbortSpeaking(AbortReason) { ++aborts; }
    void SendStopListening() {}
    void SendStartListening(ListeningMode) { ++listens; }
    void SendWakeWordDetected(const std::string& text) { texts.push_back(text); }
};
struct Application {
    DeviceStateMachine state_machine_;
    AudioService audio_service_;
    std::unique_ptr<Protocol> protocol_ = std::make_unique<Protocol>();
    std::atomic<uint32_t> conversation_generation_{0}, audio_channel_generation_{0};
    CAMERA_FIELDS
    std::atomic<bool> keep_alive_{false};
    bool network_connected_=true, aborted_=false, tts_completion_pending_=false;
    uint32_t tts_generation_=0, pending_tts_generation_=0;
    int64_t reconnect_at_ms_=0, vad_listen_start_ms_=0;
    int reconnect_delay_ms_=2000, clock_ticks_=0;
    bool play_popup_on_listening_=false, vad_speech_started_=false, vad_speech_pending_=false, tts_just_finished_=false;
    ListeningMode listening_mode_=kListeningModeRealtime;
    AecMode aec_mode_=kAecOff;
    void* playback_timer_handle_=nullptr;
    std::deque<std::function<void()>> tasks;
    std::thread::id main_thread = std::this_thread::get_id();
    Application() {
        state_machine_.TransitionTo(kDeviceStateStarting);
        state_machine_.TransitionTo(kDeviceStateActivating);
        state_machine_.TransitionTo(kDeviceStateIdle);
        protocol_->on_open = [this] { ++audio_channel_generation_; };
    }
    DeviceState GetDeviceState() const { return state_machine_.GetState(); }
    bool SetDeviceState(DeviceState state) { return state_machine_.TransitionTo(state); }
    void Schedule(std::function<void()>&& fn) { tasks.push_back(std::move(fn)); }
    void Drain() {
        assert(std::this_thread::get_id()==main_thread);
        int remaining=100;
        while (!tasks.empty()) { assert(--remaining>0); auto fn=std::move(tasks.front()); tasks.pop_front(); fn(); }
    }
    void PrepareCameraSession(std::function<void(bool)>);
    void HandleCameraPreparation(); void CompleteCameraPreparation(bool); void CancelCameraSession();
    void CancelTtsCompletion(); void HandlePlaybackProgress(); void EndConversation(); void ResetProtocol();
    void NotifySTT(const std::string&); void HandleNetworkDisconnectedEvent(); void HandleStateChangedEvent();
    void HandleWakeWordDetectedEvent(); void ContinueWakeWordInvoke(const std::string&);
    void SetListeningMode(ListeningMode);
    ListeningMode GetDefaultListeningMode() { return kListeningModeRealtime; }
    void AbortSpeaking(AbortReason) {} void ScheduleReconnect() {}
    void IncomingTts(const std::string&);
};
void fresh_vision() {
    cJSON caps;
    caps.children["vision"].children["url"] = cJSON{false,"test-only",{}};
    caps.children["vision"].children["token"] = cJSON{false,"test-only",{}};
    McpServer::GetInstance().ParseCapabilities(&caps);
}
'''

TESTS = r'''
int main() {
    { // Only valid live capabilities advance readiness; restored values and invalid MCP cannot.
        auto& m=McpServer::GetInstance(); auto rev=m.VisionCapabilityRevision();
        Board::GetInstance().camera.SetExplainUrl("test-only","test-only");
        m.ParseCapabilities(nullptr); cJSON invalid; m.ParseCapabilities(&invalid);
        invalid.children["vision"].children["url"]=cJSON{false,"",{}}; m.ParseCapabilities(&invalid);
        assert(m.VisionCapabilityRevision()==rev); fresh_vision(); assert(m.VisionCapabilityRevision()==rev+1);
        std::cout << "PASS live vision revision versus restored/invalid configuration\n";
    }
    { // Hello alone and a previous boot's revision cannot unlock upload; main queue keeps draining.
        Application a; int calls=0; bool ready=false;
        a.PrepareCameraSession([&](bool ok){++calls;ready=ok;assert(std::this_thread::get_id()==a.main_thread);});
        assert(calls==0); a.Drain(); assert(calls==0 && a.protocol_->opens==1);
        a.HandleStateChangedEvent(); assert(!a.audio_service_.voice && !a.audio_service_.wake && !a.audio_service_.uplink);
        bool replied=false; a.Schedule([&]{replied=true;}); a.Drain(); assert(replied && !ready);
        a.HandleCameraPreparation(); assert(calls==0); fresh_vision(); a.HandleCameraPreparation();
        assert(calls==1 && ready && a.protocol_->opened && a.camera_result_pending_);
        a.HandleCameraPreparation(); assert(calls==1);
        std::cout << "PASS asynchronous hello/vision ordering and exactly-once main callback\n";
    }
    { // Fresh update can arrive before OpenAudioChannel returns; next photo reuses the same connection.
        Application a; int calls=0;
        a.protocol_->on_open=[&]{++a.audio_channel_generation_;fresh_vision();};
        a.PrepareCameraSession([&](bool ok){assert(ok);++calls;}); a.Drain(); assert(calls==1);
        a.NotifySTT("result"); a.Drain(); assert(a.protocol_->opens==1 && !a.camera_result_pending_);
        a.IncomingTts("start"); a.Drain(); assert(a.GetDeviceState()==kDeviceStateSpeaking);
        a.HandleStateChangedEvent(); assert(!a.audio_service_.voice && !a.audio_service_.wake);
        a.PrepareCameraSession([&](bool ok){assert(ok);++calls;}); a.Drain();
        assert(calls==2 && a.protocol_->opens==1 && a.protocol_->aborts==1 && a.camera_result_pending_);
        a.IncomingTts("start"); a.Drain(); assert(a.GetDeviceState()==kDeviceStateIdle);
        a.HandleWakeWordDetectedEvent(); a.ContinueWakeWordInvoke("late");
        assert(a.audio_service_.encoded_wake==0 && a.protocol_->listens==0);
        std::cout << "PASS reused camera session, old TTS cancellation and wake/microphone isolation\n";
    }
    { // Waiting session times out and closes only its own connection.
        Application a; int calls=0; now_us=1000000;
        a.PrepareCameraSession([&](bool ok){assert(!ok);++calls;}); a.Drain();
        now_us=11000000; a.HandleCameraPreparation(); assert(calls==1 && !a.protocol_->opened);
        a.HandleCameraPreparation(); assert(calls==1 && !a.camera_session_active_);
        std::cout << "PASS bounded capability timeout and cleanup\n";
    }
    { // End before scheduling, during hello, and while waiting never reports success.
        for(int phase=0;phase<3;++phase) {
            Application a; int calls=0;
            if(phase==1) a.protocol_->on_open=[&]{++a.audio_channel_generation_;a.EndConversation();fresh_vision();};
            a.PrepareCameraSession([&](bool ok){assert(!ok);++calls;});
            if(phase==0) a.EndConversation();
            a.Drain(); if(phase==2){a.EndConversation();a.Drain();}
            assert(calls==1 && !a.protocol_->opened && !a.camera_session_active_);
            fresh_vision();a.HandleCameraPreparation();assert(calls==1);
        }
        std::cout << "PASS cancellation before/during/after transport open\n";
    }
    { // A cancelled request's queued cleanup cannot close a newer camera session.
        Application a; int old_calls=0,new_calls=0;
        a.PrepareCameraSession([&](bool ok){assert(!ok);++old_calls;});a.Drain();
        a.EndConversation();
        a.protocol_->on_open=[&]{++a.audio_channel_generation_;fresh_vision();};
        a.PrepareCameraSession([&](bool ok){assert(ok);++new_calls;});a.Drain();
        assert(old_calls==1 && new_calls==1 && a.protocol_->opened && a.camera_session_ready_);
        std::cout << "PASS stale cancellation cannot close replacement session\n";
    }
    { // Offline/reset completes failure exactly once and cannot leave a delayed upload callback.
        for(bool reset : {false,true}) {
            Application a;int calls=0;
            a.PrepareCameraSession([&](bool ok){assert(!ok);++calls;});a.Drain();
            if(reset){a.ResetProtocol();a.Drain();}else a.HandleNetworkDisconnectedEvent();
            fresh_vision();a.HandleCameraPreparation();assert(calls==1 && !a.camera_session_active_);
            assert(!a.protocol_ || !a.protocol_->opened);
        }
        std::cout << "PASS network loss/protocol reset cancellation\n";
    }
    { // Real state machine rejects Starting -> Idle; ending a camera request preserves activation.
        for(DeviceState state : {kDeviceStateStarting,kDeviceStateActivating,kDeviceStateWifiConfiguring}) {
            Application a; DeviceStateMachine machine;
            assert(machine.TransitionTo(kDeviceStateStarting));
            assert(!machine.TransitionTo(kDeviceStateIdle));
            if(state==kDeviceStateStarting) {
                // Start a second real state machine in the desired state by placement reconstruction.
                a.state_machine_.~DeviceStateMachine();new(&a.state_machine_) DeviceStateMachine;
                assert(a.SetDeviceState(kDeviceStateStarting));
            } else assert(a.SetDeviceState(state));
            a.EndConversation();a.Drain();assert(a.GetDeviceState()==state);
            int calls=0;a.PrepareCameraSession([&](bool ok){assert(ok);++calls;});a.Drain();
            assert(calls==0 && a.GetDeviceState()==state && a.protocol_->opens==0);
            if(state!=kDeviceStateActivating)assert(a.SetDeviceState(kDeviceStateActivating));
            assert(a.SetDeviceState(kDeviceStateIdle));
            a.protocol_->on_open=[&]{++a.audio_channel_generation_;fresh_vision();};
            a.HandleCameraPreparation();assert(calls==1);
        }
        std::cout << "PASS production state machine and activation readiness\n";
    }
    { // Open failure and an expired reuse window cause a fresh bounded handshake.
        Application a;int failures=0;a.protocol_->open_result=false;
        a.PrepareCameraSession([&](bool ok){assert(!ok);++failures;});a.Drain();assert(failures==1);
        a.protocol_->open_result=true;a.protocol_->on_open=[&]{++a.audio_channel_generation_;fresh_vision();};
        a.PrepareCameraSession([](bool ok){assert(ok);});a.Drain();int opens=a.protocol_->opens;
        now_us+=10LL*60*1000000;a.PrepareCameraSession([](bool ok){assert(ok);});a.Drain();
        assert(a.protocol_->opens==opens+1);
        std::cout << "PASS open failure and fresh handshake after reuse deadline\n";
    }
    { // A late successful hello still observes the deadline; disconnect preserves activation.
        Application a;int calls=0;
        a.protocol_->on_open=[&]{++a.audio_channel_generation_;now_us+=11000000;fresh_vision();};
        a.PrepareCameraSession([&](bool ok){assert(!ok);++calls;});a.Drain();
        assert(calls==1 && !a.protocol_->opened);
        Application b;assert(b.SetDeviceState(kDeviceStateActivating));int cancelled=0;
        b.PrepareCameraSession([&](bool ok){assert(!ok);++cancelled;});b.Drain();
        b.HandleNetworkDisconnectedEvent();assert(cancelled==1 && b.GetDeviceState()==kDeviceStateActivating);
        std::cout << "PASS delayed hello deadline and offline activation state\n";
    }
    { // Result playback finishes in idle with the microphone/wake detector still off.
        Application a;a.protocol_->on_open=[&]{++a.audio_channel_generation_;fresh_vision();};
        a.PrepareCameraSession([](bool ok){assert(ok);});a.Drain();a.NotifySTT("result");a.Drain();
        a.IncomingTts("start");a.Drain();a.IncomingTts("stop");a.Drain();
        a.HandleStateChangedEvent();assert(a.GetDeviceState()==kDeviceStateIdle);
        assert(!a.audio_service_.voice && !a.audio_service_.wake && a.protocol_->listens==0);
        a.EndConversation();a.Drain();assert(a.audio_service_.wake && !a.camera_session_active_);
        std::cout << "PASS result TTS completion and explicit exit restores wake word\n";
    }
}
'''

header = (ROOT / "main/application.h").read_text(encoding="utf-8")
camera_fields = header[header.index("    std::atomic<uint32_t> camera_request_generation_"):header.index("    uint32_t tts_generation_")]
parts = [HEAD.replace("    CAMERA_FIELDS", camera_fields)]
for name in ["CancelCameraSession", "CompleteCameraPreparation", "PrepareCameraSession", "HandleCameraPreparation", "CancelTtsCompletion", "HandlePlaybackProgress", "EndConversation", "ResetProtocol", "NotifySTT", "HandleNetworkDisconnectedEvent", "HandleStateChangedEvent", "HandleWakeWordDetectedEvent", "ContinueWakeWordInvoke", "SetListeningMode"]:
    parts.append(definition("main/application.cc", "Application::" + name))
parts.append(definition("main/mcp_server.cc", "McpServer::ParseCapabilities"))
source = (ROOT / "main/application.cc").read_text(encoding="utf-8")
start = source.index('if (strcmp(state->valuestring, "start") == 0) {')
end = source.index('} else if (strcmp(state->valuestring, "sentence_start") == 0)', start)
parts.append('void Application::IncomingTts(const std::string& input) { if(camera_result_pending_)return; struct { const char* valuestring; } value{input.c_str()};auto* state=&value;' + source[start:end] + '}}')
parts.append(TESTS)
with tempfile.TemporaryDirectory(prefix="xiaozhi-camera-session-") as directory:
    temp = Path(directory)
    (temp / "esp_log.h").write_text('#pragma once\n#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGE(...) ((void)0)\n')
    cpp = temp / "test.cc"
    cpp.write_text("\n".join(parts), encoding="utf-8")
    binary = temp / "test"
    subprocess.run(["g++", "-std=c++20", "-O1", "-g", "-pthread", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-I", str(temp), "-I", str(ROOT / "main"), str(cpp), str(ROOT / "main/device_state_machine.cc"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=20)
