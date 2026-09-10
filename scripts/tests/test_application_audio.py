"""Test current production method bodies against host hardware stubs.

Requires Python 3 and a host g++ supporting C++20. This does not invoke ESP-IDF.
Run from the repository root on Linux, or inside WSL on Windows:
    python3 scripts/tests/test_application_audio.py
An optional repository root argument selects another checkout. All generated
C++ and executables are created in a temporary directory and removed afterward.
"""
import pathlib
import re
import subprocess
import sys
import tempfile

repo = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[2]

def method(path, name):
    source = (repo / path).read_text(encoding="utf-8")
    hit = source.index(name + "(")
    begin = source.rfind("\n", 0, hit) + 1
    brace = source.index("{", hit)
    depth = 0
    tokens = re.finditer(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', source[brace:])
    for token in tokens:
        if token.group() == "{": depth += 1
        if token.group() == "}": depth -= 1
        if depth == 0:
            return source[begin:brace + token.end()]
    raise AssertionError(name)

head = r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define CONFIG_USE_SERVER_AEC 0
#define MAX_PLAYBACK_TASKS_IN_QUEUE 8
#define MAX_SEND_PACKETS_IN_QUEUE 30
#define MAX_DECODE_PACKETS_IN_QUEUE 30
#define MAX_ENCODE_TASKS_IN_QUEUE 8
#define MAX_TIMESTAMPS_IN_QUEUE 3
#define OPUS_FRAME_DURATION_MS 20
#define AUDIO_POWER_CHECK_INTERVAL_MS 1000
#define MAIN_EVENT_VAD_CHANGE 8
#define ESP_AUDIO_ERR_OK 0
#define ESP_AUDIO_ERR_FAIL -1
#define ESP_AUDIO_DEC_RECOVERY_NONE 0
int64_t fake_time_us = 1000000;
int64_t esp_timer_get_time() { return fake_time_us; }
int esp_timer_stop(void*) { return 0; }
int esp_timer_start_periodic(void*, int64_t) { return 0; }
using esp_ae_sample_t = void*;
struct esp_audio_dec_in_raw_t { uint8_t* buffer; uint32_t len; int consumed; int frame_recover; };
struct esp_audio_dec_out_frame_t { uint8_t* buffer; uint32_t len; uint32_t decoded_size; };
struct esp_audio_dec_info_t {};
struct esp_audio_enc_in_frame_t { uint8_t* buffer; uint32_t len; };
struct esp_audio_enc_out_frame_t { uint8_t* buffer; uint32_t len; uint32_t encoded_bytes; };
std::function<void()> encode_hook;
std::atomic<int> decode_calls{0};
int esp_opus_dec_decode(void*, esp_audio_dec_in_raw_t*, esp_audio_dec_out_frame_t* out, esp_audio_dec_info_t*) { ++decode_calls; out->decoded_size = 4; return 0; }
int esp_opus_enc_process(void*, esp_audio_enc_in_frame_t*, esp_audio_enc_out_frame_t* out) { if (encode_hook) encode_hook(); out->encoded_bytes = 1; return 0; }
int esp_opus_dec_reset(void*) { return 0; }
void esp_ae_rate_cvt_get_max_out_sample_num(void*, size_t n, uint32_t* out) { *out = n; }
void esp_ae_rate_cvt_process(void*, void*, size_t, void*, uint32_t*) {}
enum AudioTaskType { kAudioTaskTypeEncodeToSendQueue, kAudioTaskTypeEncodeToTestingQueue, kAudioTaskTypeDecodeToPlaybackQueue, kAudioTaskTypeLocalPlayback };
struct AudioTask { AudioTaskType type; std::vector<int16_t> pcm; uint32_t timestamp = 0; };
struct AudioStreamPacket { int sample_rate = 16000; int frame_duration = 20; uint32_t timestamp = 0; std::vector<uint8_t> payload; };
struct FakeCodec {
    std::function<void()> output_hook;
    bool output_enabled() { return true; }
    int output_sample_rate() { return 16000; }
    void EnableOutput(bool) {}
    void OutputData(std::vector<int16_t>&) { if (output_hook) output_hook(); }
};
struct AudioService {
    std::mutex audio_queue_mutex_, decoder_mutex_, output_mutex_;
    std::condition_variable audio_queue_cv_;
    std::deque<std::unique_ptr<AudioTask>> audio_encode_queue_, audio_playback_queue_;
    std::deque<std::unique_ptr<AudioStreamPacket>> audio_decode_queue_, audio_send_queue_, audio_testing_queue_;
    std::deque<uint32_t> timestamp_queue_;
    std::atomic<uint32_t> decode_generation_{0}; uint32_t encode_generation_ = 0;
    std::atomic<uint32_t> local_playback_token_{0}; bool local_playback_paused_ = false;
    bool IsLocalPlaybackActive() const { return local_playback_token_.load() != 0; }
    std::atomic<bool> service_stopped_{false}, keep_uplink_{false};
    bool decoding_ = false, playing_ = false, processing = false, speaking = false;
    std::chrono::steady_clock::time_point last_output_time_ = std::chrono::steady_clock::now() - 1s;
    void *opus_decoder_ = this, *opus_encoder_ = this, *output_resampler_ = nullptr, *audio_power_timer_ = nullptr;
    int decoder_sample_rate_ = 16000, decoder_frame_size_ = 320, encoder_frame_size_ = 320, encoder_outbuf_size_ = 1024;
    struct { std::atomic<int> encode_count{0}, decode_count{0}, playback_count{0}; } debug_statistics_;
    struct { std::function<void()> on_send_queue_available; } callbacks_;
    FakeCodec fake_codec; FakeCodec* codec_ = &fake_codec;
    std::function<void()> sample_rate_hook;
    void SetDecodeSampleRate(int,int) { if (sample_rate_hook) sample_rate_hook(); }
    void EnableVoiceProcessing(bool p) { processing = p; }
    void EnableWakeWordDetection(bool) {}
    bool IsAudioProcessorRunning() { return processing; }
    bool IsVoiceDetected() { return speaking; }
    void PlaySound(const char*) {}
    void SetKeepUplink(bool p) { keep_uplink_ = p; }
    void ResetDecoder(); void ResetEncoder(); bool IsPlaybackComplete();
    void OpusCodecTask(); void AudioOutputTask();
    bool PushPacketToDecodeQueue(std::unique_ptr<AudioStreamPacket>,bool);
    bool EnqueueDecodePacket(std::unique_ptr<AudioStreamPacket>,bool,uint32_t);
    void PushTaskToEncodeQueue(AudioTaskType,std::vector<int16_t>&&);
};
enum DeviceState { kDeviceStateIdle, kDeviceStateConnecting, kDeviceStateListening, kDeviceStateSpeaking, kDeviceStateStarting, kDeviceStateActivating, kDeviceStateWifiConfiguring };
enum ListeningMode { kListeningModeManualStop, kListeningModeRealtime };
enum AbortReason { kAbortReasonNone };
struct FakeLed { void OnStateChanged() {} };
struct Board {
    static Board& GetInstance() { static Board b; return b; }
    FakeLed* GetLed() { static FakeLed led; return &led; }
};
namespace Lang::Sounds { const char* OGG_POPUP = "popup"; }
struct Protocol {
    bool opened = false, open_result = true; int opens = 0, closes = 0, aborts = 0;
    std::vector<std::string> texts; std::function<void()> open_hook;
    bool IsAudioChannelOpened() { return opened; }
    bool OpenAudioChannel() { ++opens; if(open_hook) open_hook(); opened = open_result; return opened; }
    void CloseAudioChannel() { ++closes; opened = false; }
    void SendAbortSpeaking(AbortReason) { ++aborts; }
    void SendWakeWordDetected(const std::string& t) { texts.push_back(t); }
};
struct Application {
    AudioService audio_service_; std::unique_ptr<Protocol> protocol_ = std::make_unique<Protocol>();
    std::atomic<bool> keep_alive_{false}; bool network_connected_ = false, aborted_ = false;
    int64_t reconnect_at_ms_ = 0, audio_channel_opened_ms_ = 0;
    int reconnect_delay_ms_ = 2000, open_requests = 0;
    std::atomic<uint32_t> audio_channel_generation_{0}, conversation_generation_{0};
    std::atomic<uint32_t> camera_request_generation_{0};
    std::atomic<bool> camera_session_active_{false}, camera_result_pending_{false};
    std::function<void(bool)> camera_prepare_callback_;
    bool camera_session_ready_ = false, camera_open_started_ = false;
    void CancelCameraSession();
    uint32_t tts_generation_ = 0, pending_tts_generation_ = 0;
    bool tts_completion_pending_ = false, tts_just_finished_ = false;
    bool vad_speech_pending_ = false, vad_speech_started_ = false;
    int64_t vad_listen_start_ms_ = 0; int stop_requests = 0;
    void* playback_timer_handle_ = nullptr;
    ListeningMode listening_mode_ = kListeningModeRealtime;
    DeviceState state = kDeviceStateIdle;
    std::deque<std::function<void()>> tasks;
    void Schedule(std::function<void()>&& f) { tasks.push_back(std::move(f)); }
    void Drain() { while (!tasks.empty()) { auto f = std::move(tasks.front()); tasks.pop_front(); f(); } }
    DeviceState GetDeviceState() { return state; }
    void SetDeviceState(DeviceState s) { state = s; }
    void HandleToggleChatEvent() { ++open_requests; }
    void ScheduleReconnect(); void HandleReconnect(); void CancelTtsCompletion(); void HandlePlaybackProgress();
    void NotifySTT(const std::string&); void EndConversation(); void AbortSpeaking(AbortReason); void SetKeepAlive(bool);
    void IncomingTts(const std::string&);
    void VadChange(); void StopListening() { ++stop_requests; }
};
'''

tests = r'''
struct Gate {
    std::promise<void> entered, release;
    std::shared_future<void> release_future = release.get_future().share();
    void Wait() { entered.set_value(); release_future.wait(); }
    void Arrived() { assert(entered.get_future().wait_for(2s) == std::future_status::ready); }
};
void stop_worker(AudioService& a, std::thread& t) { a.service_stopped_ = true; a.audio_queue_cv_.notify_all(); t.join(); }
int main() {
    { // Retry debounce, exponential cap, offline suppression and immediate user cancellation.
        Application a; a.ScheduleReconnect(); assert(a.reconnect_at_ms_ == 0);
        a.keep_alive_ = true; a.ScheduleReconnect(); assert(a.reconnect_at_ms_ == 0);
        a.network_connected_ = true; a.ScheduleReconnect(); assert(a.reconnect_at_ms_ == 3000);
        a.ScheduleReconnect(); assert(a.reconnect_delay_ms_ == 4000);
        fake_time_us = 2999000; a.HandleReconnect(); assert(a.open_requests == 0);
        fake_time_us = 3000000; a.HandleReconnect(); assert(a.open_requests == 1);
        a.ScheduleReconnect(); assert(a.reconnect_at_ms_ == 7000);
        for(int i=0;i<10;i++) { a.reconnect_at_ms_=0; a.ScheduleReconnect(); }
        assert(a.reconnect_delay_ms_ == 60000);
        a.network_connected_=false; fake_time_us=1000000000; a.HandleReconnect(); assert(a.open_requests==1);
        a.network_connected_=true; a.EndConversation(); a.HandleReconnect(); assert(a.open_requests==1);
        a.Drain(); assert(!a.keep_alive_ && a.reconnect_at_ms_==0);
        std::cout << "PASS reconnect backoff/offline/cancel\n";
    }
    { // Old TTS completions cannot reset a newer response; finishing is idempotent.
        Application a; a.state=kDeviceStateSpeaking; a.tts_generation_=2;
        a.pending_tts_generation_=1; a.tts_completion_pending_=true;
        a.HandlePlaybackProgress(); assert(a.state==kDeviceStateSpeaking && !a.tts_completion_pending_);
        a.pending_tts_generation_=a.tts_generation_; a.tts_completion_pending_=true;
        a.audio_service_.playing_=true; a.HandlePlaybackProgress(); assert(a.tts_completion_pending_);
        a.audio_service_.playing_=false; a.listening_mode_=kListeningModeManualStop;
        a.HandlePlaybackProgress(); assert(a.state==kDeviceStateIdle);
        auto generation=a.tts_generation_; a.HandlePlaybackProgress(); assert(a.tts_generation_==generation);
        a.state=kDeviceStateSpeaking; a.listening_mode_=kListeningModeRealtime;
        a.AbortSpeaking(kAbortReasonNone); assert(a.state==kDeviceStateListening && a.aborted_);
        std::cout << "PASS TTS stale generation/drain/idempotence/abort\n";
    }
    { // Photo text never activates mic input; a failed open cannot retain stale text.
        Application a; a.audio_service_.processing=true;
        a.audio_service_.audio_send_queue_.push_back(std::make_unique<AudioStreamPacket>());
        a.NotifySTT("photo"); a.Drain();
        assert(a.state==kDeviceStateIdle && !a.audio_service_.processing);
        assert(a.audio_service_.audio_send_queue_.empty() && a.protocol_->texts==std::vector<std::string>{"photo"});
        a.protocol_->opened=false; a.protocol_->open_result=false;
        a.NotifySTT("failed"); a.Drain(); assert(a.state==kDeviceStateIdle && a.protocol_->texts.size()==1);
        a.protocol_->open_result=true; a.protocol_->OpenAudioChannel(); assert(a.protocol_->texts.size()==1);
        std::cout << "PASS photo text-only/no stale resend\n";
    }
    { // Execute the production JSON callback bodies, including generation capture.
        Application a; a.protocol_->opened=true;
        a.IncomingTts("start"); a.EndConversation(); a.Drain();
        assert(a.state==kDeviceStateIdle);
        a.protocol_->opened=true; a.EndConversation(); a.IncomingTts("start"); a.Drain();
        assert(a.state==kDeviceStateIdle);
        a.protocol_->opened=true; a.IncomingTts("start"); a.Drain();
        assert(a.state==kDeviceStateSpeaking);
        a.audio_service_.playing_=true;
        a.IncomingTts("stop"); a.IncomingTts("stop"); a.Drain();
        assert(a.tts_completion_pending_);
        auto generation=a.tts_generation_;
        a.IncomingTts("stop"); a.Drain(); assert(a.tts_generation_==generation);
        a.IncomingTts("start"); a.Drain();
        assert(!a.tts_completion_pending_ && a.tts_generation_!=generation);
        a.audio_service_.playing_=false; a.HandlePlaybackProgress(); assert(a.state==kDeviceStateSpeaking);
        std::cout << "PASS queued TTS cancellation/duplicate stop/new response\n";
    }
    { // User leaves before the queued notification runs, or during blocking channel open.
        Application a; a.NotifySTT("cancel-before"); a.EndConversation(); a.Drain();
        assert(a.protocol_->texts.empty() && a.protocol_->opens==0);
        a.protocol_->open_hook=[&]{a.EndConversation();};
        a.NotifySTT("cancel-during-open"); a.Drain();
        assert(a.protocol_->texts.empty() && !a.protocol_->opened && a.state==kDeviceStateIdle);
        std::cout << "PASS photo cancellation before/during open\n";
    }
    { // A speech edge inside warmup must survive until silence after warmup; manual hold remains manual.
        Application a; a.state=kDeviceStateListening; a.vad_listen_start_ms_=1000;
        fake_time_us=2200000; a.audio_service_.speaking=true; a.VadChange();
        assert(!a.vad_speech_started_ && a.stop_requests==0);
        fake_time_us=2800000; a.audio_service_.speaking=false; a.VadChange(); assert(a.stop_requests==1);
        a.listening_mode_=kListeningModeManualStop;
        fake_time_us=3000000; a.audio_service_.speaking=true; a.VadChange();
        fake_time_us=3500000; a.audio_service_.speaking=false; a.VadChange(); assert(a.stop_requests==1);
        a.listening_mode_=kListeningModeRealtime; a.keep_alive_=true;
        a.audio_service_.speaking=true; a.VadChange(); a.audio_service_.speaking=false; a.VadChange();
        assert(a.stop_requests==1);
        std::cout << "PASS VAD warmup speech/manual hold/call silence\n";
    }
    { // Queue emptiness excludes a frame already handed to the I2S output call.
        AudioService a; Gate gate; a.fake_codec.output_hook=[&]{gate.Wait();};
        a.audio_playback_queue_.push_back(std::make_unique<AudioTask>());
        std::thread worker([&]{a.AudioOutputTask();}); gate.Arrived();
        assert(a.audio_playback_queue_.empty() && !a.IsPlaybackComplete());
        gate.release.set_value();
        { std::unique_lock<std::mutex> l(a.audio_queue_mutex_); a.audio_queue_cv_.wait_for(l,2s,[&]{return !a.playing_;}); }
        assert(!a.IsPlaybackComplete());
        { std::lock_guard<std::mutex> l(a.audio_queue_mutex_); a.last_output_time_=std::chrono::steady_clock::now()-201ms; }
        assert(a.IsPlaybackComplete()); stop_worker(a,worker);
        std::cout << "PASS playback in-flight/DMA-tail boundary\n";
    }
    { // Reset during a popped-but-not-decoded frame must invalidate decoder history and output.
        AudioService a; Gate gate; a.sample_rate_hook=[&]{gate.Wait();};
        a.audio_decode_queue_.push_back(std::make_unique<AudioStreamPacket>());
        auto before=decode_calls.load(); std::thread worker([&]{a.OpusCodecTask();}); gate.Arrived();
        assert(!a.IsPlaybackComplete()); a.ResetDecoder(); gate.release.set_value();
        { std::unique_lock<std::mutex> l(a.audio_queue_mutex_); assert(a.audio_queue_cv_.wait_for(l,2s,[&]{return !a.decoding_;})); }
        assert(a.audio_playback_queue_.empty() && decode_calls==before); stop_worker(a,worker);
        std::cout << "PASS decoder reset invalidates in-flight frame\n";
    }
    { // ResetEncoder during an encode call must prevent that frame from reaching the send queue.
        AudioService a; Gate gate; encode_hook=[&]{gate.Wait();};
        auto task=std::make_unique<AudioTask>(); task->type=kAudioTaskTypeEncodeToSendQueue; task->pcm.resize(320);
        a.audio_encode_queue_.push_back(std::move(task));
        std::thread worker([&]{a.OpusCodecTask();}); gate.Arrived();
        a.ResetEncoder(); gate.release.set_value();
        auto deadline=std::chrono::steady_clock::now()+2s;
        while(a.debug_statistics_.encode_count.load()==0 && std::chrono::steady_clock::now()<deadline) std::this_thread::sleep_for(1ms);
        assert(a.debug_statistics_.encode_count.load()==1);
        { std::lock_guard<std::mutex> l(a.audio_queue_mutex_); assert(a.audio_send_queue_.empty()); }
        stop_worker(a,worker); encode_hook={};
        std::cout << "PASS encoder reset drops in-flight uplink\n";
    }
    { // A producer blocked on a full downlink queue is released by service shutdown.
        AudioService a; for(int i=0;i<30;i++) a.audio_decode_queue_.push_back(std::make_unique<AudioStreamPacket>());
        auto waiter=std::async(std::launch::async,[&]{return a.PushPacketToDecodeQueue(std::make_unique<AudioStreamPacket>(),true);});
        a.service_stopped_=true; a.audio_queue_cv_.notify_all(); assert(!waiter.get());
        std::cout << "PASS decode backpressure shutdown\n";
    }
    { // Use the production AudioTask declaration and enqueue function for timestamp defaults.
        AudioTask direct; assert(direct.timestamp==0);
        AudioService a; a.processing=true;
        a.PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue,std::vector<int16_t>(320));
        assert(a.audio_encode_queue_.front()->timestamp==0);
        a.timestamp_queue_.push_back(1234);
        a.PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue,std::vector<int16_t>(320));
        assert(a.audio_encode_queue_.back()->timestamp==1234);
        a.timestamp_queue_={1,2,3,4};
        a.PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue,std::vector<int16_t>(320));
        assert(a.audio_encode_queue_.back()->timestamp==0);
        std::cout << "PASS default/missing/overflow AEC timestamp\n";
    }
}
'''

audio_header = (repo / "main/audio/audio_service.h").read_text(encoding="utf-8")
audio_task = re.search(r"struct AudioTask\s*\{[\s\S]*?\n\};", audio_header).group()
head = re.sub(r"struct AudioTask \{[^\n]*\};", lambda _: audio_task, head)
parts = [head]
for name in ["ResetDecoder", "ResetEncoder", "IsPlaybackComplete", "OpusCodecTask", "AudioOutputTask", "PushPacketToDecodeQueue", "EnqueueDecodePacket", "PushTaskToEncodeQueue"]:
    parts.append(method("main/audio/audio_service.cc", "AudioService::" + name))
for name in ["ScheduleReconnect", "HandleReconnect", "CancelTtsCompletion", "HandlePlaybackProgress", "NotifySTT", "EndConversation", "AbortSpeaking", "SetKeepAlive", "CancelCameraSession"]:
    parts.append(method("main/application.cc", "Application::" + name))
application_source = (repo / "main/application.cc").read_text(encoding="utf-8")
start = application_source.index('if (strcmp(state->valuestring, "start") == 0) {')
end = application_source.index('} else if (strcmp(state->valuestring, "sentence_start") == 0)', start)
parts.append('void Application::IncomingTts(const std::string& input) { struct { const char* valuestring; } value{input.c_str()}; auto* state=&value; ' + application_source[start:end] + '}}')
start = application_source.index('if (bits & MAIN_EVENT_VAD_CHANGE) {')
end = application_source.index('if (bits & MAIN_EVENT_SCHEDULE)', start)
parts.append('void Application::VadChange() { const int bits=MAIN_EVENT_VAD_CHANGE; ' + application_source[start:end] + '}')
parts.append(tests)
with tempfile.TemporaryDirectory(prefix="xiaozhi-audio-host-") as directory:
    directory = pathlib.Path(directory)
    source = directory / "regression.cc"
    source.write_text("\n".join(parts), encoding="utf-8")
    binary = directory / "regression"
    subprocess.run(["g++", "-std=c++20", "-O1", "-pthread", "-Wall", "-Wextra", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=15)
