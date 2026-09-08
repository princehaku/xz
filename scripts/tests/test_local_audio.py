"""Exercise production local-audio methods with concurrent host hardware stubs.

Run with Python 3 and g++ (or WSL); no ESP-IDF build or device is required.
"""
import ast
import pathlib
import re
import subprocess
import sys
import tempfile

repo = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[2]
base = ast.parse((repo / "scripts/tests/test_application_audio.py").read_text(encoding="utf-8"))
# Reuse the existing hardware/Opus stubs without executing its separate suite.
head = next(ast.literal_eval(node.value) for node in base.body if isinstance(node, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "head" for t in node.targets))
extractor = next(node for node in base.body if isinstance(node, ast.FunctionDef) and node.name == "method")
exec(compile(ast.Module(body=[extractor], type_ignores=[]), "method-extractor", "exec"))

head = head.replace("#define ESP_LOGI", "#define ESP_LOGD(...) ((void)0)\n#define ESP_LOGI", 1)
head = head.replace("struct FakeCodec {", r'''
struct FakeCodec {
    std::mutex writes_mutex;
    std::vector<int16_t> writes;
    int output_channels() { return 1; }
''', 1)
head = head.replace("void OutputData(std::vector<int16_t>&) { if (output_hook) output_hook(); }", r'''
    void OutputData(std::vector<int16_t>& data) {
        if (output_hook) output_hook();
        std::lock_guard<std::mutex> lock(writes_mutex);
        writes.push_back(data.empty() ? 0 : data.front());
    }
''')
head = head.replace("struct AudioService {", r'''
#define AS_EVENT_AUDIO_TESTING_RUNNING 1
#define AS_EVENT_WAKE_WORD_RUNNING 2
#define AS_EVENT_AUDIO_PROCESSOR_RUNNING 4
using EventGroupHandle_t = std::atomic<unsigned>*;
void xEventGroupSetBits(EventGroupHandle_t e, unsigned bits) { *e |= bits; }
void xEventGroupClearBits(EventGroupHandle_t e, unsigned bits) { *e &= ~bits; }
void esp_ae_rate_cvt_reset(void*) {}
struct FakeProcessor {
    bool running = false;
    bool Initialize(FakeCodec*, int, void*) { return true; }
    void Start() { running = true; }
    void Stop() { running = false; }
};
struct FakeWakeWord : FakeProcessor {
    static inline std::function<void()> construct_hook;
    FakeWakeWord() { if (construct_hook) construct_hook(); }
    bool Initialize(FakeCodec*, void*) { return true; }
    void OnWakeWordDetected(std::function<void(const std::string&)>) {}
};
using EspWakeWord = FakeWakeWord;
using srmodel_list_t = int;
#define ESP_WN_PREFIX "wake"
void* esp_srmodel_filter(void* models, const char*, void*) { return models; }
struct OggDemuxer {
    static inline std::function<void()> hook;
    std::function<void(const uint8_t*, int, size_t)> callback;
    void OnDemuxerFinished(decltype(callback) cb) { callback = cb; }
    void Reset() {}
    void Process(const uint8_t* b, size_t n) { if(hook) hook(); callback(b,16000,n); }
};
struct AudioService {
    std::mutex input_control_mutex_, input_resampler_mutex_;
    uint32_t local_playback_sequence_ = 0;
    std::atomic<unsigned> event_storage{0};
    EventGroupHandle_t event_group_ = &event_storage;
    void* input_resampler_ = nullptr;
    void* models_list_ = nullptr;
    bool wake_word_initialized_ = false, audio_processor_initialized_ = false;
    bool audio_input_need_warmup_ = false;
    int encoder_sample_rate_ = 16000;
    std::unique_ptr<FakeWakeWord> wake_word_ = std::make_unique<FakeWakeWord>();
    std::unique_ptr<FakeProcessor> audio_processor_ = std::make_unique<FakeProcessor>();
    uint32_t BeginLocalPlayback();
    bool PushLocalPcm(uint32_t, std::vector<int16_t>&);
    void PauseLocalPlayback(uint32_t, bool);
    void EndLocalPlayback(uint32_t);
    void EnableWakeWordDetection(bool);
    void EnableAudioTesting(bool);
    void SetModelsList(srmodel_list_t*);
    void Stop();
''', 1)
head = head.replace("void EnableVoiceProcessing(bool p) { processing = p; }", "void EnableVoiceProcessing(bool);")
head = head.replace("void PlaySound(const char*) {}", "void PlaySound(const std::string_view&);")
head = head.replace("std::function<void()> on_send_queue_available;", "std::function<void()> on_send_queue_available; std::function<void(const std::string&)> on_wake_word_detected;")
audio_header = (repo / "main/audio/audio_service.h").read_text(encoding="utf-8")
audio_task = re.search(r"struct AudioTask\s*\{[\s\S]*?\n\};", audio_header).group()
head = re.sub(r"struct AudioTask \{[^\n]*\};", lambda _: audio_task, head)

tests = r'''
struct Gate {
    std::promise<void> entered, release;
    std::shared_future<void> release_future = release.get_future().share();
    void Wait() { entered.set_value(); release_future.wait(); }
    void Arrived() { assert(entered.get_future().wait_for(2s) == std::future_status::ready); }
};
void stop_worker(AudioService& a, std::thread& t) { a.Stop(); t.join(); }
void fill(AudioService& a, uint32_t token) {
    for(int i=0; i<MAX_PLAYBACK_TASKS_IN_QUEUE; ++i) {
        std::vector<int16_t> p(320, i+1); assert(a.PushLocalPcm(token,p) && p.empty());
    }
}
std::unique_ptr<AudioTask> opus(uint32_t generation, int16_t value) {
    auto task = std::make_unique<AudioTask>(); task->type=kAudioTaskTypeDecodeToPlaybackQueue;
    task->generation=generation; task->pcm.assign(320,value); return task;
}
void wait_empty(AudioService& a) {
    std::unique_lock<std::mutex> lock(a.audio_queue_mutex_);
    assert(a.audio_queue_cv_.wait_for(lock,2s,[&]{return a.audio_playback_queue_.empty() && !a.playing_;}));
}
int main() {
    {
        AudioService a;
        a.EnableWakeWordDetection(true); a.EnableVoiceProcessing(true); a.EnableAudioTesting(true);
        a.audio_decode_queue_.push_back(std::make_unique<AudioStreamPacket>());
        a.audio_send_queue_.push_back(std::make_unique<AudioStreamPacket>());
        auto token=a.BeginLocalPlayback(); assert(token && a.IsLocalPlaybackActive());
        assert(!a.wake_word_->running && !a.audio_processor_->running && a.event_storage==0);
        assert(a.audio_decode_queue_.empty() && a.audio_send_queue_.empty());
        a.EnableWakeWordDetection(true); a.EnableVoiceProcessing(true); a.EnableAudioTesting(true);
        assert(!a.wake_word_->running && !a.audio_processor_->running && a.event_storage==0);
        assert(!a.PushPacketToDecodeQueue(std::make_unique<AudioStreamPacket>(),true));
        a.PlaySound("sound"); assert(a.audio_decode_queue_.empty());
        std::vector<int16_t> invalid(321,7), empty;
        assert(!a.PushLocalPcm(token,invalid) && invalid.size()==321);
        assert(!a.PushLocalPcm(token,empty));
        a.EndLocalPlayback(token); a.EnableWakeWordDetection(true); assert(a.wake_word_->running);
        std::cout << "PASS local exclusivity/network/prompt/microphone gates and format bounds\n";
    }
    {
        AudioService a; Gate models; FakeWakeWord::construct_hook=[&]{models.Wait();};
        srmodel_list_t list=1;
        auto loader=std::async(std::launch::async,[&]{a.SetModelsList(&list);}); models.Arrived();
        auto begin=std::async(std::launch::async,[&]{return a.BeginLocalPlayback();});
        assert(begin.wait_for(20ms)==std::future_status::timeout);
        models.release.set_value(); loader.get(); auto token=begin.get(); FakeWakeWord::construct_hook={};
        a.EnableWakeWordDetection(true);
        assert(token && !a.wake_word_initialized_ && !a.wake_word_->running);
        a.EndLocalPlayback(token); a.EnableWakeWordDetection(true);
        assert(a.wake_word_initialized_ && a.wake_word_->running);
        std::cout << "PASS startup model creation serializes with local entry and later wake enable\n";
    }
    {
        AudioService a; auto token=a.BeginLocalPlayback(); fill(a,token);
        a.PauseLocalPlayback(token,true); a.ResetDecoder();
        assert(a.audio_playback_queue_.size()==MAX_PLAYBACK_TASKS_IN_QUEUE && !a.IsPlaybackComplete());
        std::thread worker([&]{a.AudioOutputTask();});
        std::vector<int16_t> p(320,9); auto start=std::chrono::steady_clock::now();
        assert(!a.PushLocalPcm(token,p) && p.size()==320);
        assert(std::chrono::steady_clock::now()-start>=80ms);
        { std::lock_guard<std::mutex> lock(a.fake_codec.writes_mutex); assert(a.fake_codec.writes.empty()); }
        a.PauseLocalPlayback(token,false); assert(a.PushLocalPcm(token,p) && p.empty());
        wait_empty(a); assert(!a.IsPlaybackComplete());
        { std::lock_guard<std::mutex> lock(a.audio_queue_mutex_); a.last_output_time_=std::chrono::steady_clock::now()-201ms; }
        assert(a.IsPlaybackComplete()); stop_worker(a,worker);
        assert((a.fake_codec.writes==std::vector<int16_t>{1,2,3,4,5,6,7,8,9}));
        std::cout << "PASS pause keeps queued PCM, bounded wait preserves data, resume order and DMA tail\n";
    }
    {
        AudioService a; auto old=a.BeginLocalPlayback(); fill(a,old);
        std::vector<int16_t> waiting(320,9);
        auto writer=std::async(std::launch::async,[&]{return a.PushLocalPcm(old,waiting);});
        assert(writer.wait_for(20ms)==std::future_status::timeout);
        auto current=a.BeginLocalPlayback(); assert(current && current!=old);
        assert(!writer.get() && waiting.size()==320);
        a.PauseLocalPlayback(current,true); a.PauseLocalPlayback(old,false); a.EndLocalPlayback(old);
        assert(a.local_playback_token_==current && a.local_playback_paused_);
        a.PauseLocalPlayback(current,false); assert(a.PushLocalPcm(current,waiting));
        assert(a.audio_playback_queue_.size()==1);
        a.EndLocalPlayback(current); assert(!a.IsLocalPlaybackActive() && a.audio_playback_queue_.empty());
        assert(!a.PushLocalPcm(old,waiting));
        std::cout << "PASS replacement wakes old producer; stale pause/end cannot change new session\n";
    }
    {
        AudioService a; Gate output;
        a.fake_codec.output_hook=[&]{output.Wait();};
        a.audio_playback_queue_.push_back(opus(a.decode_generation_,1));
        a.audio_playback_queue_.push_back(opus(a.decode_generation_,2));
        std::thread worker([&]{a.AudioOutputTask();}); output.Arrived();
        auto begin=std::async(std::launch::async,[&]{return a.BeginLocalPlayback();});
        assert(begin.wait_for(20ms)==std::future_status::timeout);
        // Reservation and queue invalidation happen before waiting for the old hardware write.
        assert(a.IsLocalPlaybackActive());
        output.release.set_value(); auto token=begin.get();
        a.fake_codec.output_hook={};
        std::vector<int16_t> p(320,3); assert(a.PushLocalPcm(token,p));
        wait_empty(a); stop_worker(a,worker);
        assert((a.fake_codec.writes==std::vector<int16_t>{1,3}));
        std::cout << "PASS source switch waits current hardware frame and discards queued old TTS\n";
    }
    {
        AudioService a; Gate decode; a.sample_rate_hook=[&]{decode.Wait();};
        a.audio_decode_queue_.push_back(std::make_unique<AudioStreamPacket>());
        auto before=decode_calls.load(); std::thread worker([&]{a.OpusCodecTask();}); decode.Arrived();
        auto token=a.BeginLocalPlayback(); decode.release.set_value();
        { std::unique_lock<std::mutex> lock(a.audio_queue_mutex_);
          assert(a.audio_queue_cv_.wait_for(lock,2s,[&]{return !a.decoding_;})); }
        assert(decode_calls==before && a.audio_playback_queue_.empty());
        a.EndLocalPlayback(token); stop_worker(a,worker);
        std::cout << "PASS in-flight Opus decode cannot enter local session\n";
    }
    {
        AudioService a; Gate demux; OggDemuxer::hook=[&]{demux.Wait();};
        auto prompt=std::async(std::launch::async,[&]{a.PlaySound("old sound");}); demux.Arrived();
        auto token=a.BeginLocalPlayback(); a.EndLocalPlayback(token);
        demux.release.set_value(); prompt.get(); OggDemuxer::hook={};
        assert(a.audio_decode_queue_.empty());
        a.PlaySound("new sound"); assert(a.audio_decode_queue_.size()==1);
        std::cout << "PASS old prompt demux cannot resume after local session exits\n";
    }
    {
        AudioService a; auto token=a.BeginLocalPlayback(); fill(a,token);
        std::vector<int16_t> p(320,9);
        auto writer=std::async(std::launch::async,[&]{return a.PushLocalPcm(token,p);});
        assert(writer.wait_for(20ms)==std::future_status::timeout); a.Stop();
        assert(!writer.get() && p.size()==320 && !a.IsLocalPlaybackActive());
        assert(a.BeginLocalPlayback()==0);
        std::cout << "PASS shutdown cancels blocked PCM producer and invalidates local token\n";
    }
}
'''

parts = [head]
for name in ["BeginLocalPlayback", "PushLocalPcm", "PauseLocalPlayback", "EndLocalPlayback", "Stop",
             "ResetDecoder", "ResetEncoder", "IsPlaybackComplete", "OpusCodecTask", "AudioOutputTask",
             "PushPacketToDecodeQueue", "EnqueueDecodePacket", "EnableWakeWordDetection",
             "EnableVoiceProcessing", "EnableAudioTesting", "SetModelsList", "PlaySound"]:
    parts.append(method("main/audio/audio_service.cc", "AudioService::" + name))
parts.append(tests)
with tempfile.TemporaryDirectory(prefix="xiaozhi-local-audio-") as directory:
    source = pathlib.Path(directory) / "local.cc"
    source.write_text("\n".join(parts), encoding="utf-8")
    binary = pathlib.Path(directory) / "local"
    subprocess.run(["g++", "-std=c++20", "-O1", "-pthread", "-Wall", "-Wextra", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=15)
