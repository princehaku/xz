#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>


#include <deque>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>


#include "audio_service.h"
#include "device_state.h"
#include "device_state_machine.h"
#include "ota.h"
#include "protocol.h"


// Main event bits
#define MAIN_EVENT_SCHEDULE (1 << 0)
#define MAIN_EVENT_SEND_AUDIO (1 << 1)
#define MAIN_EVENT_WAKE_WORD_DETECTED (1 << 2)
#define MAIN_EVENT_VAD_CHANGE (1 << 3)
#define MAIN_EVENT_ERROR (1 << 4)
#define MAIN_EVENT_ACTIVATION_DONE (1 << 5)
#define MAIN_EVENT_CLOCK_TICK (1 << 6)
#define MAIN_EVENT_NETWORK_CONNECTED (1 << 7)
#define MAIN_EVENT_NETWORK_DISCONNECTED (1 << 8)
#define MAIN_EVENT_TOGGLE_CHAT (1 << 9)
#define MAIN_EVENT_START_LISTENING (1 << 10)
#define MAIN_EVENT_STOP_LISTENING (1 << 11)
#define MAIN_EVENT_STATE_CHANGED (1 << 12)
#define MAIN_EVENT_PLAYBACK_PROGRESS (1 << 13)

enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // Delete copy constructor and assignment operator
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /**
     * Initialize the application
     * This sets up display, audio, network callbacks, etc.
     * Network connection starts asynchronously.
     */
    void Initialize();

    /**
     * Run the main event loop
     * This function runs in the main task and never returns.
     * It handles all events including network, state changes, and user interactions.
     */
    void Run();

    DeviceState GetDeviceState() const { return state_machine_.GetState(); }
    bool IsVoiceDetected() const { return audio_service_.IsVoiceDetected(); }

    /**
     * Request state transition
     * Returns true if transition was successful
     */
    bool SetDeviceState(DeviceState state);

    /**
     * Schedule a callback to be executed in the main task
     */
    void Schedule(std::function<void()>&& callback);

    /**
     * Alert with status, message, emotion and optional sound
     */
    void Alert(const char* status, const char* message, const char* emotion = "",
               const std::string_view& sound = "");
    void DismissAlert();

    void AbortSpeaking(AbortReason reason);

    /**
     * Toggle chat state (event-based, thread-safe)
     * Sends MAIN_EVENT_TOGGLE_CHAT to be handled in Run()
     */
    void ToggleChatState();

    /**
     * Start listening (event-based, thread-safe)
     * Sends MAIN_EVENT_START_LISTENING to be handled in Run()
     */
    void StartListening();

    /**
     * Stop listening (event-based, thread-safe)
     * Sends MAIN_EVENT_STOP_LISTENING to be handled in Run()
     */
    void StopListening();

    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    bool UpgradeFirmware(const std::string& url, const std::string& version = "");
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    void PlaySound(const std::string_view& sound);
    AudioService& GetAudioService() { return audio_service_; }
    Protocol* GetProtocol() { return protocol_.get(); }
    void NotifySTT(const std::string& text);
    // Completes on the main task after this connection receives fresh vision capabilities.
    void PrepareCameraSession(std::function<void(bool)> callback);
    void SetKeepAlive(bool enable);
    void EndConversation();

    /**
     * Reset protocol resources (thread-safe)
     * Can be called from any task to release resources allocated after network connected
     * This includes closing audio channel, resetting protocol and ota objects
     */
    void ResetProtocol();

private:
    Application();
    ~Application();

    std::mutex mutex_;
    std::deque<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    esp_timer_handle_t playback_timer_handle_ = nullptr;
    DeviceStateMachine state_machine_;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;
    std::atomic<bool> keep_alive_{false};
    bool network_connected_ = false;
    int64_t reconnect_at_ms_ = 0;
    int reconnect_delay_ms_ = 2000;
    int64_t audio_channel_opened_ms_ = 0;
    std::atomic<uint32_t> audio_channel_generation_{0};
    std::atomic<uint32_t> conversation_generation_{0};
    std::atomic<uint32_t> camera_request_generation_{0};
    std::atomic<bool> camera_session_active_{false};
    std::atomic<bool> camera_result_pending_{false};
    std::function<void(bool)> camera_prepare_callback_;
    uint32_t camera_prepare_generation_ = 0;
    uint32_t camera_channel_generation_ = 0;
    uint32_t camera_vision_baseline_ = 0;
    int64_t camera_prepare_deadline_ms_ = 0;
    int64_t camera_ready_ms_ = 0;
    bool camera_open_started_ = false;
    bool camera_session_ready_ = false;
    uint32_t tts_generation_ = 0;
    uint32_t pending_tts_generation_ = 0;
    bool tts_completion_pending_ = false;
    std::string last_error_message_;
    AudioService audio_service_;
    std::unique_ptr<Ota> ota_;

    bool has_server_time_ = false;
    bool aborted_ = false;
    bool assets_version_checked_ = false;
    bool play_popup_on_listening_ =
        false;                         // Flag to play popup sound after state changes to listening
    bool vad_speech_started_ = false;  // 标志：当前监听会话中用户是否已开口（用于自动停止）
    bool vad_speech_pending_ = false;
    int64_t vad_listen_start_ms_ = 0;  // 进入 Listening 状态的时间戳（ms），用于过滤 AFE 热身误报
    bool tts_just_finished_ =
        false;  // TTS 刚结束标志：进入 Listening 时需延长 VAD 热身以防扬声器尾音误判
    int clock_ticks_ = 0;
    TaskHandle_t activation_task_handle_ = nullptr;

    // Event handlers
    void HandleStateChangedEvent();
    void HandleToggleChatEvent();
    void HandleStartListeningEvent();
    void HandleStopListeningEvent();
    void HandleNetworkConnectedEvent();
    void HandleNetworkDisconnectedEvent();
    void HandleActivationDoneEvent();
    void HandleWakeWordDetectedEvent();
    void ContinueOpenAudioChannel(ListeningMode mode);
    void ContinueWakeWordInvoke(const std::string& wake_word);
    void ScheduleReconnect();
    void HandleReconnect();
    void CancelTtsCompletion();
    void HandlePlaybackProgress();
    void HandleCameraPreparation();
    void CompleteCameraPreparation(bool ready);
    void CancelCameraSession();

    // Activation task (runs in background)
    void ActivationTask();

    // Helper methods
    void CheckAssetsVersion();
    void CheckNewVersion();
    void InitializeProtocol();
    void ShowActivationCode(const std::string& code, const std::string& message);
    void SetListeningMode(ListeningMode mode);
    ListeningMode GetDefaultListeningMode() const;

    // State change handler called by state machine
    void OnStateChanged(DeviceState old_state, DeviceState new_state);
};

class TaskPriorityReset {
public:
    TaskPriorityReset(BaseType_t priority) {
        original_priority_ = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, priority);
    }
    ~TaskPriorityReset() { vTaskPrioritySet(NULL, original_priority_); }

private:
    BaseType_t original_priority_;
};

#endif  // _APPLICATION_H_
