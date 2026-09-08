#include "audio_service.h"
#include <esp_log.h>
#include <cstring>
#include <algorithm>
#include <freertos/idf_additions.h>
#include <esp_heap_caps.h>

#define RATE_CVT_CFG(_src_rate, _dest_rate, _channel)        \
    (esp_ae_rate_cvt_cfg_t)                                  \
    {                                                        \
        .src_rate        = (uint32_t)(_src_rate),            \
        .dest_rate       = (uint32_t)(_dest_rate),           \
        .channel         = (uint8_t)(_channel),              \
        .bits_per_sample = ESP_AUDIO_BIT16,                  \
        .complexity      = 2,                                \
        .perf_type       = ESP_AE_RATE_CVT_PERF_TYPE_SPEED,  \
    }

#define OPUS_DEC_CFG(_sample_rate, _frame_duration_ms)                                                    \
    (esp_opus_dec_cfg_t)                                                                                  \
    {                                                                                                     \
        .sample_rate    = (uint32_t)(_sample_rate),                                                       \
        .channel        = ESP_AUDIO_MONO,                                                                 \
        .frame_duration = (esp_opus_dec_frame_duration_t)AS_OPUS_GET_FRAME_DRU_ENUM(_frame_duration_ms),  \
        .self_delimited = false,                                                                          \
    }

#if CONFIG_USE_AUDIO_PROCESSOR
#include "processors/afe_audio_processor.h"
#else
#include "processors/no_audio_processor.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#include "wake_words/afe_wake_word.h"
#include "wake_words/custom_wake_word.h"
#else
#include "wake_words/esp_wake_word.h"
#endif

#define TAG "AudioService"

AudioService::AudioService() {
    event_group_ = xEventGroupCreate();
}

AudioService::~AudioService() {
    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
    }
    if (opus_encoder_ != nullptr) {
        esp_opus_enc_close(opus_encoder_);
    }
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_close(opus_decoder_);
    }
    if (input_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(input_resampler_);
    }
    if (output_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(output_resampler_);
    }
}

void AudioService::Initialize(AudioCodec* codec) {
    codec_ = codec;
    codec_->Start();

    /* Decoder is configured for the maximum Opus frame size (120ms) so it can handle
     * any TTS packet the server sends, regardless of the server's declared frame_duration. */
    esp_opus_dec_cfg_t opus_dec_cfg = OPUS_DEC_CFG(codec->output_sample_rate(), 120);
    auto ret = esp_opus_dec_open(&opus_dec_cfg, sizeof(esp_opus_dec_cfg_t), &opus_decoder_);
    if (opus_decoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio decoder, error code: %d", ret);
    } else {
        decoder_sample_rate_ = codec->output_sample_rate();
        decoder_duration_ms_ = 120;
        decoder_frame_size_ = decoder_sample_rate_ / 1000 * 120;
    }
    esp_opus_enc_config_t opus_enc_cfg = AS_OPUS_ENC_CONFIG();
    ret = esp_opus_enc_open(&opus_enc_cfg, sizeof(esp_opus_enc_config_t), &opus_encoder_);
    if (opus_encoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio encoder, error code: %d", ret);
    } else {
        encoder_sample_rate_ = 16000;
        encoder_duration_ms_ = OPUS_FRAME_DURATION_MS;
        esp_opus_enc_get_frame_size(opus_encoder_, &encoder_frame_size_, &encoder_outbuf_size_);
        encoder_frame_size_ = encoder_frame_size_ / sizeof(int16_t);
    }

    ESP_LOGI(TAG, "Audio init: codec_in=%dHz codec_out=%dHz encoder_frame=%u encoder_outbuf=%u",
             codec->input_sample_rate(), codec->output_sample_rate(),
             (unsigned)encoder_frame_size_, (unsigned)encoder_outbuf_size_);

    if (codec->input_sample_rate() != 16000) {
        ESP_LOGI(TAG, "Input resampler: %d -> 16000 Hz (channels=%d)",
                 codec->input_sample_rate(), codec->input_channels());
        esp_ae_rate_cvt_cfg_t input_resampler_cfg = RATE_CVT_CFG(
            codec->input_sample_rate(), ESP_AUDIO_SAMPLE_RATE_16K, codec->input_channels());
        auto resampler_ret = esp_ae_rate_cvt_open(&input_resampler_cfg, &input_resampler_);
        if (input_resampler_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create input resampler, error code: %d", resampler_ret);
        }
    } else {
        ESP_LOGI(TAG, "Input resampler: not needed (codec already at 16000 Hz)");
    }

#if CONFIG_USE_AUDIO_PROCESSOR
    audio_processor_ = std::make_unique<AfeAudioProcessor>();
#else
    audio_processor_ = std::make_unique<NoAudioProcessor>();
#endif

    audio_processor_->OnOutput([this](std::vector<int16_t>&& data) {
        PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue, std::move(data));
    });

    audio_processor_->OnVadStateChange([this](bool speaking) {
        voice_detected_ = speaking;
        if (callbacks_.on_vad_change) {
            callbacks_.on_vad_change(speaking);
        }
    });

#if CONFIG_USE_AUDIO_PROCESSOR
    // Create AFE processing task early to avoid allocation failure later
    // when entering listening state under fragmented heap conditions.
    audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
    audio_processor_initialized_ = true;
#endif

    esp_timer_create_args_t audio_power_timer_args = {
        .callback = [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->CheckAndUpdateAudioPowerState();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "audio_power_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&audio_power_timer_args, &audio_power_timer_);
}

void AudioService::Start() {
    service_stopped_ = false;
    xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING | AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    esp_timer_start_periodic(audio_power_timer_, 1000000);

#if CONFIG_USE_AUDIO_PROCESSOR
    /* Start the audio input task */
    xTaskCreatePinnedToCore([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioInputTask();
        vTaskDelete(NULL);
    }, "audio_input", 2048 * 3, this, 8, &audio_input_task_handle_, 0);

    /* Start the audio output task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioOutputTask();
        vTaskDelete(NULL);
    }, "audio_output", 2048 * 2, this, 4, &audio_output_task_handle_);
#else
    /* Start the audio input task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioInputTask();
        vTaskDelete(NULL);
    }, "audio_input", 2048 * 2, this, 8, &audio_input_task_handle_);

    /* Start the audio output task */
    xTaskCreate([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->AudioOutputTask();
        vTaskDelete(NULL);
    }, "audio_output", 2048, this, 4, &audio_output_task_handle_);
#endif

    /* Start the opus codec task with PSRAM stack to avoid internal SRAM pressure.
     * SILK encoder (VOIP mode) needs ~40KB stack; PSRAM is fine for non-ISR codec work. */
    xTaskCreateWithCaps([](void* arg) {
        AudioService* audio_service = (AudioService*)arg;
        audio_service->OpusCodecTask();
        vTaskDeleteWithCaps(NULL);
    /* Priority above typical worker tasks so decode keeps playback queue fed (avoids speaker underrun). */
    }, "opus_codec", 2048 * 20, this, 5, &opus_codec_task_handle_,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void AudioService::Stop() {
    esp_timer_stop(audio_power_timer_);
    service_stopped_ = true;
    xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING |
        AS_EVENT_WAKE_WORD_RUNNING |
        AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    ++decode_generation_;
    ++encode_generation_;
    local_playback_token_ = 0;
    local_playback_paused_ = false;
    audio_encode_queue_.clear();
    audio_send_queue_.clear();
    audio_decode_queue_.clear();
    audio_playback_queue_.clear();
    audio_testing_queue_.clear();
    audio_queue_cv_.notify_all();
}

bool AudioService::ReadAudioData(std::vector<int16_t>& data, int sample_rate, int samples) {
    std::unique_lock<std::mutex> input_lock(input_control_mutex_);
    if (IsLocalPlaybackActive()) return false;
    if (!codec_->input_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableInput(true);
    }
    input_lock.unlock();

    if (codec_->input_sample_rate() != sample_rate) {
        data.resize(samples * codec_->input_sample_rate() / sample_rate * codec_->input_channels());
        if (!codec_->InputData(data)) {
            return false;
        }
        if (input_resampler_ != nullptr) {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            uint32_t in_sample_num = data.size() / codec_->input_channels();
            uint32_t output_samples = 0;
            esp_ae_rate_cvt_get_max_out_sample_num(input_resampler_, in_sample_num, &output_samples);
            auto resampled = std::vector<int16_t>(output_samples * codec_->input_channels());
            uint32_t actual_output = output_samples;
            esp_ae_rate_cvt_process(input_resampler_, (esp_ae_sample_t)data.data(), in_sample_num,
                                   (esp_ae_sample_t)resampled.data(), &actual_output);
            resampled.resize(actual_output * codec_->input_channels());
            data = std::move(resampled);
        }
    } else {
        data.resize(samples * codec_->input_channels());
        if (!codec_->InputData(data)) {
            return false;
        }
    }

    /* Update the last input time */
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        last_input_time_ = std::chrono::steady_clock::now();
    }
    debug_statistics_.input_count++;

#if CONFIG_USE_AUDIO_DEBUGGER
    // 音频调试：发送原始音频数据
    if (audio_debugger_ == nullptr) {
        audio_debugger_ = std::make_unique<AudioDebugger>();
    }
    audio_debugger_->Feed(data);
#endif

    return true;
}

void AudioService::AudioInputTask() {
    while (true) {
        EventBits_t bits = xEventGroupWaitBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING |
            AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING,
            pdFALSE, pdFALSE, portMAX_DELAY);

        if (service_stopped_) {
            break;
        }
        if (audio_input_need_warmup_) {
            audio_input_need_warmup_ = false;
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        /* Used for audio testing in NetworkConfiguring mode by clicking the BOOT button */
        if (bits & AS_EVENT_AUDIO_TESTING_RUNNING) {
            if (audio_testing_queue_.size() >= AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS) {
                ESP_LOGW(TAG, "Audio testing queue is full, stopping audio testing");
                EnableAudioTesting(false);
                continue;
            }
            std::vector<int16_t> data;
            int samples = OPUS_FRAME_DURATION_MS * 16000 / 1000;
            if (ReadAudioData(data, 16000, samples)) {
                if (IsLocalPlaybackActive()) continue;
                // If input channels is 2, we need to fetch the left channel data
                if (codec_->input_channels() == 2) {
                    auto mono_data = std::vector<int16_t>(data.size() / 2);
                    for (size_t i = 0, j = 0; i < mono_data.size(); ++i, j += 2) {
                        mono_data[i] = data[j];
                    }
                    data = std::move(mono_data);
                }
                PushTaskToEncodeQueue(kAudioTaskTypeEncodeToTestingQueue, std::move(data));
                continue;
            }
        }

        /* Feed the wake word and/or audio processor */
        if (bits & (AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING)) {
            // Read audio in AFE feed-chunk aligned size to reduce feed/fetch mismatch.
            size_t wake_feed = 0;
            size_t proc_feed = 0;
            {
                std::lock_guard<std::mutex> input_lock(input_control_mutex_);
                if (IsLocalPlaybackActive()) continue;
                if ((bits & AS_EVENT_WAKE_WORD_RUNNING) && wake_word_) {
                    wake_feed = wake_word_->GetFeedSize();
                }
                if (bits & AS_EVENT_AUDIO_PROCESSOR_RUNNING) {
                    proc_feed = audio_processor_->GetFeedSize();
                }
            }
            int samples = static_cast<int>(std::max<size_t>(160, std::max(wake_feed, proc_feed)));
            std::vector<int16_t> data;
            if (ReadAudioData(data, 16000, samples)) {
                std::lock_guard<std::mutex> input_lock(input_control_mutex_);
                if (IsLocalPlaybackActive()) continue;
                if ((bits & AS_EVENT_WAKE_WORD_RUNNING) && wake_word_) {
                    wake_word_->Feed(data);
                }
                if (bits & AS_EVENT_AUDIO_PROCESSOR_RUNNING) {
                    audio_processor_->Feed(std::move(data));
                }
                continue;
            }
        }

        // Read timeout/error should not terminate the input task.
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "Audio input task stopped");
}

void AudioService::AudioOutputTask() {
    while (true) {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]() {
            return service_stopped_ || (!audio_playback_queue_.empty() &&
                !(audio_playback_queue_.front()->type == kAudioTaskTypeLocalPlayback &&
                  local_playback_paused_));
        });
        if (service_stopped_) {
            break;
        }

        // Source changes use the same barrier. Never wait for it with the queue
        // locked: the active writer needs that lock to publish its completion.
        lock.unlock();
        std::unique_lock<std::mutex> output_lock(output_mutex_);
        lock.lock();
        if (service_stopped_) break;
        if (audio_playback_queue_.empty()) continue;
        const auto& front = audio_playback_queue_.front();
        if (front->type == kAudioTaskTypeLocalPlayback && local_playback_paused_) continue;

        auto task = std::move(audio_playback_queue_.front());
        audio_playback_queue_.pop_front();
        const bool local = task->type == kAudioTaskTypeLocalPlayback;
        if ((local && (task->generation == 0 || task->generation != local_playback_token_)) ||
            (!local && (IsLocalPlaybackActive() || task->generation != decode_generation_))) {
            audio_queue_cv_.notify_all();
            continue;
        }
        playing_ = true;
        audio_queue_cv_.notify_all();
        lock.unlock();

        if (!codec_->output_enabled()) {
            esp_timer_stop(audio_power_timer_);
            esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
            codec_->EnableOutput(true);
        }

        codec_->OutputData(task->pcm);

        /* Update the last output time */
        lock.lock();
        last_output_time_ = std::chrono::steady_clock::now();
        playing_ = false;
        audio_queue_cv_.notify_all();
        debug_statistics_.playback_count++;

#if CONFIG_USE_SERVER_AEC
        /* Record the timestamp for server AEC */
        if (task->timestamp > 0) {
            timestamp_queue_.push_back(task->timestamp);
        }
#endif
    }

    ESP_LOGW(TAG, "Audio output task stopped");
}

void AudioService::OpusCodecTask() {
    while (true) {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]() {
            return service_stopped_ ||
                !audio_encode_queue_.empty() ||
                (!audio_decode_queue_.empty() && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE);
        });
        if (service_stopped_) {
            break;
        }

        /* Decode the audio from decode queue */
        if (!audio_decode_queue_.empty() && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE) {
            auto packet = std::move(audio_decode_queue_.front());
            audio_decode_queue_.pop_front();
            const auto generation = decode_generation_.load();
            decoding_ = true;
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto task = std::make_unique<AudioTask>();
            task->type = kAudioTaskTypeDecodeToPlaybackQueue;
            task->timestamp = packet->timestamp;
            task->generation = generation;

            SetDecodeSampleRate(packet->sample_rate, packet->frame_duration);
            if (opus_decoder_ != nullptr) {
                task->pcm.resize(decoder_frame_size_);
                esp_audio_dec_in_raw_t raw = {
                    .buffer = (uint8_t *)(packet->payload.data()),
                    .len = (uint32_t)(packet->payload.size()),
                    .consumed = 0,
                    .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
                };
                esp_audio_dec_out_frame_t out_frame = {
                    .buffer = (uint8_t *)(task->pcm.data()),
                    .len = (uint32_t)(task->pcm.size() * sizeof(int16_t)),
                    .decoded_size = 0,
                };
                esp_audio_dec_info_t dec_info = {};
                std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
                // ResetDecoder may have invalidated this frame while the queue lock
                // was released. Do not let it change the new decoder history.
                auto ret = generation == decode_generation_.load()
                    ? esp_opus_dec_decode(opus_decoder_, &raw, &out_frame, &dec_info)
                    : ESP_AUDIO_ERR_FAIL;
                decoder_lock.unlock();
                if (ret == ESP_AUDIO_ERR_OK) {
                    task->pcm.resize(out_frame.decoded_size / sizeof(int16_t));
                    if (decoder_sample_rate_ != codec_->output_sample_rate() && output_resampler_ != nullptr) {
                        uint32_t target_size = 0;
                        esp_ae_rate_cvt_get_max_out_sample_num(output_resampler_, task->pcm.size(), &target_size);
                        std::vector<int16_t> resampled(target_size);
                        uint32_t actual_output = target_size;
                        esp_ae_rate_cvt_process(output_resampler_, (esp_ae_sample_t)task->pcm.data(), task->pcm.size(),
                                                (esp_ae_sample_t)resampled.data(), &actual_output);
                        resampled.resize(actual_output);
                        task->pcm = std::move(resampled);
                    }
                    lock.lock();
                    if (generation == decode_generation_.load() && !service_stopped_ && !IsLocalPlaybackActive()) {
                        audio_playback_queue_.push_back(std::move(task));
                    }
                    audio_queue_cv_.notify_all();
                    debug_statistics_.decode_count++;
                } else {
                    if (generation == decode_generation_.load()) {
                        ESP_LOGE(TAG, "Failed to decode audio after resize, error code: %d", ret);
                    }
                    lock.lock();
                }
            } else {
                ESP_LOGE(TAG, "Audio decoder is not configured");
                lock.lock();
            }
            decoding_ = false;
            audio_queue_cv_.notify_all();
        }
        /* Encode the audio to send queue */
        if (!audio_encode_queue_.empty()) {
            /* If the send queue is full (no consumer active, e.g. during camera HTTP upload),
             * silently drain the encode queue to prevent backlog and warning spam. */
            if (audio_send_queue_.size() >= MAX_SEND_PACKETS_IN_QUEUE) {
                audio_encode_queue_.pop_front();
                audio_queue_cv_.notify_all();
                /* lock still held – continue to next loop iteration */
            /* Software echo suppression: discard mic audio while TTS is playing or
             * queued for playback, and for 200ms after playback stops to cover tailings.
             * This prevents the speaker output from feeding back into the ASR pipeline
             * when AEC hardware reference is unavailable.
             * When keep_uplink_ is set (server-side barge-in mode), mic audio is forwarded
             * to the server even during playback so the server can perform VAD/ASR and
             * decide whether to interrupt the current TTS. */
            } else if (!keep_uplink_ &&
                       (playing_ || decoding_ || !audio_playback_queue_.empty() || !audio_decode_queue_.empty() ||
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - last_output_time_).count() < 200)) {
                audio_encode_queue_.pop_front();
                audio_queue_cv_.notify_all();
                /* lock still held – continue to next loop iteration */
            } else {
                auto task = std::move(audio_encode_queue_.front());
                audio_encode_queue_.pop_front();
                const auto generation = encode_generation_;
                audio_queue_cv_.notify_all();
                lock.unlock();

                auto packet = std::make_unique<AudioStreamPacket>();
                packet->frame_duration = OPUS_FRAME_DURATION_MS;
                packet->sample_rate = 16000;
                packet->timestamp = task->timestamp;

                if (opus_encoder_ != nullptr && task->pcm.size() == encoder_frame_size_) {
                    packet->payload.resize(encoder_outbuf_size_);
                    esp_audio_enc_in_frame_t in = {
                        .buffer = (uint8_t *)(task->pcm.data()),
                        .len = (uint32_t)(encoder_frame_size_ * sizeof(int16_t)),
                    };
                    esp_audio_enc_out_frame_t out = {
                        .buffer = packet->payload.data(),
                        .len = (uint32_t)encoder_outbuf_size_,
                        .encoded_bytes = 0,
                    };
                    auto ret = esp_opus_enc_process(opus_encoder_, &in, &out);
                    if (ret == ESP_AUDIO_ERR_OK) {
                        packet->payload.resize(out.encoded_bytes);
                        static uint32_t encode_count = 0;
                        encode_count++;
                        if (encode_count == 1 || encode_count % 50 == 0) {
                            ESP_LOGI(TAG, "Opus encoded #%lu: pcm=%u bytes=%u",
                                     (unsigned long)encode_count,
                                     (unsigned)encoder_frame_size_,
                                     (unsigned)out.encoded_bytes);
                        }

                        if (task->type == kAudioTaskTypeEncodeToSendQueue) {
                            bool queued = false;
                            {
                                std::lock_guard<std::mutex> lock2(audio_queue_mutex_);
                                if (generation == encode_generation_ && !service_stopped_ &&
                                    audio_send_queue_.size() < MAX_SEND_PACKETS_IN_QUEUE) {
                                    audio_send_queue_.push_back(std::move(packet));
                                    queued = true;
                                }
                            }
                            if (queued && callbacks_.on_send_queue_available) {
                                callbacks_.on_send_queue_available();
                            }
                        } else if (task->type == kAudioTaskTypeEncodeToTestingQueue) {
                            std::lock_guard<std::mutex> lock2(audio_queue_mutex_);
                            if (generation == encode_generation_ && !service_stopped_ && !IsLocalPlaybackActive()) {
                                audio_testing_queue_.push_back(std::move(packet));
                            }
                        }
                        debug_statistics_.encode_count++;
                    } else {
                        ESP_LOGE(TAG, "Failed to encode audio, error code: %d", ret);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to encode audio: encoder not configured or invalid frame size (got %u, expected %u)",
                             task->pcm.size(), encoder_frame_size_);
                }
                lock.lock();
            } /* else (encode normally) */
        } /* if encode_queue non-empty */
    } /* while */

    ESP_LOGW(TAG, "Opus codec task stopped");
}

void AudioService::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    /* Only recreate decoder when sample_rate changes. Frame duration is ignored because
     * the decoder is always configured for 120ms (max) to handle any TTS packet size. */
    if (decoder_sample_rate_ == sample_rate) {
        return;
    }
    ESP_LOGI(TAG, "SetDecodeSampleRate: %d Hz (server frame_duration=%dms, decoder fixed at 120ms)",
             sample_rate, frame_duration);
    std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_close(opus_decoder_);
        opus_decoder_ = nullptr;
    }
    esp_opus_dec_cfg_t opus_dec_cfg = OPUS_DEC_CFG(sample_rate, 120);
    auto ret = esp_opus_dec_open(&opus_dec_cfg, sizeof(esp_opus_dec_cfg_t), &opus_decoder_);
    if (opus_decoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio decoder, error code: %d", ret);
        return;
    }
    decoder_sample_rate_ = sample_rate;
    decoder_duration_ms_ = 120;
    decoder_frame_size_ = decoder_sample_rate_ / 1000 * 120;

    auto codec = Board::GetInstance().GetAudioCodec();
    if (decoder_sample_rate_ != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", decoder_sample_rate_, codec->output_sample_rate());
        if (output_resampler_ != nullptr) {
            esp_ae_rate_cvt_close(output_resampler_);
            output_resampler_ = nullptr;
        }
        esp_ae_rate_cvt_cfg_t output_resampler_cfg = RATE_CVT_CFG(
            decoder_sample_rate_, codec->output_sample_rate(), ESP_AUDIO_MONO);
        auto resampler_ret = esp_ae_rate_cvt_open(&output_resampler_cfg, &output_resampler_);
        if (output_resampler_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create output resampler, error code: %d", resampler_ret);
        }
    }
}

void AudioService::PushTaskToEncodeQueue(AudioTaskType type, std::vector<int16_t>&& pcm) {
    auto task = std::make_unique<AudioTask>();
    task->type = type;
    task->pcm = std::move(pcm);
    /* Push the task to the encode queue */
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    if (service_stopped_ || IsLocalPlaybackActive() ||
        (type == kAudioTaskTypeEncodeToSendQueue && !IsAudioProcessorRunning())) {
        return;
    }
    const auto generation = encode_generation_;

    /* If the task is to send queue, we need to set the timestamp */
    if (type == kAudioTaskTypeEncodeToSendQueue && !timestamp_queue_.empty()) {
        if (timestamp_queue_.size() <= MAX_TIMESTAMPS_IN_QUEUE) {
            task->timestamp = timestamp_queue_.front();
        } else {
            ESP_LOGW(TAG, "Timestamp queue (%u) is full, dropping timestamp", timestamp_queue_.size());
        }
        timestamp_queue_.pop_front();
    }

    if (audio_encode_queue_.size() >= MAX_ENCODE_TASKS_IN_QUEUE) {
        if (type == kAudioTaskTypeEncodeToSendQueue) {
            // Do not block AFE/fetch thread. Drop one oldest frame to keep real-time pipeline alive.
            audio_encode_queue_.pop_front();
            ESP_LOGW(TAG, "Encode queue full, dropping oldest frame");
        } else {
            audio_queue_cv_.wait(lock, [this, generation]() {
                return service_stopped_ || IsLocalPlaybackActive() || generation != encode_generation_ ||
                    audio_encode_queue_.size() < MAX_ENCODE_TASKS_IN_QUEUE;
            });
            if (service_stopped_ || IsLocalPlaybackActive() || generation != encode_generation_) {
                return;
            }
        }
    }
    audio_encode_queue_.push_back(std::move(task));
    audio_queue_cv_.notify_all();
}

bool AudioService::PushPacketToDecodeQueue(std::unique_ptr<AudioStreamPacket> packet, bool wait) {
    return EnqueueDecodePacket(std::move(packet), wait, decode_generation_.load());
}

bool AudioService::EnqueueDecodePacket(std::unique_ptr<AudioStreamPacket> packet, bool wait, uint32_t generation) {
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    if (service_stopped_ || IsLocalPlaybackActive() || generation != decode_generation_) return false;
    if (audio_decode_queue_.size() >= MAX_DECODE_PACKETS_IN_QUEUE) {
        if (wait) {
            if (!audio_queue_cv_.wait_for(lock, std::chrono::milliseconds(120), [this, generation]() {
                return audio_decode_queue_.size() < MAX_DECODE_PACKETS_IN_QUEUE || service_stopped_ ||
                    IsLocalPlaybackActive() || generation != decode_generation_;
            })) {
                ESP_LOGW(TAG, "Decode queue full, backpressure timeout, dropping packet");
                return false;
            }
        } else {
            return false;
        }
    }
    if (service_stopped_ || IsLocalPlaybackActive() || generation != decode_generation_) {
        return false;
    }
    audio_decode_queue_.push_back(std::move(packet));
    audio_queue_cv_.notify_all();
    return true;
}

std::unique_ptr<AudioStreamPacket> AudioService::PopPacketFromSendQueue() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    if (audio_send_queue_.empty()) {
        return nullptr;
    }
    auto packet = std::move(audio_send_queue_.front());
    audio_send_queue_.pop_front();
    audio_queue_cv_.notify_all();
    return packet;
}

void AudioService::EncodeWakeWord() {
    if (wake_word_) {
        wake_word_->EncodeWakeWordData();
    }
}

const std::string& AudioService::GetLastWakeWord() const {
    return wake_word_->GetLastDetectedWakeWord();
}

std::unique_ptr<AudioStreamPacket> AudioService::PopWakeWordPacket() {
    auto packet = std::make_unique<AudioStreamPacket>();
    if (wake_word_->GetWakeWordOpus(packet->payload)) {
        return packet;
    }
    return nullptr;
}

void AudioService::EnableWakeWordDetection(bool enable) {
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
    if (enable && IsLocalPlaybackActive()) return;
    if (!wake_word_) {
        return;
    }

    ESP_LOGD(TAG, "%s wake word detection", enable ? "Enabling" : "Disabling");
    if (enable) {
        if (!wake_word_initialized_) {
            if (!wake_word_->Initialize(codec_, models_list_)) {
                ESP_LOGE(TAG, "Failed to initialize wake word");
                return;
            }
            wake_word_initialized_ = true;
        }
        // Reset input resampler to clear cached data from previous mode (e.g. AudioProcessor)
        // This prevents buffer overflow when switching between different feed sizes
        {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            if (input_resampler_ != nullptr) {
                esp_ae_rate_cvt_reset(input_resampler_);
            }
        }
        wake_word_->Start();
        xEventGroupSetBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    } else {
        wake_word_->Stop();
        xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    }
}

void AudioService::EnableVoiceProcessing(bool enable) {
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
    if (enable && IsLocalPlaybackActive()) return;
    ESP_LOGI(TAG, "%s voice processing (encoder_frame=%u sample_rate=%d)",
             enable ? "Enabling" : "Disabling",
             (unsigned)encoder_frame_size_, encoder_sample_rate_);
    if (enable) {
        if (!audio_processor_initialized_) {
            audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
            audio_processor_initialized_ = true;
        }

        /* We should make sure no audio is playing */
        ResetDecoder();
        audio_input_need_warmup_ = true;
        // Reset input resampler to clear cached data from previous mode (e.g. WakeWord)
        // This prevents buffer overflow when switching between different feed sizes
        {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            if (input_resampler_ != nullptr) {
                esp_ae_rate_cvt_reset(input_resampler_);
            }
        }
        audio_processor_->Start();
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    } else {
        audio_processor_->Stop();
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    }
}

void AudioService::EnableAudioTesting(bool enable) {
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
    if (enable && IsLocalPlaybackActive()) return;
    ESP_LOGI(TAG, "%s audio testing", enable ? "Enabling" : "Disabling");
    if (enable) {
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
    } else {
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
        /* Copy audio_testing_queue_ to audio_decode_queue_ */
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (IsLocalPlaybackActive()) {
            audio_testing_queue_.clear();
        } else {
            audio_decode_queue_ = std::move(audio_testing_queue_);
        }
        audio_queue_cv_.notify_all();
    }
}

void AudioService::EnableDeviceAec(bool enable) {
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
    ESP_LOGI(TAG, "%s device AEC", enable ? "Enabling" : "Disabling");
    if (!audio_processor_initialized_) {
        audio_processor_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_);
        audio_processor_initialized_ = true;
    }

    audio_processor_->EnableDeviceAec(enable);
}

void AudioService::SetCallbacks(AudioServiceCallbacks& callbacks) {
    callbacks_ = callbacks;
}

void AudioService::PlaySound(const std::string_view& ogg) {
    const auto generation = decode_generation_.load();
    if (IsLocalPlaybackActive()) return;
    if (!codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableOutput(true);
    }

    const auto* buf = reinterpret_cast<const uint8_t*>(ogg.data());
    size_t size = ogg.size();

    auto demuxer = std::make_unique<OggDemuxer>();
    demuxer->OnDemuxerFinished([this, generation](const uint8_t* data, int sample_rate, size_t size){
        if (IsLocalPlaybackActive() || generation != decode_generation_) return;
        auto packet = std::make_unique<AudioStreamPacket>();
        packet->sample_rate = sample_rate;
        packet->frame_duration = 60;
        packet->payload.resize(size);
        std::memcpy(packet->payload.data(), data, size);
        EnqueueDecodePacket(std::move(packet), true, generation);
    });
    demuxer->Reset();
    demuxer->Process(buf, size);
}

bool AudioService::IsIdle() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return !decoding_ && !playing_ && audio_encode_queue_.empty() && audio_decode_queue_.empty() && audio_playback_queue_.empty() && audio_testing_queue_.empty();
}

bool AudioService::IsPlaybackComplete() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return service_stopped_ ||
        (!decoding_ && !playing_ && audio_decode_queue_.empty() && audio_playback_queue_.empty() &&
         std::chrono::steady_clock::now() - last_output_time_ >= std::chrono::milliseconds(200));
}

void AudioService::WaitForPlaybackQueueEmpty() {
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    audio_queue_cv_.wait(lock, [this]() { 
        return service_stopped_ || (!decoding_ && !playing_ && audio_decode_queue_.empty() && audio_playback_queue_.empty());
    });
}

void AudioService::ResetDecoder() {
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        ++decode_generation_;
        std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
        if (opus_decoder_ != nullptr) {
            esp_opus_dec_reset(opus_decoder_);
        }
        decoder_lock.unlock();
        timestamp_queue_.clear();
        audio_decode_queue_.clear();
        audio_playback_queue_.erase(std::remove_if(audio_playback_queue_.begin(), audio_playback_queue_.end(),
            [](const auto& task) { return task->type != kAudioTaskTypeLocalPlayback; }), audio_playback_queue_.end());
        audio_testing_queue_.clear();
        audio_queue_cv_.notify_all();
    }
    // Invalidate first so the writer cannot start another old frame while this
    // caller waits for an already submitted I2S write to finish.
    std::lock_guard<std::mutex> output_lock(output_mutex_);
}

uint32_t AudioService::BeginLocalPlayback() {
    // Serialize against a concurrent microphone/wake-word enable. The output
    // barrier lets an already submitted write finish (Opus frames are <=120 ms).
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
    uint32_t token;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (service_stopped_ || !codec_ || codec_->output_channels() != 1) return 0;
        token = ++local_playback_sequence_;
        if (token == 0) token = ++local_playback_sequence_;
        local_playback_token_ = token;
        local_playback_paused_ = false;
        ++decode_generation_;
        ++encode_generation_;
        keep_uplink_ = false;
        audio_decode_queue_.clear();
        audio_playback_queue_.clear();
        audio_testing_queue_.clear();
        audio_encode_queue_.clear();
        audio_send_queue_.clear();
        timestamp_queue_.clear();
        std::lock_guard<std::mutex> decoder_lock(decoder_mutex_);
        if (opus_decoder_) esp_opus_dec_reset(opus_decoder_);
        audio_queue_cv_.notify_all();
    }
    if (wake_word_) wake_word_->Stop();
    if (audio_processor_) audio_processor_->Stop();
    xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING |
        AS_EVENT_AUDIO_PROCESSOR_RUNNING | AS_EVENT_AUDIO_TESTING_RUNNING);
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    return token;
}

bool AudioService::PushLocalPcm(uint32_t token, std::vector<int16_t>& pcm) {
    if (!codec_ || token == 0 || pcm.empty() || codec_->output_channels() != 1 ||
        pcm.size() > static_cast<size_t>(codec_->output_sample_rate()) * 20 / 1000) return false;
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    if (!audio_queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this, token]() {
            return service_stopped_ || token != local_playback_token_ ||
                (!local_playback_paused_ && audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE);
        })) return false;
    if (service_stopped_ || token != local_playback_token_) return false;
    auto task = std::make_unique<AudioTask>();
    task->type = kAudioTaskTypeLocalPlayback;
    task->generation = token;
    task->pcm = std::move(pcm);
    audio_playback_queue_.push_back(std::move(task));
    audio_queue_cv_.notify_all();
    return true;
}

void AudioService::PauseLocalPlayback(uint32_t token, bool paused) {
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (token == 0 || token != local_playback_token_) return;
        local_playback_paused_ = paused;
        audio_queue_cv_.notify_all();
    }
    std::lock_guard<std::mutex> output_lock(output_mutex_);
}

void AudioService::EndLocalPlayback(uint32_t token) {
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (token == 0 || token != local_playback_token_) return;
        local_playback_token_ = 0;
        local_playback_paused_ = false;
        ++decode_generation_;
        audio_playback_queue_.erase(std::remove_if(audio_playback_queue_.begin(), audio_playback_queue_.end(),
            [token](const auto& task) {
                return task->type == kAudioTaskTypeLocalPlayback && task->generation == token;
            }), audio_playback_queue_.end());
        audio_queue_cv_.notify_all();
    }
    std::lock_guard<std::mutex> output_lock(output_mutex_);
}

void AudioService::ResetEncoder() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    ++encode_generation_;
    audio_encode_queue_.clear();
    audio_send_queue_.clear();
    audio_queue_cv_.notify_all();
}

void AudioService::CheckAndUpdateAudioPowerState() {
    auto now = std::chrono::steady_clock::now();
    int64_t input_elapsed;
    int64_t output_elapsed;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        input_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_input_time_).count();
        output_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_output_time_).count();
    }
    if (input_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->input_enabled()) {
        codec_->EnableInput(false);
    }
    if (output_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->output_enabled()) {
        // Keep TX clock when duplex RX is active; otherwise RX may stall on some boards.
        if (!(codec_->duplex() && codec_->input_enabled())) {
            codec_->EnableOutput(false);
        }
    }
    if (!codec_->input_enabled() && !codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
    }
}

void AudioService::SetModelsList(srmodel_list_t* models_list) {
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
    models_list_ = models_list;
    wake_word_initialized_ = false;
    xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
    if (esp_srmodel_filter(models_list_, ESP_MN_PREFIX, NULL) != nullptr) {
        wake_word_ = std::make_unique<CustomWakeWord>();
    } else if (esp_srmodel_filter(models_list_, ESP_WN_PREFIX, NULL) != nullptr) {
        wake_word_ = std::make_unique<AfeWakeWord>();
    } else {
        wake_word_ = nullptr;
    }
#else
    if (esp_srmodel_filter(models_list_, ESP_WN_PREFIX, NULL) != nullptr) {
        wake_word_ = std::make_unique<EspWakeWord>();
    } else {
        wake_word_ = nullptr;
    }
#endif

    if (wake_word_) {
        wake_word_->OnWakeWordDetected([this](const std::string& wake_word) {
            if (!IsLocalPlaybackActive() && callbacks_.on_wake_word_detected) {
                callbacks_.on_wake_word_detected(wake_word);
            }
        });
    }
}

bool AudioService::IsAfeWakeWord() {
    std::lock_guard<std::mutex> input_lock(input_control_mutex_);
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
    return wake_word_ != nullptr && dynamic_cast<AfeWakeWord*>(wake_word_.get()) != nullptr;
#else
    return false;
#endif
}
