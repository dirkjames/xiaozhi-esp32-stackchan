/*
 * 测试专用：WAV 麦克风注入 + 无人值守回放脚本。见 wav_test_feeder.h。
 */
#include "wav_test_feeder.h"

#ifdef STACKCHAN_WAV_TEST

#include <esp_log.h>
#include <esp_timer.h>

#include <cstring>

#define TAG "wavtest"

// EMBED_FILES 生成的符号（文件名 → C 标识符）
extern const char wavtest_dance_start[] asm("_binary_wavtest_dance_wav_start");
extern const char wavtest_dance_end[] asm("_binary_wavtest_dance_wav_end");
extern const char wavtest_story_start[] asm("_binary_wavtest_story_wav_start");
extern const char wavtest_story_end[] asm("_binary_wavtest_story_wav_end");
extern const char wavtest_digits_start[] asm("_binary_wavtest_digits_wav_start");
extern const char wavtest_digits_end[] asm("_binary_wavtest_digits_wav_end");

namespace {

struct Clip {
    const char* name = nullptr;
    const int16_t* pcm = nullptr;
    size_t frames = 0;   // 源采样帧数（单声道）
    int rate = 0;        // 源采样率
};

Clip g_clips[3];

// 回放脚本：dance → story → digits 各两轮（WAV_TEST 关闭时不参与编译）
constexpr int kOrder[] = {0, 1, 2, 0, 1, 2};
constexpr int kRounds = (int)(sizeof(kOrder) / sizeof(kOrder[0]));

// 一轮结束后等云端回复的超时（网络/服务端出问题时不让测试卡死）
constexpr int64_t kReplyTimeoutUs = 30LL * 1000 * 1000;
// 回复播完、重新回到 listening 之后再留一点余量，避免抢话
constexpr int64_t kListenGuardUs = 800LL * 1000;

enum Phase {
    kPhaseIdle,       // 还没开始
    kPhasePlaying,    // 正在把录音当麦克风喂给 AFE
    kPhaseWaitReply,  // 录音放完，等云端回复
    kPhaseDone,       // 全部轮次跑完
};

Phase g_phase = kPhaseIdle;
int g_round = 0;
int g_clip = 0;
double g_pos = 0.0;
int64_t g_phase_us = 0;
int64_t g_listen_since_us = 0;
bool g_reply_seen = false;
bool g_reply_announced = false;
DeviceState g_state = kDeviceStateUnknown;
bool g_inited = false;

const char* StateName(DeviceState s) {
    switch (s) {
        case kDeviceStateIdle: return "Idle";
        case kDeviceStateConnecting: return "Connecting";
        case kDeviceStateListening: return "Listening";
        case kDeviceStateSpeaking: return "Speaking";
        default: return "Other";
    }
}

// 解析 canonical WAV；只取 fmt(单声道/16bit) 与 data 块
bool ParseWav(Clip& c, const char* begin, const char* end, const char* name) {
    const size_t total = (size_t)(end - begin);
    c.name = name;
    if (total < 44 || memcmp(begin, "RIFF", 4) != 0 || memcmp(begin + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "%s: not a RIFF/WAVE blob (%u bytes)", name, (unsigned)total);
        return false;
    }
    const char* q = begin + 12;
    int channels = 0;
    int bits = 0;
    int rate = 0;
    const char* data = nullptr;
    size_t data_len = 0;
    while (q + 8 <= end) {
        uint32_t sz = 0;
        memcpy(&sz, q + 4, 4);
        if (memcmp(q, "fmt ", 4) == 0 && q + 8 + 16 <= end) {
            uint16_t ch = 0, bps = 0;
            uint32_t sr = 0;
            memcpy(&ch, q + 10, 2);
            memcpy(&sr, q + 12, 4);
            memcpy(&bps, q + 22, 2);
            channels = ch;
            rate = (int)sr;
            bits = bps;
        } else if (memcmp(q, "data", 4) == 0) {
            data = q + 8;
            data_len = sz;
            if (data + data_len > end) {
                data_len = (size_t)(end - data);
            }
            break;
        }
        q += 8 + sz + (sz & 1);
    }
    if (data == nullptr || channels <= 0 || bits != 16) {
        ESP_LOGE(TAG, "%s: unsupported wav (ch %d, bits %d, data %u)", name, channels, bits,
                 (unsigned)data_len);
        return false;
    }
    c.pcm = reinterpret_cast<const int16_t*>(data);
    c.frames = data_len / sizeof(int16_t) / (size_t)channels;
    c.rate = rate;
    ESP_LOGI(TAG, "clip '%s': %d Hz, %d ch, %u frames (%.2f s)", name, rate, channels,
             (unsigned)c.frames, c.frames / (double)rate);
    return true;
}

void Init() {
    g_inited = true;
    ParseWav(g_clips[0], wavtest_dance_start, wavtest_dance_end, "dance");
    ParseWav(g_clips[1], wavtest_story_start, wavtest_story_end, "story");
    ParseWav(g_clips[2], wavtest_digits_start, wavtest_digits_end, "digits");
    ESP_LOGI(TAG, "ready: %d rounds (%d clips x 2)", kRounds, (int)(sizeof(g_clips) / sizeof(g_clips[0])));
}

void Silence(int16_t* dest, int frames, int channels) {
    memset(dest, 0, (size_t)frames * channels * sizeof(int16_t));
}

// 把当前录音写进 dest；返回 true 表示这条录音已经放完
bool RenderClip(int16_t* dest, int frames, int channels, int sample_rate) {
    const Clip& c = g_clips[g_clip];
    if (c.pcm == nullptr || c.frames < 2 || c.rate <= 0) {
        return true;
    }
    const double step = (double)c.rate / (double)sample_rate;
    bool finished = false;
    int i = 0;
    for (; i < frames; ++i) {
        if (g_pos >= (double)(c.frames - 1)) {
            finished = true;
            break;
        }
        const size_t i0 = (size_t)g_pos;
        const double frac = g_pos - (double)i0;
        const double s = (double)c.pcm[i0] * (1.0 - frac) + (double)c.pcm[i0 + 1] * frac;
        int32_t v = (int32_t)(s + (s >= 0 ? 0.5 : -0.5));
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        dest[i * channels] = (int16_t)v;  // 通道 0 = 麦克风；其余保持静音
        g_pos += step;
    }
    return finished;
}

void StartRound(int64_t now) {
    g_clip = kOrder[g_round];
    g_pos = 0.0;
    g_phase = kPhasePlaying;
    g_phase_us = now;
    const Clip& c = g_clips[g_clip];
    ESP_LOGI(TAG, "round %d/%d start: '%s' (%.2f s)", g_round + 1, kRounds, c.name,
             c.frames / (double)c.rate);
}

void FinishRound(int64_t now, const char* why) {
    (void)now;
    ++g_round;
    if (g_round >= kRounds) {
        g_phase = kPhaseDone;
        ESP_LOGI(TAG, "ALL ROUNDS DONE (%d) — %s; injecting silence from now on", kRounds, why);
        return;
    }
    StartRound(esp_timer_get_time());
}

}  // namespace

namespace wav_test {

void OnDeviceState(DeviceState state) {
    if (!g_inited) {
        Init();
    }
    if (state == g_state) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if (state == kDeviceStateListening || state == kDeviceStateSpeaking ||
        state == kDeviceStateIdle || g_state == kDeviceStateUnknown) {
        ESP_LOGI(TAG, "state %s -> %s (round %d/%d)", StateName(g_state), StateName(state),
                 g_round + 1, kRounds);
    }
    g_state = state;
    if (state == kDeviceStateListening) {
        g_listen_since_us = now;
    } else if (state == kDeviceStateSpeaking && g_phase == kPhaseWaitReply) {
        g_reply_seen = true;
        if (!g_reply_announced) {
            g_reply_announced = true;
            ESP_LOGI(TAG, "round %d/%d reply started", g_round + 1, kRounds);
        }
    }
}

bool Fill(int16_t* dest, int frames, int channels, int sample_rate) {
    if (!g_inited) {
        Init();
    }
    Silence(dest, frames, channels);
    if (frames <= 0 || channels <= 0) {
        return true;
    }

    const int64_t now = esp_timer_get_time();
    const DeviceState st = g_state;

    switch (g_phase) {
        case kPhaseIdle:
            // 开机 greeting 播完、真正进入 listening 后开始第一轮
            if (st == kDeviceStateListening) {
                StartRound(now);
            }
            break;

        case kPhasePlaying: {
            if (RenderClip(dest, frames, channels, sample_rate)) {
                const Clip& c = g_clips[g_clip];
                ESP_LOGI(TAG, "round %d/%d '%s' injected (%.2f s audio), waiting for reply",
                         g_round + 1, kRounds, c.name, c.frames / (double)c.rate);
                g_phase = kPhaseWaitReply;
                // 以「录音放完」为超时基线；若此刻状态已经是 Speaking（极快回复），
                // 也要把 reply_seen 补上，否则会一直等到超时。
                g_phase_us = now;
                g_reply_seen = (st == kDeviceStateSpeaking);
                g_reply_announced = false;
            }
            break;
        }

        case kPhaseWaitReply:
            if (g_reply_seen && st == kDeviceStateListening &&
                (now - g_listen_since_us) >= kListenGuardUs) {
                ESP_LOGI(TAG, "round %d/%d reply done", g_round + 1, kRounds);
                FinishRound(now, "normal");
            } else if ((now - g_phase_us) >= kReplyTimeoutUs) {
                ESP_LOGW(TAG, "round %d/%d TIMEOUT waiting reply (%d s), moving on", g_round + 1,
                         kRounds, (int)(kReplyTimeoutUs / 1000000));
                FinishRound(now, "timeout");
            }
            break;

        case kPhaseDone:
        default:
            break;
    }
    return true;
}

}  // namespace wav_test

#endif  // STACKCHAN_WAV_TEST
