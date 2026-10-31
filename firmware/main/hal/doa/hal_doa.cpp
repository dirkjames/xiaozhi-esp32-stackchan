/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal_doa.h"

#include <application.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/board/hal_bridge.h>
#include <hal/hal.h>
#include <math.h>
#include <settings.h>
#include <stackchan/modifiers/beat_dance.h>
#include <stackchan/stackchan.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>

#define TAG "HalDoa"

using namespace stackchan;

namespace hal_doa {
namespace {

constexpr float kPi           = 3.14159265358979323846f;
constexpr int kWinN           = 1024;  // 64ms @16k
constexpr int kHop            = 320;   // 20ms @16k
constexpr int kHopMs          = 20;    // 每 hop 对应的毫秒数
constexpr int kMedN           = 250;   // 活动门限滑动中位数窗 (5s)
constexpr int kVoteN          = 9;     // 多数表决窗 (180ms)
// 三分类门限沿用 firmware_capture demo 的 1.23 / 4.03（用户三条定位录音自动寻优，
// touch_test2 盲测 91.6%）。**不要**改成围绕 0 对称的窗口：IPD 含约 +2° 的固定
// 通道间偏置（ES7210 双 ADC 顺序采样），正前方说话落在 +2° 附近，对称窗口会把
// MIDDLE 判成 L，头转过去之后就再也不会动。
constexpr float kGateR        = 1.25f;
constexpr float kMinConf      = 0.70f;
constexpr int kBinN           = 21;

// 与离线 doa3class.py BANDS=range(300,825,25) 的 argmin 选 bin 完全一致 (15.625Hz/bin)
constexpr int kBins[kBinN] = {19, 21, 22, 24, 26, 27, 29, 30, 32, 34, 35,
                              37, 38, 40, 42, 43, 45, 46, 48, 50, 51};

// 控制律 v9 = firmware_capture demo 的 v7（2026-10-16 九轮实录定案）原样移植。
// 关键：IPD 含 +2° 固定偏置，所以「中心判决」和「某一侧的返向判决」都必须是
// 非对称门限。头朝左（_head_deg > 0）时所有方向证据都同号且整体 +2°，只能靠
// 幅度分级：> +4.0 回中、> +8.0 换向右；头朝右（_head_deg < 0）时偏置友好：
// < -2.5 回中、< -4.5 换向左。
constexpr int kCtlRunN      = 15;  // 15 个活动窗 (~0.3s 有效语音) 填满才判决
constexpr float kCenterTurnL = -1.0f;  // 中心: 均值 < 此值 → 转左 (左用户 ~-2)
constexpr float kCenterTurnR = 3.8f;   // 中心: 均值 > 此值 → 转右 (中用户 +2.5~3.3)
constexpr float kUndoNeg     = 2.5f;   // 头朝右: < -此值 → 回中
constexpr float kFlipNeg     = 4.5f;   // 头朝右: < -此值 → 换向左
constexpr float kUndoPos     = 4.0f;   // 头朝左: > +此值 → 回中
constexpr float kFlipPos     = 8.0f;   // 头朝左: > +此值 → 换向右
constexpr int kUndoRunN      = 8;
constexpr int kFlipRunN      = 2;
constexpr int kHeadCoolN     = 50;   // 目标下发后冷却 1s
constexpr int kHeadFreezeN   = 50;   // 冻结窗 (1s)
constexpr int kHeadSettleN   = 25;   // 额外缓动余量 (~0.5s)
constexpr int kHeadSpeed     = 700;

constexpr size_t kRingFrames = 2048;  // 128ms 立体声 @16k

struct Buffers {
    float wl[kWinN];
    float wr[kWinN];
    float re1[kWinN];
    float im1[kWinN];
    float re2[kWinN];
    float im2[kWinN];
    float hann[kWinN];
    float tw_re[kWinN / 2];
    float tw_im[kWinN / 2];
    uint16_t bitrev[kWinN];
    float rms_ring[kMedN];
    float tmp[kMedN];
};

// DSP 工作区走内部 RAM（FFT 每秒跑 100 次，PSRAM 带宽不够）；
// 环形缓冲只做顺序读写，放 PSRAM 即可。
Buffers* _buf  = nullptr;
int16_t* _ring = nullptr;

std::atomic<bool> _enabled{true};
// 默认开启「听声转头」：mcp 工具 robot.look_at_sound 仍可随时关掉。
// 之前默认 false，云端从不主动开启，导致现场看 DOA「没生效」。
std::atomic<bool> _look_at_sound{true};
std::atomic<uint32_t> _wpos{0};
uint32_t _rpos = 0;
int _fill      = 0;

// 滚动窗/表决状态
int _rms_n = 0, _rms_idx = 0, _med_cnt = 0;
float _gate = 0;
char _vote[kVoteN];
int _vote_n = 0, _vote_idx = 0;
char _last_cls = '-';
// Listening 下连续无有效语音的累计时长（ms），用于「长时间不说话自动回正」
uint32_t _quiet_ms = 0;
uint32_t _last_notify_ms = 0;
uint32_t _frames = 0;

// v7 控制环状态
float _ctl_buf[kCtlRunN];
int _ctl_n = 0, _ctl_idx = 0;
int _undo_run = 0, _flip_run = 0;
int _freeze = 0;
int _cool   = 0;
float _head_deg = 0;
char _head_cls  = 0;

// NVS 参数
std::atomic<float> _th_lo{1.23f};
std::atomic<float> _th_hi{4.03f};
std::atomic<int> _yaw_sign{-1};
std::atomic<int> _yaw_zero{460};
std::atomic<int> _max_deg{60};
std::atomic<float> _gate_floor{150.0f};

Result _result;
std::mutex _result_mutex;

// 3:2 抽取残组（仅生产者访问）
int16_t _grp[3][2];
int _grp_n = 0;

void _ctl_reset()
{
    _ctl_n = _ctl_idx = 0;
    _undo_run = _flip_run = 0;
}

void _load_params()
{
    Settings settings("doa", false);
    _th_lo.store(settings.GetInt("th_lo_x100", 123) / 100.0f);
    _th_hi.store(settings.GetInt("th_hi_x100", 403) / 100.0f);
    _yaw_sign.store(settings.GetInt("sign", -1));
    _yaw_zero.store(settings.GetInt("zero", 460));
    _max_deg.store(settings.GetInt("max_deg", 60));
    _gate_floor.store((float)settings.GetInt("gate_floor", 150));
}

void _push_frame(int16_t l, int16_t r)
{
    const uint32_t w = _wpos.load(std::memory_order_relaxed);
    const size_t idx = (w % kRingFrames) * 2;
    _ring[idx]     = l;
    _ring[idx + 1] = r;
    _wpos.store(w + 1, std::memory_order_release);
}

// y[2m] = x[3m]; y[2m+1] = (x[3m+1] + x[3m+2]) / 2
// 3:2 抽取 (24k -> 16k)；L/R 过同一线性系统，300-800Hz 的 IPD 保持不变
void _emit_group()
{
    _push_frame(_grp[0][0], _grp[0][1]);
    _push_frame((int16_t)((_grp[1][0] + _grp[2][0]) >> 1), (int16_t)((_grp[1][1] + _grp[2][1]) >> 1));
}

void _fft(float* re, float* im)
{
    for (int i = 0; i < kWinN; ++i) {
        const int j = _buf->bitrev[i];
        if (j > i) {
            float t = re[i];
            re[i]   = re[j];
            re[j]   = t;
            t       = im[i];
            im[i]   = im[j];
            im[j]   = t;
        }
    }
    for (int len = 2; len <= kWinN; len <<= 1) {
        const int step = kWinN / len, half = len >> 1;
        for (int i = 0; i < kWinN; i += len) {
            for (int k = 0; k < half; ++k) {
                const float wr = _buf->tw_re[k * step], wi = _buf->tw_im[k * step];
                const float xr = re[i + k + half], xi = im[i + k + half];
                const float tr = xr * wr - xi * wi;
                const float ti = xr * wi + xi * wr;
                re[i + k + half] = re[i + k] - tr;
                im[i + k + half] = im[i + k] - ti;
                re[i + k] += tr;
                im[i + k] += ti;
            }
        }
    }
}

bool _should_freeze()
{
    if (!_enabled.load(std::memory_order_relaxed)) {
        return true;
    }
    // 只在「听用户说话」时转头。待机/连接中/播报中的方向要么不可信（喇叭回采），
    // 要么根本不该动：之前只排除 Speaking，待机时车载底噪（风噪/路噪）会被判成语音方向，
    // 导致不聊天也一直左右甩头。
    if (Application::GetInstance().GetDeviceState() != kDeviceStateListening) {
        return true;
    }
    // 喇叭正在出声（TTS 尾音 / 本地 sfx 比如情绪音效）：麦克风回采到的是自己的声音，
    // 方向完全不可信。实测 Listening 期间播 "happy" 音效后 130ms 就会误判成 R 并甩头 -60°。
    if (Application::GetInstance().GetAudioService().IsOutputActive()) {
        return true;
    }
    // 舞蹈中：与 DOA 转头互斥
    if (isBeatDanceActive()) {
        return true;
    }
    return false;
}

void _command_head(float deg)
{
    const float delta = fabsf(deg - _head_deg);
    _head_deg         = deg;

    {
        // 只下发舵机目标角。不要在 DOA 任务里改 UI（display_apply_expression 会
        // 往 StackChan 的 modifier 列表里加定时表情）：那是跨任务的 LVGL 图元修改，
        // 会和主任务的状态切换（SetStatus/SetEmotion）抢同一把锁，实测会把主任务
        // 拖到秒级甚至卡死 —— 主任务一停，上行音频和 MCP 响应就全停了。
        LvglLockGuard lock;
        GetStackChan().motion().yawServo().moveWithSpeed((int)(deg * 10.0f), kHeadSpeed);
    }

    // 缓动中 + 到位后 1s 冻结判决（舵机噪声 IPD 随机，会撕裂表决）
    _freeze = kHeadFreezeN + kHeadSettleN + (int)(delta / 3.0f);
    ESP_LOGI(TAG, "head -> %+.0f deg", (double)deg);
}

void _send_notification(char cls, float ipd, float confidence)
{
    char payload[256];
    snprintf(payload, sizeof(payload),
             R"({"jsonrpc":"2.0","method":"notifications/robotics","params":{"event":"sound_direction","data":{"cls":"%c","ipd_deg":%.2f,"confidence":%.2f,"yaw_deg":%.0f},"ts":%u}})",
             cls, (double)ipd, (double)confidence, (double)_head_deg,
             (unsigned)(GetHAL().millis()));
    Application::GetInstance().SendMcpMessage(payload);
}

void _update_head(float ipd, bool active)
{
    if (!_look_at_sound.load(std::memory_order_relaxed)) {
        return;
    }
    if (isBeatDanceActive()) {
        return;
    }
    if (_cool > 0) {
        --_cool;
    }

    // 不做「静默 N 秒自动回中」：用户明确要求的时序是
    //   转向我 -> 我说完 -> TTS 播报（播报中不转头）-> 播报完回 Listening
    //   -> 我再说话 -> 重新做声源定位 -> 再转向我
    // 也就是说脑袋应该停在上一次的方向，只在「新的一句话给出新方向」时才动。

    // 控制环只喂活动窗
    if (active) {
        _ctl_buf[_ctl_idx] = ipd;
        _ctl_idx           = (_ctl_idx + 1) % kCtlRunN;
        if (_ctl_n < kCtlRunN) {
            ++_ctl_n;
        }
    }

    if (_ctl_n >= kCtlRunN && _cool == 0) {
        float mean = 0;
        for (int i = 0; i < kCtlRunN; ++i) {
            mean += _ctl_buf[i];
        }
        mean /= kCtlRunN;

        const float max_deg = (float)_max_deg.load(std::memory_order_relaxed);
        const float sign    = (float)_yaw_sign.load(std::memory_order_relaxed);

        if (_head_deg == 0.0f) {
            char tgt = 0;
            if (mean < kCenterTurnL) {
                tgt = 'L';
            } else if (mean > kCenterTurnR) {
                tgt = 'R';
            }
            if (tgt != 0) {
                _head_cls = tgt;
                _cool     = kHeadCoolN;
                _ctl_reset();
                _command_head(sign * ((tgt == 'L') ? -max_deg : max_deg));
            }
        } else {
            bool opp = false, deep = false;
            if (_head_deg < 0.0f) {
                opp  = mean < -kUndoNeg;
                deep = mean < -kFlipNeg;
            } else {
                opp  = mean > kUndoPos;
                deep = mean > kFlipPos;
            }
            _undo_run = opp ? (_undo_run + 1) : 0;
            _flip_run = deep ? (_flip_run + 1) : 0;

            // demo v7 原样：深证据连续 2 次 → 直接换到对侧；浅证据连续 8 次 → 回中。
            // 注意**不能**用「mean 落在哪个三分类区间」来推目标：头已经偏到一侧时，
            // 中间用户的证据会整体平移（头朝左时中间用户就是 +5），必须按 demo 的
            // 幅度分级来判断「回中」还是「换向」，否则中间用户会被判成「换到右」。
            if (_flip_run >= kFlipRunN) {
                _undo_run = _flip_run = 0;
                _cool                 = kHeadCoolN;
                _ctl_reset();
                _head_cls = (_head_deg < 0.0f) ? 'L' : 'R';
                _command_head((_head_deg < 0.0f) ? (sign * -max_deg) : (sign * max_deg));
            } else if (_undo_run >= kUndoRunN) {
                _undo_run = 0;
                _cool     = kHeadCoolN;
                _ctl_reset();
                _head_cls = 0;
                _command_head(0.0f);
            }
        }
    }
}

void _process_window()
{
    bool frozen = (_freeze > 0) || _should_freeze();
    if (_freeze > 0) {
        --_freeze;
    }

    // 加窗 + RMS（L 通道，同离线口径）
    float rms2 = 0;
    for (int i = 0; i < kWinN; ++i) {
        const float l = _buf->wl[i] * _buf->hann[i];
        const float r = _buf->wr[i] * _buf->hann[i];
        _buf->re1[i]  = l;
        _buf->im1[i]  = 0;
        _buf->re2[i]  = r;
        _buf->im2[i]  = 0;
        rms2 += l * l;
    }
    const float rms = sqrtf(rms2 / kWinN);

    // 滑动中位数活动门限（每 25 窗=0.5s 重算，窗 5s）
    // 冻结窗不入环：舵机噪声会把门限抬高，解冻后误吞语音
    if (!frozen) {
        _buf->rms_ring[_rms_idx] = rms;
        _rms_idx                 = (_rms_idx + 1) % kMedN;
        if (_rms_n < kMedN) {
            ++_rms_n;
        }
    }
    if (++_med_cnt >= 25 && _rms_n >= 100) {
        _med_cnt = 0;
        memcpy(_buf->tmp, _buf->rms_ring, sizeof(float) * _rms_n);
        std::nth_element(_buf->tmp, _buf->tmp + _rms_n / 2, _buf->tmp + _rms_n);
        _gate = kGateR * _buf->tmp[_rms_n / 2];
        if (_gate < _gate_floor.load(std::memory_order_relaxed)) {
            _gate = _gate_floor.load(std::memory_order_relaxed);
        }
    }

    _fft(_buf->re1, _buf->im1);
    _fft(_buf->re2, _buf->im2);

    // 跨谱幅度 |FL||FR| 加权平均 IPD（度）
    float wsum = 0, isum = 0;
    for (int bi = 0; bi < kBinN; ++bi) {
        const int b    = kBins[bi];
        const float lr = _buf->re1[b], li = _buf->im1[b];
        const float rr = _buf->re2[b], ri = _buf->im2[b];
        const float cr = rr * lr + ri * li;  // FR * conj(FL)
        const float ci = ri * lr - rr * li;
        const float w  = sqrtf(lr * lr + li * li) * sqrtf(rr * rr + ri * ri);
        if (w <= 0) {
            continue;  // 静音 bin: atan2(0,0) 未定义
        }
        isum += atan2f(ci, cr) * w;
        wsum += w;
    }
    const float ipd = (wsum > 0) ? isum / wsum * 57.29578f : 0.0f;

    // 判决 + 多数表决（冻结窗不判决并清空表决环）
    // 活动门限与 demo 完全一致：中位数(5s)×1.25，绝对下限 150（实测环境噪声 rms 43~110）。
    // 之前额外叠加了 AFE VAD 闸门 + 600 绝对音量下限，实测语音窗 rms 只有 300~1200，
    // 大量有效窗被吞掉，控制环长期填不满 → 头不动。这里恢复 demo 口径。
    const bool vad    = Application::GetInstance().GetAudioService().IsVoiceDetected();
    const bool active = !frozen && (_gate > 0) && (rms > _gate);

    const float th_lo = _th_lo.load(std::memory_order_relaxed);
    const float th_hi = _th_hi.load(std::memory_order_relaxed);
    const char cls    = active ? (ipd < th_lo ? 'L' : (ipd > th_hi ? 'R' : 'M')) : '-';

    if (frozen) {
        _vote_n = _vote_idx = 0;
    } else {
        _vote[_vote_idx] = cls;
        _vote_idx        = (_vote_idx + 1) % kVoteN;
        if (_vote_n < kVoteN) {
            ++_vote_n;
        }
    }

    int cnt_l = 0, cnt_m = 0, cnt_r = 0, cnt_act = 0;
    for (int i = 0; i < _vote_n; ++i) {
        if (_vote[i] == 'L') {
            ++cnt_l;
            ++cnt_act;
        } else if (_vote[i] == 'M') {
            ++cnt_m;
            ++cnt_act;
        } else if (_vote[i] == 'R') {
            ++cnt_r;
            ++cnt_act;
        }
    }

    char maj = '-';
    if (!frozen && cnt_act >= 5) {
        maj = (cnt_l >= cnt_m && cnt_l >= cnt_r) ? 'L' : (cnt_m >= cnt_r) ? 'M' : 'R';
    }

    const int max_cnt = (maj == 'L') ? cnt_l : (maj == 'M') ? cnt_m : (maj == 'R') ? cnt_r : 0;
    const float confidence = (cnt_act > 0) ? (float)max_cnt / (float)cnt_act : 0.0f;

    ++_frames;

    {
        std::lock_guard<std::mutex> lock(_result_mutex);
        _result.cls        = maj;
        _result.ipd_deg    = ipd;
        _result.confidence = confidence;
        _result.ts_ms      = GetHAL().millis();
    }

    const bool changed = (maj != _last_cls);
    // 类别变化即打；另外每 50 窗（1s）打一条，方便用户复测时对照「正前/左/右」看 ipd，
    // 不用等类别变化才出日志（之前只有变化才打，复测时几乎看不到数据）。
    if (changed || (_frames % 50) == 0) {
        // 双通道能量诊断：两路 rms 应当接近但不相等（同一壳体内两只麦），
        // 若 R 恒为 0 说明 I2S slot 映射错了，右路根本没数据。
        float sl = 0.0f, sr = 0.0f;
        for (int i = 0; i < kWinN; ++i) {
            sl += _buf->wl[i] * _buf->wl[i];
            sr += _buf->wr[i] * _buf->wr[i];
        }
        const float rms_l = sqrtf(sl / kWinN);
        const float rms_r = sqrtf(sr / kWinN);
        ESP_LOGI(TAG, "dir %c (ipd %+.2f rms %.0f L/R %.0f/%.0f gate %.0f conf %.2f vad %d)%s", maj, (double)ipd,
                 (double)rms, (double)rms_l, (double)rms_r, (double)_gate, (double)confidence, vad ? 1 : 0, frozen ? " F" : "");
        // M/-/M 快速抖动时不要让通知刷屏：至少间隔 800ms 才上报一次
        const uint32_t now_ms = GetHAL().millis();
        if (changed && maj != '-' && (now_ms - _last_notify_ms) >= 800) {
            _last_notify_ms = now_ms;
            _send_notification(maj, ipd, confidence);
        }
        _last_cls = maj;
    }

    // 只有「音量够 + 表决够一致」的窗才进控制环，避免噪声把脑袋甩来甩去
    _update_head(ipd, active && confidence >= kMinConf);

    // 用户要求的兜底：Listening 下连续 30s 没有有效语音 → 头回正中间。
    // 播报中（frozen）不计时，避免"播报 30s 后被强行回正"。
    if (!frozen && !active) {
        _quiet_ms += kHopMs;
        if (_quiet_ms >= 30000 && _head_deg != 0.0f) {
            _quiet_ms = 0;
            _head_cls = 0;
            _ctl_reset();
            _command_head(0.0f);
            ESP_LOGI(TAG, "head recenter: 30s idle without speech");
        }
    } else {
        _quiet_ms = 0;
    }

    // 窗移位（hop = 320）
    memmove(_buf->wl, _buf->wl + kHop, sizeof(float) * (kWinN - kHop));
    memmove(_buf->wr, _buf->wr + kHop, sizeof(float) * (kWinN - kHop));
    _fill = kWinN - kHop;
}

void _drain()
{
    const uint32_t w = _wpos.load(std::memory_order_acquire);
    if (w == _rpos) {
        return;
    }
    if (w - _rpos > kRingFrames) {
        // 生产过快：丢弃旧数据，从最新窗口重新开始
        _rpos  = w - kRingFrames;
        _fill  = 0;
        _vote_n = _vote_idx = 0;
    }

    while (_rpos != w) {
        const int space = kWinN - _fill;
        const int avail = (int)std::min<uint32_t>(w - _rpos, (uint32_t)space);
        for (int i = 0; i < avail; ++i) {
            const size_t idx     = ((_rpos + i) % kRingFrames) * 2;
            _buf->wl[_fill + i]  = (float)_ring[idx];
            _buf->wr[_fill + i]  = (float)_ring[idx + 1];
        }
        _fill += avail;
        _rpos += avail;
        if (_fill == kWinN) {
            _process_window();
        }
    }
}

void _task_entry(void*)
{
    while (true) {
        _drain();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}  // namespace

bool init()
{
    static bool inited = false;
    if (inited) {
        return true;
    }

    // 优先 PSRAM：Buffers 约 36KB，xiozhi 启动后内部 SRAM 只剩 6KB 左右，
    // 内部 RAM 被吃掉后会连带 TLS/WebSocket 的接收任务创建失败（xTaskCreate 返回失败），
    // 表现为 OTA 请求发出去后收不到任何响应。
    _buf = static_cast<Buffers*>(heap_caps_malloc(sizeof(Buffers), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (_buf == nullptr) {
        _buf = static_cast<Buffers*>(heap_caps_malloc(sizeof(Buffers), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (_buf == nullptr) {
        ESP_LOGE(TAG, "no memory for DOA buffers (%u bytes)", (unsigned)sizeof(Buffers));
        return false;
    }
    memset(_buf, 0, sizeof(Buffers));

    _ring = static_cast<int16_t*>(heap_caps_malloc(sizeof(int16_t) * kRingFrames * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (_ring == nullptr) {
        _ring = static_cast<int16_t*>(heap_caps_malloc(sizeof(int16_t) * kRingFrames * 2, MALLOC_CAP_8BIT));
    }
    if (_ring == nullptr) {
        ESP_LOGE(TAG, "no memory for DOA ring");
        heap_caps_free(_buf);
        _buf = nullptr;
        return false;
    }
    memset(_ring, 0, sizeof(int16_t) * kRingFrames * 2);

    for (int i = 0; i < kWinN; ++i) {
        _buf->hann[i] = 0.5f * (1.0f - cosf(2.0f * kPi * i / (kWinN - 1)));
        uint32_t r = 0, x = (uint32_t)i;
        for (int b = 0; b < 10; ++b) {
            r = (r << 1) | (x & 1);
            x >>= 1;
        }
        _buf->bitrev[i] = (uint16_t)r;
    }
    for (int k = 0; k < kWinN / 2; ++k) {
        const float a  = -2.0f * kPi * k / kWinN;
        _buf->tw_re[k] = cosf(a);
        _buf->tw_im[k] = sinf(a);
    }

    _load_params();
    reset();

    xTaskCreatePinnedToCore(_task_entry, "hal_doa", 4096, nullptr, 2, nullptr, 0);
    inited = true;

    ESP_LOGI(TAG, "init done: win=%d hop=%d ring=%u", kWinN, kHop, (unsigned)kRingFrames);
    return true;
}

void on_capture(const int16_t* stereo, int frames)
{
    if (_buf == nullptr || stereo == nullptr || frames <= 0) {
        return;
    }
    if (!_enabled.load(std::memory_order_relaxed)) {
        return;
    }

    int i = 0;

    // 补齐上次跨调用残留的 3 帧组
    while (_grp_n > 0 && i < frames) {
        _grp[_grp_n][0] = stereo[i * 2];
        _grp[_grp_n][1] = stereo[i * 2 + 1];
        ++_grp_n;
        ++i;
        if (_grp_n == 3) {
            _emit_group();
            _grp_n = 0;
        }
    }

    // 整组 3:2 抽取
    for (; i + 2 < frames; i += 3) {
        const int16_t* g = stereo + i * 2;
        _push_frame(g[0], g[1]);
        _push_frame((int16_t)((g[2] + g[4]) >> 1), (int16_t)((g[3] + g[5]) >> 1));
    }

    // 末尾不足一组的留到下次
    for (; i < frames; ++i) {
        _grp[_grp_n][0] = stereo[i * 2];
        _grp[_grp_n][1] = stereo[i * 2 + 1];
        ++_grp_n;
    }
}

Result get_result()
{
    std::lock_guard<std::mutex> lock(_result_mutex);
    return _result;
}

void set_enabled(bool enabled)
{
    _enabled.store(enabled, std::memory_order_relaxed);
}

bool is_enabled()
{
    return _enabled.load(std::memory_order_relaxed);
}

void set_look_at_sound(bool enabled)
{
    _look_at_sound.store(enabled, std::memory_order_relaxed);
    if (!enabled && _head_deg != 0.0f) {
        _command_head(0.0f);
    }
    ESP_LOGI(TAG, "look_at_sound: %d", enabled ? 1 : 0);
}

bool is_look_at_sound()
{
    return _look_at_sound.load(std::memory_order_relaxed);
}

void reset()
{
    _rms_n = _rms_idx = _med_cnt = 0;
    _gate     = 0;
    _vote_n   = _vote_idx = 0;
    _last_cls = '-';
    _frames   = 0;
    _cool     = 0;
    _freeze   = 0;
    _grp_n    = 0;
    _ctl_reset();

    std::lock_guard<std::mutex> lock(_result_mutex);
    _result = Result{};
}

bool set_params(float th_lo, float th_hi, int sign, int zero, int max_deg, float gate_floor)
{
    if (th_lo < 0.0f || th_hi <= th_lo || th_hi > 45.0f) {
        ESP_LOGW(TAG, "invalid ipd thresholds: %.2f / %.2f", (double)th_lo, (double)th_hi);
        return false;
    }
    if (max_deg < 0 || max_deg > 120 || gate_floor < 0.0f) {
        ESP_LOGW(TAG, "invalid params: max_deg=%d gate_floor=%.0f", max_deg, (double)gate_floor);
        return false;
    }

    _th_lo.store(th_lo);
    _th_hi.store(th_hi);
    _yaw_sign.store(sign < 0 ? -1 : 1);
    _yaw_zero.store(zero);
    _max_deg.store(max_deg);
    _gate_floor.store(gate_floor);

    Settings settings("doa", true);
    settings.SetInt("th_lo_x100", (int)(th_lo * 100.0f));
    settings.SetInt("th_hi_x100", (int)(th_hi * 100.0f));
    settings.SetInt("sign", _yaw_sign.load());
    settings.SetInt("zero", zero);
    settings.SetInt("max_deg", max_deg);
    settings.SetInt("gate_floor", (int)gate_floor);

    ESP_LOGI(TAG, "params updated: %s", get_params_json().c_str());
    return true;
}

std::string get_params_json()
{
    char buf[192];
    snprintf(buf, sizeof(buf),
             R"({"th_lo":%.2f,"th_hi":%.2f,"sign":%d,"zero":%d,"max_deg":%d,"gate_floor":%.0f})",
             (double)_th_lo.load(), (double)_th_hi.load(), _yaw_sign.load(), _yaw_zero.load(), _max_deg.load(),
             (double)_gate_floor.load());
    return std::string(buf);
}

}  // namespace hal_doa
