/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "beat_dance.h"
#include "../stackchan.h"
#include <hal/hal.h>
#include <algorithm>
#include <cstdio>
#include <esp_log.h>
#include <esp_timer.h>

#define TAG "BeatDance"

namespace stackchan {

static std::atomic<int> _beat_dance_active_count{0};

bool isBeatDanceActive()
{
    return _beat_dance_active_count.load() > 0;
}

namespace {

constexpr int kDefaultBpm    = 120;
constexpr int kFeatureWeight = 60;  // eye weight while dancing (squint)
constexpr int kMouthWeight   = 50;

animation::Keyframe make_frame(int yaw, int pitch, int eye_weight, int mouth_weight, int speed, uint32_t duration_ms)
{
    return animation::Keyframe(animation::FeatureKeyframe(0, 0, 0, eye_weight),
                               animation::FeatureKeyframe(0, 0, 0, eye_weight),
                               animation::FeatureKeyframe(0, 0, 0, mouth_weight),
                               animation::ServoKeyframe(yaw, speed), animation::ServoKeyframe(pitch, speed),
                               duration_ms);
}

int clamp_int(int v, int lo, int hi)
{
    return std::max(lo, std::min(hi, v));
}

}  // namespace

BeatDanceModifier::BeatDanceModifier(const std::string& pattern, int bpm, uint32_t duration_ms, int intensity)
    : _timeline(_build_sequence(pattern, bpm, duration_ms, intensity), false), _duration_ms(duration_ms)
{
    _beat_dance_active_count.fetch_add(1);
    _timeline.start();

    BaseType_t ret = xTaskCreatePinnedToCore(_task_entry, "beat_dance", 4096, this, 4, &_task, 0);
    if (ret != pdPASS) {
        // 退化：没有独立任务时至少恢复状态，由 _update 周期驱动（~8 Hz）
        _task = nullptr;
        ESP_LOGE(TAG, "[dance] task create failed, fell back to pool tick");
    }
}

BeatDanceModifier::~BeatDanceModifier()
{
    // 走到这里时 _update 已看到 _done，任务已经完成姿态恢复并退出，不再访问 this
    if (_task != nullptr) {
        // 兜底：常规路径不会进来
        vTaskDelete(_task);
        _task = nullptr;
    }
    _beat_dance_active_count.fetch_sub(1);
}

void BeatDanceModifier::_update(Modifiable& stackchan)
{
    if (_done.load()) {
        requestDestroy();
        return;
    }

    if (_task == nullptr) {
        // 任务创建失败的降级路径：用池子的低频 tick 推进
        _timeline.update();
        if (_timeline.isFinished()) {
            _done.store(true);
        }
    }
}

void BeatDanceModifier::_task_entry(void* param)
{
    static_cast<BeatDanceModifier*>(param)->_run();
}

void BeatDanceModifier::_run()
{
    const uint32_t started = GetHAL().millis();
    uint32_t lock_wait_max = 0;
    uint32_t lock_wait_cnt = 0;
    uint32_t next_sample   = 1000;

    // 现场排障：确认舞蹈任务真的在跑、舵机电流目标有没有变化
    ESP_LOGI(TAG, "[dance] start duration=%u ms, task=%p", (unsigned)_duration_ms, (void*)_task);

    while (true) {
        const bool stop = _stop_requested.load();
        bool finished   = false;

        {
            // 探针：LVGL 锁被别的任务长期霸占（口型动画/图片加载/舵机 I2C）时，
            // 舞蹈帧会被饿住，现场表现就是「跳舞时整个设备像卡死」。
            const int64_t lock_begin_us = esp_timer_get_time();
            LvglLockGuard lock;
            const uint32_t lock_wait_ms = (uint32_t)((esp_timer_get_time() - lock_begin_us) / 1000);
            if (lock_wait_ms > lock_wait_max) {
                lock_wait_max = lock_wait_ms;
            }
            if (lock_wait_ms > 100) {
                ++lock_wait_cnt;
                ESP_LOGW(TAG, "[dance] lvgl lock wait %u ms", (unsigned)lock_wait_ms);
            }
            if (!stop) {
                _timeline.update();
                finished = _timeline.isFinished();
            }
        }

        if (stop || finished) {
            break;
        }

        // 安全阀：任何异常都不让任务永久驻留
        if (GetHAL().millis() - started > _duration_ms + 3000) {
            break;
        }

        // 每秒采样一次实际舵机角度：角度一直不动 = 舵机/总线没响应，
        // 角度在变但人眼看不到 = 幅度太小或被别的 modifier 覆盖。
        const uint32_t elapsed = GetHAL().millis() - started;
        if (elapsed >= next_sample) {
            next_sample += 1000;
            auto& motion = GetStackChan().motion();
            auto angles  = motion.getCurrentAngles();
            auto p_lim   = motion.pitchServo().getAngleLimit();
            ESP_LOGI(TAG, "[dance] t=%u yaw=%d pitch=%d pitch_limit=[%d,%d] lvgl_wait_max=%u", (unsigned)elapsed,
                     angles.x, angles.y, p_lim.x, p_lim.y, (unsigned)lock_wait_max);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    {
        LvglLockGuard lock;
        auto& stackchan = GetStackChan();

        // 回中 + 收起舞蹈表情，回到 happy
        if (_restore_pose.load() && stackchan.hasAvatar()) {
            make_frame(0, 0, 100, 0, 200, 0).apply();
            stackchan.avatar().setEmotion(avatar::Emotion::Happy);
        }
    }

    ESP_LOGI(TAG, "[dance] finished in %u ms (lvgl lock wait max %u ms, >100ms x%u)",
             (unsigned)(GetHAL().millis() - started), (unsigned)lock_wait_max, (unsigned)lock_wait_cnt);

    _task = nullptr;
    _done.store(true);
    vTaskDelete(NULL);
}

animation::KeyframeSequence BeatDanceModifier::_build_sequence(const std::string& pattern, int bpm,
                                                               uint32_t duration_ms, int intensity)
{
    if (bpm <= 0) {
        bpm = kDefaultBpm;
    }
    bpm = clamp_int(bpm, 30, 300);
    intensity = clamp_int(intensity, 0, 100);

    const uint32_t beat_ms = static_cast<uint32_t>(60000 / bpm);
    const uint32_t half_ms = std::max<uint32_t>(40, beat_ms / 2);

    // 俯仰轴的 0° 是「理论上」的中位，但舵机配置里 pitch angleLimit 从 30 起，
    // 任何往 0 走的目标都会被 clamp 成 30。以 0 为基准的摆动（bounce/nod）
    // 因此被砍掉一半幅度，现场看就是「在跳舞但头几乎不动」。
    // 用实际可达下限作为基准，摆动幅度才是完整的。
    int pitch_base = 0;
    {
        auto limit = GetStackChan().motion().pitchServo().getAngleLimit();
        pitch_base = clamp_int(0, limit.x, limit.y);
    }

    // 强度 -> 偏航幅度 5°~20° (舵机单位 0.1°)；速度 150~750
    const int amp_deg = 5 + intensity * 15 / 100;
    const int amp     = clamp_int(amp_deg * 10, 50, 200);
    const int half    = amp / 2;
    const int speed   = clamp_int(150 + intensity * 6, 150, 750);
    const int slow    = clamp_int(120 + intensity * 2, 120, 320);

    // 一个 pattern 单元包含的帧（每帧时长已固定），单位为拍的整数/半整数
    animation::KeyframeSequence unit;
    uint32_t unit_ms = 0;

    if (pattern == "nod") {
        unit = {
            make_frame(0, pitch_base + half, kFeatureWeight, kMouthWeight, speed, beat_ms),
            make_frame(0, pitch_base, kFeatureWeight, kMouthWeight, speed, beat_ms),
        };
    } else if (pattern == "bounce") {
        // bounce 只动俯仰轴，原来只抬 half(默认 7°)，俯仰下限 clamp 掉一半后
        // 只剩 4°，肉眼等于没动。这里用整幅 amp 保证看得出来是在弹。
        unit = {
            make_frame(0, pitch_base + amp, kFeatureWeight, kMouthWeight, speed, half_ms),
            make_frame(0, pitch_base, kFeatureWeight, kMouthWeight, speed, half_ms),
        };
    } else if (pattern == "robot") {
        unit = {
            make_frame(amp, pitch_base, kFeatureWeight, kMouthWeight, 1000, beat_ms),
            make_frame(-amp, pitch_base + half, kFeatureWeight, kMouthWeight, 1000, beat_ms),
        };
    } else if (pattern == "panic") {
        unit = {
            make_frame(amp, pitch_base + half, kFeatureWeight, kMouthWeight, 1000, half_ms),
            make_frame(-amp, pitch_base, kFeatureWeight, kMouthWeight, 1000, half_ms),
        };
    } else if (pattern == "look_around") {
        unit = {
            make_frame(amp, pitch_base, kFeatureWeight, 0, slow, beat_ms * 2),
            make_frame(-amp, pitch_base, kFeatureWeight, 0, slow, beat_ms * 2),
        };
    } else {  // sway (default)
        unit = {
            make_frame(amp, pitch_base, kFeatureWeight, kMouthWeight, speed, beat_ms),
            make_frame(-amp, pitch_base, kFeatureWeight, kMouthWeight, speed, beat_ms),
        };
    }

    for (const auto& frame : unit) {
        unit_ms += frame.durationMs;
    }
    if (unit_ms == 0) {
        unit_ms = beat_ms;
    }

    animation::KeyframeSequence sequence;
    uint32_t total = 0;
    while (total < duration_ms) {
        for (const auto& frame : unit) {
            if (total >= duration_ms) {
                break;
            }
            sequence.push_back(frame);
            total += frame.durationMs;
        }
    }

    if (!sequence.empty() && total > duration_ms) {
        uint32_t excess = total - duration_ms;
        auto& last      = sequence.back();
        last.durationMs = last.durationMs > excess ? last.durationMs - excess : 1;
    }

    return sequence;
}

}  // namespace stackchan
