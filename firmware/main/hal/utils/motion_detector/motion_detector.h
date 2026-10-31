/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <cstdint>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Gesture level motion events derived from the accelerometer
 *
 * Shake  keeps the historical differential/high-pass rule and is still polled
 * through isShakeDetected(); the other gestures are queued and drained with
 * pollEvent() so that a single sample only reports one event.
 */
enum class MotionEvent {
    None = 0,
    Shake,
    PickUp,
    Brake,
    Turn,
    Bump,
};

class MotionDetector {
public:
    static constexpr float kGravity = 9.81f;

    /**
     * @brief Set the shake detection threshold.
     * @param threshold Higher value needs stronger shake. Default is 4.0f.
     *                  - 2.0f: Very sensitive (light shake)
     *                  - 4.0f: Normal
     *                  - 8.0f: Hard shake required
     */
    void setShakeThreshold(float threshold)
    {
        _shake_threshold = threshold;
    }

    void update(const float& acc_x, const float& acc_y, const float& acc_z)
    {
        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

        // 首个采样点没有参考值，只记录基线
        if (!_has_prev) {
            _prev_acc_x = acc_x;
            _prev_acc_y = acc_y;
            _prev_acc_z = acc_z;
            _has_prev   = true;
            // 头部静止时的姿态角作为 Turn 判据基线，避免开机初始倾角误判
            _roll_base = std::atan2(acc_y, acc_z) * 57.2957795f;
            return;
        }

        const float horiz      = std::sqrt(acc_x * acc_x + acc_y * acc_y);
        const float horiz_prev = std::sqrt(_prev_acc_x * _prev_acc_x + _prev_acc_y * _prev_acc_y);

        // Use differential (change in acceleration) for shake detection.
        // This effectively acts as a high-pass filter and is independent of gravity orientation.
        float diff = std::abs(acc_x - _prev_acc_x) + std::abs(acc_y - _prev_acc_y) + std::abs(acc_z - _prev_acc_z);
        const float dz = acc_z - _prev_acc_z;

        _prev_acc_x = acc_x;
        _prev_acc_y = acc_y;
        _prev_acc_z = acc_z;

        // --- Shake Detection ---
        if (diff > _shake_threshold) {
            if (now - _last_shake_peak_time > 100) {       // Debounce 100ms
                if (now - _last_shake_peak_time < 1000) {  // Window 1s
                    _shake_count++;
                } else {
                    _shake_count = 1;  // Reset sequence
                }
                _last_shake_peak_time = now;

                if (_shake_count >= 3) {
                    _shake_detected = true;
                    _shake_count    = 0;
                }
            }
        }

        // --- Bump: 竖直方向冲击 ---
        if (std::abs(dz) > 15.0f) {
            _push_event(MotionEvent::Bump, now);
        }

        // --- PickUp: 失重后 1s 内恢复重力 ---
        const float mag = std::sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
        if (mag < 3.0f) {
            _freefall      = true;
            _freefall_time = now;
        } else if (_freefall) {
            if (now - _freefall_time < 1000 && mag > 7.0f) {
                _freefall = false;
                _push_event(MotionEvent::PickUp, now);
            } else if (now - _freefall_time >= 1000) {
                _freefall = false;
            }
        }

        // --- Brake: 水平分量突变 ---
        if (std::abs(horiz - horiz_prev) > 15.0f) {
            _push_event(MotionEvent::Brake, now);
        }

        // --- Turn: 相对静止基线侧倾持续 3 帧 ---
        const float roll_deg = std::atan2(acc_y, acc_z) * 57.2957795f;
        const float roll_delta = roll_deg - _roll_base;
        if (std::abs(roll_delta) > 40.0f) {
            if (++_tilt_frames == 3) {
                _push_event(MotionEvent::Turn, now);
            }
        } else {
            _tilt_frames = 0;
            // 静止时让基线缓慢跟随，避免挪动过设备后一直被判定为侧倾
            if (std::abs(roll_delta) < 10.0f) {
                _roll_base += roll_delta * 0.05f;
            }
        }
    }

    bool isShakeDetected()
    {
        if (_shake_detected) {
            _shake_detected = false;
            return true;
        }
        return false;
    }

    /**
     * @brief Pop the pending gesture event
     *
     * @return MotionEvent::None when nothing was detected since the last poll
     */
    MotionEvent pollEvent()
    {
        MotionEvent event = _pending_event;
        _pending_event    = MotionEvent::None;
        return event;
    }

private:
    void _push_event(MotionEvent event, uint32_t now)
    {
        // 500ms 全局冷却，避免一次动作被多个判据重复上报
        if (_last_event_time != 0 && now - _last_event_time < 500) {
            return;
        }
        _last_event_time = now;
        _pending_event   = event;
    }

    int _shake_count               = 0;
    uint32_t _last_shake_peak_time = 0;
    bool _shake_detected           = false;
    float _shake_threshold         = 4.0f;

    MotionEvent _pending_event = MotionEvent::None;
    uint32_t _last_event_time  = 0;

    bool _freefall          = false;
    uint32_t _freefall_time = 0;

    int _tilt_frames = 0;
    float _roll_base = 0.0f;

    bool _has_prev    = false;
    float _prev_acc_x = 0;
    float _prev_acc_y = 0;
    float _prev_acc_z = 0;
};
