/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include "../modifiable.h"
#include "../animation/animation.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <string>

namespace stackchan {

/**
 * @brief Whether a beat dance is currently running (used to mute DOA head turns)
 *
 */
bool isBeatDanceActive();

/**
 * @brief BPM driven head dance
 *
 * Why a dedicated 20 ms task instead of relying on `StackChan::update()`:
 * while a xiaozhi session is active the HAL update task adds another 100 ms
 * delay per loop (`hal.cpp`), so the modifier pool is only ticked at ~8 Hz.
 * That is far too coarse for a 120 BPM beat, so this modifier drives its own
 * `animation::Timeline` from a FreeRTOS task and only reports completion
 * through `_update()` (which then requests destruction from the normal pool).
 *
 * Lifetime contract:
 * - the task never touches `this` after setting `_done`;
 * - `_update()` only requests destruction once `_done` is set, i.e. after the
 *   task has already restored the pose and stopped taking the lvgl lock.
 *
 * @param pattern sway / nod / bounce / robot / panic / look_around
 * @param bpm beats per minute, <=0 falls back to 120
 * @param duration_ms total dance length
 * @param intensity 0~100, maps to yaw amplitude (5°~20°) and speed
 */
class BeatDanceModifier : public Modifier {
public:
    BeatDanceModifier(const std::string& pattern, int bpm, uint32_t duration_ms, int intensity);
    ~BeatDanceModifier() override;

    void _update(Modifiable& stackchan) override;

    /**
     * @brief Ask the dance to stop at the next tick
     *
     * @param restore_pose false when the dance is immediately superseded by a
     *                     new one, so the old task must not fight the new pose
     */
    void stop(bool restore_pose = true)
    {
        _restore_pose.store(restore_pose);
        _stop_requested.store(true);
    }

    bool isDone() const
    {
        return _done.load();
    }

private:
    static void _task_entry(void* param);
    void _run();
    static animation::KeyframeSequence _build_sequence(const std::string& pattern, int bpm, uint32_t duration_ms,
                                                       int intensity);

    animation::Timeline _timeline;
    uint32_t _duration_ms = 6000;
    TaskHandle_t _task    = nullptr;

    std::atomic<bool> _done{false};
    std::atomic<bool> _stop_requested{false};
    std::atomic<bool> _restore_pose{true};
};

}  // namespace stackchan
