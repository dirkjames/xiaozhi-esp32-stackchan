/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include "board/hal_bridge.h"
#include "drivers/bmi270/bmi270.h"
#include "utils/motion_detector/motion_detector.h"
#include <application.h>
#include <mooncake_log.h>
#include <cstdio>
#include <memory>

static const std::string_view _tag = "HAL-IMU";

static std::unique_ptr<BMI270> _bmi270;

namespace {

/**
 * @brief 应用是否已经初始化到可以播放音效 / 显示表情
 *
 * Application::Initialize() 里 audio_service_.Initialize() 之前 codec_ 为空，
 * 此时 PlaySound() 会空指针崩溃，而 IMU 任务在 board init 阶段就已开始采样。
 */
bool _local_feedback_ready()
{
    return Application::GetInstance().GetAudioService().IsStarted();
}

/**
 * @brief 手势事件 -> 本地反馈 + 云端上报
 *
 * Shake 交给 ImuEventModifier 做动画，这里只补音效，避免两套表情互相覆盖。
 */
void _react(MotionEvent event, const BMI270_Data& data)
{
    const char* type  = nullptr;
    const char* expr  = nullptr;
    const char* sound = nullptr;

    switch (event) {
        case MotionEvent::Shake:
            type  = "shake";
            sound = "error";
            break;
        case MotionEvent::PickUp:
            type  = "pick_up";
            expr  = "surprised";
            sound = "wake";
            break;
        case MotionEvent::Brake:
            type  = "brake";
            expr  = "dizzy";
            // 急停/侧倾这类轻微触碰也会触发，不给音效，避免碰一下就"当当"响
            break;
        case MotionEvent::Turn:
            type  = "turn";
            expr  = "confused";
            // 同上：侧倾只换表情，不播放大声效
            break;
        case MotionEvent::Bump:
            type  = "bump";
            expr  = "dizzy";
            sound = "error";
            break;
        default:
            return;
    }

    if (expr != nullptr && _local_feedback_ready()) {
        hal_bridge::display_apply_expression(expr, 2000);
    }
    if (sound != nullptr && _local_feedback_ready()) {
        hal_bridge::app_play_sound_by_name(sound);
    }

    // 上报云端；未连接时 SendMcpMessage 内部会丢弃
    char payload[256];
    std::snprintf(payload, sizeof(payload),
                  "{\"jsonrpc\":\"2.0\",\"method\":\"notifications/robotics\","
                  "\"params\":{\"event\":\"motion\",\"data\":{\"type\":\"%s\","
                  "\"accel\":[%.2f,%.2f,%.2f],\"gyro\":[%.2f,%.2f,%.2f]},\"ts\":%u}}",
                  type, data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z,
                  static_cast<unsigned>(GetHAL().millis()));
    Application::GetInstance().SendMcpMessage(payload);
}

}  // namespace

static void _imu_task(void* param)
{
    auto motion_detector = std::make_unique<MotionDetector>();
    motion_detector->setShakeThreshold(16.0f);

    // 前 2s 只等传感器与应用初始化稳定，不做判据，避免上电姿态被当成 Turn
    int warmup_frames = 20;

    while (1) {
        if (warmup_frames > 0) {
            --warmup_frames;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (_bmi270 && _bmi270->update()) {
            auto& data = _bmi270->getData();
            // mclog::debug("IMU Accel: {:.2f}\t{:.2f}\t{:.2f}", data.accel_x, data.accel_y, data.accel_z);

            motion_detector->update(data.accel_x, data.accel_y, data.accel_z);

            if (motion_detector->isShakeDetected()) {
                mclog::tagInfo(_tag, "Shake Detected!");
                GetHAL().onImuMotionEvent.emit(ImuMotionEvent::Shake);
                _react(MotionEvent::Shake, data);
            } else {
                auto event = motion_detector->pollEvent();
                switch (event) {
                    case MotionEvent::PickUp:
                        mclog::tagInfo(_tag, "Pick Up Detected!");
                        GetHAL().onImuMotionEvent.emit(ImuMotionEvent::PickUp);
                        break;
                    case MotionEvent::Brake:
                        mclog::tagInfo(_tag, "Brake Detected!");
                        GetHAL().onImuMotionEvent.emit(ImuMotionEvent::Brake);
                        break;
                    case MotionEvent::Turn:
                        mclog::tagInfo(_tag, "Turn Detected!");
                        GetHAL().onImuMotionEvent.emit(ImuMotionEvent::Turn);
                        break;
                    case MotionEvent::Bump:
                        mclog::tagInfo(_tag, "Bump Detected!");
                        GetHAL().onImuMotionEvent.emit(ImuMotionEvent::Bump);
                        break;
                    default:
                        break;
                }
                if (event != MotionEvent::None) {
                    _react(event, data);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Hal::imu_init()
{
    mclog::tagInfo(_tag, "init");

    auto i2c_bus = hal_bridge::board_get_i2c_bus();

    _bmi270 = std::make_unique<BMI270>(i2c_bus, 0x69);
    if (!_bmi270->begin()) {
        _bmi270.reset();
        mclog::tagError(_tag, "BMI270 init failed");
        return;
    }
    mclog::tagInfo(_tag, "BMI270 init ok");

    // xTaskCreateWithCaps(_imu_task, "imu", 4096, NULL, 2, NULL, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCoreWithCaps(_imu_task, "imu", 4096, NULL, 2, NULL, 1, MALLOC_CAP_SPIRAM);
}
