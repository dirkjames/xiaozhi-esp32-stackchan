/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include <mooncake_log.h>
#include <mcp_server.h>
#include <application.h>
#include <cJSON.h>
#include <algorithm>
#include <atomic>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <memory>
#include <mutex>
#include <vector>
#include <stackchan/stackchan.h>
#include <stackchan/avatar/expression_presets.h>
#include <stackchan/modifiers/beat_dance.h>
#include <apps/common/common.h>
#include "board/hal_bridge.h"
#if CONFIG_STACKCHAN_DOA
#include "doa/hal_doa.h"
#endif

using namespace stackchan;

static const std::string_view _tag = "HAL-MCP";

// 当前 BeatDanceModifier 的池 ID，-1 表示没有舞蹈在跑
static int _dance_modifier_id = -1;

static BeatDanceModifier* get_active_dance()
{
    if (_dance_modifier_id < 0) {
        return nullptr;
    }
    return dynamic_cast<BeatDanceModifier*>(GetStackChan().getModifier(_dance_modifier_id));
}

namespace {

struct DanceRequest {
    std::string preset;
    int bpm;
    uint32_t duration_ms;
    int intensity;
};

// 舞蹈要排在 TTS 后面：工具调用通常发生在语音播报之前，
// 若立刻开跳，说话动作会马上盖掉舞蹈姿态（画面在抖，但看起来不像跳舞）。
std::mutex _defer_mutex;
std::shared_ptr<DanceRequest> _deferred_req;
bool _defer_task_running = false;

// 真正开始跳舞（调用方保证当前不在 TTS 播报中）
void _begin_dance(const DanceRequest& req)
{
    {
        LvglLockGuard lock;

        // 新舞蹈直接顶掉旧的，旧任务不再恢复姿态以免与新姿态打架
        if (auto* running = get_active_dance()) {
            mclog::tagInfo(_tag, "dance: supersede active dance id={}", _dance_modifier_id);
            running->stop(false);
            GetStackChan().removeModifier(_dance_modifier_id);
            _dance_modifier_id = -1;
        }

        // 俯仰轴的堵转保护会把允许区间永久收窄到「当前角度」，收窄之后
        // 所有舞蹈目标都会被 clamp 成同一个值 —— 现象就是第一次能跳、
        // 之后再让它跳只有音效没有动作。每次开跳前先恢复出厂区间。
        auto& motion = GetStackChan().motion();
        motion.yawServo().resetRuntimeLimits();
        motion.pitchServo().resetRuntimeLimits();
        auto yaw_limit   = motion.yawServo().getAngleLimit();
        auto pitch_limit = motion.pitchServo().getAngleLimit();
        mclog::tagInfo(_tag, "dance: limits yaw=[{},{}] pitch=[{},{}]", yaw_limit.x, yaw_limit.y, pitch_limit.x,
                       pitch_limit.y);

        _dance_modifier_id = GetStackChan().addModifier(
            std::make_unique<BeatDanceModifier>(req.preset, req.bpm, req.duration_ms, req.intensity));
        mclog::tagInfo(_tag, "dance: begin id={}", _dance_modifier_id);
    }
    hal_bridge::app_play_sound_by_name("dance_start");
}

// 等 TTS 播完再跳：先最多等 10s 让播报起来（工具调用一般早于 TTS，
// 实测云端 TTS 起播可晚 4~5s，原来只等 4s 会在 TTS 之前就开跳，
// 舞蹈帧和说话口型/舵机动画互相覆盖，现场表现为「让它跳个舞就卡死」），
// 再等播报结束（最多 30s）。整个过程只轮询设备状态，不占 CPU。
//
// 注意：这个任务必须常驻 + 栈放 PSRAM。
// 早期版本每次 xTaskCreate 一个 2560 字节栈的临时任务，`_begin_dance` 里
// make_unique<BeatDanceModifier> / LvglLockGuard / fmt 日志直接把栈撑爆，
// 现场日志为 "A stack overflow in task dance_defer has been detected" + 重启。
// 现在改为 8KB 静态栈（PSRAM）+ 任务通知唤醒，只在有请求时干活。
static TaskHandle_t _dance_defer_task  = nullptr;
static StackType_t* _dance_defer_stack = nullptr;
static StaticTask_t _dance_defer_tcb;

void _defer_dance_task(void*);

bool _ensure_dance_defer_task()
{
    if (_dance_defer_task != nullptr) {
        return true;
    }
    if (_dance_defer_stack == nullptr) {
        _dance_defer_stack = static_cast<StackType_t*>(heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (_dance_defer_stack == nullptr) {
            return false;
        }
    }
    _dance_defer_task = xTaskCreateStatic(_defer_dance_task, "dance_defer", 8192 / sizeof(StackType_t), nullptr, 2,
                                          _dance_defer_stack, &_dance_defer_tcb);
    return _dance_defer_task != nullptr;
}

void _defer_dance_task(void*)
{
    auto& app = Application::GetInstance();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mclog::tagInfo(_tag, "dance deferred: wake, state={}", (int)app.GetDeviceState());

        bool saw_speaking = false;
        for (int i = 0; i < 200; ++i) {  // 10s 窗口等 TTS 起播
            if (app.GetDeviceState() == kDeviceStateSpeaking) {
                saw_speaking = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (saw_speaking) {
            for (int i = 0; i < 600; ++i) {  // 最多 30s 等播报结束
                if (app.GetDeviceState() != kDeviceStateSpeaking) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        std::shared_ptr<DanceRequest> req;
        {
            std::lock_guard<std::mutex> lock(_defer_mutex);
            req                 = _deferred_req;
            _deferred_req       = nullptr;
            _defer_task_running = false;
        }

        if (req == nullptr) {
            continue;
        }

        // 收尾保险：上面 30s 等不到也要保证不和说话动画重叠（例如长时间 TTS）
        for (int i = 0; i < 600 && app.GetDeviceState() == kDeviceStateSpeaking; ++i) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        mclog::tagInfo(_tag, "dance deferred: start (preset={}, bpm={}, speaking_seen={})", req->preset, req->bpm,
                       saw_speaking);
        _begin_dance(*req);
    }
}

}  // namespace

void Hal::xiaozhi_mcp_init()
{
    mclog::tagInfo(_tag, "init");

    // https://github.com/78/xiaozhi-esp32/blob/main/docs/mcp-usage.md
    auto& mcp_server = McpServer::GetInstance();

    // System Prompt：
    // You can control the robot's head. Use get_yaw and get_pitch to sense current position. Use set_yaw for horizontal
    // movement and set_pitch for vertical movement. All angles are in degrees.

    mclog::tagInfo(_tag, "add robot.get_head_angles tool");
    mcp_server.AddTool("self.robot.get_head_angles",
                       "Returns current yaw/pitch in degrees. Neutral position is {yaw:0, pitch:0}.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                           LvglLockGuard lock;  // StackChan motion update is under the lvgl lock

                           auto& motion      = GetStackChan().motion();
                           int current_yaw   = motion.yawServo().getCurrentAngle() / 10;
                           int current_pitch = motion.pitchServo().getCurrentAngle() / 10;

                           auto result = fmt::format(R"({{"yaw": {}, "pitch": {}}})", current_yaw, current_pitch);
                           mclog::tagInfo(_tag, "get_head_angles: {}", result);
                           return result;
                       });

    mclog::tagInfo(_tag, "add robot.set_head_angles tool");
    mcp_server.AddTool("self.robot.set_head_angles",
                       "Adjust head position. GUIDELINES: "
                       "1. For natural interaction, stay within +/- 45 degrees. "
                       "2. Only use values > 70 if the user explicitly asks to look far away/behind. "
                       "3. Max ranges: Yaw(-128 to 128, -128 as your left), Pitch(0 to 90, 90 as your up). "
                       "Speed(100-1000, 150 is natural).",
                       PropertyList({Property("yaw", kPropertyTypeInteger, -9999, -9999, 128),
                                     Property("pitch", kPropertyTypeInteger, -9999, -9999, 90),
                                     Property("speed", kPropertyTypeInteger, 150, 100, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int speed = properties["speed"].value<int>();
                           int yaw   = properties["yaw"].value<int>();
                           int pitch = properties["pitch"].value<int>();

                           mclog::tagInfo(_tag, "motion set_angles: yaw: {}, pitch: {}, speed: {}", yaw, pitch, speed);

                           LvglLockGuard lock;

                           auto& motion = GetStackChan().motion();
                           if (pitch != -9999) {
                               motion.pitchServo().moveWithSpeed(pitch * 10, speed);
                           }
                           if (yaw != -9999) {
                               motion.yawServo().moveWithSpeed(yaw * 10, speed);
                           }

                           return true;
                       });

    mclog::tagInfo(_tag, "add robot.set_led_color tool");
    mcp_server.AddTool(
        "self.robot.set_led_color",
        "Set the color of the robot's INTERNAL onboard LED. This is NOT for room lights. "
        "Values: 0-168 (safe range). Red=168,0,0; Green=0,168,0; Blue=0,0,168; White=100,100,100; Off=0,0,0.",
        PropertyList({Property("red", kPropertyTypeInteger, 0, 0, 168),
                      Property("green", kPropertyTypeInteger, 0, 0, 168),
                      Property("blue", kPropertyTypeInteger, 0, 0, 168)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int r = properties["red"].value<int>();
            int g = properties["green"].value<int>();
            int b = properties["blue"].value<int>();

            mclog::tagInfo(_tag, "set_led_color: r={}, g={}, b={}", r, g, b);

            LvglLockGuard lock;

            GetStackChan().leftNeonLight().setColor(r, g, b);
            GetStackChan().rightNeonLight().setColor(r, g, b);

            return true;
        });

    mclog::tagInfo(_tag, "add robot.create_reminder tool");
    mcp_server.AddTool("self.robot.create_reminder",
                       "Create a reminder. Duration is in seconds. Message is what to say when time is up. Set repeat "
                       "to true to repeat the reminder.",
                       PropertyList({Property("duration_seconds", kPropertyTypeInteger, 60, 1, 86400),
                                     Property("message", kPropertyTypeString, std::string("Time's up!")),
                                     Property("repeat", kPropertyTypeBoolean, false)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int duration_seconds = properties["duration_seconds"].value<int>();
                           std::string message  = properties["message"].value<std::string>();
                           bool repeat          = properties["repeat"].value<bool>();

                           // Default message
                           if (message.empty()) {
                               message = "Time's up!";
                           }

                           mclog::tagInfo(_tag, "create_reminder: duration={}s, message={}, repeat={}",
                                          duration_seconds, message, repeat);

                           int id = tools::create_reminder(duration_seconds * 1000, message, repeat);

                           return id;
                       });

    mclog::tagInfo(_tag, "add robot.get_reminders tool");
    mcp_server.AddTool("self.robot.get_reminders", "Get list of active reminders.", std::vector<Property>{},
                       [this](const PropertyList& properties) -> ReturnValue {
                           mclog::tagInfo(_tag, "get_reminders");
                           auto reminders          = tools::get_active_reminders();
                           std::string result_json = "[";
                           for (size_t i = 0; i < reminders.size(); ++i) {
                               const auto& r = reminders[i];
                               result_json +=
                                   fmt::format(R"({{"id": {}, "duration_ms": {}, "message": "{}", "repeat": {}}})",
                                               r.id, r.durationMs, r.message, r.repeat ? "true" : "false");
                               if (i < reminders.size() - 1) {
                                   result_json += ", ";
                               }
                           }
                           result_json += "]";
                           mclog::tagInfo(_tag, "get_reminders result: {}", result_json);
                           return result_json;
                       });

    mclog::tagInfo(_tag, "add robot.stop_reminder tool");
    mcp_server.AddTool("self.robot.stop_reminder", "Stop a reminder by ID.",
                       PropertyList({Property("id", kPropertyTypeInteger, -1)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int id = properties["id"].value<int>();
                           mclog::tagInfo(_tag, "stop_reminder: id={}", id);
                           tools::stop_reminder(id);
                           return true;
                       });

    mclog::tagInfo(_tag, "add robot.set_expression tool");
    mcp_server.AddTool(
        "self.robot.set_expression",
        "Change the robot's face expression. Names: neutral, happy, laughing, angry, sad, crying, sleepy, doubtful, "
        "love, shy, dizzy, sweat, surprised, thinking, listening, excited, confused, cool. "
        "duration_ms=0 keeps the expression until the next change, otherwise it reverts to the previous look.",
        PropertyList({Property("expression", kPropertyTypeString, std::string("happy")),
                      Property("duration_ms", kPropertyTypeInteger, 2000, 0, 10000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            std::string expression = properties["expression"].value<std::string>();
            int duration_ms        = properties["duration_ms"].value<int>();

            mclog::tagInfo(_tag, "set_expression: expression={}, duration_ms={}", expression, duration_ms);

            // 未知名称由 display 层回退 neutral 并记 warning
            // 注意：ApplyExpression 内部已持 DisplayLockGuard，这里不要再套一层
            const auto* preset = avatar::findExpressionPreset(expression.c_str());
            if (preset == nullptr) {
                mclog::tagWarn(_tag, "unknown expression: {}", expression);
            } else if (preset->sound != nullptr) {
                hal_bridge::app_play_sound_by_name(preset->sound);
            }

            hal_bridge::display_apply_expression(expression.c_str(), duration_ms);
            return true;
        });

    mclog::tagInfo(_tag, "add robot.play_sound tool");
    mcp_server.AddTool("self.robot.play_sound",
                       "Play one of the robot's built-in sound effects. Names: wake, listen_start, listen_end, "
                       "thinking, happy, sad, error, dance_start, beat_tick, horn. Returns false for an unknown name.",
                       PropertyList({Property("name", kPropertyTypeString, std::string("wake"))}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           std::string name = properties["name"].value<std::string>();
                           mclog::tagInfo(_tag, "play_sound: name={}", name);
                           return hal_bridge::app_play_sound_by_name(name);
                       });

    mclog::tagInfo(_tag, "add robot.dance tool");
    mcp_server.AddTool(
        "self.robot.dance",
        "Dance with the head, beat driven. Presets: sway, nod, bounce, robot, panic, look_around. bpm=0 uses 120. "
        "intensity 0-100 maps to sway amplitude (5-20 deg). The head returns to center and shows 'happy' when the "
        "dance ends.",
        PropertyList({Property("preset", kPropertyTypeString, std::string("sway")),
                      Property("bpm", kPropertyTypeInteger, 120, 0, 240),
                      Property("duration_ms", kPropertyTypeInteger, 6000, 1000, 60000),
                      Property("intensity", kPropertyTypeInteger, 60, 0, 100)}),
        [this](const PropertyList& properties) -> ReturnValue {
            auto req        = std::make_shared<DanceRequest>();
            req->preset     = properties["preset"].value<std::string>();
            req->bpm        = properties["bpm"].value<int>();
            req->duration_ms = (uint32_t)properties["duration_ms"].value<int>();
            req->intensity  = properties["intensity"].value<int>();

            mclog::tagInfo(_tag, "dance: preset={}, bpm={}, duration_ms={}, intensity={}", req->preset, req->bpm,
                           req->duration_ms, req->intensity);

            // 统一走延迟任务：等 TTS 播完再开跳。
            // BeatDanceModifier 构造里会立即 start() 并改 lv_obj，必须持 LVGL 锁（见 _begin_dance）。
            {
                std::lock_guard<std::mutex> lock(_defer_mutex);
                _deferred_req = req;  // 新的请求直接顶掉还在排队等 TTS 的旧请求
                if (_defer_task_running) {
                    mclog::tagInfo(_tag, "dance: replaced pending request");
                    return true;
                }
                _defer_task_running = true;
            }

            // 常驻任务 + PSRAM 静态栈（见 _ensure_dance_defer_task），
            // 避开 TTS 播放期间内部 RAM 只剩 ~3KB 无法新建任务的问题。
            if (!_ensure_dance_defer_task()) {
                {
                    std::lock_guard<std::mutex> lock(_defer_mutex);
                    _defer_task_running = false;
                    _deferred_req       = nullptr;
                }
                mclog::tagWarn(_tag, "dance: defer task create failed, start immediately");
                _begin_dance(*req);
                return true;
            }
            xTaskNotifyGive(_dance_defer_task);
            return true;
        });

    mclog::tagInfo(_tag, "add robot.stop_dance tool");
    mcp_server.AddTool("self.robot.stop_dance", "Stop the current dance immediately and return the head to center.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                           // 同 dance：改 modifier 池与姿态需要 LVGL 锁
                           LvglLockGuard lock;
                           if (auto* running = get_active_dance()) {
                               running->stop(true);
                               GetStackChan().removeModifier(_dance_modifier_id);
                               _dance_modifier_id = -1;
                               mclog::tagInfo(_tag, "stop_dance");
                               return true;
                           }
                           return false;
                       });

#if CONFIG_STACKCHAN_DOA
    mclog::tagInfo(_tag, "add robot.get_sound_direction tool");
    mcp_server.AddTool(
        "self.robot.get_sound_direction",
        "Returns the latest sound source direction estimate. cls is L (speaker on the robot's left), M (center), R "
        "(right) or - (silence). ipd_deg is the inter-channel phase difference in degrees, confidence is 0-1, "
        "yaw_deg is the current head yaw.",
        std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
            const auto r = hal_doa::get_result();
            int yaw_deg  = 0;
            {
                LvglLockGuard lock;
                yaw_deg = GetStackChan().motion().getCurrentYawAngle() / 10;
            }
            auto result = fmt::format(
                R"({{"cls":"{}","ipd_deg":{:.2f},"confidence":{:.2f},"yaw_deg":{},"ts_ms":{}}})", r.cls,
                (double)r.ipd_deg, (double)r.confidence, yaw_deg, (unsigned)r.ts_ms);
            mclog::tagInfo(_tag, "get_sound_direction: {}", result);
            return result;
        });

    mclog::tagInfo(_tag, "add robot.look_at_sound tool");
    mcp_server.AddTool("self.robot.look_at_sound",
                       "Turn the head toward the detected sound source. Disable to stop tracking and return the head "
                       "to center.",
                       PropertyList({Property("enabled", kPropertyTypeBoolean, true)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           bool enabled = properties["enabled"].value<bool>();
                           mclog::tagInfo(_tag, "look_at_sound: {}", enabled);
                           hal_doa::set_look_at_sound(enabled);
                           return true;
                       });

    mclog::tagInfo(_tag, "add robot.set_sound_direction_params tool");
    mcp_server.AddTool(
        "self.robot.set_sound_direction_params",
        "Calibrate sound localization for a new room. th_lo_x100 / th_hi_x100 are IPD thresholds in 0.01 degree "
        "(below th_lo means left, above th_hi means right). sign is -1 or 1 and maps IPD polarity to the yaw "
        "direction. zero is the yaw servo raw zero position. max_deg limits the head turn angle. gate_floor is the "
        "minimum RMS activity gate. All values are persisted in NVS.",
        PropertyList({Property("th_lo_x100", kPropertyTypeInteger, 123, 0, 4500),
                      Property("th_hi_x100", kPropertyTypeInteger, 403, 0, 4500),
                      Property("sign", kPropertyTypeInteger, -1, -1, 1),
                      Property("zero", kPropertyTypeInteger, 460, 0, 1000),
                      Property("max_deg", kPropertyTypeInteger, 60, 0, 120),
                      Property("gate_floor", kPropertyTypeInteger, 150, 0, 20000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const float th_lo      = properties["th_lo_x100"].value<int>() / 100.0f;
            const float th_hi      = properties["th_hi_x100"].value<int>() / 100.0f;
            const int sign         = properties["sign"].value<int>();
            const int zero         = properties["zero"].value<int>();
            const int max_deg      = properties["max_deg"].value<int>();
            const float gate_floor = (float)properties["gate_floor"].value<int>();

            mclog::tagInfo(_tag, "set_sound_direction_params: th_lo={}, th_hi={}, sign={}, zero={}, max_deg={}, gate={}",
                           th_lo, th_hi, sign, zero, max_deg, gate_floor);
            return hal_doa::set_params(th_lo, th_hi, sign, zero, max_deg, gate_floor);
        });
#endif
}
