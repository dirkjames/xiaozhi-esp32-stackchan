/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal_bridge.h"
#include "stackchan_display.h"
#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <application.h>
#include <board.h>
#include <display.h>
#include <mutex>
#include <assets.h>
#include <assets/assets.h>
#include <deque>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <settings.h>

static const char* _tag = "HAL_BRIDGE";

static constexpr std::string_view _xiaozhi_config_nvs_ns                           = "xiaozhi";
static constexpr std::string_view _xiaozhi_config_idle_shutdown_time_key           = "idle_sec";
static constexpr std::string_view _xiaozhi_config_allow_shutdown_when_charging_key = "ext_pwr";
static constexpr std::string_view _xiaozhi_config_idle_random_movement_key         = "idle_lv";
static constexpr std::string_view _xiaozhi_config_start_ai_agent_on_boot_key       = "boot_ai";
static constexpr std::string_view _xiaozhi_config_cloud_profile_key                = "cloud";

/* OTA 服务地址：仅允许这两个内置 profile，避免手误写错 */
static constexpr std::string_view _xiaozhi_ota_url_official = "https://api.tenclass.net/xiaozhi/ota/";
static constexpr std::string_view _xiaozhi_ota_url_self     = "https://yinxuxian-test.superacme.com/xiaozhi/ota/";

namespace hal_bridge {

/* -------------------------------------------------------------------------- */
/*                            State and touch point                           */
/* -------------------------------------------------------------------------- */

static std::mutex _mutex;
static Data_t _data;

void lock()
{
    _mutex.lock();
}

void unlock()
{
    _mutex.unlock();
}

Data_t& get_data()
{
    return _data;
}

void set_touch_point(int num, int x, int y)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _data.touchPoint.num = num;
    _data.touchPoint.x   = x;
    _data.touchPoint.y   = y;
}

TouchPoint_t get_touch_point()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _data.touchPoint;
}

bool is_xiaozhi_mode()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _data.isXiaozhiMode;
}

void set_xiaozhi_mode(bool mode)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _data.isXiaozhiMode = mode;
}

/* -------------------------------------------------------------------------- */
/*                                   Display                                  */
/* -------------------------------------------------------------------------- */
#define DISPLAY_TYPE StackChanAvatarDisplay

lv_disp_t* display_get_lvgl_display()
{
    auto display = static_cast<DISPLAY_TYPE*>(Board::GetInstance().GetDisplay());
    return display->GetLvglDisplay();
}

void display_apply_expression(const char* name, int duration_ms)
{
    auto display = static_cast<DISPLAY_TYPE*>(Board::GetInstance().GetDisplay());
    display->ApplyExpression(name, duration_ms);
}

void disply_lvgl_lock()
{
    auto display = static_cast<DISPLAY_TYPE*>(Board::GetInstance().GetDisplay());
    display->LvglLock();
}

void disply_lvgl_unlock()
{
    auto display = static_cast<DISPLAY_TYPE*>(Board::GetInstance().GetDisplay());
    display->LvglUnlock();
}

/* -------------------------------------------------------------------------- */
/*                                 Application                                */
/* -------------------------------------------------------------------------- */

void xiaozhi_board_init()
{
    // Init board
    auto& board = Board::GetInstance();
}

void start_xiaozhi_app()
{
    set_xiaozhi_mode(true);

    // 按配置的云 profile 覆写 OTA 地址（xiaozhi 的 Ota::GetCheckVersionUrl 优先读 wifi/ota_url）
    auto config     = get_xiaozhi_config();
    std::string url = std::string(config.cloudProfile == 1 ? _xiaozhi_ota_url_self : _xiaozhi_ota_url_official);
    {
        Settings settings("wifi", true);
        settings.SetString("ota_url", url);
    }
    ESP_LOGI(_tag, "xiaozhi cloud profile: %d, ota url: %s", static_cast<int>(config.cloudProfile), url.c_str());

    // Initialize and run the application
    auto& app = Application::GetInstance();
    app.Initialize();
    app.Run();  // This function runs the main event loop and never returns
}

XiaozhiConfig_t get_xiaozhi_config()
{
    XiaozhiConfig_t config;

    Settings settings(_xiaozhi_config_nvs_ns.data(), false);
    config.idleShutdownTimeSeconds = settings.GetInt(_xiaozhi_config_idle_shutdown_time_key.data(),
                                                     static_cast<int>(config.idleShutdownTimeSeconds));
    config.allowShutdownWhenCharging =
        settings.GetBool(_xiaozhi_config_allow_shutdown_when_charging_key.data(), config.allowShutdownWhenCharging);
    config.idleRandomMovementLevel =
        settings.GetInt(_xiaozhi_config_idle_random_movement_key.data(), config.idleRandomMovementLevel);
    config.startAiAgentOnBoot =
        settings.GetBool(_xiaozhi_config_start_ai_agent_on_boot_key.data(), config.startAiAgentOnBoot);
    // [DEV-AUTO] 无人值守回归：开机直接进入 AI.AGENT，无需人工点击（发版前移除）
    config.startAiAgentOnBoot = true;
    // 关闭待机时的随机小动作：车载场景下无人说话时来回转头非常干扰
    config.idleRandomMovementLevel = 0;
    config.cloudProfile = settings.GetInt(_xiaozhi_config_cloud_profile_key.data(), config.cloudProfile);

    return config;
}

void set_xiaozhi_config(const XiaozhiConfig_t& config)
{
    Settings settings(_xiaozhi_config_nvs_ns.data(), true);
    settings.SetInt(_xiaozhi_config_idle_shutdown_time_key.data(), config.idleShutdownTimeSeconds);
    settings.SetBool(_xiaozhi_config_allow_shutdown_when_charging_key.data(), config.allowShutdownWhenCharging);
    settings.SetInt(_xiaozhi_config_idle_random_movement_key.data(), config.idleRandomMovementLevel);
    settings.SetBool(_xiaozhi_config_start_ai_agent_on_boot_key.data(), config.startAiAgentOnBoot);
    settings.SetInt(_xiaozhi_config_cloud_profile_key.data(), config.cloudProfile);
}

void app_play_sound(const std::string_view& sound)
{
    auto& app = Application::GetInstance();
    // audio_service_.Start() 之前 codec_ 为空，PlaySound() 会空指针
    if (!app.GetAudioService().IsStarted()) {
        return;
    }
    app.PlaySound(sound);
}

/* -------------------------------------------------------------------------- */
/*                                    SFX                                     */
/* -------------------------------------------------------------------------- */

static std::mutex _sfx_mutex;
static std::deque<std::string> _sfx_pending;
static constexpr size_t _sfx_pending_max = 4;

// 播放工作队列：Ogg 解封装 + 整段解码的调用链非常深（OggDemuxer -> 回调 ->
// Packet 分配），实测在 stackchan 刷新任务（4096 栈、只剩 ~1.4KB）或
// WebSocket 接收任务上直接调用会把栈踩穿，破坏相邻堆块后随机 Guru。
// 因此所有 sfx 播放都投递到独立的 sfx_play 任务；栈用 PSRAM（8KB），
// 不占已经非常紧张的内部 SRAM。
static std::mutex _sfx_play_mutex;
static std::deque<std::string> _sfx_play_queue;
static constexpr size_t _sfx_play_queue_max = 4;
static TaskHandle_t _sfx_play_task           = nullptr;
static StaticTask_t _sfx_play_tcb;
static StackType_t* _sfx_play_stack = nullptr;

static bool _is_tts_speaking()
{
    return Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking;
}

static void _sfx_play_task_entry(void*)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (true) {
            std::string name;
            {
                std::lock_guard<std::mutex> lock(_sfx_play_mutex);
                if (_sfx_play_queue.empty()) {
                    break;
                }
                name = std::move(_sfx_play_queue.front());
                _sfx_play_queue.pop_front();
            }
            auto ogg = assets::find_sound(name);
            if (ogg.empty()) {
                continue;
            }
            ESP_LOGI(_tag, "sfx play: %s begin", name.c_str());
            app_play_sound(ogg);
            ESP_LOGI(_tag, "sfx play: %s end", name.c_str());
        }
    }
}

static bool _ensure_sfx_play_task()
{
    if (_sfx_play_task != nullptr) {
        return true;
    }
    if (_sfx_play_stack == nullptr) {
        _sfx_play_stack = (StackType_t*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (_sfx_play_stack == nullptr) {
            ESP_LOGE(_tag, "failed to alloc sfx play stack");
            return false;
        }
    }
    _sfx_play_task = xTaskCreateStatic(_sfx_play_task_entry, "sfx_play", 8192 / sizeof(StackType_t), nullptr, 3,
                                       _sfx_play_stack, &_sfx_play_tcb);
    return _sfx_play_task != nullptr;
}

/**
 * @brief 把 sfx 投递给独立的播放任务（不阻塞调用方）
 */
static void _post_sfx(std::string_view name)
{
    if (!_ensure_sfx_play_task()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(_sfx_play_mutex);
        if (_sfx_play_queue.size() >= _sfx_play_queue_max) {
            _sfx_play_queue.pop_front();
        }
        _sfx_play_queue.emplace_back(name);
    }
    xTaskNotifyGive(_sfx_play_task);
}

bool app_play_sound_by_name(std::string_view name)
{
    auto ogg = assets::find_sound(name);
    if (ogg.empty()) {
        ESP_LOGW(_tag, "unknown sound: %s", std::string(name).c_str());
        return false;
    }

    if (_is_tts_speaking()) {
        std::lock_guard<std::mutex> lock(_sfx_mutex);
        if (_sfx_pending.size() >= _sfx_pending_max) {
            _sfx_pending.pop_front();
        }
        _sfx_pending.emplace_back(name);
        ESP_LOGI(_tag, "sfx deferred (tts speaking): %s", std::string(name).c_str());
        return true;
    }

    _post_sfx(name);
    return true;
}

void process_deferred_sounds()
{
    if (_is_tts_speaking()) {
        return;
    }

    std::string next;
    {
        std::lock_guard<std::mutex> lock(_sfx_mutex);
        if (_sfx_pending.empty()) {
            return;
        }
        next = std::move(_sfx_pending.front());
        _sfx_pending.pop_front();
    }

    ESP_LOGI(_tag, "sfx deferred play: %s", next.c_str());
    _post_sfx(next);
}

}  // namespace hal_bridge
