/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "app_ai_agent.h"
#include <hal/hal.h>
#include <mooncake.h>
#include <mooncake_log.h>
#include <assets/assets.h>
#include <smooth_lvgl.hpp>
#include <stackchan/stackchan.h>
#include <apps/common/common.h>

#ifdef XIAOZHI_AUTO_TEST
#include "hal/board/hal_bridge.h"
#endif

using namespace mooncake;
using namespace smooth_ui_toolkit::lvgl_cpp;

AppAiAgent::AppAiAgent()
{
    // Configure App name
    setAppInfo().name = "AI.AGENT";
    // Configure App icon
    static auto icon  = assets::get_image("icon_ai_agent.bin");
    setAppInfo().icon = (void*)&icon;
    // Configure App theme color
    static uint32_t theme_color = 0x33CC99;
    setAppInfo().userData       = (void*)&theme_color;
}

// Called when the App is installed
void AppAiAgent::onCreate()
{
    mclog::tagInfo(getAppInfo().name, "on create");
}

// Called when the App is opened
// You can construct UI, initialize operations, etc. here
void AppAiAgent::onOpen()
{
    mclog::tagInfo(getAppInfo().name, "on open");

    // Request to start Xiaozhi service
    // All apps will be uninstall in next mooncake update
    GetHAL().requestXiaozhiStart();

#ifdef XIAOZHI_AUTO_TEST
    _autotest_mark = GetHAL().millis();
    mclog::tagInfo(getAppInfo().name, "[auto-test] armed, first chat in 8s");
#endif
}

// Called repeatedly while the App is running
void AppAiAgent::onRunning()
{
#ifdef XIAOZHI_AUTO_TEST
    auto now = GetHAL().millis();
    if (!_autotest_chat_started) {
        // xiaozhi 就绪且空闲后, 打开 8 秒模拟单击屏幕进入聊天
        if (hal_bridge::is_xiaozhi_ready() && hal_bridge::is_xiaozhi_idle() && now - _autotest_mark > 8000) {
            mclog::tagInfo(getAppInfo().name, "[auto-test] toggle chat (simulate tap)");
            hal_bridge::toggle_xiaozhi_chat_state();
            _autotest_chat_started = true;
            _autotest_mark         = now;
        }
    } else if (hal_bridge::is_xiaozhi_idle()) {
        // 会话结束(服务端断开/连接失败)回到空闲, 静置 20 秒后开新一轮
        if (!_autotest_idle) {
            _autotest_idle = true;
            _autotest_mark = now;
            mclog::tagInfo(getAppInfo().name, "[auto-test] back to idle, next chat in 20s");
        } else if (now - _autotest_mark > 20000) {
            _autotest_idle = false;
            mclog::tagInfo(getAppInfo().name, "[auto-test] toggle chat (restart loop)");
            hal_bridge::toggle_xiaozhi_chat_state();
            _autotest_mark = now;
        }
    }
#endif
}

// Called when the App is closed
// You can destroy UI, release resources, etc. here
void AppAiAgent::onClose()
{
    mclog::tagInfo(getAppInfo().name, "on close");
}
