/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <mooncake.h>

/**
 * @brief Derived App
 *
 */
class AppAiAgent : public mooncake::AppAbility {
public:
    AppAiAgent();

    // Override lifecycle callbacks
    void onCreate() override;
    void onOpen() override;
    void onRunning() override;
    void onClose() override;

#ifdef XIAOZHI_AUTO_TEST
private:
    // 自动化压测状态: 模拟点击进入聊天, 会话结束后自动重开
    uint32_t _autotest_mark = 0;
    bool _autotest_chat_started = false;
    bool _autotest_idle = false;
#endif
};
