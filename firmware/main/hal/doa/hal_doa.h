/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <cstdint>
#include <string>

/**
 * 传统声源定位（300-800Hz 加权 IPD 三分类）与转头闭环。
 *
 * 数据流：
 *   ES7210 双麦（MIC1/MIC2）24kHz 交错 L/R 原始样本
 *     -> hal_doa::on_capture()   （audio input 任务，3:2 抽取到 16kHz）
 *     -> 环形缓冲
 *     -> 内部任务每 20ms 处理一个 1024 点 Hann 窗（hop=320 @16kHz）
 *     -> 类别/置信度 + v7 非对称控制律 -> Motion yaw
 *
 * 算法口径与 firmware_capture（demo）一致：21 个低频 bin 的幅度加权平均 IPD、
 * 5s 滑动中位数活动门限 ×1.25（下限 150）、9 窗多数表决、冻结窗。
 */
namespace hal_doa {

struct Result {
    char cls         = '-';  // 'L' 左 / 'M' 中 / 'R' 右 / '-' 静默
    float ipd_deg    = 0;
    float confidence = 0;  // 0..1，表决环内胜出类占比
    uint32_t ts_ms   = 0;
};

/**
 * @brief 分配缓冲、生成 FFT 表、读取 NVS 参数并启动处理任务（幂等）
 */
bool init();

/**
 * @brief 音频采集回调，24kHz 交错双声道样本，仅由 audio input 任务调用
 *
 * @param stereo 交错样本 [L,R,L,R,...]
 * @param frames 帧数（不含通道维度）
 */
void on_capture(const int16_t* stereo, int frames);

/**
 * @brief 最近一次判决结果
 */
Result get_result();

/**
 * @brief 开启/关闭整个 DOA 通路（关闭时 on_capture 直接丢弃）
 */
void set_enabled(bool enabled);
bool is_enabled();

/**
 * @brief 声源方向触发转头（默认关闭，由 MCP 工具 look_at_sound 打开）
 */
void set_look_at_sound(bool enabled);
bool is_look_at_sound();

/**
 * @brief 清空表决/门限/控制环状态
 */
void reset();

/**
 * @brief 校准参数（换房间后重标定，写入 NVS）
 *
 * @param th_lo     IPD 左门限（度）
 * @param th_hi     IPD 右门限（度）
 * @param sign      左右符号（-1 表示 IPD 负值对应左）
 * @param zero      yaw 舵机零位 raw（仅记录，实际零位由 servo 模块维护）
 * @param max_deg   转头限幅（度）
 * @param gate_floor 活动门限绝对下限（RMS）
 */
bool set_params(float th_lo, float th_hi, int sign, int zero, int max_deg, float gate_floor);

std::string get_params_json();

}  // namespace hal_doa
