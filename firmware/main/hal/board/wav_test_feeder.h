/*
 * 测试专用：把预先录好的 16 kHz/16 bit 单声道 WAV 当作麦克风输入回放，
 * 用于无人值守地端到端验证「listening / speaking 不卡顿、不卡死」。
 *
 * 仅在编译期打开 STACKCHAN_WAV_TEST（见 main/CMakeLists.txt）时生效，
 * 正式固件不会包含这段逻辑。
 */
#pragma once

#include <stdint.h>

#include "device_state.h"

namespace wav_test {

/**
 * @brief 主任务在状态机切换时通知测试脚本
 *
 * Speaking / Idle 期间语音处理是关掉的，Read() 不会被调用；只靠 Fill() 观察
 * 状态会漏掉「云端已经回复」这一事实，所以状态由主任务直接推过来。
 */
void OnDeviceState(DeviceState state);

/**
 * @brief 用 WAV 内容覆盖本次采集到的 PCM
 *
 * 调用方（CoreS3AudioCodec::Read）已经照常读了一次真实 I2S 数据用来保持
 * 24 kHz 的实时节拍，这里只是把内容替换掉：
 *   - 通道 0 → 录音内容（相当于"人对着麦克风说话"）
 *   - 其余通道 → 静音（AEC 参考通道必须安静，否则 AEC 会把近端语音当成回声消掉）
 *
 * @param dest        交错排列的 int16 PCM 缓冲
 * @param frames      帧数（不是采样点数）
 * @param channels    通道数
 * @param sample_rate 输出采样率（StackChan 为 24000）
 * @return 永远返回 true（测试模式下麦克风完全由 WAV 驱动）
 */
bool Fill(int16_t* dest, int frames, int channels, int sample_rate);

}  // namespace wav_test
