/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include "avatar.h"
#include "../modifiable.h"
#include "../modifiers/timed.h"
#include <cstdint>

namespace stackchan::avatar {

/**
 * @brief Optional decorator attached with an expression preset
 *
 */
enum class ExpressionDecorator {
    None = 0,
    Heart,
    Angry,
    Sweat,
    Shy,
    Dizzy,
};

/**
 * @brief One addressable expression preset (name -> look)
 *
 */
struct ExpressionPreset {
    const char* name;              //!< Cloud addressable name
    Emotion emotion;               //!< Base emotion
    int eye_weight;                //!< 0~100, 100 = fully open
    int mouth_weight;              //!< 0~100
    int eye_size;                  //!< -100~100, 0 = normal
    ExpressionDecorator decorator; //!< Optional decorator
    const char* sound;             //!< Optional sfx name (see assets::find_sound), nullptr = none
    uint32_t default_duration_ms;  //!< 0 = persistent
};

/**
 * @brief Look up a preset by name
 *
 * @param name preset name, null-safe
 * @return const ExpressionPreset* nullptr if not found
 */
const ExpressionPreset* findExpressionPreset(const char* name);

/**
 * @brief Apply a preset to the avatar (emotion + eye/mouth weights + decorator)
 *
 * The caller must hold the lvgl lock, and is responsible for calling
 * BlinkModifier::resyncEyeWeights() afterwards.
 *
 * @param stackchan
 * @param preset
 * @param decorator_lifetime_ms decorator lifetime, 0 = preset default
 */
void applyExpressionPreset(Modifiable& stackchan, const ExpressionPreset& preset, uint32_t decorator_lifetime_ms = 0);

/**
 * @brief Apply a preset for the given duration, then restore the previous look
 *
 * Only used by the cloud facing `set_expression(..., duration_ms)` path; the
 * `llm.emotion` channel keeps its historical "persistent until changed" behaviour.
 */
class TimedExpressionModifier : public TimedEventModifier {
public:
    TimedExpressionModifier(const ExpressionPreset& preset, uint32_t durationMs)
        : TimedEventModifier(durationMs), _preset(preset), _lifetime_ms(durationMs)
    {
    }

    void _on_start(Modifiable& stackchan) override
    {
        if (!stackchan.hasAvatar()) {
            return;
        }

        _prev_emotion = stackchan.avatar().getEmotion();
        applyExpressionPreset(stackchan, _preset, _lifetime_ms);
    }

    void _on_end(Modifiable& stackchan) override
    {
        if (!stackchan.hasAvatar()) {
            return;
        }

        auto& avatar = stackchan.avatar();
        avatar.leftEye().setSize(0);
        avatar.rightEye().setSize(0);
        avatar.leftEye().setWeight(100);
        avatar.rightEye().setWeight(100);
        avatar.mouth().setWeight(0);
        avatar.setEmotion(_prev_emotion);
    }

private:
    ExpressionPreset _preset;
    uint32_t _lifetime_ms = 0;
    Emotion _prev_emotion = Emotion::Neutral;
};

}  // namespace stackchan::avatar
