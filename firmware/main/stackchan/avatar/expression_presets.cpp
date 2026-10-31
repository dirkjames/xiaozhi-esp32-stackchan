/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "expression_presets.h"
#include <cstring>
#include <memory>

namespace stackchan::avatar {

/**
 * @brief Expression preset table (18 presets)
 *
 * Built from the 6 base emotions + eye/mouth weights + the 5 existing
 * decorators. Names are the cloud facing contract, see
 * docs/specs (add-lumi-like-robot-features §3).
 */
static const ExpressionPreset _presets[] = {
    // name        emotion         eye  mouth size  decorator                     sound           duration
    {"neutral",    Emotion::Neutral, 100,  0,    0, ExpressionDecorator::None,    nullptr,              0},
    {"happy",      Emotion::Happy,   100, 40,    0, ExpressionDecorator::None,    "happy",           2500},
    {"laughing",   Emotion::Happy,    60, 70,    0, ExpressionDecorator::Heart,   "happy",           2500},
    {"angry",      Emotion::Angry,   100, 30,    0, ExpressionDecorator::Angry,   "horn",            2000},
    {"sad",        Emotion::Sad,      90, 20,    0, ExpressionDecorator::None,    "sad",             2500},
    {"crying",     Emotion::Sad,      60, 60,    0, ExpressionDecorator::Sweat,   "sad",             3000},
    {"sleepy",     Emotion::Sleepy,   30, 20,    0, ExpressionDecorator::None,    nullptr,              0},
    {"doubtful",   Emotion::Doubt,   100, 10,    0, ExpressionDecorator::None,    nullptr,           2500},
    {"love",       Emotion::Happy,    70, 50,    0, ExpressionDecorator::Heart,   "happy",           3000},
    {"shy",        Emotion::Happy,    55, 30,    0, ExpressionDecorator::Shy,     nullptr,           2500},
    {"dizzy",      Emotion::Doubt,    90, 40,    0, ExpressionDecorator::Dizzy,   "error",           2500},
    {"sweat",      Emotion::Sad,      85, 35,    0, ExpressionDecorator::Sweat,   nullptr,           2500},
    {"surprised",  Emotion::Happy,   100, 80,   30, ExpressionDecorator::None,    "wake",            2000},
    {"thinking",   Emotion::Doubt,    85, 15,  -10, ExpressionDecorator::None,    "thinking",        2500},
    {"listening",  Emotion::Neutral, 100, 10,   10, ExpressionDecorator::None,    "listen_start",       0},
    {"excited",    Emotion::Happy,    95, 75,   20, ExpressionDecorator::None,    "dance_start",     2500},
    {"confused",   Emotion::Doubt,    90, 25,  -20, ExpressionDecorator::Sweat,   "error",           2500},
    {"cool",       Emotion::Neutral,  75, 25,  -15, ExpressionDecorator::None,    nullptr,           2500},
};

static constexpr size_t _preset_count = sizeof(_presets) / sizeof(_presets[0]);

const ExpressionPreset* findExpressionPreset(const char* name)
{
    if (!name) {
        return nullptr;
    }

    // Aliases: legacy / upstream names that are not part of the 18 presets.
    // "doubt" is the base Emotion name used by the cloud llm.emotion fallback,
    // "microchip_ai" is what the xiaozhi boot screen requests.
    struct Alias {
        const char* from;
        const char* to;
    };
    static const Alias _aliases[] = {
        {"doubt", "doubtful"},
        {"microchip_ai", "neutral"},
    };
    for (const auto& alias : _aliases) {
        if (strcmp(alias.from, name) == 0) {
            name = alias.to;
            break;
        }
    }

    for (size_t i = 0; i < _preset_count; ++i) {
        if (strcmp(_presets[i].name, name) == 0) {
            return &_presets[i];
        }
    }
    return nullptr;
}

void applyExpressionPreset(Modifiable& stackchan, const ExpressionPreset& preset, uint32_t decorator_lifetime_ms)
{
    if (!stackchan.hasAvatar()) {
        return;
    }

    auto& avatar = stackchan.avatar();

    avatar.setEmotion(preset.emotion);
    avatar.leftEye().setWeight(preset.eye_weight);
    avatar.rightEye().setWeight(preset.eye_weight);
    avatar.leftEye().setSize(preset.eye_size);
    avatar.rightEye().setSize(preset.eye_size);
    avatar.mouth().setWeight(preset.mouth_weight);

    if (preset.decorator == ExpressionDecorator::None) {
        return;
    }

    uint32_t lifetime = decorator_lifetime_ms;
    if (lifetime == 0) {
        lifetime = preset.default_duration_ms > 0 ? preset.default_duration_ms : 2000;
    }

    switch (preset.decorator) {
        case ExpressionDecorator::Heart:
            avatar.addDecorator(std::make_unique<HeartDecorator>(lv_screen_active(), lifetime, 500));
            break;
        case ExpressionDecorator::Angry:
            avatar.addDecorator(std::make_unique<AngryDecorator>(lv_screen_active(), lifetime, 500));
            break;
        case ExpressionDecorator::Sweat:
            avatar.addDecorator(std::make_unique<SweatDecorator>(lv_screen_active(), lifetime, 700));
            break;
        case ExpressionDecorator::Shy:
            avatar.addDecorator(std::make_unique<ShyDecorator>(lv_screen_active(), lifetime));
            break;
        case ExpressionDecorator::Dizzy:
            avatar.addDecorator(std::make_unique<DizzyDecorator>(lv_screen_active(), lifetime, 300));
            break;
        case ExpressionDecorator::None:
        default:
            break;
    }
}

}  // namespace stackchan::avatar
