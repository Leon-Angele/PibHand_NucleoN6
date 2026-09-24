/**
 * @file hand_config.hpp
 * @brief Single-hand configuration for six STS3215 axes.
 */
#ifndef HAND_CONFIG_HPP
#define HAND_CONFIG_HPP

#ifdef __cplusplus
#include <array>
#include <cstdint>
#include <string_view>
#else
#include <stdint.h>
#endif

#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif

#ifndef AS5600_ENABLED
#define AS5600_ENABLED 0
#endif

#if DEBUG_PRINTS
#ifdef __cplusplus
#include <cstdio>
#define HAND_DEBUG(fmt, ...) std::printf("[HAND] " fmt "\r\n", ##__VA_ARGS__)
#else
#include <stdio.h>
#define HAND_DEBUG(fmt, ...) printf("[HAND] " fmt "\r\n", ##__VA_ARGS__)
#endif
#else
#define HAND_DEBUG(fmt, ...) ((void)0)
#endif

#ifdef __cplusplus
namespace HandControl {

enum class Finger : uint8_t {
    Thumb = 0,
    Index,
    Middle,
    Ring,
    Pinky,
    ThumbRotation,
    Count
};

enum class GripType : uint8_t {
    Open,
    Spitzgriff,
    Dreipunktgriff,
    Schluesselgriff,
    Zylindergriff,
    Hakengriff,
    SphaerischerGriff,
    Mittelfinger,
    Count
};

struct FingerConfig {
    std::string_view name;
    uint8_t servoId;
    uint16_t zeroPos;
    uint16_t maxPos;
};

struct GripConfig {
    GripType type;
    std::string_view name;
    std::array<uint8_t, static_cast<size_t>(Finger::Count)> positionsPercent;
};

inline constexpr size_t FINGER_COUNT = static_cast<size_t>(Finger::Count);
inline constexpr size_t CONTROLLED_FINGER_FIRST = static_cast<size_t>(Finger::Index);
inline constexpr size_t CONTROLLED_FINGER_LAST = static_cast<size_t>(Finger::Pinky);

inline constexpr uint16_t DEFAULT_SPEED_DEG_PER_S = 200;
inline constexpr uint16_t DEFAULT_TORQUE_LIMIT_PERCENT = 50;
inline constexpr float DEFAULT_FORCE_LIMIT_N = 5.0f;
inline constexpr float DEFAULT_FORCE_DEADBAND_N = 0.05f;
inline constexpr float DEFAULT_ADMITTANCE_STIFFNESS_N_PER_PERCENT = 0.1f;
inline constexpr float DEFAULT_ADMITTANCE_NATURAL_FREQUENCY_HZ = 3.0f;
inline constexpr float CONTROL_DT_S = 0.002f;

inline constexpr std::array<FingerConfig, FINGER_COUNT> AxisSettings = {{
    {"Thumb",          1, 4095, 0},
    {"Index",          2, 4095, 0},
    {"Middle",         3, 4095, 400},
    {"Ring",           4, 4095, 0},
    {"Pinky",          5, 4095, 400},
    {"ThumbRotation",  6, 2047, 4095}
}};

/* Positions are expressed as percentage of each axis travel. */
inline constexpr std::array<GripConfig, static_cast<size_t>(GripType::Count)> GripDatabase = {{
    {GripType::Open,             "OPEN",              {0,   0,  0,  0,  0,  0}},
    {GripType::Spitzgriff,       "ZEIGEN",            {100, 0, 100, 100, 100, 50}},
    {GripType::Dreipunktgriff,   "DREIPUNKTGRIFF",    {78,  78, 78, 0,   0,  50}},
    {GripType::Schluesselgriff,  "SCHLUESSELGRIFF",   {67,  33, 0,  0,  0, 67}},
    {GripType::Zylindergriff,    "ZYLINDERGRIFF",     {89,  89, 89, 89, 89, 33}},
    {GripType::Hakengriff,       "HAKENGRIFF",        {0,   89, 89, 89, 89, 0}},
    {GripType::SphaerischerGriff,"SPHAERISCHER_GRIFF",{67,  67, 67, 67, 67, 44}},
    {GripType::Mittelfinger,     "MITTELFINGER",      {100, 100, 0, 100, 100, 49}}
}};

class Hand {
public:
    static constexpr uint8_t getServoID(Finger finger)
    {
        return AxisSettings[static_cast<size_t>(finger)].servoId;
    }

    static constexpr const FingerConfig& getAxisConfig(Finger finger)
    {
        return AxisSettings[static_cast<size_t>(finger)];
    }

    static constexpr uint16_t percentToServoPos(Finger finger, float percent)
    {
        if (percent <= 0.0f) return getAxisConfig(finger).zeroPos;
        if (percent >= 100.0f) return getAxisConfig(finger).maxPos;

        const FingerConfig& cfg = getAxisConfig(finger);
        const int32_t delta = static_cast<int32_t>(cfg.maxPos) - static_cast<int32_t>(cfg.zeroPos);
        int32_t value = static_cast<int32_t>(cfg.zeroPos + (delta * percent) / 100.0f);
        if (value < 0) value = 0;
        if (value > 4095) value = 4095;
        return static_cast<uint16_t>(value);
    }

    static constexpr uint16_t percentToServoPos(Finger finger, uint8_t percent)
    {
        return percentToServoPos(finger, static_cast<float>(percent));
    }

    static constexpr float servoPosToPercent(Finger finger, uint16_t servoPos)
    {
        const FingerConfig& cfg = getAxisConfig(finger);
        const int32_t delta = static_cast<int32_t>(cfg.maxPos) - static_cast<int32_t>(cfg.zeroPos);
        if (delta == 0) return 0.0f;
        float percent = (static_cast<float>(static_cast<int32_t>(servoPos) - cfg.zeroPos) * 100.0f) /
                        static_cast<float>(delta);
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        return percent;
    }
};

} // namespace HandControl
#endif

#endif // HAND_CONFIG_HPP
