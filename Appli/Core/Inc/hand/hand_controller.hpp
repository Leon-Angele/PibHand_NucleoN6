// hand_controller.hpp
// Non-blocking 100Hz HandController with smooth interpolation and placeholder AI hook

#ifndef HAND_CONTROLLER_HPP
#define HAND_CONTROLLER_HPP

#include "hand/hand_config.hpp"
#include "hand/servo.hpp"
#include "main.h"

#include <array>
#include <cstdint>
#include <cstddef>

class HandController {
public:
    static constexpr size_t AXIS_COUNT = static_cast<size_t>(HandControl::Finger::Count);

    HandController(HandControl::Hand::Side side, ServoBus& bus) noexcept;

    // Start a smooth trajectory to the predefined grip over duration_ms milliseconds
    void setTargetGrip(HandControl::GripType grip, uint16_t duration_ms) noexcept;

    // Called at 100Hz from main loop; non-blocking
    void update() noexcept;

private:
    HandControl::Hand::Side side_;
    ServoBus& bus_;

    // Per-axis trajectory state
    std::array<uint16_t, AXIS_COUNT> start_pos_{};
    std::array<uint16_t, AXIS_COUNT> target_pos_{};
    std::array<uint16_t, AXIS_COUNT> current_pos_{};
    std::array<uint32_t, AXIS_COUNT> start_time_ms_{};
    std::array<uint32_t, AXIS_COUNT> duration_ms_{};
    std::array<bool, AXIS_COUNT> moving_{};

    // Torque lifecycle: enable before first non-Open grip, disable after Open completes
    HandControl::GripType current_grip_ = HandControl::GripType::Open;
    bool torque_enabled_ = false;
    bool pending_torque_disable_ = false;

    // Helpers
    static float smoothstep(float t) noexcept;

    // AI placeholder to adjust grasp; return correction factor (0..1) or negative to indicate stop
    float predictGraspAdjustment(uint8_t finger_idx, int16_t current_load, int16_t current_speed, float position_error) noexcept;

    // Build and send sync-write for current_pos_
    void sendSyncWrite(uint16_t time_ms) noexcept;
};

#endif // HAND_CONTROLLER_HPP