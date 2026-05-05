#include "hand/hand_controller.hpp"
#include "hand/hand_config.hpp"
#include <algorithm>
#include <cstring>

using namespace HandControl;

HandController::HandController(Hand::Side side, ServoBus& bus) noexcept
    : side_(side), bus_(bus)
{
    // Initialize current positions from GripDatabase Open (safe default)
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        current_pos_[i] = Hand::mapToServoPos(static_cast<uint16_t>(0));
        start_pos_[i] = current_pos_[i];
        target_pos_[i] = current_pos_[i];
        duration_ms_[i] = 0;
        start_time_ms_[i] = 0;
        moving_[i] = false;
    }
}

void HandController::setTargetGrip(GripType grip, uint16_t duration_ms) noexcept
{
    const auto& cfg = GripDatabase[static_cast<size_t>(grip)];
    HAND_DEBUG("Side %d starting grip %d (Duration: %d ms)", static_cast<int>(side_), static_cast<int>(grip), duration_ms);

    // Enable torque for all axes before any non-Open grip
    if (!torque_enabled_) {
        HAND_DEBUG("Side %d enabling torque for all servos", static_cast<int>(side_));
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            uint8_t id = Hand::getServoID(side_, static_cast<Finger>(i));
            Servo s(id, bus_);
            if (!s.setTorqueEnable(true)) {
                HAND_DEBUG("Side %d torque enable FAILED for servo id %d", static_cast<int>(side_), static_cast<int>(id));
            }
        }
        torque_enabled_ = true;
    }

    current_grip_ = grip;
    pending_torque_disable_ = (grip == GripType::Open);

    uint32_t now = HAL_GetTick();
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        uint16_t tgt = Hand::mapToServoPos(cfg.positions[i]);
        start_pos_[i] = current_pos_[i];
        target_pos_[i] = tgt;
        start_time_ms_[i] = now;
        duration_ms_[i] = duration_ms;
        moving_[i] = (start_pos_[i] != target_pos_[i]);
    }
}

float HandController::smoothstep(float t) noexcept
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

float HandController::predictGraspAdjustment(uint8_t /*finger_idx*/, int16_t /*current_load*/, int16_t /*current_speed*/, float /*position_error*/) noexcept
{
    // Placeholder: no adjustment by default. Return 0.0f meaning no correction.
    // Replace with actual AI model integration (X-CUBE-AI) later.
    return 0.0f;
}

void HandController::sendSyncWrite(uint16_t time_ms) noexcept
{
    uint8_t ids[AXIS_COUNT];
    uint16_t poses[AXIS_COUNT];
    uint16_t times[AXIS_COUNT];
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        ids[i] = Hand::getServoID(side_, static_cast<Finger>(i));
        poses[i] = current_pos_[i];
        times[i] = time_ms;
    }
    // best-effort; ignore return value (could log)
    (void) bus_.syncWritePositions(ids, poses, times, AXIS_COUNT);
}

void HandController::update() noexcept
{
    const uint32_t now = HAL_GetTick();

    // 1) Interpolate trajectories
    bool all_done = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!moving_[i]) continue;
        all_done = false;
        uint32_t elapsed = (now >= start_time_ms_[i]) ? (now - start_time_ms_[i]) : 0;
        float t = (duration_ms_[i] == 0) ? 1.0f : (static_cast<float>(elapsed) / static_cast<float>(duration_ms_[i]));
        if (t >= 1.0f) {
            current_pos_[i] = target_pos_[i];
            moving_[i] = false;
            if (i == 0) {
                HAND_DEBUG("Side %d representative finger %d reached target", static_cast<int>(side_), static_cast<int>(i));
            }
        } else {
            float s = smoothstep(t);
            int32_t val = static_cast<int32_t>(start_pos_[i]) +
                          static_cast<int32_t>((static_cast<int32_t>(target_pos_[i]) - static_cast<int32_t>(start_pos_[i])) * s);
            if (val < 0) val = 0;
            if (val > 0xFFFF) val = 0xFFFF;
            current_pos_[i] = static_cast<uint16_t>(val);
        }
    }

    // 2) Disable torque after Open grip fully reaches target
    if (pending_torque_disable_ && torque_enabled_ && all_done) {
        HAND_DEBUG("Side %d disabling torque after Open completed", static_cast<int>(side_));
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            uint8_t id = Hand::getServoID(side_, static_cast<Finger>(i));
            Servo s(id, bus_);
            if (!s.setTorqueEnable(false)) {
                HAND_DEBUG("Side %d torque disable FAILED for servo id %d", static_cast<int>(side_), static_cast<int>(id));
            }
        }
        torque_enabled_ = false;
        pending_torque_disable_ = false;
    }

    // 3) Send sync-write only when torque is active
    if (torque_enabled_) {
        sendSyncWrite(50);
    }
}
