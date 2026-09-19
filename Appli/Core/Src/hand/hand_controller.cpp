#include "hand/hand_controller.hpp"

#include <algorithm>
#include <cmath>

namespace HandControl {

namespace {
constexpr uint16_t MICROSTEP_TIME_MS = 10;
constexpr uint32_t TICK_MS = 2;
constexpr uint32_t FEEDBACK_STALE_MS = 500;
}

HandController::HandController(ServoBus& bus)
    : bus_(bus),
      admittance_{
          AdmittanceController({DEFAULT_ADMITTANCE_STIFFNESS_N_PER_PERCENT,
                                DEFAULT_ADMITTANCE_NATURAL_FREQUENCY_HZ,
                                DEFAULT_FORCE_DEADBAND_N, 120.0f}),
          AdmittanceController({DEFAULT_ADMITTANCE_STIFFNESS_N_PER_PERCENT,
                                DEFAULT_ADMITTANCE_NATURAL_FREQUENCY_HZ,
                                DEFAULT_FORCE_DEADBAND_N, 120.0f}),
          AdmittanceController({DEFAULT_ADMITTANCE_STIFFNESS_N_PER_PERCENT,
                                DEFAULT_ADMITTANCE_NATURAL_FREQUENCY_HZ,
                                DEFAULT_FORCE_DEADBAND_N, 120.0f}),
          AdmittanceController({DEFAULT_ADMITTANCE_STIFFNESS_N_PER_PERCENT,
                                DEFAULT_ADMITTANCE_NATURAL_FREQUENCY_HZ,
                                DEFAULT_FORCE_DEADBAND_N, 120.0f})}
{
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        reference_percent_[i] = 0.0f;
        reference_start_percent_[i] = 0.0f;
        reference_target_percent_[i] = 0.0f;
        command_percent_[i] = 0.0f;
        command_ticks_[i] = Hand::percentToServoPos(static_cast<Finger>(i), 0.0f);
        actual_ticks_[i] = command_ticks_[i];
    }
    for (auto& controller : admittance_) controller.reset(0.0f);
}

void HandController::setPose(const GripConfig& grip)
{
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        startTrajectory(i, static_cast<float>(grip.positionsPercent[i]));
    }
    mode_ = ControllerMode::Move;
}

void HandController::setTargetGrip(GripType grip)
{
    if (static_cast<size_t>(grip) >= static_cast<size_t>(GripType::Count)) return;
    setPose(GripDatabase[static_cast<size_t>(grip)]);
}

void HandController::setTargetGripWithForce(GripType grip, float force_newton)
{
    if (static_cast<size_t>(grip) >= static_cast<size_t>(GripType::Count)) return;
    if (!setForceAll(force_newton)) return;
    setPose(GripDatabase[static_cast<size_t>(grip)]);
}

bool HandController::setSingleFingerPercent(Finger finger, float percent)
{
    const size_t index = static_cast<size_t>(finger);
    if (index >= FINGER_COUNT || !std::isfinite(percent) || percent < 0.0f || percent > 100.0f) {
        return false;
    }
    startTrajectory(index, percent);
    mode_ = ControllerMode::Position;
    return true;
}

bool HandController::setSingleFingerPercentWithForce(Finger finger, float percent, float force_newton)
{
    const size_t index = static_cast<size_t>(finger);
    if (index < CONTROLLED_FINGER_FIRST || index > CONTROLLED_FINGER_LAST) return false;
    if (!setForce(finger, force_newton)) return false;
    return setSingleFingerPercent(finger, percent);
}

bool HandController::setForceAll(float force_newton)
{
    if (!std::isfinite(force_newton) || force_newton < 0.0f || force_newton > DEFAULT_FORCE_LIMIT_N) {
        return false;
    }
    for (size_t i = CONTROLLED_FINGER_FIRST; i <= CONTROLLED_FINGER_LAST; ++i) {
        force_setpoint_n_[i] = force_newton;
    }
    return true;
}

bool HandController::setForce(Finger finger, float force_newton)
{
    const size_t index = static_cast<size_t>(finger);
    if (index < CONTROLLED_FINGER_FIRST || index > CONTROLLED_FINGER_LAST ||
        !std::isfinite(force_newton) || force_newton < 0.0f || force_newton > DEFAULT_FORCE_LIMIT_N) {
        return false;
    }
    force_setpoint_n_[index] = force_newton;
    return true;
}

void HandController::setSpeed(uint16_t speed_deg_per_s)
{
    speed_deg_per_second_ = std::clamp<uint16_t>(speed_deg_per_s, 1U, 270U);
}

void HandController::setAdmittanceEnabled(bool enabled)
{
    admittance_enabled_ = enabled;
    output_pending_ = true;
    for (size_t i = CONTROLLED_FINGER_FIRST; i <= CONTROLLED_FINGER_LAST; ++i) {
        admittance_[i - CONTROLLED_FINGER_FIRST].setEnabled(enabled);
        admittance_[i - CONTROLLED_FINGER_FIRST].reset(reference_percent_[i]);
        command_percent_[i] = reference_percent_[i];
    }
    mode_ = enabled ? ControllerMode::Admittance : ControllerMode::Position;
}

void HandController::stopImmediate()
{
    admittance_enabled_ = false;
    output_pending_ = true;
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        moving_[i] = false;
        reference_start_percent_[i] = reference_percent_[i];
        reference_target_percent_[i] = reference_percent_[i];
        trajectory_elapsed_ms_[i] = 0;
        trajectory_duration_ms_[i] = 0;
        command_percent_[i] = reference_percent_[i];
    }
    for (auto& controller : admittance_) {
        controller.setEnabled(false);
        controller.reset(0.0f);
    }
    mode_ = ControllerMode::Hold;
}

void HandController::holdCurrent()
{
    stopImmediate();
}

bool HandController::isMoving() const
{
    for (bool moving : moving_) {
        if (moving) return true;
    }
    return false;
}

void HandController::startTrajectory(size_t index, float target_percent)
{
    if (index >= FINGER_COUNT) return;
    output_pending_ = true;
    target_percent = std::clamp(target_percent, 0.0f, 100.0f);
    reference_start_percent_[index] = reference_percent_[index];
    reference_target_percent_[index] = target_percent;
    trajectory_elapsed_ms_[index] = 0;

    const float distance_percent = std::fabs(target_percent - reference_percent_[index]);
    const float degrees_per_percent = 180.0f / 100.0f;
    const float speed = std::max(1.0f, static_cast<float>(speed_deg_per_second_));
    trajectory_duration_ms_[index] = static_cast<uint32_t>(
        std::max(1.0f, distance_percent * degrees_per_percent * 1000.0f / speed));
    moving_[index] = distance_percent > 0.01f;
    if (!moving_[index]) {
        reference_percent_[index] = target_percent;
        command_percent_[index] = target_percent;
        if (index >= CONTROLLED_FINGER_FIRST && index <= CONTROLLED_FINGER_LAST) {
            admittance_[index - CONTROLLED_FINGER_FIRST].reset(target_percent);
        }
    }
}

float HandController::forceForFinger(size_t index, const FSR_Snapshot& fsr) const
{
    if (index >= FSR400_SENSOR_COUNT) return 0.0f;
    return fsr.force_newton[index];
}

void HandController::update(const FSR_Snapshot& fsr)
{
    ++sequence_;
    fault_flags_ &= ~(FAULT_FORCE_UNREACHED | FAULT_EXTENSION_LIMIT);

    bool any_moving = false;

    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        if (moving_[i]) {
            trajectory_elapsed_ms_[i] += TICK_MS;
            if (trajectory_elapsed_ms_[i] >= trajectory_duration_ms_[i]) {
                reference_percent_[i] = reference_target_percent_[i];
                moving_[i] = false;
                if (i >= CONTROLLED_FINGER_FIRST && i <= CONTROLLED_FINGER_LAST) {
                    admittance_[i - CONTROLLED_FINGER_FIRST].reset(reference_percent_[i]);
                }
            } else {
                const float t = static_cast<float>(trajectory_elapsed_ms_[i]) /
                                static_cast<float>(trajectory_duration_ms_[i]);
                const float alpha = smoothstep(t);
                reference_percent_[i] = reference_start_percent_[i] +
                                        alpha * (reference_target_percent_[i] - reference_start_percent_[i]);
                any_moving = true;
            }
        }

        float command = reference_percent_[i];
        if (i >= CONTROLLED_FINGER_FIRST && i <= CONTROLLED_FINGER_LAST &&
            admittance_enabled_ && !moving_[i]) {
            const size_t regulator = i - CONTROLLED_FINGER_FIRST;
            measured_force_n_[i] = forceForFinger(i, fsr);
            command = admittance_[regulator].step(reference_percent_[i], force_setpoint_n_[i],
                                                  measured_force_n_[i], CONTROL_DT_S);
            if (command >= 99.99f && measured_force_n_[i] + 0.05f < force_setpoint_n_[i]) {
                fault_flags_ |= FAULT_FORCE_UNREACHED;
            }
            if (command <= 0.01f && measured_force_n_[i] > force_setpoint_n_[i] + 0.05f) {
                fault_flags_ |= FAULT_EXTENSION_LIMIT;
            }
        } else if (i < FSR400_SENSOR_COUNT) {
            measured_force_n_[i] = forceForFinger(i, fsr);
        }

        command_percent_[i] = std::clamp(command, 0.0f, 100.0f);
        command_ticks_[i] = Hand::percentToServoPos(static_cast<Finger>(i), command_percent_[i]);
        any_moving = any_moving || moving_[i] ||
                     (i >= CONTROLLED_FINGER_FIRST && i <= CONTROLLED_FINGER_LAST &&
                      admittance_enabled_ &&
                      std::fabs(admittance_[i - CONTROLLED_FINGER_FIRST].velocity()) > 0.01f);
    }

    if (admittance_enabled_) mode_ = ControllerMode::Admittance;
    else if (any_moving) mode_ = ControllerMode::Move;

    output_pending_ = output_pending_ || any_moving || admittance_enabled_;
}

bool HandController::copyOutputFrame(uint8_t* ids, uint16_t* positions, uint16_t* times,
                                     size_t count) const
{
    if (ids == nullptr || positions == nullptr || times == nullptr || count < FINGER_COUNT) return false;
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        ids[i] = Hand::getServoID(static_cast<Finger>(i));
        positions[i] = command_ticks_[i];
        times[i] = MICROSTEP_TIME_MS;
    }
    return true;
}

void HandController::setActualPosition(Finger finger, uint16_t position, uint32_t now_ms)
{
    const size_t index = static_cast<size_t>(finger);
    if (index >= FINGER_COUNT) return;
    actual_ticks_[index] = position;
    position_feedback_time_ms_[index] = now_ms;
    position_feedback_valid_[index] = true;
}

void HandController::setActualCurrent(Finger finger, int32_t current_mA, uint32_t now_ms)
{
    const size_t index = static_cast<size_t>(finger);
    if (index >= FINGER_COUNT) return;
    current_mA_[index] = current_mA;
    current_feedback_time_ms_[index] = now_ms;
    current_feedback_valid_[index] = true;
}

void HandController::getStatus(ControllerStatus* status, uint32_t now_ms) const
{
    if (status == nullptr) return;
    status->mode = mode_;
    status->sequence = sequence_;
    status->speedDegPerSecond = speed_deg_per_second_;
    status->torqueLimitPercent = torque_limit_percent_;
    status->faultFlags = fault_flags_;
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        if (!position_feedback_valid_[i] ||
            (now_ms - position_feedback_time_ms_[i]) > FEEDBACK_STALE_MS) {
            status->faultFlags |= FAULT_SERVO_POSITION_STALE;
        }
        if (!current_feedback_valid_[i] ||
            (now_ms - current_feedback_time_ms_[i]) > FEEDBACK_STALE_MS) {
            status->faultFlags |= FAULT_SERVO_CURRENT_STALE;
        }
        status->referencePercent[i] = reference_percent_[i];
        status->commandPercent[i] = command_percent_[i];
        status->actualPercent[i] = Hand::servoPosToPercent(static_cast<Finger>(i), actual_ticks_[i]);
        status->targetTicks[i] = Hand::percentToServoPos(static_cast<Finger>(i), reference_target_percent_[i]);
        status->commandTicks[i] = command_ticks_[i];
        status->actualTicks[i] = actual_ticks_[i];
        status->currentMilliamp[i] = current_mA_[i];
        status->forceSetpoint[i] = force_setpoint_n_[i];
        status->forceMeasured[i] = measured_force_n_[i];
    }
}

float HandController::smoothstep(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace HandControl
