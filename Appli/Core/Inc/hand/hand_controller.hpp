#ifndef HAND_CONTROLLER_HPP
#define HAND_CONTROLLER_HPP

#include "hand/admittance_controller.hpp"
#include "hand/fsr400.hpp"
#include "hand/hand_config.hpp"
#include "hand/servo.hpp"

#include <array>
#include <cstdint>

namespace HandControl {

enum class ControllerMode : uint8_t {
    Boot,
    Tare,
    Move,
    Position,
    Admittance,
    Hold,
    Fault
};

struct ControllerStatus {
    ControllerMode mode = ControllerMode::Boot;
    uint32_t sequence = 0;
    uint16_t targetTicks[FINGER_COUNT]{};
    uint16_t commandTicks[FINGER_COUNT]{};
    uint16_t actualTicks[FINGER_COUNT]{};
    int32_t currentMilliamp[FINGER_COUNT]{};
    float referencePercent[FINGER_COUNT]{};
    float commandPercent[FINGER_COUNT]{};
    float actualPercent[FINGER_COUNT]{};
    float forceSetpoint[FINGER_COUNT]{};
    float forceMeasured[FINGER_COUNT]{};
    uint16_t speedDegPerSecond = DEFAULT_SPEED_DEG_PER_S;
    uint16_t torqueLimitPercent = DEFAULT_TORQUE_LIMIT_PERCENT;
    uint32_t faultFlags = 0;
};

class HandController {
public:
    static constexpr size_t FINGER_COUNT_LOCAL = FINGER_COUNT;

    explicit HandController(ServoBus& bus);

    void setTargetGrip(GripType grip);
    void setTargetGripWithForce(GripType grip, float force_newton);
    bool setSingleFingerPercent(Finger finger, float percent);
    bool setSingleFingerPercentWithForce(Finger finger, float percent, float force_newton);
    bool setForceAll(float force_newton);
    bool setForce(Finger finger, float force_newton);

    void setSpeed(uint16_t speed_deg_per_s);
    uint16_t speed() const { return speed_deg_per_second_; }
    void setTorqueLimit(uint16_t percent) { torque_limit_percent_ = percent > 100 ? 100 : percent; }
    uint16_t torqueLimit() const { return torque_limit_percent_; }

    void setAdmittanceEnabled(bool enabled);
    bool admittanceEnabled() const { return admittance_enabled_; }
    bool isMoving() const;
    void stopImmediate();
    void holdCurrent();

    /* Called after one complete ADC scan. It is deterministic and non-blocking. */
    void update(const FSR_Snapshot& fsr);
    void getStatus(ControllerStatus* status) const;
    bool outputPending() const { return output_pending_; }
    bool copyOutputFrame(uint8_t* ids, uint16_t* positions, uint16_t* times,
                         size_t count) const;
    uint32_t outputSequence() const { return sequence_; }
    void markOutputSent(uint32_t sequence) {
        if (sequence_ == sequence) output_pending_ = false;
    }

    void setActualFeedback(Finger finger, uint16_t position, int32_t current_mA,
                           uint32_t now_ms);
    void setActualCurrent(Finger finger, int32_t current_mA, uint32_t now_ms);

private:
    static float smoothstep(float t);
    void startTrajectory(size_t index, float target_percent);
    void setPose(const GripConfig& grip);
    float forceForFinger(size_t index, const FSR_Snapshot& fsr) const;

    ServoBus& bus_;
    std::array<float, FINGER_COUNT> reference_percent_{};
    std::array<float, FINGER_COUNT> reference_start_percent_{};
    std::array<float, FINGER_COUNT> reference_target_percent_{};
    std::array<float, FINGER_COUNT> command_percent_{};
    std::array<float, FINGER_COUNT> force_setpoint_n_{};
    std::array<float, FINGER_COUNT> measured_force_n_{};
    std::array<uint16_t, FINGER_COUNT> command_ticks_{};
    std::array<uint16_t, FINGER_COUNT> actual_ticks_{};
    std::array<int32_t, FINGER_COUNT> current_mA_{};
    std::array<uint32_t, FINGER_COUNT> feedback_time_ms_{};
    std::array<uint32_t, FINGER_COUNT> trajectory_elapsed_ms_{};
    std::array<uint32_t, FINGER_COUNT> trajectory_duration_ms_{};
    std::array<bool, FINGER_COUNT> moving_{};
    std::array<AdmittanceController, 4> admittance_;

    uint16_t speed_deg_per_second_ = DEFAULT_SPEED_DEG_PER_S;
    uint16_t torque_limit_percent_ = DEFAULT_TORQUE_LIMIT_PERCENT;
    bool admittance_enabled_ = false;
    bool output_pending_ = true;
    ControllerMode mode_ = ControllerMode::Boot;
    uint32_t sequence_ = 0;
    uint32_t fault_flags_ = 0;
};

} // namespace HandControl

#endif // HAND_CONTROLLER_HPP
