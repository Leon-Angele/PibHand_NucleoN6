#include "hand/admittance_controller.hpp"

#include <algorithm>
#include <cmath>

namespace HandControl {

AdmittanceController::AdmittanceController(const AdmittanceParameters& parameters)
    : parameters_(parameters)
{
    const float omega = 2.0f * 3.14159265359f * parameters_.natural_frequency_hz;
    if (omega > 0.0f && parameters_.stiffness_n_per_percent > 0.0f) {
        mass_ = parameters_.stiffness_n_per_percent / (omega * omega);
        damping_ = 2.0f * std::sqrt(parameters_.stiffness_n_per_percent * mass_);
    } else {
        mass_ = 1.0f;
        damping_ = 0.0f;
    }
}

void AdmittanceController::reset(float reference_percent)
{
    position_percent_ = reference_percent;
    velocity_percent_s_ = 0.0f;
}

float AdmittanceController::step(float reference_percent, float force_setpoint_n,
                                 float measured_force_n, float dt_s,
                                 float min_percent, float max_percent)
{
    if (!enabled_ || dt_s <= 0.0f || mass_ <= 0.0f) {
        position_percent_ = std::clamp(reference_percent, min_percent, max_percent);
        velocity_percent_s_ = 0.0f;
        return position_percent_;
    }

    float force_error = force_setpoint_n - measured_force_n;
    if (std::fabs(force_error) <= parameters_.deadband_n) {
        force_error = 0.0f;
    }

    const float spring_force = parameters_.stiffness_n_per_percent *
                               std::min(position_percent_ - reference_percent, 0.0f);
    const float acceleration = (force_error - damping_ * velocity_percent_s_ - spring_force) / mass_;

    velocity_percent_s_ += acceleration * dt_s;
    velocity_percent_s_ = std::clamp(velocity_percent_s_,
                                     -parameters_.max_velocity_percent_s,
                                     parameters_.max_velocity_percent_s);

    const float next_position = position_percent_ + velocity_percent_s_ * dt_s;
    position_percent_ = std::clamp(next_position, min_percent, max_percent);

    if ((position_percent_ <= min_percent && velocity_percent_s_ < 0.0f) ||
        (position_percent_ >= max_percent && velocity_percent_s_ > 0.0f)) {
        velocity_percent_s_ = 0.0f;
    }
    return position_percent_;
}

} // namespace HandControl
