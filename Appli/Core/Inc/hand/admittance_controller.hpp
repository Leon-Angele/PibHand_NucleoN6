#ifndef ADMITTANCE_CONTROLLER_HPP
#define ADMITTANCE_CONTROLLER_HPP

#include <cstdint>

namespace HandControl {

struct AdmittanceParameters {
    float stiffness_n_per_percent;
    float natural_frequency_hz;
    float deadband_n;
    float max_velocity_percent_s;
};

class AdmittanceController {
public:
    explicit AdmittanceController(const AdmittanceParameters& parameters);

    void reset(float reference_percent);
    float step(float reference_percent, float force_setpoint_n, float measured_force_n,
               float dt_s, float min_percent = 0.0f, float max_percent = 100.0f);

    float position() const { return position_percent_; }
    float velocity() const { return velocity_percent_s_; }
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool enabled() const { return enabled_; }

private:
    AdmittanceParameters parameters_;
    float mass_ = 0.0f;
    float damping_ = 0.0f;
    float position_percent_ = 0.0f;
    float velocity_percent_s_ = 0.0f;
    bool enabled_ = false;
};

} // namespace HandControl

#endif // ADMITTANCE_CONTROLLER_HPP
