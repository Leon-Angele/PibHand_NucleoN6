/**
 * @file hand_controller.hpp
 * @brief Application layer for robotic hand control with 6 STS3215 servos.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Features:
 * - Non-blocking trajectory interpolation (smoothstep)
 * - Round-robin telemetry polling (1 servo per update cycle)
 * - Async state machine for closed-loop control
 *
 * Called from main loop at ~100Hz - MUST be non-blocking!
 */
#ifndef HAND_CONTROLLER_HPP
#define HAND_CONTROLLER_HPP

#include "hand/hand_config.hpp"
#include "hand/servo.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace HandControl {

/**
 * @brief Hand controller with non-blocking update loop
 * 
 * Architecture:
 * - Part 1 (Movement): Interpolates trajectories and sends syncWritePositions
 * - Part 2 (Telemetry): Round-robin polling of servo current for closed-loop control
 * 
 * Update frequency: ~100Hz (called from main loop)
 * Round-robin cycle: 60ms (6 servos × 10ms timeout)
 */
class HandController {
public:
    static constexpr size_t FINGER_COUNT = static_cast<size_t>(Finger::Count);
    
    /**
     * @brief Constructor
     * @param side Hand side (Left or Right)
     * @param bus Reference to servo bus (shared between hands)
     */
    HandController(Hand::Side side, ServoBus& bus);
    
    /**
     * @brief Set target grip with smooth trajectory
     * 
     * Each finger moves at its configured maxSpeed from AxisSettings.
     * Movement duration is calculated individually per finger based on
     * distance and speed: duration = (delta * 1000) / maxSpeed
     * 
     * @param grip Target grip type
     */
    void setTargetGrip(GripType grip);
    /**
     * @brief Set target grip with per-finger percent speeds (0..100)
     * @param grip Grip type
     * @param perFingerPercent Pointer to 6 values (percent 0..100)
     */
    void setTargetGripWithPercent(GripType grip, const uint16_t* perFingerPercent);

    /**
     * @brief Set a single finger to a position with optional speed (deg/s). If speed==0, use axis maxSpeed.
     */
    void setSingleFingerPosition(Finger finger, uint16_t position, uint16_t speed_deg_per_s = 0);

    /**
     * @brief Immediately stop all movements for this hand.
     */
    void stopImmediate();

    /**
     * @brief Hold current positions (stop trajectories and command hold to servos).
     */
    void holdCurrent();
    
    /**
     * @brief Non-blocking update (called at ~100Hz from main loop)
     * 
     * Part 1: Trajectory interpolation + syncWritePositions
     * Part 2: Round-robin telemetry polling (1 servo per cycle)
     * 
     * CRITICAL: This function MUST be non-blocking! Max execution time < 500μs
     */
    void update();

private:
    Hand::Side side_;
    ServoBus& bus_;
    
    // ===== TRAJECTORY STATE (per finger) =====
    std::array<uint16_t, FINGER_COUNT> start_pos_{};
    std::array<uint16_t, FINGER_COUNT> target_pos_{};
    std::array<uint16_t, FINGER_COUNT> current_pos_{};
    std::array<uint32_t, FINGER_COUNT> start_time_ms_{};
    std::array<uint32_t, FINGER_COUNT> duration_ms_{};
    std::array<bool, FINGER_COUNT> moving_{};
    
    // ===== ROUND-ROBIN TELEMETRY STATE =====
    uint8_t poll_finger_idx_ = 0;  // Current finger being polled (0-5)
    
    // ===== HELPERS =====
    
    /**
     * @brief Smoothstep interpolation (cubic easing)
     * @param t Normalized time [0.0, 1.0]
     * @return Interpolation factor [0.0, 1.0]
     */
    static float smoothstep(float t);
    
    /**
     * @brief Interpolate position for given finger
     * @param finger_idx Finger index (0-5)
     * @param now Current time in milliseconds
     * @return Interpolated position
     */
    uint16_t interpolatePosition(size_t finger_idx, uint32_t now);
    
    /**
     * @brief Placeholder for AI-based grasp adjustment (future X-CUBE-AI integration)
     * @param finger_idx Finger index
     * @param current Measured current in mA
     */
    void predictGraspAdjustment(uint8_t finger_idx, int16_t current);

    /**
     * @brief Handle an over-current event for a finger by stopping its motor.
     * @param finger_idx Finger index that triggered the over-current
     * @param measured_current Measured current in mA
     *
     * This will immediately stop movement of the specific finger and
     * send a position hold command to the servo to prevent further motion.
     */
    void handleOverCurrent(uint8_t finger_idx, int16_t measured_current);
};

} // namespace HandControl

#endif // HAND_CONTROLLER_HPP
