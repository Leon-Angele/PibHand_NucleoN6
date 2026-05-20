/**
 * @file hand_controller.cpp
 * @brief Implementation of the non-blocking HandController and helpers.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Non-blocking trajectory interpolation and round-robin telemetry polling
 * for the robotic hand (6 servos per hand).
 */

#include "hand/hand_controller.hpp"
#include <cmath>
// BSP LED control for Nucleo board
#include "stm32n6xx_nucleo.h"

namespace HandControl {

// Microstep time (ms) used when controller drives micro-steps
static constexpr uint16_t MICROSTEP_TIME = 10;
// LED toggle interval when servos are moving (ms)
static constexpr uint32_t LED_TOGGLE_MS = 100;
// Last tick when we toggled the blue LED
static uint32_t led_last_toggle_ms = 0;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

HandController::HandController(Hand::Side side, ServoBus& bus)
    : side_(side), bus_(bus)
{
    // Initialize all fingers to open position (0)
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        start_pos_[i] = 0;
        target_pos_[i] = 0;
        current_pos_[i] = 0;
        start_time_ms_[i] = 0;
        duration_ms_[i] = 0;
        moving_[i] = false;
    }
    
}

/**
 * @brief Construct a new HandController instance.
 *
 * Initializes internal state for trajectory interpolation and telemetry.
 * @param side Hand::Side indicating left or right hand
 * @param bus Reference to the shared ServoBus instance
 */

// ============================================================================
// PUBLIC API
// ============================================================================

void HandController::setTargetGrip(GripType grip)
{
    // Lookup grip configuration from database
    if (static_cast<size_t>(grip) >= static_cast<size_t>(GripType::Count)) {
        HAND_DEBUG("Invalid grip type: %d", static_cast<int>(grip));
        return;
    }
    
    const GripConfig& grip_cfg = GripDatabase[static_cast<size_t>(grip)];
    
    uint32_t now = HAL_GetTick();
    
    // Setup trajectory for all 6 fingers with individual speed control
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        start_pos_[i] = current_pos_[i];
        target_pos_[i] = grip_cfg.positions[i];
        start_time_ms_[i] = now;
        
        // Calculate individual duration based on distance and configured maxSpeed
        uint16_t delta = (target_pos_[i] > start_pos_[i]) 
                         ? (target_pos_[i] - start_pos_[i]) 
                         : (start_pos_[i] - target_pos_[i]);
        
        uint16_t max_speed_deg_per_sec = Hand::getAxisConfig(side_, static_cast<Finger>(i)).maxSpeed;  // degrees per second
        
        // Handle edge cases
        if (delta == 0) {
            duration_ms_[i] = 0;
            moving_[i] = false;
        } else if (max_speed_deg_per_sec == 0) {
            // Fallback to default speed if maxSpeed is zero
            duration_ms_[i] = (delta * 1000) / 1000;  // 1000 units/s default
            moving_[i] = true;
            HAND_DEBUG("Warning: maxSpeed=0 for finger %d, using default 1000 units/s", i);
        } else {
            // Convert degrees/s to servo units/s: 360° = 4095 units
            // units/s = deg/s * (4095 / 360)
            uint32_t max_speed_units_per_sec = (static_cast<uint32_t>(max_speed_deg_per_sec) * 4095) / 360;
            
            // Normal case: duration = distance / speed
            duration_ms_[i] = (static_cast<uint32_t>(delta) * 1000) / max_speed_units_per_sec;
            moving_[i] = true;
        }
    }
    
    HAND_DEBUG("Grip set: %s (side=%d, per-finger speeds)", 
               grip_cfg.name.data(), static_cast<int>(side_));
}

void HandController::setTargetGripWithPercent(GripType grip, const uint16_t* perFingerPercent)
{
    if (static_cast<size_t>(grip) >= static_cast<size_t>(GripType::Count)) {
        HAND_DEBUG("Invalid grip type: %d", static_cast<int>(grip));
        return;
    }
    const GripConfig& grip_cfg = GripDatabase[static_cast<size_t>(grip)];

    uint32_t now = HAL_GetTick();

    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        start_pos_[i] = current_pos_[i];
        target_pos_[i] = grip_cfg.positions[i];
        start_time_ms_[i] = now;

        uint16_t delta = (target_pos_[i] > start_pos_[i]) ? (target_pos_[i] - start_pos_[i]) : (start_pos_[i] - target_pos_[i]);

        uint16_t pct = (perFingerPercent) ? perFingerPercent[i] : 100;
        uint16_t axis_max_deg_s = Hand::getAxisConfig(side_, static_cast<Finger>(i)).maxSpeed;
        uint32_t use_deg_s = (axis_max_deg_s == 0) ? 1000 : ((static_cast<uint32_t>(axis_max_deg_s) * pct) / 100);

        uint32_t max_speed_units_per_sec = (use_deg_s * 4095) / 360;
        if (delta == 0) { duration_ms_[i] = 0; moving_[i] = false; }
        else if (max_speed_units_per_sec == 0) { duration_ms_[i] = (delta * 1000) / 1000; moving_[i] = true; }
        else { duration_ms_[i] = (static_cast<uint32_t>(delta) * 1000) / max_speed_units_per_sec; moving_[i] = true; }
    }

    HAND_DEBUG("Grip set with per-finger percent: %s (side=%d)", grip_cfg.name.data(), static_cast<int>(side_));
}

void HandController::setSingleFingerPosition(Finger finger, uint16_t position, uint16_t speed_deg_per_s)
{
    size_t i = static_cast<size_t>(finger);
    if (i >= FINGER_COUNT) return;
    uint32_t now = HAL_GetTick();
    start_pos_[i] = current_pos_[i];
    target_pos_[i] = position;
    start_time_ms_[i] = now;

    uint16_t delta = (target_pos_[i] > start_pos_[i]) ? (target_pos_[i] - start_pos_[i]) : (start_pos_[i] - target_pos_[i]);
    uint32_t use_deg_s = speed_deg_per_s;
    if (use_deg_s == 0) use_deg_s = Hand::getAxisConfig(side_, finger).maxSpeed;
    if (use_deg_s == 0) use_deg_s = 1000;
    uint32_t max_speed_units_per_sec = (use_deg_s * 4095) / 360;
    if (delta == 0) { duration_ms_[i] = 0; moving_[i] = false; }
    else { duration_ms_[i] = (static_cast<uint32_t>(delta) * 1000) / max_speed_units_per_sec; moving_[i] = true; }
}

void HandController::stopImmediate()
{
    // Stop all movements immediately
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        moving_[i] = false;
        target_pos_[i] = current_pos_[i];
    }
    // Command servos to hold current positions
    std::array<uint8_t, FINGER_COUNT> ids{};
    std::array<uint16_t, FINGER_COUNT> positions{};
    std::array<uint16_t, FINGER_COUNT> times{};
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        ids[i] = Hand::getServoID(side_, static_cast<Finger>(i));
        positions[i] = Hand::mapToServoPos(side_, static_cast<Finger>(i), current_pos_[i]);
        times[i] = MICROSTEP_TIME;
    }
    bus_.syncWritePositions(ids.data(), positions.data(), times.data(), FINGER_COUNT);
}

void HandController::holdCurrent()
{
    // Similar to stopImmediate but keep motors in hold (no state change beyond stopping)
    stopImmediate();
}

/**
 * @brief Schedule a target grip for this hand.
 *
 * Sets up start/target positions and timing for all fingers. Each finger
 * moves at its configured maxSpeed from AxisSettings. Movement is applied
 * non-blocking via the `update()` method with smoothstep interpolation.
 * 
 * @param grip Target `GripType`
 */

void HandController::update()
{
    uint32_t now = HAL_GetTick();
    
    // ========================================================================
    // PART 1: TRAJECTORY INTERPOLATION + MOVEMENT
    // ========================================================================
    
    bool any_moving = false;
    std::array<uint8_t, FINGER_COUNT> servo_ids{};
    std::array<uint16_t, FINGER_COUNT> positions{};
    std::array<uint16_t, FINGER_COUNT> times{};
    
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        // Get servo ID for this finger
        servo_ids[i] = Hand::getServoID(side_, static_cast<Finger>(i));
        
        // Smoothstep interpolation for smooth S-curve movement
        if (moving_[i]) {
            uint32_t elapsed = now - start_time_ms_[i];
            
            // Check if movement is complete
            if (duration_ms_[i] == 0 || elapsed >= duration_ms_[i]) {
                current_pos_[i] = target_pos_[i];
                moving_[i] = false;
            } else {
                // Use smoothstep interpolation for S-curve
                current_pos_[i] = interpolatePosition(i, now);
                any_moving = true;
            }
        }
        
        // Map logical position (0=open, 4095=closed) to physical servo position
        // This accounts for different zero positions and rotation directions
        positions[i] = Hand::mapToServoPos(side_, static_cast<Finger>(i), current_pos_[i]);
        // Use microstep time for smooth servo execution
        times[i] = MICROSTEP_TIME;
    }
    
    // Send positions to servos (fire and forget, non-blocking)
    if (any_moving || (now % 500) == 0) {  // Send updates while moving, or every 500ms to maintain position
        bus_.syncWritePositions(servo_ids.data(), positions.data(), times.data(), FINGER_COUNT);
    }

    // Blue LED heartbeat while any servo is moving: toggle every 100ms.
    if (any_moving) {
        if ((now - led_last_toggle_ms) >= LED_TOGGLE_MS) {
            BSP_LED_Toggle(LED_BLUE);
            led_last_toggle_ms = now;
        }
    } else {
        // Ensure LED is off when idle
        BSP_LED_Off(LED_BLUE);
    }
    
    // Poll the bus state machine (non-blocking). Telemetry/admittance logic removed.
    bus_.poll();
}

/**
 * @brief Periodic non-blocking update.
 *
 * Performs trajectory interpolation, sends sync write packets to servos and
 * advances the round-robin telemetry state machine. Intended to be called
 * from the main loop at ~100Hz.
 */

// ============================================================================
// PRIVATE HELPERS
// ============================================================================

float HandController::smoothstep(float t)
{
    // Clamp to [0, 1]
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // Cubic smoothstep: 3t² - 2t³ -> S-Curve easing
    return t * t * (3.0f - 2.0f * t);
}

/**
 * @brief Cubic smoothstep easing function.
 * @param t Normalized time in [0,1]
 * @return float Interpolation factor
 */

uint16_t HandController::interpolatePosition(size_t finger_idx, uint32_t now)
{
    if (finger_idx >= FINGER_COUNT) return 0;
    
    // Calculate normalized time [0.0, 1.0]
    uint32_t elapsed = now - start_time_ms_[finger_idx];
    float t = static_cast<float>(elapsed) / static_cast<float>(duration_ms_[finger_idx]);
    
    // Apply smoothstep interpolation
    float alpha = smoothstep(t);
    
    // Linear interpolation with smoothstep easing
    float start = static_cast<float>(start_pos_[finger_idx]);
    float target = static_cast<float>(target_pos_[finger_idx]);
    float pos = start + alpha * (target - start);
    
    // Clamp to valid range [0, 4095]
    if (pos < 0.0f) pos = 0.0f;
    if (pos > 4095.0f) pos = 4095.0f;
    
    return static_cast<uint16_t>(pos);
}

/**
 * @brief Compute interpolated servo position for a finger.
 * @param finger_idx Finger index (0..FINGER_COUNT-1)
 * @param now Current time (HAL_GetTick())
 * @return uint16_t Servo position in native units (0..4095)
 */

// telemetry/admittance helpers removed

/**
 * @brief Placeholder for future AI-based grasp adjustment.
 *
 * Analyzes measured current for contact/slip detection and may modify
 * trajectories in future iterations.
 * @param finger_idx Finger index
 * @param current Measured current in mA
 */

} // namespace HandControl
