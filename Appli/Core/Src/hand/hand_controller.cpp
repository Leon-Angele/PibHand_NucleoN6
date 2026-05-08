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

namespace HandControl {

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
    
    poll_finger_idx_ = 0;
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

void HandController::setTargetGrip(GripType grip, uint16_t duration_ms)
{
    // Lookup grip configuration from database
    if (static_cast<size_t>(grip) >= static_cast<size_t>(GripType::Count)) {
        HAND_DEBUG("Invalid grip type: %d", static_cast<int>(grip));
        return;
    }
    
    const GripConfig& grip_cfg = GripDatabase[static_cast<size_t>(grip)];
    
    uint32_t now = HAL_GetTick();
    
    // Setup trajectory for all 6 fingers
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        start_pos_[i] = current_pos_[i];
        target_pos_[i] = grip_cfg.positions[i];
        start_time_ms_[i] = now;
        duration_ms_[i] = duration_ms;
        
        // Only mark as moving if there's actual movement
        if (start_pos_[i] != target_pos_[i]) {
            moving_[i] = true;
        }
    }
    
    HAND_DEBUG("Grip set: %s (side=%d, duration=%dms)", 
               grip_cfg.name.data(), static_cast<int>(side_), duration_ms);
}

/**
 * @brief Schedule a target grip for this hand.
 *
 * Sets up start/target positions and timing for all fingers. Movement is
 * applied non-blocking via the `update()` method.
 * @param grip Target `GripType`
 * @param duration_ms Interpolation duration in milliseconds
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
        
        // Interpolate position
        if (moving_[i]) {
            current_pos_[i] = interpolatePosition(i, now);
            
            // Check if target reached
            uint32_t elapsed = now - start_time_ms_[i];
            if (elapsed >= duration_ms_[i]) {
                current_pos_[i] = target_pos_[i];
                moving_[i] = false;
            } else {
                any_moving = true;
            }
        }
        
        positions[i] = current_pos_[i];
        times[i] = 10;  // Move time for next update cycle (10ms = 100Hz update rate)
    }
    
    // Send positions to servos (fire and forget, non-blocking)
    if (any_moving || (now % 500) == 0) {  // Send updates while moving, or every 500ms to maintain position
        bus_.syncWritePositions(servo_ids.data(), positions.data(), times.data(), FINGER_COUNT);
    }
    
    // ========================================================================
    // PART 2: ROUND-ROBIN TELEMETRY POLLING
    // ========================================================================
    
    // Poll the bus state machine (non-blocking)
    bus_.poll();
    
    BusState bus_state = bus_.getState();
    
    switch (bus_state) {
        case BusState::IDLE:
            // Bus is free - start reading current for next finger
            {
                uint8_t servo_id = Hand::getServoID(side_, static_cast<Finger>(poll_finger_idx_));
                if (bus_.startReadCurrent(servo_id)) {
                    // Read started successfully (state is now WAIT_RX)
                } else {
                    // Failed to start read (bus might be busy) - try again next cycle
                    HAND_DEBUG("Failed to start read for finger %d", poll_finger_idx_);
                }
            }
            break;
            
        case BusState::DATA_READY:
            // Response received - extract current value
            {
                auto current_opt = bus_.getReadResult();
                if (current_opt.has_value()) {
                    int16_t current = current_opt.value();
                    
                    // Feed to AI placeholder (future closed-loop control)
                    predictGraspAdjustment(poll_finger_idx_, current);
                    
                  
                    #if DEBUG_PRINTS
                    /*
                    static int16_t currents[6] = {0};
                    currents[poll_finger_idx_] = current;
                    static uint32_t last_log_ms = 0;
                    if ((now - last_log_ms) > 1000) {  
                        HAND_DEBUG("Ströme: [0]:%d [1]:%d [2]:%d [3]:%d [4]:%d [5]:%d mA", 
                                currents[0], currents[1], currents[2], 
                                currents[3], currents[4], currents[5]);
                        last_log_ms = now;
                    }
                        */
                    #endif
                }
                
                // Reset bus state and advance to next finger
                bus_.resetState();
                poll_finger_idx_ = (poll_finger_idx_ + 1) % FINGER_COUNT;
            }
            break;
            
        case BusState::TIMEOUT:
            // Timeout occurred - log error (rate-limited) and advance to next finger
            {
                // Global rate-limited logging: only log every 10 seconds total
                uint32_t now = HAL_GetTick();
                static uint32_t last_timeout_log_ms = 0;
                constexpr uint32_t TIMEOUT_LOG_INTERVAL_MS = 10000; // 10s
                if ((now - last_timeout_log_ms) > TIMEOUT_LOG_INTERVAL_MS) {
                    uint8_t servo_id = Hand::getServoID(side_, static_cast<Finger>(poll_finger_idx_));
                    HAND_DEBUG("Read timeout for servo %d (finger %d)", servo_id, poll_finger_idx_);
                    last_timeout_log_ms = now;
                }
                
                // Reset bus state and advance to next finger
                bus_.resetState();
                poll_finger_idx_ = (poll_finger_idx_ + 1) % FINGER_COUNT;
            }
            break;
            
        case BusState::TX_BUSY:
        case BusState::WAIT_RX:
            // Bus is busy - wait for next update cycle
            break;
    }
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

void HandController::predictGraspAdjustment(uint8_t finger_idx, int16_t current)
{
    // Placeholder for future AI-based closed-loop control (X-CUBE-AI)
    // 
    // This function will analyze servo current to detect:
    // - Object contact (current spike)
    // - Slip detection (oscillating current)
    // - Grip force optimization
    // 
    // Based on sensor fusion (current + position + speed), the AI model
    // will output trajectory adjustments to improve grasp stability.
    
    (void)finger_idx;  // Suppress unused warning
    (void)current;
    
    // Example future implementation:
    // if (current > AxisSettings[finger_idx].maxCurrent * 0.8f) {
    //     HAND_DEBUG("High current on finger %d - possible contact", finger_idx);
    //     // Reduce target position to prevent damage
    //     // target_pos_[finger_idx] = current_pos_[finger_idx];
    // }
}

/**
 * @brief Placeholder for future AI-based grasp adjustment.
 *
 * Analyzes measured current for contact/slip detection and may modify
 * trajectories in future iterations.
 * @param finger_idx Finger index
 * @param current Measured current in mA
 */

} // namespace HandControl
