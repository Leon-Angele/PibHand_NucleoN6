#include "hand/hand_bridge.h"

/**
 * @file hand_bridge.cpp
 * @brief C API bridge between firmware `main` and the C++ HandControl subsystem.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Provides C-callable wrappers to initialize, update and command the
 * C++ `HandController` instances from the rest of the C application.
 */

#include "hand/hand_config.hpp"
#include "hand/servo.hpp"
#include "hand/hand_controller.hpp"
#include "hand/serial_commander.hpp"
#include "main.h"

#include <cstdint>

using namespace HandControl;

// Forward declare HAL handles (defined in main.c)
extern UART_HandleTypeDef huart3;    // Servo bus
extern UART_HandleTypeDef hlpuart1;  // VCP

// C executor callback pointer
static hand_grip_executor_t c_executor_cb = nullptr;

// ASYNC DMA-based ports for both servo bus and VCP
static Stm32UartDmaPort servoPort(&huart3);
static Stm32UartDmaPort vcpPort(&hlpuart1);
static ServoBus servoBus(servoPort);
static SerialCommander commander(vcpPort);
static HandController rightHand(Hand::Side::Right, servoBus);
static HandController leftHand(Hand::Side::Left, servoBus);

// Default executor: calls C++ controllers directly
class DefaultGripExecutor : public ICommandExecutor {
public:
    bool executeGrip(Hand::Side side, GripType grip) override {
        if (side == Hand::Side::Right) {
            rightHand.setTargetGrip(grip, 1000);
        } else {
            leftHand.setTargetGrip(grip, 1000);
        }
        return true;
    }
};
static DefaultGripExecutor defaultExecutor;

// Adapter that forwards to C callback
class CExecutorAdapter : public ICommandExecutor {
public:
    bool executeGrip(Hand::Side side, GripType grip) override {
        if (!c_executor_cb) return false;
        return c_executor_cb(static_cast<uint8_t>(side), static_cast<uint8_t>(grip));
    }
};
static CExecutorAdapter cExecutorAdapter;

extern "C" {

/**
 * @brief Initialize the hand bridge subsystem.
 *
 * Must be called after HAL/peripheral initialization. This sets the
 * `SerialCommander` executor to the default C++ executor which forwards
 * commands to the local `HandController` instances.
 */
void hand_bridge_init(void) {

    commander.setExecutor(&defaultExecutor);
    printf("[BRIDGE] Hand controller initialized\r\n");
}

/**
 * @brief Register a C callback executor for grip commands.
 *
 * If `cb` is non-NULL, incoming serial grip commands are forwarded to the
 * provided C callback via the `CExecutorAdapter`. Passing NULL restores the
 * default C++ executor implementation.
 *
 * @param cb Function pointer of type `hand_grip_executor_t` (or NULL).
 */
void hand_bridge_set_executor(hand_grip_executor_t cb) {
    c_executor_cb = cb;
    if (cb) {
        commander.setExecutor(&cExecutorAdapter);
    } else {
        commander.setExecutor(&defaultExecutor);
    }
}

/**
 * @brief Set a target grip on the specified hand.
 *
 * This is a C-callable helper that maps numeric `side` and `grip` values to
 * the internal C++ enums and schedules a smooth trajectory with the given
 * duration.
 *
 * @param side 0 = Left, 1 = Right
 * @param grip Grip identifier as `uint8_t` (maps to `GripType`)
 * @param duration_ms Duration of the interpolated movement in milliseconds
 * @return true if the request was accepted
 */
bool hand_bridge_set_target_grip(uint8_t side, uint8_t grip, uint16_t duration_ms) {
    Hand::Side s = (side == 1) ? Hand::Side::Right : Hand::Side::Left;
    HandControl::GripType g = static_cast<HandControl::GripType>(grip);
    if (s == Hand::Side::Right) {
        rightHand.setTargetGrip(g, duration_ms);
    } else {
        leftHand.setTargetGrip(g, duration_ms);
    }
    return true;
}

/**
 * @brief Periodic update called from the main loop.
 *
 * Calls the per-hand `update()` method which performs non-blocking
 * interpolation and telemetry polling. Should be executed at ~100Hz.
 */
void hand_bridge_update(void) {
    rightHand.update();
    // leftHand.update();
}

/**
 * @brief Feed a received UART byte into the commander (ISR-safe).
 *
 * Typically called from the HAL UART RX IRQ to push incoming bytes into the
 * ring buffer. Returns false if the internal buffer is full.
 *
 * @param b Received byte
 * @return true if byte was accepted, false on overflow
 */
bool commander_bridge_feed_byte(uint8_t b) {
    return commander.feedByte(b);
}

/**
 * @brief Process pending ASCII commands (call from non-ISR/main loop).
 *
 * Parses complete lines from the internal buffer and executes them via the
 * registered executor.
 */
void commander_bridge_process(void) {
    commander.processCommand();
}

/**
 * @brief HAL TX complete callback bridge.
 *
 * Forward the HAL UART TX complete event to the `Stm32UartDmaPort` router.
 * Should be called from `HAL_UART_TxCpltCallback` with the `UART_HandleTypeDef*`.
 *
 * @param huart Pointer to the UART handle provided by HAL
 */
void bridge_on_uart_tx(void* huart) {
    Stm32UartDmaPort::onTxComplete(static_cast<UART_HandleTypeDef*>(huart));
}

/**
 * @brief HAL RX complete callback bridge.
 *
 * Forward the HAL UART RX complete event to the `Stm32UartDmaPort` router.
 * Should be called from `HAL_UART_RxCpltCallback` with the `UART_HandleTypeDef*`.
 *
 * @param huart Pointer to the UART handle provided by HAL
 */
void bridge_on_uart_rx(void* huart) {
    Stm32UartDmaPort::onRxComplete(static_cast<UART_HandleTypeDef*>(huart));
}

} // extern "C"

// NOTE: hand_bridge_set_servo_deg is disabled in the new async implementation
// because it uses blocking calls that violate the non-blocking architecture.
// Use HandController::setTargetGrip() instead for coordinated hand movements.
/*
extern "C" bool hand_bridge_set_servo_deg(uint8_t id, int16_t degrees, uint16_t time_ms) {
    if (degrees < -90) degrees = -90;
    if (degrees > 90) degrees = 90;
    uint32_t pos = static_cast<uint32_t>(static_cast<int32_t>(degrees) + 90);
    uint16_t servo_pos = static_cast<uint16_t>((pos * 4095u) / 180u);
    // FIXME: Servo class no longer exists in async implementation
    // Would need to use ServoBus::writeRegister() with proper async handling
    return false;
}
*/
