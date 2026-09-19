#include "hand/hand_bridge.h"

#include "hand/fsr400.hpp"
#include "hand/hand_config.hpp"
#include "hand/hand_controller.hpp"
#include "hand/serial_commander.hpp"

#include "main.h"

#include <array>
#include <cstdio>
#include <cstring>

using namespace HandControl;

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef hlpuart1;

static feetech::PortHandlerSTM32 servoPort(&huart3);
static feetech::STSPacketHandler servoPacketHandler(servoPort);
static Stm32UartDmaPort vcpPort(&hlpuart1);
static ServoBus servoBus(servoPacketHandler);
static SerialCommander commander(vcpPort);
static HandController hand(servoBus);

enum class ServoTransaction : uint8_t {
    None,
    PositionWrite,
    TorqueWrite,
    PositionRead,
    CurrentRead
};

static bool servo_port_ready = false;
static ServoTransaction servo_transaction = ServoTransaction::None;
static uint32_t position_sequence_in_flight = 0;
static uint8_t torque_index_in_flight = 0;
static uint32_t torque_generation = 0;
static uint32_t torque_generation_in_flight = 0;
static bool torque_update_pending = false;
static uint8_t torque_update_index = 0;
static uint8_t status_stream_hz = 0;
static uint32_t status_last_ms = 0;
static bool status_once_pending = false;
static uint32_t status_once_deadline_ms = 0;
static uint32_t status_once_target_sweep = 0;
static uint8_t feedback_step = 0;
static uint8_t feedback_axis_in_flight = 0;
static uint8_t feedback_position_success_mask = 0;
static uint8_t feedback_current_success_mask = 0;
static uint8_t last_position_success_mask = 0;
static uint8_t last_current_success_mask = 0;
static uint32_t feedback_sweep_generation = 0;
static uint32_t feedback_next_read_ms = 0;
static bool feedback_preferred = true;
static constexpr uint8_t ALL_SERVO_MASK = (1U << FINGER_COUNT) - 1U;
static constexpr uint8_t FEEDBACK_STEPS_PER_SWEEP = static_cast<uint8_t>(FINGER_COUNT * 2U);
static constexpr uint32_t FEEDBACK_READ_INTERVAL_MS = 2U;
static constexpr uint32_t STATUS_ONCE_TIMEOUT_MS = 350U;
static constexpr uint32_t FSR_STALE_MS = 100U;
static constexpr uint32_t LED_PULSE_MS = 100U;
static uint32_t blue_led_until_ms = 0;
static uint32_t red_led_until_ms = 0;

namespace {

void pulseLed(Led_TypeDef led, uint32_t& until_ms) noexcept
{
    const uint32_t now = HAL_GetTick();
    until_ms = now + LED_PULSE_MS;
    BSP_LED_On(led);
}

void updateLedPulses(uint32_t now_ms) noexcept
{
    if (blue_led_until_ms != 0U && static_cast<int32_t>(now_ms - blue_led_until_ms) >= 0) {
        BSP_LED_Off(LED_BLUE);
        blue_led_until_ms = 0U;
    }
    if (red_led_until_ms != 0U && static_cast<int32_t>(now_ms - red_led_until_ms) >= 0) {
        BSP_LED_Off(LED_RED);
        red_led_until_ms = 0U;
    }
}

void signalSerialError() noexcept
{
    pulseLed(LED_RED, red_led_until_ms);
}

void signalPoseAccepted() noexcept
{
    pulseLed(LED_BLUE, blue_led_until_ms);
}

const char* modeName(ControllerMode mode)
{
    switch (mode) {
        case ControllerMode::Boot: return "BOOT";
        case ControllerMode::Tare: return "TARE";
        case ControllerMode::Move: return "MOVE";
        case ControllerMode::Position: return "POS";
        case ControllerMode::Admittance: return "ADM";
        case ControllerMode::Hold: return "HOLD";
        case ControllerMode::Fault: return "FAULT";
        default: return "FAULT";
    }
}

void appendStatus(void)
{
    ControllerStatus status;
    FSR_Snapshot fsr{};
    const uint32_t now = HAL_GetTick();
    hand.getStatus(&status, now);
    FSR_GetSnapshot(&fsr);
    if (last_position_success_mask != ALL_SERVO_MASK ||
        last_current_success_mask != ALL_SERVO_MASK) {
        status.faultFlags |= FAULT_SERVO_COMMUNICATION;
    }
    if (!fsr.acquisition_ok || fsr.sequence == 0U ||
        (now - fsr.last_update_ms) > FSR_STALE_MS) {
        status.faultFlags |= FAULT_FSR_ACQUISITION;
    }
    if (!fsr.tared) status.faultFlags |= FAULT_FSR_NOT_TARED;
    if (fsr.saturated) status.faultFlags |= FAULT_FSR_SATURATED;

    char message[384]{};
    int used = std::snprintf(message, sizeof(message), "STAT:%lu:%s:%08lX:R:",
                             static_cast<unsigned long>(status.sequence), modeName(status.mode),
                             static_cast<unsigned long>(status.faultFlags));
    if (used < 0 || static_cast<size_t>(used) >= sizeof(message)) return;

    auto append = [&](const char* format, auto value) {
        if (used < 0 || static_cast<size_t>(used) >= sizeof(message)) return;
        used += std::snprintf(message + used, sizeof(message) - static_cast<size_t>(used), format, value);
    };
    auto appendFixed = [&](float value, int decimals, bool comma) {
        if (used < 0 || static_cast<size_t>(used) >= sizeof(message)) return;
        const long scale = decimals == 2 ? 100L : 10L;
        const long scaled = static_cast<long>(value * static_cast<float>(scale) + 0.5f);
        const long whole = scaled / scale;
        const long fraction = scaled % scale;
        used += std::snprintf(message + used, sizeof(message) - static_cast<size_t>(used),
                              comma ? ",%ld.%0*ld" : "%ld.%0*ld", whole, decimals, fraction);
    };
    for (size_t i = 0; i < FINGER_COUNT; ++i) appendFixed(status.referencePercent[i], 1, i != 0);
    append(":C:", 0);
    for (size_t i = 0; i < FINGER_COUNT; ++i) appendFixed(status.commandPercent[i], 1, i != 0);
    append(":P:", 0);
    for (size_t i = 0; i < FINGER_COUNT; ++i) appendFixed(status.actualPercent[i], 1, i != 0);
    append(":PT:", 0);
    for (size_t i = 0; i < FINGER_COUNT; ++i) append(i == 0 ? "%u" : ",%u", status.actualTicks[i]);
    append(":S:", 0);
    for (size_t i = CONTROLLED_FINGER_FIRST; i <= CONTROLLED_FINGER_LAST; ++i) appendFixed(status.forceSetpoint[i], 2, i != CONTROLLED_FINGER_FIRST);
    append(":F:", 0);
    for (size_t i = 0; i < FSR400_SENSOR_COUNT; ++i) appendFixed(fsr.force_newton[i], 2, i != 0);
    append(":A:", 0);
    for (size_t i = 0; i < FSR400_SENSOR_COUNT; ++i) append(i == 0 ? "%u" : ",%u", fsr.raw[i]);
    append(":I:", 0);
    for (size_t i = 0; i < FINGER_COUNT; ++i) append(i == 0 ? "%ld" : ",%ld", static_cast<long>(status.currentMilliamp[i]));
    if (used >= 0 && static_cast<size_t>(used) < sizeof(message)) {
        std::snprintf(message + used, sizeof(message) - static_cast<size_t>(used),
                      ":SP:%u:TQ:%u\n", status.speedDegPerSecond, status.torqueLimitPercent);
        commander.sendText(message);
    }
}

bool feedbackRequested() noexcept
{
    return status_stream_hz != 0U || status_once_pending;
}

void resetFeedbackSweep(uint32_t now_ms) noexcept
{
    feedback_step = 0U;
    feedback_position_success_mask = 0U;
    feedback_current_success_mask = 0U;
    feedback_next_read_ms = now_ms;
    feedback_preferred = true;
}

void completeStatusOnce() noexcept
{
    appendStatus();
    commander.completeDeferred(true);
    status_once_pending = false;
}

void completeFeedbackStep(bool success, uint32_t now_ms) noexcept
{
    const uint8_t mask = static_cast<uint8_t>(1U << feedback_axis_in_flight);
    if (servo_transaction == ServoTransaction::PositionRead) {
        if (success) feedback_position_success_mask |= mask;
    } else if (servo_transaction == ServoTransaction::CurrentRead) {
        if (success) feedback_current_success_mask |= mask;
    }

    ++feedback_step;
    feedback_next_read_ms = now_ms + FEEDBACK_READ_INTERVAL_MS;
    feedback_preferred = false;
    if (feedback_step < FEEDBACK_STEPS_PER_SWEEP) return;

    last_position_success_mask = feedback_position_success_mask;
    last_current_success_mask = feedback_current_success_mask;
    ++feedback_sweep_generation;
    feedback_step = 0U;
    feedback_position_success_mask = 0U;
    feedback_current_success_mask = 0U;
    if (status_once_pending && feedback_sweep_generation >= status_once_target_sweep) {
        completeStatusOnce();
    }
}

bool startFeedbackRead(uint32_t now_ms) noexcept
{
    feedback_axis_in_flight = static_cast<uint8_t>(feedback_step / 2U);
    const Finger finger = static_cast<Finger>(feedback_axis_in_flight);
    const uint8_t id = Hand::getServoID(finger);
    const bool position = (feedback_step % 2U) == 0U;
    const bool started = servo_port_ready &&
        (position ? servoBus.startReadPosition(id) : servoBus.startReadCurrent(id));
    servo_transaction = position ? ServoTransaction::PositionRead
                                 : ServoTransaction::CurrentRead;
    if (started) return true;

    signalSerialError();
    servoBus.resetState();
    completeFeedbackStep(false, now_ms);
    servo_transaction = ServoTransaction::None;
    return false;
}

class DefaultGripExecutor final : public ICommandExecutor {
public:
    Result executeCommand(const Command& command) override
    {
        const auto result = [](bool success) {
            return success ? Result::Success : Result::Failure;
        };
        switch (command.type) {
            case CommandType::Pose:
                if (command.has_force) {
                    const bool accepted = setPoseWithForce(command);
                    if (accepted) signalPoseAccepted();
                    return result(accepted);
                }
                hand.setTargetGrip(command.grip);
                signalPoseAccepted();
                return Result::Success;
            case CommandType::SinglePosition:
                if (command.has_force) {
                    const bool accepted = hand.setSingleFingerPercentWithForce(
                        command.finger, command.position_percent, command.force_newton);
                    if (accepted) signalPoseAccepted();
                    return result(accepted);
                }
                {
                    const bool accepted = hand.setSingleFingerPercent(
                        command.finger, command.position_percent);
                    if (accepted) signalPoseAccepted();
                    return result(accepted);
                }
            case CommandType::ForceAll:
                return result(hand.setForceAll(command.force_newton));
            case CommandType::ForceFinger:
                return result(hand.setForce(command.finger, command.force_newton));
            case CommandType::AdmittanceOn:
                if (!FSR_IsTared()) return Result::Failure;
                hand.setAdmittanceEnabled(true);
                return Result::Success;
            case CommandType::AdmittanceOff:
                hand.setAdmittanceEnabled(false);
                return Result::Success;
            case CommandType::FsrTare:
                if (hand.admittanceEnabled() || hand.isMoving()) return Result::Failure;
                return result(FSR_Tare());
            case CommandType::Speed:
                hand.setSpeed(command.speed_deg_per_s);
                return Result::Success;
            case CommandType::Torque:
                hand.setTorqueLimit(command.torque_percent);
                ++torque_generation;
                torque_update_pending = true;
                torque_update_index = 0;
                return Result::Success;
            case CommandType::Stop:
            case CommandType::Hold:
                hand.stopImmediate();
                return Result::Success;
            case CommandType::GetStatus:
                if (status_once_pending) return Result::Failure;
                status_once_pending = true;
                status_once_deadline_ms = HAL_GetTick() + STATUS_ONCE_TIMEOUT_MS;
                if (status_stream_hz == 0U) {
                    resetFeedbackSweep(HAL_GetTick());
                    status_once_target_sweep = feedback_sweep_generation + 1U;
                } else {
                    const bool sweep_not_started = feedback_step == 0U &&
                        servo_transaction != ServoTransaction::PositionRead &&
                        servo_transaction != ServoTransaction::CurrentRead;
                    status_once_target_sweep = feedback_sweep_generation +
                        (sweep_not_started ? 1U : 2U);
                }
                return Result::Deferred;
            case CommandType::StatusStream:
                if (status_stream_hz == 0U && command.status_rate_hz != 0U) {
                    resetFeedbackSweep(HAL_GetTick());
                }
                status_stream_hz = command.status_rate_hz;
                status_last_ms = HAL_GetTick();
                return Result::Success;
            default:
                return Result::Failure;
        }
    }

private:
    bool setPoseWithForce(const Command& command)
    {
        return hand.setForceAll(command.force_newton) &&
               (hand.setTargetGrip(command.grip), true);
    }
};

static DefaultGripExecutor defaultExecutor;

} // namespace

extern "C" {

void hand_bridge_init(void)
{
    commander.setExecutor(&defaultExecutor);
    commander.setErrorCallback(&signalSerialError);
    BSP_LED_Off(LED_BLUE);
    BSP_LED_Off(LED_RED);
    servo_port_ready = servoPort.openPort();
    if (!servo_port_ready) signalSerialError();
    hand.setTorqueLimit(DEFAULT_TORQUE_LIMIT_PERCENT);
    torque_update_pending = servo_port_ready;
    torque_update_index = 0;
}

bool hand_bridge_set_target_grip(uint8_t grip)
{
    if (grip >= static_cast<uint8_t>(GripType::Count)) return false;
    hand.setTargetGrip(static_cast<GripType>(grip));
    return true;
}

void hand_bridge_update(void)
{
    FSR_Snapshot snapshot{};
    FSR_GetSnapshot(&snapshot);
    hand.update(snapshot);
}

void hand_bridge_service(void)
{
    servoBus.poll();
    const uint32_t now = HAL_GetTick();
    updateLedPulses(now);

    const BusState bus_state = servoBus.getState();
    if (bus_state == BusState::DATA_READY) {
        bool success = false;
        if (servo_transaction == ServoTransaction::PositionRead) {
            const auto position = servoBus.getPositionResult();
            success = position.has_value();
            if (success) {
                hand.setActualPosition(static_cast<Finger>(feedback_axis_in_flight),
                                       *position, now);
            }
            completeFeedbackStep(success, now);
        } else if (servo_transaction == ServoTransaction::CurrentRead) {
            const auto current = servoBus.getReadResult();
            success = current.has_value();
            if (success) {
                hand.setActualCurrent(static_cast<Finger>(feedback_axis_in_flight),
                                      *current, now);
            }
            completeFeedbackStep(success, now);
        } else {
            servoBus.resetState();
        }
        if (!success) signalSerialError();
        servo_transaction = ServoTransaction::None;
    } else if (bus_state == BusState::TIMEOUT) {
        signalSerialError();
        if (servo_transaction == ServoTransaction::PositionRead ||
            servo_transaction == ServoTransaction::CurrentRead) {
            completeFeedbackStep(false, now);
        }
        servoBus.resetState();
        servo_transaction = ServoTransaction::None;
    } else if (bus_state == BusState::IDLE) {
        if (servo_transaction == ServoTransaction::PositionWrite) {
            hand.markOutputSent(position_sequence_in_flight);
            feedback_preferred = true;
            servo_transaction = ServoTransaction::None;
        } else if (servo_transaction == ServoTransaction::TorqueWrite) {
            if (torque_generation == torque_generation_in_flight &&
                torque_update_index == torque_index_in_flight) {
                ++torque_update_index;
            }
            servo_transaction = ServoTransaction::None;
        }
    }

    if (status_once_pending &&
        static_cast<int32_t>(now - status_once_deadline_ms) >= 0) {
        last_position_success_mask = feedback_position_success_mask;
        last_current_success_mask = feedback_current_success_mask;
        completeStatusOnce();
    }

    if (servo_transaction == ServoTransaction::None && torque_update_pending &&
        servoBus.getState() == BusState::IDLE) {
        if (torque_update_index >= FINGER_COUNT) {
            torque_update_pending = false;
        } else if (servoBus.writeTorqueLimit(
                       Hand::getServoID(static_cast<Finger>(torque_update_index)), hand.torqueLimit())) {
            torque_index_in_flight = torque_update_index;
            torque_generation_in_flight = torque_generation;
            servo_transaction = ServoTransaction::TorqueWrite;
        } else {
            signalSerialError();
        }
    }

    const bool feedback_due = feedbackRequested() &&
        static_cast<int32_t>(now - feedback_next_read_ms) >= 0;
    if (servo_transaction == ServoTransaction::None && !torque_update_pending &&
        servoBus.getState() == BusState::IDLE && feedback_due &&
        (feedback_preferred || !hand.outputPending())) {
        (void)startFeedbackRead(now);
    }

    if (servo_transaction == ServoTransaction::None && !torque_update_pending &&
        servoBus.getState() == BusState::IDLE && hand.outputPending()) {
        std::array<uint8_t, FINGER_COUNT> ids{};
        std::array<uint16_t, FINGER_COUNT> positions{};
        std::array<uint16_t, FINGER_COUNT> times{};
        const uint32_t output_sequence = hand.outputSequence();
        if (hand.copyOutputFrame(ids.data(), positions.data(), times.data(), FINGER_COUNT) &&
            servoBus.syncWritePositions(ids.data(), positions.data(), times.data(), FINGER_COUNT)) {
            position_sequence_in_flight = output_sequence;
            servo_transaction = ServoTransaction::PositionWrite;
        } else {
            signalSerialError();
        }
    }

    if (servo_transaction == ServoTransaction::None && !torque_update_pending &&
        servoBus.getState() == BusState::IDLE && feedbackRequested() &&
        static_cast<int32_t>(now - feedback_next_read_ms) >= 0) {
        (void)startFeedbackRead(now);
    }

    commander.processCommand();
    commander.serviceTx();

    const uint8_t rate = status_stream_hz;
    if (rate != 0U) {
        const uint32_t interval = 1000U / rate;
        const uint32_t now = HAL_GetTick();
        const uint32_t elapsed = now - status_last_ms;
        if (elapsed >= interval) {
            status_last_ms += (elapsed / interval) * interval;
            appendStatus();
        }
    }
}

void commander_bridge_process(void)
{
    commander.processCommand();
}

bool commander_bridge_feed_byte(uint8_t byte)
{
    return commander.feedByte(byte);
}

void hand_bridge_ping_all_servos(void)
{
    std::printf("\r\n[BRIDGE] Pinging configured servos...\r\n");
    if (!servo_port_ready) {
        std::printf("[BRIDGE] Servo port initialization failed.\r\n\r\n");
        return;
    }
    for (size_t i = 0; i < FINGER_COUNT; ++i) {
        const Finger finger = static_cast<Finger>(i);
        const bool online = servoBus.pingServo(Hand::getServoID(finger), 100);
        std::printf("  [%u] %s (ID %u): %s\r\n", static_cast<unsigned>(i),
                    Hand::getAxisConfig(finger).name.data(), Hand::getServoID(finger),
                    online ? "OK" : "TIMEOUT");
        HAL_Delay(20);
    }
    std::printf("[BRIDGE] Ping complete.\r\n\r\n");
}

void bridge_on_uart_tx(void* huart)
{
    auto* uart = static_cast<UART_HandleTypeDef*>(huart);
    Stm32UartDmaPort::onTxComplete(uart);
    feetech::PortHandlerSTM32::handleTxComplete(uart);
}

void bridge_on_uart_rx(void* huart)
{
    auto* uart = static_cast<UART_HandleTypeDef*>(huart);
    Stm32UartDmaPort::onRxComplete(uart);
    feetech::PortHandlerSTM32::handleRxComplete(uart);
}

void bridge_on_uart_error(void* huart)
{
    auto* uart = static_cast<UART_HandleTypeDef*>(huart);
    Stm32UartDmaPort::onError(uart);
    feetech::PortHandlerSTM32::handleUartError(uart);
    if (uart == &hlpuart1) signalSerialError();
}

} // extern "C"
