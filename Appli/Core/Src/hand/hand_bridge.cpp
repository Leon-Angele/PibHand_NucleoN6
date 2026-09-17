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

static Stm32UartDmaPort servoPort(&huart3);
static Stm32UartDmaPort vcpPort(&hlpuart1);
static ServoBus servoBus(servoPort);
static SerialCommander commander(vcpPort);
static HandController hand(servoBus);

static bool torque_update_pending = false;
static uint8_t torque_update_index = 0;
static uint8_t status_stream_hz = 0;
static uint32_t status_last_ms = 0;
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
    hand.getStatus(&status);
    FSR_GetSnapshot(&fsr);

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

class DefaultGripExecutor final : public ICommandExecutor {
public:
    bool executeCommand(const Command& command) override
    {
        switch (command.type) {
            case CommandType::Pose:
                if (command.has_force) {
                    const bool accepted = setPoseWithForce(command);
                    if (accepted) signalPoseAccepted();
                    return accepted;
                }
                hand.setTargetGrip(command.grip);
                signalPoseAccepted();
                return true;
            case CommandType::SinglePosition:
                if (command.has_force) {
                    const bool accepted = hand.setSingleFingerPercentWithForce(
                        command.finger, command.position_percent, command.force_newton);
                    if (accepted) signalPoseAccepted();
                    return accepted;
                }
                {
                    const bool accepted = hand.setSingleFingerPercent(
                        command.finger, command.position_percent);
                    if (accepted) signalPoseAccepted();
                    return accepted;
                }
            case CommandType::ForceAll:
                return hand.setForceAll(command.force_newton);
            case CommandType::ForceFinger:
                return hand.setForce(command.finger, command.force_newton);
            case CommandType::AdmittanceOn:
                if (!FSR_IsTared()) return false;
                hand.setAdmittanceEnabled(true);
                return true;
            case CommandType::AdmittanceOff:
                hand.setAdmittanceEnabled(false);
                return true;
            case CommandType::FsrTare:
                if (hand.admittanceEnabled() || hand.isMoving()) return false;
                return FSR_Tare();
            case CommandType::Speed:
                hand.setSpeed(command.speed_deg_per_s);
                return true;
            case CommandType::Torque:
                hand.setTorqueLimit(command.torque_percent);
                torque_update_pending = true;
                torque_update_index = 0;
                return true;
            case CommandType::Stop:
            case CommandType::Hold:
                hand.stopImmediate();
                return true;
            case CommandType::GetStatus:
                appendStatus();
                return true;
            case CommandType::StatusStream:
                status_stream_hz = command.status_rate_hz;
                status_last_ms = HAL_GetTick();
                return true;
            default:
                return false;
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
    hand.setTorqueLimit(DEFAULT_TORQUE_LIMIT_PERCENT);
    torque_update_pending = true;
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
    if (servoBus.getState() == BusState::TIMEOUT) signalSerialError();

    if (!torque_update_pending && servoBus.getState() == BusState::IDLE && hand.outputPending()) {
        std::array<uint8_t, FINGER_COUNT> ids{};
        std::array<uint16_t, FINGER_COUNT> positions{};
        std::array<uint16_t, FINGER_COUNT> times{};
        if (hand.copyOutputFrame(ids.data(), positions.data(), times.data(), FINGER_COUNT) &&
            servoBus.syncWritePositions(ids.data(), positions.data(), times.data(), FINGER_COUNT)) {
            hand.markOutputSent();
        } else {
            signalSerialError();
        }
    }

    if (torque_update_pending && servoBus.getState() == BusState::IDLE) {
        if (torque_update_index >= FINGER_COUNT) {
            torque_update_pending = false;
        } else if (servoBus.writeTorqueLimit(
                       Hand::getServoID(static_cast<Finger>(torque_update_index)), hand.torqueLimit())) {
            ++torque_update_index;
        } else {
            signalSerialError();
        }
    }

    commander.processCommand();
    commander.serviceTx();

    const uint8_t rate = status_stream_hz;
    if (rate != 0U) {
        const uint32_t interval = 1000U / rate;
        const uint32_t now = HAL_GetTick();
        if ((now - status_last_ms) >= interval) {
            status_last_ms = now;
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
    Stm32UartDmaPort::onTxComplete(static_cast<UART_HandleTypeDef*>(huart));
}

void bridge_on_uart_rx(void* huart)
{
    Stm32UartDmaPort::onRxComplete(static_cast<UART_HandleTypeDef*>(huart));
}

} // extern "C"
