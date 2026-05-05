#include "hand/hand_bridge.h"

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

// Default C++ objects (mirror what main.cpp had)
static Stm32UartDmaPort servoPort(&huart3);   // USART3 for servo bus
static Stm32UartDmaPort vpcPort(&hlpuart1);   // LPUART1 for VCP/commands
static ServoBus servoBus(servoPort);
static SerialCommander commander(vpcPort);    // Commander uses VCP port!
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

void hand_bridge_init(void) {
    // Set default executor
    commander.setExecutor(&defaultExecutor);
}

void hand_bridge_set_executor(hand_grip_executor_t cb) {
    c_executor_cb = cb;
    if (cb) {
        commander.setExecutor(&cExecutorAdapter);
    } else {
        commander.setExecutor(&defaultExecutor);
    }
}

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

void hand_bridge_update(void) {
    rightHand.update();
    leftHand.update();
}

bool commander_bridge_feed_byte(uint8_t b) {
    return commander.feedByte(b);
}

void commander_bridge_process(void) {
    commander.processCommand();
}

void bridge_on_uart_rx(void* huart) {
    UART_HandleTypeDef* h = static_cast<UART_HandleTypeDef*>(huart);
    if (h->Instance == USART3) {
        Stm32UartDmaPort::onRxComplete(h);
    }
    else if (h->Instance == LPUART1) {
        Stm32UartDmaPort::onRxComplete(h);
    }
}

void bridge_on_uart_tx(void* huart) {
    UART_HandleTypeDef* h = static_cast<UART_HandleTypeDef*>(huart);
    if (h->Instance == USART3) {
        Stm32UartDmaPort::onTxComplete(h);
    }
    else if (h->Instance == LPUART1) {
        Stm32UartDmaPort::onTxComplete(h);
    }
}

} // extern "C"

// C-wrapper to set a raw servo position using degrees (-90..+90)
extern "C" bool hand_bridge_set_servo_deg(uint8_t id, int16_t degrees, uint16_t time_ms) {
    if (degrees < -90) degrees = -90;
    if (degrees > 90) degrees = 90;
    // Map -90..+90 -> 0..4095
    uint32_t pos = static_cast<uint32_t>(static_cast<int32_t>(degrees) + 90); // 0..180
    uint16_t servo_pos = static_cast<uint16_t>((pos * 4095u) / 180u);
    Servo s(id, servoBus);
    s.setTorqueEnable(true);
    return s.setPosition(servo_pos, time_ms);
}
