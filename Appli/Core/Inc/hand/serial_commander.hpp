// serial_commander.hpp
// ISR-safe ASCII command parser for ROS2 over USB
// Protocol: "G:<Side>:<GripID>\n" where <Side> is 0 (Left) or 1 (Right)

#ifndef SERIAL_COMMANDER_HPP
#define SERIAL_COMMANDER_HPP

#include "hand/hand_config.hpp"
#include "hand/servo.hpp"
#include "main.h"

#include <cstdint>
#include <cstddef>
#include <array>

class ICommandExecutor {
public:
    virtual ~ICommandExecutor() = default;
    // Execute a pre-defined Grip on given side. Return true on success.
    virtual bool executeGrip(HandControl::Hand::Side side, HandControl::GripType grip) = 0;
};

class SerialCommander {
public:
    explicit SerialCommander(ISerialPort& port) noexcept;
    void setExecutor(ICommandExecutor* exec) noexcept { executor_ = exec; }

    // ISR-safe feedByte API; processCommand must be called from non-ISR context.
    // Returns true if byte was accepted, false on buffer full.
    bool feedByte(uint8_t b) noexcept;

    // Called from main loop (non-ISR) to parse and execute complete commands.
    void processCommand() noexcept;

private:
    ISerialPort& port_;
    ICommandExecutor* executor_ = nullptr;

    static constexpr size_t RX_BUF_SIZE = 32;
    alignas(1) uint8_t rx_buf_[RX_BUF_SIZE];
    volatile uint16_t rx_head_ = 0;
    volatile uint16_t rx_tail_ = 0;
    volatile bool overflow_flag_ = false;

    // Helpers (non-ISR)
    void sendResponse(const char* msg, size_t len) noexcept;
    static bool parseGripCommand(const uint8_t* data, size_t len,
                                 HandControl::Hand::Side& outSide,
                                 uint8_t& outGripId) noexcept;
};

#endif // SERIAL_COMMANDER_HPP
