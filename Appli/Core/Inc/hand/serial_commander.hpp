/**
 * @file serial_commander.hpp
 * @brief ISR-safe ASCII command parser for hand control via VCP.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Protocol: "G:<Side>:<GripID>\n" where <Side> is 0 (Left) or 1 (Right)
 */

#ifndef SERIAL_COMMANDER_HPP
#define SERIAL_COMMANDER_HPP

#include "hand/hand_config.hpp"
#include "hand/servo.hpp"
#include "main.h"

#include <cstdint>
#include <cstddef>
#include <array>

// ============================================================================
// VCP UART PORT WRAPPER
// ============================================================================

/**
 * @brief Polling UART port (blocking TX, no DMA) - for VCP/debug only
 */
class PollUartPort : public HandControl::ISerialPort {
public:
    explicit PollUartPort(UART_HandleTypeDef* huart, uint32_t timeout_ms = 1000);
    ~PollUartPort() override = default;

    bool transmitDMA(const uint8_t* data, uint16_t length) override;
    bool receiveDMA(uint8_t*, uint16_t) override { return false; }
    bool isTxDone() const override { return true; }
    bool isRxDone() const override { return false; }
    void process() override {}
    void abortRx() override {}

private:
    UART_HandleTypeDef* huart_;
    uint32_t timeout_ms_;
};

// ============================================================================
// COMMAND EXECUTOR INTERFACE
// ============================================================================

class ICommandExecutor {
public:
    virtual ~ICommandExecutor() = default;

    // High-level command types parsed from ASCII input
    enum class CommandType : uint8_t {
        GripDefault,    // G:<Side>:<GripID>
        GripGlobalPct,  // G:<Side>:<GripID>:V:<percent>
        GripPerFingerPct, // G:<Side>:<GripID>:Vx:<v0>,...,<v5>
        SingleFinger,   // F:<Side>:<Finger>:<Pos>[:<Speed>]
        Stop,           // STOP:<Side>
        Hold,           // HOLD:<Side>
        GetStatus,      // GET:STATUS
        Unknown
    };

    struct Command {
        CommandType type = CommandType::Unknown;
        HandControl::Hand::Side side = HandControl::Hand::Side::Left;
        HandControl::GripType grip = HandControl::GripType::Open;

        // For global percent
        uint16_t percent = 100; // 0..100

        // For per-finger percent (0..100)
        std::array<uint16_t, static_cast<size_t>(HandControl::Finger::Count)> perFingerPercent{{0,0,0,0,0,0}};

        // For single finger command
        HandControl::Finger finger = HandControl::Finger::Thumb;
        uint16_t position = 0; // 0..4095
        uint16_t speed_deg_per_s = 0; // 0 means use axis maxSpeed
    };

    // Execute an abstracted command parsed from ASCII input. Return true on success.
    virtual bool executeCommand(const Command& cmd) = 0;
};

class SerialCommander {
public:
    explicit SerialCommander(HandControl::ISerialPort& port) noexcept;
    void setExecutor(ICommandExecutor* exec) noexcept { executor_ = exec; }

    // ISR-safe feedByte API; processCommand must be called from non-ISR context.
    // Returns true if byte was accepted, false on buffer full.
    bool feedByte(uint8_t b) noexcept;

    // Called from main loop (non-ISR) to parse and execute complete commands.
    void processCommand() noexcept;

private:
    HandControl::ISerialPort& port_;
    ICommandExecutor* executor_ = nullptr;

    static constexpr size_t RX_BUF_SIZE = 32;
    alignas(1) uint8_t rx_buf_[RX_BUF_SIZE];
    volatile uint16_t rx_head_ = 0;
    volatile uint16_t rx_tail_ = 0;
    volatile bool overflow_flag_ = false;

    // Helpers (non-ISR)
    void sendResponse(const char* msg, size_t len) noexcept;
    static bool parseCommand(const uint8_t* data, size_t len,
                             ICommandExecutor::Command& outCmd) noexcept;
};

#endif // SERIAL_COMMANDER_HPP
