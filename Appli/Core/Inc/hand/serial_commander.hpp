#ifndef SERIAL_COMMANDER_HPP
#define SERIAL_COMMANDER_HPP

#include "hand/hand_config.hpp"
#include "hand/servo.hpp"
#include "main.h"

#include <array>
#include <cstddef>
#include <cstdint>

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

class ICommandExecutor {
public:
    virtual ~ICommandExecutor() = default;

    enum class CommandType : uint8_t {
        Pose,
        SinglePosition,
        ForceAll,
        ForceFinger,
        AdmittanceOn,
        AdmittanceOff,
        FsrTare,
        Speed,
        Torque,
        Stop,
        Hold,
        GetStatus,
        StatusStream,
        Unknown
    };

    struct Command {
        CommandType type = CommandType::Unknown;
        HandControl::GripType grip = HandControl::GripType::Open;
        HandControl::Finger finger = HandControl::Finger::Thumb;
        float position_percent = 0.0f;
        float force_newton = 0.0f;
        uint16_t speed_deg_per_s = 0;
        uint16_t torque_percent = 0;
        uint8_t status_rate_hz = 0;
        bool has_force = false;
    };

    virtual bool executeCommand(const Command& cmd) = 0;
};

class SerialCommander {
public:
    explicit SerialCommander(HandControl::ISerialPort& port) noexcept;
    void setExecutor(ICommandExecutor* exec) noexcept { executor_ = exec; }

    bool feedByte(uint8_t b) noexcept;
    void processCommand() noexcept;
    void serviceTx() noexcept;
    void sendText(const char* text) noexcept;

private:
    HandControl::ISerialPort& port_;
    ICommandExecutor* executor_ = nullptr;

    static constexpr size_t RX_BUF_SIZE = 256;
    static constexpr size_t MAX_COMMAND_LENGTH = 128;
    static constexpr size_t TX_QUEUE_DEPTH = 8;
    static constexpr size_t TX_MESSAGE_SIZE = 384;

    uint8_t rx_buf_[RX_BUF_SIZE]{};
    volatile uint16_t rx_head_ = 0;
    volatile uint16_t rx_tail_ = 0;
    volatile bool overflow_flag_ = false;

    std::array<std::array<char, TX_MESSAGE_SIZE>, TX_QUEUE_DEPTH> tx_queue_{};
    std::array<uint16_t, TX_QUEUE_DEPTH> tx_lengths_{};
    uint8_t tx_head_ = 0;
    uint8_t tx_tail_ = 0;
    bool tx_active_ = false;
    alignas(32) std::array<uint8_t, TX_MESSAGE_SIZE> tx_dma_buffer_{};

    void sendResponse(const char* msg, size_t len) noexcept;
    static bool parseCommand(const uint8_t* data, size_t len,
                             ICommandExecutor::Command& outCmd) noexcept;
};

#endif // SERIAL_COMMANDER_HPP
