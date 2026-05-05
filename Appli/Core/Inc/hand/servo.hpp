/*
 * servo.hpp
 * Object-oriented driver for serial bus servos using STM32 HAL + DMA (USART3)
 * No dynamic allocation. C++17 (std::array, std::optional).
 */
#ifndef SERVO_HPP
#define SERVO_HPP

#include "main.h"
#include <array>
#include <cstdint>
#include <optional>

class ISerialPort {
public:
    virtual ~ISerialPort() = default;
    virtual bool transmitDMA(const uint8_t* data, uint16_t length) = 0;
    virtual bool receiveDMA(uint8_t* buffer, uint16_t length) = 0;
};

class Stm32UartDmaPort : public ISerialPort {
public:
    Stm32UartDmaPort(UART_HandleTypeDef* huart, uint32_t tx_timeout_ms = 200, uint32_t rx_timeout_ms = 200);
    ~Stm32UartDmaPort() override = default;

    bool transmitDMA(const uint8_t* data, uint16_t length) override;
    bool receiveDMA(uint8_t* buffer, uint16_t length) override;

    // Called from HAL callbacks
    static void onTxComplete(UART_HandleTypeDef* huart);
    static void onRxComplete(UART_HandleTypeDef* huart);

private:
    UART_HandleTypeDef* huart_;
    volatile bool tx_done_;
    volatile bool rx_done_;
    uint32_t tx_timeout_ms_;
    uint32_t rx_timeout_ms_;

    // Multi-port registry: supports USART3 (servo bus) + LPUART1 (VCP) simultaneously.
    // Each Stm32UartDmaPort instance registers itself here so that onTxComplete/onRxComplete
    // can route the HAL callback to the correct port object.
    static constexpr uint8_t MAX_INSTANCES = 2;
    static Stm32UartDmaPort* instances_[MAX_INSTANCES];
    static uint8_t instance_count_;
};

class ServoBus {
public:
    explicit ServoBus(ISerialPort& port) : port_(port) {}

    enum class Instruction : uint8_t {
        Ping = 0x01,
        Read  = 0x02,
        Write = 0x03
    };

    enum class Reg : uint8_t {
        TorqueEnable = 0x28,
        Position     = 0x2A,
        PosRead      = 0x38,
        Speed        = 0x3A,
        Load         = 0x3C,
        Voltage      = 0x3E,
        Temperature  = 0x3F,
        Current      = 0x45
    };

    // Generic write: writes 'len' bytes from data to register 'reg' on servo 'id'
    bool writeRegister(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len);

    // Generic read: reads 'len' bytes from register 'reg' on servo 'id' into out (out must be at least len)
    bool readRegister(uint8_t id, uint8_t reg, uint8_t len, uint8_t* out);

    // Send a Ping instruction to the given ID and expect a status packet
    bool ping(uint8_t id);

    // Sync write positions to multiple servos at once (SCS_SYNC_WRITE 0x83)
    // ids: array of servo IDs, positions: array of 16-bit positions, times_ms: per-servo time (can be same for all)
    // count: number of servos (max 6 expected)
    bool syncWritePositions(const uint8_t* ids, const uint16_t* positions, const uint16_t* times_ms, size_t count);

    // Split read (non-blocking request/finish pattern): startRead only transmits the read request packet,
    // finishRead performs the receive and copies params into out (out must be sized for len bytes).
    // startReadRegister returns expected_rx (total bytes to receive) via expected_rx_out and true on TX success.
    bool startReadRegister(uint8_t id, uint8_t reg, uint8_t len, uint16_t& expected_rx_out);
    bool finishReadRegister(uint8_t id, uint8_t len, uint8_t* out, uint16_t expected_rx);

private:
    ISerialPort& port_;
    static constexpr size_t BUF_SIZE = 64;
    std::array<uint8_t, BUF_SIZE> tx_buf_{};
    std::array<uint8_t, BUF_SIZE> rx_buf_{};

    static uint8_t checksum(const uint8_t* data, size_t len);
};

class Servo {
public:
    explicit Servo(uint8_t id, ServoBus& bus) : id_(id), bus_(bus) {}

    bool ping();
    bool setTorqueEnable(bool enable);
    bool setPosition(uint16_t position, uint16_t time_ms);

    std::optional<int16_t> getPosition();
    std::optional<int16_t> getSpeed();
    std::optional<int16_t> getLoad();
    std::optional<uint8_t> getVoltage();
    std::optional<uint8_t> getTemperature();
    std::optional<int16_t> getCurrent();

private:
    uint8_t id_;
    ServoBus& bus_;
};

#endif // SERVO_HPP
