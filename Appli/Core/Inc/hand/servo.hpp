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

// State for port-level DMA operations
enum class PortState : uint8_t {
    IDLE,
    TX_BUSY,
    RX_BUSY,
    COMPLETE,
    TIMEOUT
};

class ISerialPort {
public:
    virtual ~ISerialPort() = default;
    virtual bool transmitDMA(const uint8_t* data, uint16_t length, bool waitForCompletion = true) = 0;
    virtual PortState getState() const = 0;
    virtual void process() = 0;
    virtual void resetState() = 0;
    
    // Ringbuffer interface
    virtual uint16_t rxAvailable() const = 0;
    virtual int16_t rxRead() = 0;
    virtual bool rxPeek(uint8_t* buffer, uint16_t length) const = 0;
    virtual void rxConsume(uint16_t length) = 0;
    virtual void rxFlush() = 0;
};

class Stm32UartDmaPort : public ISerialPort {
public:
    Stm32UartDmaPort(UART_HandleTypeDef* huart, uint32_t tx_timeout_ms = 200, uint32_t rx_timeout_ms = 200);
    ~Stm32UartDmaPort() override = default;

    bool transmitDMA(const uint8_t* data, uint16_t length, bool waitForCompletion = false) override;
    
    // State machine interface
    PortState getState() const override { return state_; }
    void process() override;
    void resetState() override;

    // DMA Continuous RX with ringbuffer
    bool startReceiveToIdle();
    uint16_t rxAvailable() const;
    int16_t rxRead();
    bool rxPeek(uint8_t* buffer, uint16_t length) const;
    void rxConsume(uint16_t length);
    void rxFlush();

    // Called from HAL callbacks
    static void onTxComplete(UART_HandleTypeDef* huart);
    static void onRxEvent(UART_HandleTypeDef* huart, uint16_t Size);

private:
    UART_HandleTypeDef* huart_;
    volatile bool tx_done_;
    uint32_t tx_timeout_ms_;
    uint32_t rx_timeout_ms_;
    
    // State machine
    PortState state_ = PortState::IDLE;
    uint32_t operation_start_ms_ = 0;

    // DMA Continuous RX: Ringbuffer (non-cacheable for DMA coherency)
    static constexpr size_t RX_RING_SIZE = 256;
    static constexpr size_t TEMP_DMA_SIZE = 64;
    uint8_t* temp_dma_buffer_;  // Points to static non-cacheable buffer
    uint8_t* rx_ring_;          // Points to static non-cacheable buffer
    volatile uint16_t rx_head_ = 0;  // Updated by DMA callback
    uint16_t rx_tail_ = 0;           // Updated by parser
    uint16_t pending_tx_echo_len_ = 0;  // For auto-echo consumption
    
    // Static non-cacheable buffers (allocated in .cpp)
    static uint8_t temp_dma_buffer_storage_[TEMP_DMA_SIZE];
    static uint8_t rx_ring_storage_[RX_RING_SIZE];

    // Multi-port registry: supports USART3 (servo bus) + LPUART1 (VCP) simultaneously.
    // Each Stm32UartDmaPort instance registers itself here so that onTxComplete/onRxEvent
    // can route the HAL callback to the correct port object.
    static constexpr uint8_t MAX_INSTANCES = 2;
    static Stm32UartDmaPort* instances_[MAX_INSTANCES];
    static uint8_t instance_count_;
};

// Lightweight UART port using blocking transmit (no DMA) — for VCP/debug output
class PollUartPort : public ISerialPort {
public:
    explicit PollUartPort(UART_HandleTypeDef* huart, uint32_t tx_timeout_ms = 1000)
        : huart_(huart), tx_timeout_ms_(tx_timeout_ms) {}
    ~PollUartPort() override = default;

    bool transmitDMA(const uint8_t* data, uint16_t length, bool waitForCompletion = true) override;
    PortState getState() const override { return PortState::IDLE; }
    void process() override {}
    void resetState() override {}
    
    // Ringbuffer stubs
    uint16_t rxAvailable() const override { return 0; }
    int16_t rxRead() override { return -1; }
    bool rxPeek(uint8_t*, uint16_t) const override { return false; }
    void rxConsume(uint16_t) override {}
    void rxFlush() override {}

private:
    UART_HandleTypeDef* huart_;
    uint32_t tx_timeout_ms_;
};

// State for bus-level operations (protocol layer)
enum class BusState : uint8_t {
    IDLE,
    WAITING_TX,
    WAITING_RX,
    COMPLETE,
    ERROR
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

    // Synchronous write (broadcast, no response expected)
    bool writeRegister(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len);
    bool syncWritePositions(const uint8_t* ids, const uint16_t* positions, const uint16_t* times_ms, size_t count);
    bool ping(uint8_t id);

    // Read register (polls ringbuffer for servo response)
    bool readRegister(uint8_t id, uint8_t reg, uint8_t len, uint8_t* out);
    
    // State machine
    BusState getState() const { return bus_state_; }
    void process();  // Update state machine, parse incoming packets from ringbuffer

private:
    ISerialPort& port_;
    static constexpr size_t BUF_SIZE = 64;
    std::array<uint8_t, BUF_SIZE> tx_buf_{};
    
    // State machine
    BusState bus_state_ = BusState::IDLE;
    
    // Helpers
    static uint8_t checksum(const uint8_t* data, size_t len);
    bool validateRxPacket(uint8_t expected_id, uint8_t expected_data_len);
};

class Servo {
public:
    explicit Servo(uint8_t id, ServoBus& bus) : id_(id), bus_(bus) {}

    // Synchronous write operations (no response expected)
    bool ping();
    bool setTorqueEnable(bool enable);
    bool setPosition(uint16_t position, uint16_t time_ms);

    // Read operations (poll ringbuffer for servo response)
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
