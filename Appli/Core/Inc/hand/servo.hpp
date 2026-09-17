/**
 * @file servo.hpp
 * @brief Non-blocking STS3215 facade backed by the FeetechSDK.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Stm32UartDmaPort remains the generic DMA UART used by the VCP commander.
 * ServoBus delegates servo communication to the FeetechSDK.
 */
#ifndef SERVO_HPP
#define SERVO_HPP

#include "main.h"
#include "sts_packet_handler.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace HandControl {

// ============================================================================
// ENUMS & CONSTANTS
// ============================================================================

/**
 * @brief Bus state for protocol-level operations
 */
enum class BusState : uint8_t {
    IDLE,        // Ready for new command
    TX_BUSY,     // Transmitting command (waiting for TX complete)
    WAIT_RX,     // Waiting for servo response
    DATA_READY,  // Response received and validated
    TIMEOUT      // Response timeout occurred
};

/**
 * @brief STS3215 Registers
 */
enum class Reg : uint8_t {
    TorqueEnable = 0x28,  // 1 byte
    TorqueLimit  = 0x30,  // 2 bytes, SRAM, 0..1000
    Position     = 0x2A,  // 4 bytes: Pos_L, Pos_H, Time_L, Time_H
    PosRead      = 0x38,  // 2 bytes (for reading current position)
    Current      = 0x45   // 2 bytes
};

// ============================================================================
// LAYER 1: HARDWARE ABSTRACTION
// ============================================================================

/**
 * @brief Abstract serial port interface
 */
class ISerialPort {
public:
    virtual ~ISerialPort() = default;
    
    /**
     * @brief Transmit data via DMA (non-blocking, returns immediately)
     * @param data Pointer to data buffer
     * @param length Number of bytes to send
     * @return true if DMA started successfully, false otherwise
     */
    virtual bool transmitDMA(const uint8_t* data, uint16_t length) = 0;
    
    /**
     * @brief Receive data via DMA (non-blocking, returns immediately)
     * @param buffer Pointer to receive buffer
     * @param length Number of bytes to receive
     * @return true if DMA started successfully, false otherwise
     */
    virtual bool receiveDMA(uint8_t* buffer, uint16_t length) = 0;
    
    /**
     * @brief Check if TX is complete
     */
    virtual bool isTxDone() const = 0;
    
    /**
     * @brief Check if RX is complete
     */
    virtual bool isRxDone() const = 0;
    
    /**
     * @brief Non-blocking process (checks timeouts, consumes echo, etc.)
     */
    virtual void process() = 0;
    
    /**
     * @brief Abort ongoing RX operation (used for timeouts)
     */
    virtual void abortRx() = 0;
};

/**
 * @brief STM32 UART port with DMA (non-blocking, async)
 * Uses RX-before-TX principle for D-Cache coherency on Cortex-M55
 */
class Stm32UartDmaPort : public ISerialPort {
public:
    explicit Stm32UartDmaPort(UART_HandleTypeDef* huart);
    ~Stm32UartDmaPort() override = default;

    // ISerialPort interface
    bool transmitDMA(const uint8_t* data, uint16_t length) override;
    bool receiveDMA(uint8_t* buffer, uint16_t length) override;
    bool isTxDone() const override { return tx_done_; }
    bool isRxDone() const override { return rx_done_; }
    void process() override;
    
    // Called from HAL callbacks (static routing)
    static void onTxComplete(UART_HandleTypeDef* huart);
    static void onRxComplete(UART_HandleTypeDef* huart);
    static void onError(UART_HandleTypeDef* huart);
    
    // Abort RX (for timeouts)
    void abortRx();

private:
    UART_HandleTypeDef* huart_;
    volatile bool tx_done_;
    volatile bool rx_done_;
    
    // Multi-port registry for callback routing
    static constexpr uint8_t MAX_INSTANCES = 2;
    static Stm32UartDmaPort* instances_[MAX_INSTANCES];
    static uint8_t instance_count_;
};

// ============================================================================
// LAYER 2: PROTOCOL LAYER (STS3215 SERVO BUS)
// ============================================================================

/**
 * @brief Stable hand-control facade around the asynchronous FeetechSDK.
 */
class ServoBus {
public:
    explicit ServoBus(feetech::STSPacketHandler& packet_handler);
    
    // State machine
    BusState getState() const { return state_; }
    void resetState();
    
    /**
     * @brief Non-blocking poll - checks for RX completion, timeouts, etc.
     * Call this from the main loop; it is non-blocking.
     */
    void poll();
    
    // ===== WRITE OPERATIONS (Fire and Forget) =====
    
    /**
     * @brief Sync write positions to multiple servos (broadcast)
     * Non-blocking: starts TX, state becomes TX_BUSY
     * @param ids Array of servo IDs
     * @param positions Array of target positions (0-4095)
     * @param times_ms Array of move times in milliseconds
     * @param count Number of servos
     * @return true if command started successfully
     */
    bool syncWritePositions(const uint8_t* ids, const uint16_t* positions, 
                           const uint16_t* times_ms, size_t count);
    
    /**
     * @brief Write single register (generic)
     * @param id Servo ID
     * @param reg Register address
     * @param data Pointer to data
     * @param len Data length
     * @return true if command started successfully
     */
    bool writeRegister(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len);
    bool writeTorqueLimit(uint8_t id, uint16_t percent);
    
    // ===== ASYNC READ OPERATIONS (RX-before-TX) =====
    
    /**
     * @brief Start async read of Current register (non-blocking)
     * Uses RX-before-TX: invalidate D-Cache → start RX DMA → start TX DMA
     * @param id Servo ID
     * @return true if read started successfully
     */
    bool startReadCurrent(uint8_t id);
    bool startReadPosition(uint8_t id);
    
    /**
     * @brief Get result of async read (call after poll() sets state to DATA_READY)
     * @return Current value in mA, or std::nullopt if not ready
     */
    std::optional<int32_t> getReadResult();
    std::optional<uint16_t> getPositionResult();
    
    // ===== BLOCKING PING (Init/Discovery only) =====
    
    /**
     * @brief Blocking PING command to test servo connectivity (for init only)
     * 
     * Sends a PING instruction and waits for response. This is a blocking
     * operation intended ONLY for initialization/discovery, not for runtime.
     * 
     * @param id Servo ID to ping
     * @param timeout_ms Timeout in milliseconds (default 50ms)
     * @return true if servo responded, false on timeout or error
     */
    bool pingServo(uint8_t id, uint32_t timeout_ms = 50);

private:
    feetech::STSPacketHandler& packet_handler_;
    BusState state_ = BusState::IDLE;
    Reg last_read_reg_ = Reg::Current;
};

} // namespace HandControl

#endif // SERVO_HPP
