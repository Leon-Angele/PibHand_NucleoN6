/**
 * @file servo.cpp
 * @brief Non-blocking async servo driver and protocol implementation.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Provides `Stm32UartDmaPort`, `PollUartPort` and `ServoBus` classes
 * implementing an RX-before-TX protocol for STS3215 servos.
 */

#include "hand/servo.hpp"
#include "hand/hand_config.hpp"
#include <cstdio>
#include <cstring>

namespace HandControl {

// ============================================================================
// STATIC STORAGE (32-byte aligned, non-cacheable for DMA)
// ============================================================================

// Stm32UartDmaPort registry
Stm32UartDmaPort* Stm32UartDmaPort::instances_[Stm32UartDmaPort::MAX_INSTANCES] = {nullptr};
uint8_t Stm32UartDmaPort::instance_count_ = 0;

// ServoBus buffers (non-cacheable for DMA coherency)
__attribute__((section(".noncacheable"), aligned(32))) 
uint8_t ServoBus::tx_buf_storage_[ServoBus::TX_BUF_SIZE];

__attribute__((section(".noncacheable"), aligned(32))) 
uint8_t ServoBus::rx_buf_storage_[ServoBus::RX_BUF_SIZE];

// ============================================================================
// LAYER 1: HARDWARE ABSTRACTION - Stm32UartDmaPort
// ============================================================================

Stm32UartDmaPort::Stm32UartDmaPort(UART_HandleTypeDef* huart)
    : huart_(huart), tx_done_(true), rx_done_(false)
{
    // Register this instance for callback routing
    if (instance_count_ < MAX_INSTANCES) {
        instances_[instance_count_++] = this;
    }
}

/**
 * @brief Construct a new Stm32UartDmaPort instance.
 * @param huart Pointer to HAL UART handle
 */

bool Stm32UartDmaPort::transmitDMA(const uint8_t* data, uint16_t length)
{
    if (!data || length == 0) return false;
    
    // D-Cache clean BEFORE TX (ensure data is written to RAM for DMA)
    SCB_CleanDCache_by_Addr((uint32_t*)data, length);
    
    // Start DMA transmission (non-blocking, returns immediately)
    tx_done_ = false;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(data), length);
    
    if (ret != HAL_OK) {
        HAND_DEBUG("TX DMA failed: %d", ret);
        tx_done_ = true;
        return false;
    }
    
    return true;
}

/**
 * @brief Start a non-blocking DMA transmit.
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return true if DMA started successfully
 */

bool Stm32UartDmaPort::receiveDMA(uint8_t* buffer, uint16_t length)
{
    if (!buffer || length == 0) return false;
    
    // D-Cache invalidate BEFORE RX (prevent reading stale cached data after DMA)
    SCB_InvalidateDCache_by_Addr((uint32_t*)buffer, length);
    
    // Start DMA reception (non-blocking, returns immediately)
    rx_done_ = false;
    HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(huart_, buffer, length);
    
    if (ret != HAL_OK) {
        HAND_DEBUG("RX DMA failed: %d", ret);
        rx_done_ = true;
        return false;
    }
    
    return true;
}

/**
 * @brief Start a non-blocking DMA receive into provided buffer.
 * @param buffer Pointer to receive buffer
 * @param length Number of bytes to receive
 * @return true if DMA started successfully
 */

void Stm32UartDmaPort::process()
{
    // Non-blocking: no busy-waiting, just state checks
    // Echo handling could go here if needed
}

/**
 * @brief Non-blocking port housekeeping (timeouts, echo handling).
 */

void Stm32UartDmaPort::abortRx()
{
    HAL_UART_AbortReceive(huart_);
    rx_done_ = true;
}

/**
 * @brief Abort an ongoing RX operation (used on timeouts).
 */

void Stm32UartDmaPort::onTxComplete(UART_HandleTypeDef* huart)
{
    // Route callback to correct instance
    for (uint8_t i = 0; i < instance_count_; ++i) {
        if (instances_[i] && instances_[i]->huart_ == huart) {
            instances_[i]->tx_done_ = true;
            break;
        }
    }
}

/**
 * @brief Static callback router for TX complete events from HAL.
 * @param huart UART handle received from HAL
 */

void Stm32UartDmaPort::onRxComplete(UART_HandleTypeDef* huart)
{
    // Route callback to correct instance
    for (uint8_t i = 0; i < instance_count_; ++i) {
        if (instances_[i] && instances_[i]->huart_ == huart) {
            instances_[i]->rx_done_ = true;
            break;
        }
    }
}

/**
 * @brief Static callback router for RX complete events from HAL.
 * @param huart UART handle received from HAL
 */

// ============================================================================
// LAYER 1: HARDWARE ABSTRACTION - PollUartPort (VCP only)
// ============================================================================

PollUartPort::PollUartPort(UART_HandleTypeDef* huart, uint32_t timeout_ms)
    : huart_(huart), timeout_ms_(timeout_ms)
{
}

/**
 * @brief Blocking transmit implementation for VCP/debug.
 * @param data Message bytes
 * @param length Byte count
 * @return true on success
 */

bool PollUartPort::transmitDMA(const uint8_t* data, uint16_t length)
{
    // Blocking transmit for VCP (debug output only)
    HAL_StatusTypeDef ret = HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), length, timeout_ms_);
    return (ret == HAL_OK);
}

// ============================================================================
// LAYER 2: PROTOCOL LAYER - ServoBus
// ============================================================================

ServoBus::ServoBus(ISerialPort& port)
    : port_(port), state_(BusState::IDLE), tx_buf_(tx_buf_storage_), rx_buf_(rx_buf_storage_)
{
}

/**
 * @brief Construct a new ServoBus instance.
 * @param port Underlying serial port implementation
 */

void ServoBus::poll()
{
    switch (state_) {
        case BusState::IDLE:
            // Nothing to do
            break;
            
        case BusState::TX_BUSY:
            // Wait for TX completion
            if (port_.isTxDone()) {
                state_ = BusState::IDLE;
            }
            // Check timeout (optional)
            if ((HAL_GetTick() - operation_start_ms_) > 100) {
                HAND_DEBUG("TX timeout");
                state_ = BusState::TIMEOUT;
            }
            break;
            
        case BusState::WAIT_RX:
            // Wait for RX completion
            if (port_.isRxDone()) {
                // Validate response
                if (validateResponse(last_read_id_, expected_rx_len_ - 6)) {
                    state_ = BusState::DATA_READY;
                } else {
                    HAND_DEBUG("RX validation failed (ID=%d)", last_read_id_);
                    state_ = BusState::TIMEOUT;
                }
            }
            // Check timeout (10ms as per spec)
            else if ((HAL_GetTick() - operation_start_ms_) > 10) {
                HAND_DEBUG("RX timeout (ID=%d)", last_read_id_);
                port_.abortRx();
                state_ = BusState::TIMEOUT;
            }
            break;
            
        case BusState::DATA_READY:
        case BusState::TIMEOUT:
            // User must call resetState() after consuming result
            break;
    }
    
    // Non-blocking process
    port_.process();
}

/**
 * @brief Poll the servo bus state machine (non-blocking).
 *
 * Handles TX/RX completion and timeouts. Call from main loop.
 */

bool ServoBus::syncWritePositions(const uint8_t* ids, const uint16_t* positions, 
                                  const uint16_t* times_ms, size_t count)
{
    if (state_ != BusState::IDLE || !ids || !positions || !times_ms || count == 0) {
        return false;
    }
    
    // Build SyncWrite packet
    size_t len = buildSyncWritePacket(ids, positions, times_ms, count, tx_buf_);
    if (len == 0) return false;
    
    // Start TX (non-blocking)
    if (!port_.transmitDMA(tx_buf_, len)) {
        return false;
    }
    
    state_ = BusState::TX_BUSY;
    operation_start_ms_ = HAL_GetTick();
    return true;
}

/**
 * @brief Send a SyncWrite packet to multiple servos (non-blocking).
 * @param ids Array of servo IDs
 * @param positions Array of positions (0..4095)
 * @param times_ms Array of move times in ms
 * @param count Number of servos
 * @return true if command started
 */

bool ServoBus::writeRegister(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len)
{
    if (state_ != BusState::IDLE || !data) {
        return false;
    }
    
    // Build Write packet
    size_t pkt_len = buildWritePacket(id, reg, data, len, tx_buf_);
    if (pkt_len == 0) return false;
    
    // Start TX (non-blocking)
    if (!port_.transmitDMA(tx_buf_, pkt_len)) {
        return false;
    }
    
    state_ = BusState::TX_BUSY;
    operation_start_ms_ = HAL_GetTick();
    return true;
}

/**
 * @brief Write a register to a single servo (non-blocking).
 */

bool ServoBus::startReadCurrent(uint8_t id)
{
    if (state_ != BusState::IDLE) {
        return false;
    }
    
    // === RX-before-TX for D-Cache coherency ===
    
    // Step 1: Calculate expected RX length (Status response: 0xFF 0xFF ID Len Error Data... Checksum)
    // For Current (2 bytes): base=6 + data=2 = 8 bytes
    expected_rx_len_ = 6 + 2;
    last_read_id_ = id;
    
    // Step 2: Invalidate D-Cache for RX buffer BEFORE starting DMA
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf_, expected_rx_len_);
    
    // Step 3: Start RX DMA FIRST (listening in background)
    if (!port_.receiveDMA(rx_buf_, expected_rx_len_)) {
        HAND_DEBUG("RX DMA start failed (ID=%d)", id);
        return false;
    }
    
    // Step 4: Build and send Read command
    size_t tx_len = buildReadPacket(id, static_cast<uint8_t>(Reg::Current), 2, tx_buf_);
    if (tx_len == 0) {
        port_.abortRx();
        return false;
    }
    
    // Step 5: Start TX DMA (command goes out)
    if (!port_.transmitDMA(tx_buf_, tx_len)) {
        port_.abortRx();
        return false;
    }
    
    // Step 6: Enter WAIT_RX state and return immediately (non-blocking!)
    state_ = BusState::WAIT_RX;
    operation_start_ms_ = HAL_GetTick();
    
    return true;
}

/**
 * @brief Start asynchronous read of the Current register using RX-before-TX.
 * @param id Servo ID
 * @return true if read started successfully
 */

std::optional<int16_t> ServoBus::getReadResult()
{
    if (state_ != BusState::DATA_READY) {
        return std::nullopt;
    }
    
    // Extract data from response packet: 0xFF 0xFF ID Len Error [Data_L Data_H] Checksum
    // Data is at offset 5 (little-endian)
    int16_t value = static_cast<int16_t>(rx_buf_[5] | (rx_buf_[6] << 8));
    
    return value;
}

/**
 * @brief Retrieve result of a completed async read.
 * @return std::optional<int16_t> Measured current in mA, or std::nullopt
 */

// ============================================================================
// PROTOCOL HELPERS
// ============================================================================

uint8_t ServoBus::calcChecksum(const uint8_t* data, size_t len)
{
    // STS3215 Checksum: ~(ID + Length + Instruction + Params...) & 0xFF
    // 'data' points to ID (skip 0xFF 0xFF header)
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(~sum & 0xFF);
}

/**
 * @brief Calculate protocol checksum for a packet (STS3215 style).
 */

bool ServoBus::validateResponse(uint8_t expected_id, uint8_t data_len)
{
    // Response format: 0xFF 0xFF ID Length Error Data... Checksum
    // Length = data_len + 2 (Error + Checksum)
    
    // Check header
    if (rx_buf_[0] != 0xFF || rx_buf_[1] != 0xFF) {
        HAND_DEBUG("Invalid header");
        return false;
    }
    
    // Check ID
    if (rx_buf_[2] != expected_id) {
        HAND_DEBUG("ID mismatch: expected=%d, got=%d", expected_id, rx_buf_[2]);
        return false;
    }
    
    // Check Length
    uint8_t expected_len = data_len + 2;
    if (rx_buf_[3] != expected_len) {
        HAND_DEBUG("Length mismatch: expected=%d, got=%d", expected_len, rx_buf_[3]);
        return false;
    }
    
    // Check Error byte
    if (rx_buf_[4] != 0) {
        HAND_DEBUG("Servo error: 0x%02X", rx_buf_[4]);
        // Continue anyway - some errors are non-fatal
    }
    
    // Verify checksum
    // Checksum is calculated on: ID + Length + Error + Data...
    uint8_t calc_cs = calcChecksum(&rx_buf_[2], 2 + data_len);
    uint8_t recv_cs = rx_buf_[5 + data_len];
    
    if (calc_cs != recv_cs) {
        HAND_DEBUG("Checksum mismatch: calc=0x%02X, recv=0x%02X", calc_cs, recv_cs);
        return false;
    }
    
    return true;
}

/**
 * @brief Validate a received status packet for expected ID/length/checksum.
 * @param expected_id Expected servo ID
 * @param data_len Expected payload data length
 * @return true if packet is valid
 */

size_t ServoBus::buildReadPacket(uint8_t id, uint8_t reg, uint8_t len, uint8_t* out_buf)
{
    // Packet format: 0xFF 0xFF ID Length Instruction Reg DataLen Checksum
    // Length = 4 (Instruction + Reg + DataLen + Checksum)
    
    out_buf[0] = 0xFF;
    out_buf[1] = 0xFF;
    out_buf[2] = id;
    out_buf[3] = 4;  // Length = Param_Count(2) + 2
    out_buf[4] = static_cast<uint8_t>(Instruction::Read);
    out_buf[5] = reg;
    out_buf[6] = len;
    out_buf[7] = calcChecksum(&out_buf[2], 5);  // ID + Length + Instruction + Params
    
    return 8;
}

/**
 * @brief Build a Read instruction packet.
 */

size_t ServoBus::buildWritePacket(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len, uint8_t* out_buf)
{
    // Packet format: 0xFF 0xFF ID Length Instruction Reg Data... Checksum
    // Length = Param_Count(1 + len) + 2
    
    out_buf[0] = 0xFF;
    out_buf[1] = 0xFF;
    out_buf[2] = id;
    out_buf[3] = (1 + len) + 2;  // Length
    out_buf[4] = static_cast<uint8_t>(Instruction::Write);
    out_buf[5] = reg;
    
    // Copy data
    for (uint8_t i = 0; i < len; ++i) {
        out_buf[6 + i] = data[i];
    }
    
    // Checksum
    out_buf[6 + len] = calcChecksum(&out_buf[2], 4 + len);
    
    return 7 + len;
}

/**
 * @brief Build a Write instruction packet.
 */

size_t ServoBus::buildSyncWritePacket(const uint8_t* ids, const uint16_t* positions, 
                                      const uint16_t* times_ms, size_t count, uint8_t* out_buf)
{
    // SyncWrite format: 0xFF 0xFF ID(0xFE) Length Instruction StartReg DataLen [ID Pos_L Pos_H Time_L Time_H]... Checksum
    // Length = Param_Count(2 + count*5) + 2
    
    if (count == 0 || count > 12) return 0;  // Sanity check
    
    out_buf[0] = 0xFF;
    out_buf[1] = 0xFF;
    out_buf[2] = 0xFE;  // Broadcast ID
    out_buf[3] = (2 + count * 5) + 2;  // Length
    out_buf[4] = static_cast<uint8_t>(Instruction::SyncWrite);
    out_buf[5] = static_cast<uint8_t>(Reg::Position);  // Start register (0x2A)
    out_buf[6] = 4;  // Data length per servo (Pos_L, Pos_H, Time_L, Time_H)
    
    // Add data for each servo
    size_t offset = 7;
    for (size_t i = 0; i < count; ++i) {
        out_buf[offset++] = ids[i];
        out_buf[offset++] = static_cast<uint8_t>(positions[i] & 0xFF);       // Pos_L
        out_buf[offset++] = static_cast<uint8_t>((positions[i] >> 8) & 0xFF); // Pos_H
        out_buf[offset++] = static_cast<uint8_t>(times_ms[i] & 0xFF);        // Time_L
        out_buf[offset++] = static_cast<uint8_t>((times_ms[i] >> 8) & 0xFF); // Time_H
    }
    
    // Checksum
    out_buf[offset] = calcChecksum(&out_buf[2], offset - 2);
    
    return offset + 1;
}

/**
 * @brief Build a SyncWrite broadcast packet for multiple servos.
 */

} // namespace HandControl
