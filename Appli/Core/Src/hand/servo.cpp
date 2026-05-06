/*
 * servo.cpp
 * Implementation of serial-bus servo classes using STM32 HAL + DMA (USART3)
 */

#include "hand/servo.hpp"
#include <cstdio>
#include "main.h"

// Multi-port registry: each Stm32UartDmaPort registers itself so that
// onTxComplete/onRxComplete can route HAL callbacks to the correct port object.
Stm32UartDmaPort* Stm32UartDmaPort::instances_[Stm32UartDmaPort::MAX_INSTANCES] = {nullptr, nullptr};
uint8_t Stm32UartDmaPort::instance_count_ = 0;

// Static non-cacheable buffers
__attribute__((section(".noncacheable"))) uint8_t Stm32UartDmaPort::temp_dma_buffer_storage_[Stm32UartDmaPort::TEMP_DMA_SIZE];
__attribute__((section(".noncacheable"))) uint8_t Stm32UartDmaPort::rx_ring_storage_[Stm32UartDmaPort::RX_RING_SIZE];

Stm32UartDmaPort::Stm32UartDmaPort(UART_HandleTypeDef* huart, uint32_t tx_timeout_ms, uint32_t rx_timeout_ms)
    : huart_(huart), tx_done_(false), tx_timeout_ms_(tx_timeout_ms), rx_timeout_ms_(rx_timeout_ms),
      state_(PortState::IDLE), operation_start_ms_(0),
      temp_dma_buffer_(temp_dma_buffer_storage_), rx_ring_(rx_ring_storage_)
{
    if (instance_count_ < MAX_INSTANCES) {
        instances_[instance_count_++] = this;
    }
}

bool Stm32UartDmaPort::transmitDMA(const uint8_t* data, uint16_t length, bool waitForCompletion)
{
    // D-Cache clean before TX (needed for cache coherency on Cortex-M55)
    SCB_CleanDCache_by_Addr((uint32_t*)data, length);
    
    // Use blocking transmit for simplicity (DMA TX callback optional later)
    HAL_StatusTypeDef ret = HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), length, tx_timeout_ms_);
    if (ret != HAL_OK) {
        printf("[SERVO] TX failed: %d\r\n", ret);
        return false;
    }
    
    // Store TX length for auto-echo consumption
    pending_tx_echo_len_ = length;
    
    return true;
}

// Start continuous RX using ReceiveToIdle DMA
bool Stm32UartDmaPort::startReceiveToIdle()
{
    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(huart_, temp_dma_buffer_, TEMP_DMA_SIZE);
    if (ret != HAL_OK) {
        printf("[SERVO] ReceiveToIdle DMA start failed: %d\r\n", ret);
        return false;
    }
    return true;
}

// Ringbuffer methods
uint16_t Stm32UartDmaPort::rxAvailable() const
{
    // Volatile read of rx_head_ (updated by ISR)
    uint16_t head = rx_head_;
    if (head >= rx_tail_) {
        return head - rx_tail_;
    } else {
        return RX_RING_SIZE - rx_tail_ + head;
    }
}

int16_t Stm32UartDmaPort::rxRead()
{
    if (rx_tail_ == rx_head_) return -1;
    uint8_t byte = rx_ring_[rx_tail_];
    rx_tail_ = (rx_tail_ + 1) % RX_RING_SIZE;
    return byte;
}

bool Stm32UartDmaPort::rxPeek(uint8_t* buffer, uint16_t length) const
{
    if (rxAvailable() < length) return false;
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = rx_ring_[(rx_tail_ + i) % RX_RING_SIZE];
    }
    return true;
}

void Stm32UartDmaPort::rxConsume(uint16_t length)
{
    rx_tail_ = (rx_tail_ + length) % RX_RING_SIZE;
}

void Stm32UartDmaPort::rxFlush()
{
    rx_tail_ = rx_head_;
}

// Non-blocking state machine update - called from ServoBus::process()
void Stm32UartDmaPort::process()
{
    // Auto-consume TX echo after brief delay
    if (pending_tx_echo_len_ > 0) {
        // Wait for echo to arrive in ringbuffer (RS485 echo via Waveshare board)
        HAL_Delay(2);
        if (rxAvailable() >= pending_tx_echo_len_) {
            rxConsume(pending_tx_echo_len_);
            pending_tx_echo_len_ = 0;
        }
    }
}

// Reset port to IDLE
void Stm32UartDmaPort::resetState()
{
    state_ = PortState::IDLE;
}

void Stm32UartDmaPort::onTxComplete(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0; i < instance_count_; ++i) {
        if (instances_[i] && instances_[i]->huart_ == huart) {
            instances_[i]->tx_done_ = true;
            break;
        }
    }
}

void Stm32UartDmaPort::onRxEvent(UART_HandleTypeDef* huart, uint16_t Size)
{
    for (uint8_t i = 0; i < instance_count_; ++i) {
        if (instances_[i] && instances_[i]->huart_ == huart) {
            // Copy received data from temp DMA buffer to ringbuffer
            for (uint16_t j = 0; j < Size; j++) {
                instances_[i]->rx_ring_[instances_[i]->rx_head_] = instances_[i]->temp_dma_buffer_[j];
                instances_[i]->rx_head_ = (instances_[i]->rx_head_ + 1) % RX_RING_SIZE;
            }
            
            // Restart ReceiveToIdle for next packet
            HAL_UARTEx_ReceiveToIdle_DMA(huart, instances_[i]->temp_dma_buffer_, TEMP_DMA_SIZE);
            break;
        }
    }
}

// PollUartPort implementation (blocking TX, no DMA)
bool PollUartPort::transmitDMA(const uint8_t* data, uint16_t length, bool waitForCompletion)
{
    (void)waitForCompletion;
    HAL_StatusTypeDef ret = HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), length, tx_timeout_ms_);
    return (ret == HAL_OK);
}

// checksum calculation: ~(ID + Length + Instruction + Params...)
uint8_t ServoBus::checksum(const uint8_t* data, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    return static_cast<uint8_t>(~(sum & 0xFF));
}

bool ServoBus::writeRegister(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len)
{
    // Packet: 0xFF,0xFF, ID, LEN, INST, PARAMS..., CHECKSUM
    // LEN = params_len + 2 (Instruction + Checksum)
    uint8_t params_len = 1 + len; // reg + data
    uint8_t pkt_len = 4 + params_len; // header(2) + ID + LEN + INST + params + checksum
    if (pkt_len > tx_buf_.size()) return false;

    size_t idx = 0;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = id;
    tx_buf_[idx++] = params_len + 2; // LEN = INST(1) + PARAMS(params_len) + CHECKSUM(1)
    tx_buf_[idx++] = static_cast<uint8_t>(Instruction::Write);
    tx_buf_[idx++] = reg;
    for (uint8_t i = 0; i < len; ++i) tx_buf_[idx++] = data[i];

    // checksum over ID, Length, Instruction and Params
    uint8_t csum = checksum(&tx_buf_[2], static_cast<size_t>(idx - 2));
    tx_buf_[idx++] = csum;

    // Transmit and return immediately (non-blocking).
    // SCS/Feetech servos only return status packets for READ commands by default
    // (Status Return Level = 1). No need to wait for completion.
    return port_.transmitDMA(tx_buf_.data(), (uint16_t)idx, false);
}

// Synchronous read: sends command + polls ringbuffer for servo response
bool ServoBus::readRegister(uint8_t id, uint8_t reg, uint8_t len, uint8_t* out)
{
    // Build read command packet
    uint8_t params_len = 2; // reg address + data length
    size_t idx = 0;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = id;
    tx_buf_[idx++] = params_len + 2; // LEN = INST(1) + PARAMS(2) + CHK(1)
    tx_buf_[idx++] = static_cast<uint8_t>(Instruction::Read);
    tx_buf_[idx++] = reg;
    tx_buf_[idx++] = len;
    uint8_t csum = checksum(&tx_buf_[2], static_cast<size_t>(idx - 2));
    tx_buf_[idx++] = csum;

    // Send read command (blocking TX with auto-echo consumption)
    if (!port_.transmitDMA(tx_buf_.data(), (uint16_t)idx, false)) {
        printf("[BUS] readReg TX failed (ID=%d, reg=%d)\r\n", id, reg);
        return false;
    }

    // Small delay for servo response time (STS3215 needs ~5ms to prepare response)
    HAL_Delay(5);

    // Poll ringbuffer for response packet
    uint16_t expected_rx = static_cast<uint16_t>(6 + len);
    uint32_t start_ms = HAL_GetTick();
    constexpr uint32_t kReadTimeoutMs = 50;
    
    while ((HAL_GetTick() - start_ms) < kReadTimeoutMs) {
        port_.process();  // Process pending echo consumption
        
        if (port_.rxAvailable() >= expected_rx) {
            // Try to find valid packet header (0xFF 0xFF)
            uint8_t header[6];
            if (port_.rxPeek(header, 6)) {
                if (header[0] == 0xFF && header[1] == 0xFF && header[2] == id) {
                    // Found matching header - validate length and error byte
                    if (header[3] == (len + 2) && header[4] == 0) {
                        // Checksum validation
                        uint8_t full_packet[64];
                        if (port_.rxPeek(full_packet, expected_rx)) {
                            uint8_t calc_sum = checksum(&full_packet[2], expected_rx - 3);
                            if (calc_sum == full_packet[expected_rx - 1]) {
                                // SUCCESS: Copy data and consume packet
                                for (uint8_t i = 0; i < len; i++) {
                                    out[i] = full_packet[5 + i];
                                }
                                port_.rxConsume(expected_rx);
                                return true;
                            }
                        }
                    }
                }
            }
            // Invalid header or checksum - consume 1 byte and retry
            port_.rxConsume(1);
        }
        HAL_Delay(1);
    }
    
    printf("[BUS] readReg RX timeout (ID=%d, reg=%d, expect=%d bytes)\r\n", id, reg, expected_rx);
    return false;
}

// Validate received packet from ringbuffer peek
bool ServoBus::validateRxPacket(uint8_t expected_id, uint8_t expected_data_len)
{
    uint8_t packet[64];
    uint16_t packet_len = 6 + expected_data_len;
    if (!port_.rxPeek(packet, packet_len)) return false;
    
    // Check header
    if (packet[0] != 0xFF || packet[1] != 0xFF) return false;
    
    // Check ID
    if (packet[2] != expected_id) return false;
    
    // Check error byte
    if (packet[4] != 0) return false;
    
    // Verify checksum
    uint8_t calc_sum = checksum(&packet[2], packet_len - 3);
    if (calc_sum != packet[packet_len - 1]) return false;
    
    return true;
}

// State machine update - simplified for ringbuffer polling
void ServoBus::process()
{
    // Update port (handles echo consumption)
    port_.process();
}

bool ServoBus::syncWritePositions(const uint8_t* ids, const uint16_t* positions, const uint16_t* times_ms, size_t count)
{
    if (!ids || !positions || count == 0) return false;
    // Instruction 0x83 - Sync Write
    constexpr uint8_t INST_SYNC_WRITE = 0x83;
    uint8_t start_reg = static_cast<uint8_t>(Reg::Position);
    uint8_t data_len = 4; // position(2) + time(2)

    // params: start_address(1) + data_len(1) + [id + data_len bytes] * count
    size_t params_len = 2 + count * (1 + data_len);
    size_t idx = 0;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = 0xFE; // broadcast
    tx_buf_[idx++] = static_cast<uint8_t>(params_len + 2); // LEN = INST(1) + PARAMS(params_len) + CHECKSUM(1)
    tx_buf_[idx++] = INST_SYNC_WRITE;
    tx_buf_[idx++] = start_reg;
    tx_buf_[idx++] = data_len;

    for (size_t i = 0; i < count; ++i) {
        tx_buf_[idx++] = ids[i];
        uint16_t pos = positions[i];
        tx_buf_[idx++] = static_cast<uint8_t>(pos & 0xFF);
        tx_buf_[idx++] = static_cast<uint8_t>((pos >> 8) & 0xFF);
        uint16_t tm = times_ms ? times_ms[i] : 0;
        tx_buf_[idx++] = static_cast<uint8_t>(tm & 0xFF);
        tx_buf_[idx++] = static_cast<uint8_t>((tm >> 8) & 0xFF);
    }

    uint8_t csum = checksum(&tx_buf_[2], static_cast<size_t>(idx - 2));
    tx_buf_[idx++] = csum;

    // Transmit broadcast packet; no status packet expected
    return port_.transmitDMA(tx_buf_.data(), (uint16_t)idx, false);
}

bool ServoBus::ping(uint8_t id)
{
    // Build ping packet: header, id, length=2, instruction=Ping, checksum
    size_t idx = 0;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = 0xFF;
    tx_buf_[idx++] = id;
    tx_buf_[idx++] = 2; // length: instruction + checksum
    tx_buf_[idx++] = static_cast<uint8_t>(Instruction::Ping);
    uint8_t csum = checksum(&tx_buf_[2], static_cast<size_t>(idx - 2));
    tx_buf_[idx++] = csum;

    // Simple ping - just send packet (response check could be added later using readRegister pattern)
    return port_.transmitDMA(tx_buf_.data(), (uint16_t)idx, false);
}

// Servo high-level methods
bool Servo::ping()
{
    return bus_.ping(id_);
}

bool Servo::setTorqueEnable(bool enable)
{
    uint8_t val = enable ? 1 : 0;
    return bus_.writeRegister(id_, static_cast<uint8_t>(ServoBus::Reg::TorqueEnable), &val, 1);
}

bool Servo::setPosition(uint16_t position, uint16_t time_ms)
{
    uint8_t payload[4];
    payload[0] = static_cast<uint8_t>(position & 0xFF);
    payload[1] = static_cast<uint8_t>((position >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>(time_ms & 0xFF);
    payload[3] = static_cast<uint8_t>((time_ms >> 8) & 0xFF);
    return bus_.writeRegister(id_, static_cast<uint8_t>(ServoBus::Reg::Position), payload, 4);
}

// Synchronous READ operations
std::optional<int16_t> Servo::getPosition()
{
    uint8_t buf[2]{};
    if (!bus_.readRegister(id_, static_cast<uint8_t>(ServoBus::Reg::PosRead), 2, buf)) return std::nullopt;
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

std::optional<int16_t> Servo::getSpeed()
{
    uint8_t buf[2]{};
    if (!bus_.readRegister(id_, static_cast<uint8_t>(ServoBus::Reg::Speed), 2, buf)) return std::nullopt;
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

std::optional<int16_t> Servo::getLoad()
{
    uint8_t buf[2]{};
    if (!bus_.readRegister(id_, static_cast<uint8_t>(ServoBus::Reg::Load), 2, buf)) return std::nullopt;
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

std::optional<uint8_t> Servo::getVoltage()
{
    uint8_t buf[1]{};
    if (!bus_.readRegister(id_, static_cast<uint8_t>(ServoBus::Reg::Voltage), 1, buf)) return std::nullopt;
    return buf[0];
}

std::optional<uint8_t> Servo::getTemperature()
{
    uint8_t buf[1]{};
    if (!bus_.readRegister(id_, static_cast<uint8_t>(ServoBus::Reg::Temperature), 1, buf)) return std::nullopt;
    return buf[0];
}

std::optional<int16_t> Servo::getCurrent()
{
    uint8_t buf[2]{};
    if (!bus_.readRegister(id_, static_cast<uint8_t>(ServoBus::Reg::Current), 2, buf)) return std::nullopt;
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

// HAL callbacks are implemented in main.cpp and forward to
// Stm32UartDmaPort::onTxComplete/onRxComplete, so no
// duplicate definitions are needed here.
