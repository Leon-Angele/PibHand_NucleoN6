/**
 * @file servo.cpp
 * @brief UART DMA adapter and FeetechSDK-backed servo facade.
 */

#include "hand/servo.hpp"
#include "hand/hand_config.hpp"

extern "C" CACHEAXI_HandleTypeDef hcacheaxi;

namespace HandControl {

Stm32UartDmaPort* Stm32UartDmaPort::instances_[Stm32UartDmaPort::MAX_INSTANCES] = {};
uint8_t Stm32UartDmaPort::instance_count_ = 0u;

Stm32UartDmaPort::Stm32UartDmaPort(UART_HandleTypeDef* huart)
    : huart_(huart), tx_done_(true), rx_done_(false)
{
    if (instance_count_ < MAX_INSTANCES) {
        instances_[instance_count_++] = this;
    }
}

bool Stm32UartDmaPort::transmitDMA(const uint8_t* data, uint16_t length)
{
    if (data == nullptr || length == 0u) return false;

    SCB_CleanDCache_by_Addr(
        reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(data)), length);
    if (HAL_CACHEAXI_CleanByAddr(
            &hcacheaxi,
            reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(data)), length) != HAL_OK) {
        return false;
    }
    tx_done_ = false;
    const HAL_StatusTypeDef result =
        HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(data), length);
    if (result != HAL_OK) {
        HAND_DEBUG("TX DMA failed: %d", result);
        tx_done_ = true;
        return false;
    }
    return true;
}

bool Stm32UartDmaPort::receiveDMA(uint8_t* buffer, uint16_t length)
{
    if (buffer == nullptr || length == 0u) return false;

    SCB_CleanInvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(buffer), length);
    if (HAL_CACHEAXI_CleanInvalidByAddr(
            &hcacheaxi, reinterpret_cast<uint32_t*>(buffer), length) != HAL_OK) {
        return false;
    }
    rx_done_ = false;
    const HAL_StatusTypeDef result = HAL_UART_Receive_DMA(huart_, buffer, length);
    if (result != HAL_OK) {
        HAND_DEBUG("RX DMA failed: %d", result);
        rx_done_ = true;
        return false;
    }
    return true;
}

void Stm32UartDmaPort::process()
{
}

void Stm32UartDmaPort::abortRx()
{
    (void)HAL_UART_AbortReceive(huart_);
    rx_done_ = true;
}

void Stm32UartDmaPort::onTxComplete(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0u; i < instance_count_; ++i) {
        if (instances_[i] != nullptr && instances_[i]->huart_ == huart) {
            instances_[i]->tx_done_ = true;
            return;
        }
    }
}

void Stm32UartDmaPort::onRxComplete(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0u; i < instance_count_; ++i) {
        if (instances_[i] != nullptr && instances_[i]->huart_ == huart) {
            instances_[i]->rx_done_ = true;
            return;
        }
    }
}

void Stm32UartDmaPort::onError(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0u; i < instance_count_; ++i) {
        if (instances_[i] != nullptr && instances_[i]->huart_ == huart) {
            instances_[i]->tx_done_ = true;
            instances_[i]->rx_done_ = true;
            return;
        }
    }
}

ServoBus::ServoBus(feetech::STSPacketHandler& packet_handler)
    : packet_handler_(packet_handler)
{
}

void ServoBus::resetState()
{
    packet_handler_.reset();
    state_ = BusState::IDLE;
}

void ServoBus::poll()
{
    if (state_ == BusState::TIMEOUT) {
        resetState();
        return;
    }
    if (state_ == BusState::DATA_READY) return;

    packet_handler_.poll();
    switch (packet_handler_.state()) {
        case feetech::PacketState::Idle:
            state_ = BusState::IDLE;
            break;
        case feetech::PacketState::TxBusy:
            state_ = BusState::TX_BUSY;
            break;
        case feetech::PacketState::WaitRx:
            state_ = BusState::WAIT_RX;
            break;
        case feetech::PacketState::DataReady:
            state_ = BusState::DATA_READY;
            break;
        case feetech::PacketState::Error:
            state_ = BusState::TIMEOUT;
            break;
    }
}

bool ServoBus::syncWritePositions(const uint8_t* ids, const uint16_t* positions,
                                  const uint16_t* times_ms, size_t count)
{
    if (state_ != BusState::IDLE ||
        !packet_handler_.startSyncWritePositions(ids, positions, times_ms, count)) {
        return false;
    }
    state_ = BusState::TX_BUSY;
    return true;
}

bool ServoBus::writeRegister(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t len)
{
    if (state_ != BusState::IDLE ||
        !packet_handler_.startWriteBytes(id, reg, data, len, false)) {
        return false;
    }
    state_ = BusState::TX_BUSY;
    return true;
}

bool ServoBus::writeTorqueLimit(uint8_t id, uint16_t percent)
{
    if (percent > 100u) percent = 100u;
    const uint16_t raw = static_cast<uint16_t>(percent * 10u);
    const uint8_t data[2] = {
        static_cast<uint8_t>(raw & 0xFFu),
        static_cast<uint8_t>((raw >> 8u) & 0xFFu),
    };
    return writeRegister(id, static_cast<uint8_t>(Reg::TorqueLimit), data, sizeof(data));
}

bool ServoBus::startReadCurrent(uint8_t id)
{
    if (state_ != BusState::IDLE ||
        !packet_handler_.startReadBytes(id, static_cast<uint8_t>(Reg::Current), 2u)) {
        return false;
    }
    last_read_reg_ = Reg::Current;
    state_ = BusState::TX_BUSY;
    return true;
}

bool ServoBus::startReadPosition(uint8_t id)
{
    if (state_ != BusState::IDLE ||
        !packet_handler_.startReadBytes(id, static_cast<uint8_t>(Reg::PosRead), 2u)) {
        return false;
    }
    last_read_reg_ = Reg::PosRead;
    state_ = BusState::TX_BUSY;
    return true;
}

std::optional<int32_t> ServoBus::getReadResult()
{
    if (state_ != BusState::DATA_READY || last_read_reg_ != Reg::Current) {
        return std::nullopt;
    }

    uint8_t data[2] = {};
    if (!packet_handler_.copyResponse(data, sizeof(data))) {
        resetState();
        return std::nullopt;
    }
    const int16_t raw = static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8u));
    resetState();
    return (static_cast<int32_t>(raw) * 13) / 2;
}

std::optional<uint16_t> ServoBus::getPositionResult()
{
    if (state_ != BusState::DATA_READY || last_read_reg_ != Reg::PosRead) {
        return std::nullopt;
    }

    uint8_t data[2] = {};
    if (!packet_handler_.copyResponse(data, sizeof(data))) {
        resetState();
        return std::nullopt;
    }
    const uint16_t position = static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8u));
    resetState();
    return position;
}

bool ServoBus::pingServo(uint8_t id, uint32_t timeout_ms)
{
    if (state_ != BusState::IDLE) resetState();
    if (!packet_handler_.startPing(id, timeout_ms)) return false;

    state_ = BusState::TX_BUSY;
    while (packet_handler_.state() == feetech::PacketState::TxBusy ||
           packet_handler_.state() == feetech::PacketState::WaitRx) {
        packet_handler_.poll();
        HAL_Delay(1u);
    }

    const bool success = packet_handler_.state() == feetech::PacketState::DataReady;
    resetState();
    return success;
}

} // namespace HandControl
