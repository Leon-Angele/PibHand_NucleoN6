#include "port_handler_stm32.h"

extern "C"
{
#include "main.h"

extern CACHEAXI_HandleTypeDef hcacheaxi;
}

namespace feetech
{

PortHandlerSTM32* PortHandlerSTM32::instances_[PortHandlerSTM32::MAX_INSTANCES] = {};
uint8_t PortHandlerSTM32::instance_count_ = 0u;

PortHandlerSTM32::PortHandlerSTM32(UART_HandleTypeDef* uart_handle)
    : uart_handle_(uart_handle)
{
    if (instance_count_ < MAX_INSTANCES)
    {
        instances_[instance_count_++] = this;
    }
}

PortHandlerSTM32::~PortHandlerSTM32()
{
    closePort();
    for (uint8_t i = 0u; i < instance_count_; ++i)
    {
        if (instances_[i] != this)
        {
            continue;
        }
        for (uint8_t j = i + 1u; j < instance_count_; ++j)
        {
            instances_[j - 1u] = instances_[j];
        }
        instances_[--instance_count_] = nullptr;
        break;
    }
}

bool PortHandlerSTM32::openPort()
{
    if (uart_handle_ == nullptr || uart_handle_->Instance == nullptr ||
        uart_handle_->hdmatx == nullptr || uart_handle_->hdmarx == nullptr ||
        uart_handle_->Init.BaudRate != DEFAULT_BAUDRATE)
    {
        return false;
    }

    is_open_ = true;
    clearPort();
    return true;
}

void PortHandlerSTM32::closePort()
{
    if (is_open_ && uart_handle_ != nullptr)
    {
        (void)HAL_UART_AbortReceive(uart_handle_);
        (void)HAL_UART_AbortTransmit(uart_handle_);
    }
    is_open_ = false;
    tx_done_ = true;
    rx_done_ = false;
}

void PortHandlerSTM32::clearPort()
{
    if (uart_handle_ == nullptr)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(uart_handle_);
    __HAL_UART_SEND_REQ(uart_handle_, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(uart_handle_, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                        UART_CLEAR_FEF | UART_CLEAR_PEF);
    tx_done_ = true;
    rx_done_ = false;
    uart_error_ = false;
}

bool PortHandlerSTM32::transmitDMA(const uint8_t* packet, uint16_t length)
{
    if (!is_open_ || packet == nullptr || length == 0u || !tx_done_)
    {
        return false;
    }

    auto* address = reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(packet));
    SCB_CleanDCache_by_Addr(address, length);
    if (HAL_CACHEAXI_CleanByAddr(&hcacheaxi, address, length) != HAL_OK)
    {
        return false;
    }
    tx_done_ = false;
    if (HAL_UART_Transmit_DMA(uart_handle_, const_cast<uint8_t*>(packet), length) != HAL_OK)
    {
        tx_done_ = true;
        return false;
    }
    return true;
}

bool PortHandlerSTM32::receiveDMA(uint8_t* packet, uint16_t length)
{
    if (!is_open_ || packet == nullptr || length == 0u)
    {
        return false;
    }

    auto* address = reinterpret_cast<uint32_t*>(packet);
    SCB_CleanInvalidateDCache_by_Addr(address, length);
    if (HAL_CACHEAXI_CleanInvalidByAddr(&hcacheaxi, address, length) != HAL_OK)
    {
        return false;
    }
    rx_done_ = false;
    if (HAL_UART_Receive_DMA(uart_handle_, packet, length) != HAL_OK)
    {
        rx_done_ = false;
        return false;
    }
    return true;
}

bool PortHandlerSTM32::completeReceive(uint8_t* packet, uint16_t length)
{
    if (packet == nullptr || length == 0u)
    {
        return false;
    }

    auto* address = reinterpret_cast<uint32_t*>(packet);
    if (HAL_CACHEAXI_CleanInvalidByAddr(&hcacheaxi, address, length) != HAL_OK)
    {
        return false;
    }
    SCB_InvalidateDCache_by_Addr(address, length);
    return true;
}

void PortHandlerSTM32::abortTransmit()
{
    if (uart_handle_ != nullptr)
    {
        (void)HAL_UART_AbortTransmit(uart_handle_);
    }
    tx_done_ = true;
}

void PortHandlerSTM32::abortReceive()
{
    if (uart_handle_ != nullptr)
    {
        (void)HAL_UART_AbortReceive(uart_handle_);
    }
    rx_done_ = false;
}

int PortHandlerSTM32::getBaudRate() const
{
    return uart_handle_ != nullptr ? static_cast<int>(uart_handle_->Init.BaudRate) : 0;
}

void PortHandlerSTM32::handleTxComplete(UART_HandleTypeDef* uart_handle)
{
    for (uint8_t i = 0u; i < instance_count_; ++i)
    {
        if (instances_[i] != nullptr && instances_[i]->uart_handle_ == uart_handle)
        {
            instances_[i]->tx_done_ = true;
            return;
        }
    }
}

void PortHandlerSTM32::handleRxComplete(UART_HandleTypeDef* uart_handle)
{
    for (uint8_t i = 0u; i < instance_count_; ++i)
    {
        if (instances_[i] != nullptr && instances_[i]->uart_handle_ == uart_handle)
        {
            instances_[i]->rx_done_ = true;
            return;
        }
    }
}

void PortHandlerSTM32::handleUartError(UART_HandleTypeDef* uart_handle)
{
    for (uint8_t i = 0u; i < instance_count_; ++i)
    {
        if (instances_[i] != nullptr && instances_[i]->uart_handle_ == uart_handle)
        {
            instances_[i]->uart_error_ = true;
            instances_[i]->tx_done_ = true;
            instances_[i]->rx_done_ = false;
            return;
        }
    }
}

} // namespace feetech
