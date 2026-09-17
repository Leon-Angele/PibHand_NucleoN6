#include "sts_packet_handler.h"

#include <cstring>

extern "C"
{
#include "main.h"
}

namespace feetech
{

namespace
{
constexpr uint8_t kHeader = 0xFFu;
constexpr uint32_t kWriteTimeoutMs = 100u;
}

__attribute__((section(".noncacheable"), aligned(32)))
uint8_t STSPacketHandler::tx_buffer_[STSPacketHandler::MAX_PACKET_SIZE];

__attribute__((section(".noncacheable"), aligned(32)))
uint8_t STSPacketHandler::rx_buffer_[STSPacketHandler::MAX_DATA_LENGTH + 6u];

STSPacketHandler::STSPacketHandler(PortHandlerSTM32& port)
    : port_(port)
{
}

uint8_t STSPacketHandler::checksum(const uint8_t* data, size_t length)
{
    uint16_t sum = 0u;
    for (size_t i = 0u; i < length; ++i)
    {
        sum = static_cast<uint16_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(~sum);
}

void STSPacketHandler::poll()
{
    if (state_ == PacketState::Idle || state_ == PacketState::DataReady ||
        state_ == PacketState::Error)
    {
        return;
    }

    if (port_.hasError())
    {
        fail();
        return;
    }

    if (static_cast<uint32_t>(HAL_GetTick() - operation_start_ms_) > timeout_ms_)
    {
        fail();
        return;
    }

    if (state_ == PacketState::TxBusy && port_.isTxDone())
    {
        state_ = expect_status_ ? PacketState::WaitRx : PacketState::Idle;
    }

    if (state_ == PacketState::WaitRx && port_.isRxDone())
    {
        if (port_.completeReceive(rx_buffer_, rx_length_) && validateResponse())
        {
            state_ = PacketState::DataReady;
        }
        else
        {
            fail();
        }
    }
}

void STSPacketHandler::reset()
{
    port_.abortReceive();
    port_.clearError();
    state_ = PacketState::Idle;
    expect_status_ = false;
    response_length_ = 0u;
    tx_length_ = 0u;
    rx_length_ = 0u;
}

bool STSPacketHandler::startPing(uint8_t id, uint32_t timeout_ms)
{
    return startTransaction(id, INST_PING, nullptr, 0u, 0u, true, timeout_ms);
}

bool STSPacketHandler::startReadBytes(uint8_t id, uint8_t address, size_t length,
                                      uint32_t timeout_ms)
{
    if (length == 0u || length > MAX_DATA_LENGTH)
    {
        return false;
    }
    const uint8_t parameters[2] = {address, static_cast<uint8_t>(length)};
    return startTransaction(id, INST_READ, parameters, sizeof(parameters),
                            length, true, timeout_ms);
}

bool STSPacketHandler::startWriteBytes(uint8_t id, uint8_t address,
                                       const uint8_t* data, size_t length,
                                       bool expect_status, uint32_t timeout_ms)
{
    if (data == nullptr || length == 0u || length > (MAX_DATA_LENGTH - 1u))
    {
        return false;
    }

    uint8_t parameters[MAX_DATA_LENGTH] = {};
    parameters[0] = address;
    std::memcpy(parameters + 1u, data, length);
    return startTransaction(id, INST_WRITE, parameters, length + 1u, 0u,
                            expect_status, timeout_ms);
}

bool STSPacketHandler::startSyncWritePositions(const uint8_t* ids,
                                                const uint16_t* positions,
                                                const uint16_t* times_ms,
                                                size_t count)
{
    if (ids == nullptr || positions == nullptr || times_ms == nullptr ||
        count == 0u || count > 12u)
    {
        return false;
    }

    uint8_t parameters[2u + 12u * 5u] = {};
    parameters[0] = 0x2Au;
    parameters[1] = 4u;
    size_t offset = 2u;
    for (size_t i = 0u; i < count; ++i)
    {
        parameters[offset++] = ids[i];
        parameters[offset++] = static_cast<uint8_t>(positions[i] & 0xFFu);
        parameters[offset++] = static_cast<uint8_t>((positions[i] >> 8u) & 0xFFu);
        parameters[offset++] = static_cast<uint8_t>(times_ms[i] & 0xFFu);
        parameters[offset++] = static_cast<uint8_t>((times_ms[i] >> 8u) & 0xFFu);
    }

    return startTransaction(BROADCAST_ID, INST_SYNC_WRITE, parameters, offset,
                            0u, false, kWriteTimeoutMs);
}

bool STSPacketHandler::copyResponse(uint8_t* data, size_t length) const
{
    if (state_ != PacketState::DataReady || length != response_length_ ||
        (length != 0u && data == nullptr))
    {
        return false;
    }
    if (length != 0u)
    {
        std::memcpy(data, rx_buffer_ + 5u, length);
    }
    return true;
}

bool STSPacketHandler::startTransaction(uint8_t id, uint8_t instruction,
                                         const uint8_t* parameters,
                                         size_t parameter_count,
                                         size_t response_length,
                                         bool expect_status,
                                         uint32_t timeout_ms)
{
    if (state_ != PacketState::Idle || parameter_count > MAX_DATA_LENGTH ||
        response_length > MAX_DATA_LENGTH ||
        (parameter_count != 0u && parameters == nullptr))
    {
        return false;
    }

    const size_t packet_size = parameter_count + 6u;
    if (packet_size > MAX_PACKET_SIZE)
    {
        return false;
    }

    tx_buffer_[0] = kHeader;
    tx_buffer_[1] = kHeader;
    tx_buffer_[2] = id;
    tx_buffer_[3] = static_cast<uint8_t>(parameter_count + 2u);
    tx_buffer_[4] = instruction;
    if (parameter_count != 0u)
    {
        std::memcpy(tx_buffer_ + 5u, parameters, parameter_count);
    }
    tx_buffer_[packet_size - 1u] = checksum(tx_buffer_ + 2u, parameter_count + 3u);

    expected_id_ = id;
    response_length_ = static_cast<uint8_t>(response_length);
    expect_status_ = expect_status && id != BROADCAST_ID;
    tx_length_ = static_cast<uint16_t>(packet_size);
    rx_length_ = static_cast<uint16_t>(response_length + 6u);
    timeout_ms_ = timeout_ms == 0u ? 1u : timeout_ms;
    operation_start_ms_ = HAL_GetTick();
    port_.clearError();

    if (expect_status_ && !port_.receiveDMA(rx_buffer_, rx_length_))
    {
        reset();
        return false;
    }
    if (!port_.transmitDMA(tx_buffer_, tx_length_))
    {
        port_.abortReceive();
        reset();
        return false;
    }

    state_ = PacketState::TxBusy;
    return true;
}

bool STSPacketHandler::validateResponse()
{
    if (rx_buffer_[0] != kHeader || rx_buffer_[1] != kHeader ||
        rx_buffer_[2] != expected_id_ ||
        rx_buffer_[3] != static_cast<uint8_t>(response_length_ + 2u) ||
        rx_buffer_[4] != 0u)
    {
        return false;
    }

    const uint8_t received_checksum = rx_buffer_[5u + response_length_];
    return checksum(rx_buffer_ + 2u, response_length_ + 3u) == received_checksum;
}

void STSPacketHandler::fail()
{
    port_.abortTransmit();
    port_.abortReceive();
    state_ = PacketState::Error;
}

} // namespace feetech
