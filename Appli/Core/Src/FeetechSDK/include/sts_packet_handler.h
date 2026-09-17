#ifndef FEETECH_STS_PACKET_HANDLER_H_
#define FEETECH_STS_PACKET_HANDLER_H_

#include <stddef.h>
#include <stdint.h>

#include "port_handler_stm32.h"

namespace feetech
{

enum class PacketState : uint8_t
{
    Idle,
    TxBusy,
    WaitRx,
    DataReady,
    Error,
};

class STSPacketHandler final
{
public:
    static constexpr uint8_t BROADCAST_ID = 0xFEu;
    static constexpr uint8_t INST_PING = 0x01u;
    static constexpr uint8_t INST_READ = 0x02u;
    static constexpr uint8_t INST_WRITE = 0x03u;
    static constexpr uint8_t INST_SYNC_WRITE = 0x83u;
    static constexpr size_t MAX_DATA_LENGTH = 64u;
    static constexpr size_t MAX_PACKET_SIZE = 128u;

    explicit STSPacketHandler(PortHandlerSTM32& port);

    PacketState state() const { return state_; }
    void poll();
    void reset();

    bool startPing(uint8_t id, uint32_t timeout_ms);
    bool startReadBytes(uint8_t id, uint8_t address, size_t length,
                        uint32_t timeout_ms = 10u);
    bool startWriteBytes(uint8_t id, uint8_t address, const uint8_t* data,
                         size_t length, bool expect_status = false,
                         uint32_t timeout_ms = 10u);
    bool startSyncWritePositions(const uint8_t* ids,
                                 const uint16_t* positions,
                                 const uint16_t* times_ms,
                                 size_t count);
    bool copyResponse(uint8_t* data, size_t length) const;

    static uint8_t checksum(const uint8_t* data, size_t length);

private:
    bool startTransaction(uint8_t id, uint8_t instruction,
                          const uint8_t* parameters, size_t parameter_count,
                          size_t response_length, bool expect_status,
                          uint32_t timeout_ms);
    bool validateResponse();
    void fail();

    PortHandlerSTM32& port_;
    PacketState state_ = PacketState::Idle;
    uint32_t operation_start_ms_ = 0u;
    uint32_t timeout_ms_ = 0u;
    uint16_t tx_length_ = 0u;
    uint16_t rx_length_ = 0u;
    uint8_t expected_id_ = 0u;
    uint8_t response_length_ = 0u;
    bool expect_status_ = false;

    alignas(32) static uint8_t tx_buffer_[MAX_PACKET_SIZE];
    alignas(32) static uint8_t rx_buffer_[MAX_DATA_LENGTH + 6u];
};

} // namespace feetech

#endif // FEETECH_STS_PACKET_HANDLER_H_
