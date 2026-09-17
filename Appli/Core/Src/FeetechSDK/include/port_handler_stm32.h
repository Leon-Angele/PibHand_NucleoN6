#ifndef FEETECH_PORT_HANDLER_STM32_H_
#define FEETECH_PORT_HANDLER_STM32_H_

#include <stdint.h>

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

namespace feetech
{

class PortHandlerSTM32 final
{
public:
    static constexpr int DEFAULT_BAUDRATE = 1000000;

    explicit PortHandlerSTM32(UART_HandleTypeDef* uart_handle);
    ~PortHandlerSTM32();

    bool openPort();
    void closePort();
    void clearPort();

    bool transmitDMA(const uint8_t* packet, uint16_t length);
    bool receiveDMA(uint8_t* packet, uint16_t length);
    bool completeReceive(uint8_t* packet, uint16_t length);
    void abortTransmit();
    void abortReceive();

    bool isTxDone() const { return tx_done_; }
    bool isRxDone() const { return rx_done_; }
    bool hasError() const { return uart_error_; }
    void clearError() { uart_error_ = false; }
    int getBaudRate() const;

    static void handleTxComplete(UART_HandleTypeDef* uart_handle);
    static void handleRxComplete(UART_HandleTypeDef* uart_handle);
    static void handleUartError(UART_HandleTypeDef* uart_handle);

private:
    static constexpr uint8_t MAX_INSTANCES = 2u;
    static PortHandlerSTM32* instances_[MAX_INSTANCES];
    static uint8_t instance_count_;

    UART_HandleTypeDef* uart_handle_;
    volatile bool tx_done_ = true;
    volatile bool rx_done_ = false;
    volatile bool uart_error_ = false;
    bool is_open_ = false;
};

} // namespace feetech

#endif // FEETECH_PORT_HANDLER_STM32_H_
