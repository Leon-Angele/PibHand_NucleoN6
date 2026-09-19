#include "hand/serial_commander.hpp"

#include <cctype>
#include <cstdlib>
#include <cstring>

using namespace HandControl;

namespace {

bool equalsIgnoreCase(const char* a, const char* b)
{
    while (*a != '\0' && *b != '\0') {
        if (std::toupper(static_cast<unsigned char>(*a)) !=
            std::toupper(static_cast<unsigned char>(*b))) return false;
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

bool parseUnsigned(const char* text, unsigned long& value)
{
    if (text == nullptr || *text == '\0') return false;
    char* end = nullptr;
    value = std::strtoul(text, &end, 10);
    return end != text && *end == '\0';
}

bool parseFloat(const char* text, float& value)
{
    if (text == nullptr || *text == '\0') return false;
    char* end = nullptr;
    value = std::strtof(text, &end);
    return end != text && *end == '\0';
}

size_t split(char* text, char* tokens[], size_t max_tokens)
{
    size_t count = 0;
    char* cursor = text;
    while (count < max_tokens) {
        tokens[count++] = cursor;
        char* separator = std::strchr(cursor, ':');
        if (separator == nullptr) break;
        *separator = '\0';
        cursor = separator + 1;
    }
    return count;
}

} // namespace

PollUartPort::PollUartPort(UART_HandleTypeDef* huart, uint32_t timeout_ms)
    : huart_(huart), timeout_ms_(timeout_ms)
{
}

bool PollUartPort::transmitDMA(const uint8_t* data, uint16_t length)
{
    if (data == nullptr || length == 0U || huart_ == nullptr) return false;
    return HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), length, timeout_ms_) == HAL_OK;
}

SerialCommander::SerialCommander(ISerialPort& port) noexcept
    : port_(port)
{
}

bool SerialCommander::feedByte(uint8_t byte) noexcept
{
    const uint16_t head = rx_head_;
    const uint16_t next = static_cast<uint16_t>((head + 1U) % RX_BUF_SIZE);
    if (next == rx_tail_) {
        overflow_flag_ = true;
        return false;
    }
    rx_buf_[head] = byte;
    rx_head_ = next;
    return true;
}

void SerialCommander::sendText(const char* text) noexcept
{
    if (text == nullptr) return;
    sendResponse(text, std::strlen(text));
}

void SerialCommander::sendResponse(const char* msg, size_t len) noexcept
{
    if (msg == nullptr || len == 0U) return;
    if (len >= TX_MESSAGE_SIZE) len = TX_MESSAGE_SIZE - 1U;

    const uint8_t next = static_cast<uint8_t>((tx_head_ + 1U) % TX_QUEUE_DEPTH);
    if (next == tx_tail_) {
        if (error_callback_ != nullptr) error_callback_();
        return;
    }
    std::memcpy(tx_queue_[tx_head_].data(), msg, len);
    tx_lengths_[tx_head_] = static_cast<uint16_t>(len);
    tx_head_ = next;
}

void SerialCommander::serviceTx() noexcept
{
    if (tx_active_) {
        if (!port_.isTxDone()) return;
        tx_active_ = false;
    }
    if (tx_tail_ == tx_head_) return;

    const uint16_t length = tx_lengths_[tx_tail_];
    std::memcpy(tx_dma_buffer_.data(), tx_queue_[tx_tail_].data(), length);
    tx_tail_ = static_cast<uint8_t>((tx_tail_ + 1U) % TX_QUEUE_DEPTH);
    if (port_.transmitDMA(tx_dma_buffer_.data(), length)) {
        tx_active_ = true;
    } else if (error_callback_ != nullptr) {
        error_callback_();
    }
}

void SerialCommander::completeDeferred(bool success) noexcept
{
    if (!command_deferred_) return;
    const char* response = success ? "OK\n" : "ERR:EXEC\n";
    if (!success && error_callback_ != nullptr) error_callback_();
    sendResponse(response, std::strlen(response));
    command_deferred_ = false;
}

void SerialCommander::processCommand() noexcept
{
    serviceTx();

    if (command_deferred_) return;

    if (overflow_flag_) {
        const char response[] = "ERR:QUEUE_FULL\n";
        sendResponse(response, sizeof(response) - 1U);
        if (error_callback_ != nullptr) error_callback_();
        rx_tail_ = rx_head_;
        overflow_flag_ = false;
        serviceTx();
        return;
    }

    while (rx_tail_ != rx_head_) {
        const uint16_t tail = rx_tail_;
        const uint16_t head = rx_head_;
        size_t available = head >= tail ? head - tail : RX_BUF_SIZE - (tail - head);
        char command[MAX_COMMAND_LENGTH]{};
        bool found_terminator = false;
        size_t length = 0;
        while (length < available && length < MAX_COMMAND_LENGTH - 1U) {
            const char c = static_cast<char>(rx_buf_[(tail + length) % RX_BUF_SIZE]);
            ++length;
            if (c == '\n' || c == '\r') {
                found_terminator = true;
                break;
            }
            command[length - 1U] = c;
        }
        if (!found_terminator) {
            if (available >= MAX_COMMAND_LENGTH - 1U) {
                rx_tail_ = static_cast<uint16_t>((tail + available) % RX_BUF_SIZE);
                const char response[] = "ERR:SYNTAX\n";
                sendResponse(response, sizeof(response) - 1U);
            }
            break;
        }

        rx_tail_ = static_cast<uint16_t>((tail + length) % RX_BUF_SIZE);
        while (rx_tail_ != rx_head_) {
            const char next = static_cast<char>(rx_buf_[rx_tail_]);
            if (next != '\r' && next != '\n') break;
            rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % RX_BUF_SIZE);
        }
        if (length == 1U) continue;

        ICommandExecutor::Command parsed;
        if (parseCommand(reinterpret_cast<const uint8_t*>(command), length - 1U, parsed) && executor_) {
            const ICommandExecutor::Result result = executor_->executeCommand(parsed);
            if (result == ICommandExecutor::Result::Deferred) {
                command_deferred_ = true;
                break;
            } else {
                const char* response = result == ICommandExecutor::Result::Success
                                     ? "OK\n" : "ERR:EXEC\n";
                if (result == ICommandExecutor::Result::Failure && error_callback_ != nullptr) {
                    error_callback_();
                }
                sendResponse(response, std::strlen(response));
            }
        } else if (!executor_) {
            const char response[] = "ERR:NOEXEC\n";
            if (error_callback_ != nullptr) error_callback_();
            sendResponse(response, sizeof(response) - 1U);
        } else {
            const char response[] = "ERR:SYNTAX\n";
            if (error_callback_ != nullptr) error_callback_();
            sendResponse(response, sizeof(response) - 1U);
        }
    }
    serviceTx();
}

bool SerialCommander::parseCommand(const uint8_t* data, size_t len,
                                   ICommandExecutor::Command& out) noexcept
{
    if (data == nullptr || len == 0U || len >= MAX_COMMAND_LENGTH) return false;
    char buffer[MAX_COMMAND_LENGTH]{};
    std::memcpy(buffer, data, len);
    buffer[len] = '\0';

    char* tokens[5]{};
    const size_t count = split(buffer, tokens, 5U);
    unsigned long integer = 0;
    float value = 0.0f;

    if (equalsIgnoreCase(tokens[0], "POSE")) {
        if ((count != 2U && count != 3U) || !parseUnsigned(tokens[1], integer) || integer >= static_cast<unsigned long>(GripType::Count)) return false;
        out.grip = static_cast<GripType>(integer);
        out.type = ICommandExecutor::CommandType::Pose;
        if (count == 3U) {
            if (!parseFloat(tokens[2], value) || value < 0.0f || value > DEFAULT_FORCE_LIMIT_N) return false;
            out.force_newton = value;
            out.has_force = true;
        }
        return true;
    }

    if (equalsIgnoreCase(tokens[0], "POS")) {
        if ((count != 3U && count != 4U) || !parseUnsigned(tokens[1], integer) || integer >= FINGER_COUNT ||
            !parseFloat(tokens[2], value) || value < 0.0f || value > 100.0f) return false;
        out.finger = static_cast<Finger>(integer);
        out.position_percent = value;
        out.type = ICommandExecutor::CommandType::SinglePosition;
        if (count == 4U) {
            if (!parseFloat(tokens[3], out.force_newton) || out.force_newton < 0.0f || out.force_newton > DEFAULT_FORCE_LIMIT_N) return false;
            out.has_force = true;
        }
        return true;
    }

    if (equalsIgnoreCase(tokens[0], "FORCE")) {
        if (count != 3U || !parseFloat(tokens[2], value) || value < 0.0f || value > DEFAULT_FORCE_LIMIT_N) return false;
        out.force_newton = value;
        if (equalsIgnoreCase(tokens[1], "ALL")) {
            out.type = ICommandExecutor::CommandType::ForceAll;
            return true;
        }
        if (!parseUnsigned(tokens[1], integer) || integer < CONTROLLED_FINGER_FIRST || integer > CONTROLLED_FINGER_LAST) return false;
        out.finger = static_cast<Finger>(integer);
        out.type = ICommandExecutor::CommandType::ForceFinger;
        return true;
    }

    if (equalsIgnoreCase(tokens[0], "ADM")) {
        if (count != 2U) return false;
        if (equalsIgnoreCase(tokens[1], "ON")) out.type = ICommandExecutor::CommandType::AdmittanceOn;
        else if (equalsIgnoreCase(tokens[1], "OFF")) out.type = ICommandExecutor::CommandType::AdmittanceOff;
        else return false;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "FSR")) {
        if (count != 2U || !equalsIgnoreCase(tokens[1], "TARE")) return false;
        out.type = ICommandExecutor::CommandType::FsrTare;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "SPEED")) {
        if (count != 2U || !parseUnsigned(tokens[1], integer) || integer < 1U || integer > 270U) return false;
        out.speed_deg_per_s = static_cast<uint16_t>(integer);
        out.type = ICommandExecutor::CommandType::Speed;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "TORQUE")) {
        if (count != 2U || !parseUnsigned(tokens[1], integer) || integer > 100U) return false;
        out.torque_percent = static_cast<uint16_t>(integer);
        out.type = ICommandExecutor::CommandType::Torque;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "STATUS?")) {
        if (count != 1U) return false;
        out.type = ICommandExecutor::CommandType::GetStatus;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "STATUS")) {
        if (count != 3U || !equalsIgnoreCase(tokens[1], "STREAM") ||
            !parseUnsigned(tokens[2], integer) || integer > 20U) return false;
        out.status_rate_hz = static_cast<uint8_t>(integer);
        out.type = ICommandExecutor::CommandType::StatusStream;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "STOP")) {
        if (count != 1U) return false;
        out.type = ICommandExecutor::CommandType::Stop;
        return true;
    }
    if (equalsIgnoreCase(tokens[0], "HOLD")) {
        if (count != 1U) return false;
        out.type = ICommandExecutor::CommandType::Hold;
        return true;
    }
    return false;
}
