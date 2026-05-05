#include "hand/serial_commander.hpp"
#include <cstring>
#include <cctype>

using namespace HandControl;

SerialCommander::SerialCommander(ISerialPort& port) noexcept
: port_(port), executor_(nullptr), rx_head_(0), rx_tail_(0), overflow_flag_(false)
{
}

bool SerialCommander::feedByte(uint8_t b) noexcept {
    // ISR-safe ring buffer push (single producer from ISR)
    uint16_t head = rx_head_;
    uint16_t next = (head + 1) % RX_BUF_SIZE;
    uint16_t tail = rx_tail_; // volatile read
    if (next == tail) {
        overflow_flag_ = true; // mark overflow; processCommand will report/clear
        return false;
    }
    rx_buf_[head] = b;
    rx_head_ = next; // volatile write
    return true;
}

void SerialCommander::processCommand() noexcept {
    // If overflow, report and clear buffer
    if (overflow_flag_) {
        const char msg[] = "ERR BUF\n";
        HAND_DEBUG("ERROR: RX Buffer overflow!");
        sendResponse(msg, sizeof(msg) - 1);
        // Drop data: advance tail to head
        rx_tail_ = rx_head_;
        overflow_flag_ = false;
        return;
    }

    // Extract commands delimited by '\n' or '\r'
    while (rx_tail_ != rx_head_) {
        size_t available = 0;
        uint16_t tail = rx_tail_;
        uint16_t head = rx_head_;
        if (head >= tail) available = head - tail;
        else available = RX_BUF_SIZE - (tail - head);

        // Copy up to available or until newline (accept both '\n' and '\r' as line terminator)
        char cmd[RX_BUF_SIZE];
        bool found_newline = false;
        size_t i = 0;
        for (; i < available && i < (RX_BUF_SIZE - 1); ++i) {
            uint16_t idx = (tail + (uint16_t)i) % RX_BUF_SIZE;
            char c = static_cast<char>(rx_buf_[idx]);
            cmd[i] = c;
            if (c == '\n' || c == '\r') { found_newline = true; ++i; break; }
        }

        if (!found_newline) break; // wait for full line

        size_t cmd_len = i;
        cmd[cmd_len] = '\0';

        // Advance tail by cmd_len
        rx_tail_ = (tail + static_cast<uint16_t>(cmd_len)) % RX_BUF_SIZE;

        // Trim trailing CR/LF
        while (cmd_len > 0 && (cmd[cmd_len - 1] == '\n' || cmd[cmd_len - 1] == '\r')) {
            cmd[--cmd_len] = '\0';
        }

        // Parse command: expected "G:<Side>:<GripID>"
        Hand::Side side;
        uint8_t gripId = 0;
        if (!parseGripCommand(reinterpret_cast<uint8_t*>(cmd), cmd_len, side, gripId)) {
            HAND_DEBUG("CMD ERROR: Invalid syntax or unknown GripID");
            const char msg[] = "ERR SYNTAX\n";
            sendResponse(msg, sizeof(msg) - 1);
            continue;
        }

        // Validate gripId
        uint8_t gripCount = static_cast<uint8_t>(GripType::Count);
        if (gripId >= gripCount) {
            HAND_DEBUG("CMD ERROR: Invalid GripID");
            const char msg[] = "ERR GRIPID\n";
            sendResponse(msg, sizeof(msg) - 1);
            continue;
        }

        if (!executor_) {
            const char msg[] = "ERR NOEXEC\n";
            sendResponse(msg, sizeof(msg) - 1);
            continue;
        }

        bool ok = executor_->executeGrip(side, static_cast<GripType>(gripId));
        if (ok) {
            HAND_DEBUG("CMD OK -> Side: %d, GripID: %d", static_cast<int>(side), static_cast<int>(gripId));
            const char msg[] = "OK\n";
            sendResponse(msg, sizeof(msg) - 1);
        } else {
            const char msg[] = "ERR EXEC\n";
            sendResponse(msg, sizeof(msg) - 1);
        }
    }
}

void SerialCommander::sendResponse(const char* msg, size_t len) noexcept {
    if (len == 0 || !msg) return;
    (void) port_.transmitDMA(reinterpret_cast<const uint8_t*>(msg), static_cast<uint16_t>(len));
}

bool SerialCommander::parseGripCommand(const uint8_t* data, size_t len,
                                       Hand::Side& outSide, uint8_t& outGripId) noexcept
{
    if (!data || len == 0) return false;

    // Copy to temporary null-terminated buffer for simple parsing
    char token[RX_BUF_SIZE];
    size_t copy_len = (len < (sizeof(token) - 1)) ? len : (sizeof(token) - 1);
    memcpy(token, data, copy_len);
    token[copy_len] = '\0';

    // Expected format: G:<Side>:<GripID>
    char* p = token;
    // Skip leading spaces
    while (*p && isspace((unsigned char)*p)) ++p;
    if (*p != 'G' && *p != 'g') return false;
    ++p;
    if (*p != ':') return false;
    ++p;

    // Side must be '0' or '1'
    if (!(*p == '0' || *p == '1')) return false;
    outSide = (*p == '0') ? Hand::Side::Left : Hand::Side::Right;
    ++p;
    if (*p != ':') return false;
    ++p;

    // Parse numeric GripID
    if (!isdigit((unsigned char)*p)) return false;
    int val = 0;
    while (*p && isdigit((unsigned char)*p)) {
        val = val * 10 + (*p - '0');
        ++p;
        if (val > 255) break;
    }
    if (val < 0) return false;
    outGripId = static_cast<uint8_t>(val);
    return true;
}
