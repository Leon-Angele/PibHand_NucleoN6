#include "hand/serial_commander.hpp"

/**
 * @file serial_commander.cpp
 * @brief ISR-safe ASCII command parser implementation for hand control.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Implements a small ring-buffered parser that accepts commands like
 * "G:<Side>:<GripID>" and forwards execution to a registered executor.
 */

#include <cstring>
#include <cctype>

using namespace HandControl;

// ============================================================================
// VCP UART PORT IMPLEMENTATION
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
// SERIAL COMMANDER IMPLEMENTATION
// ============================================================================

SerialCommander::SerialCommander(HandControl::ISerialPort& port) noexcept
: port_(port), executor_(nullptr), rx_head_(0), rx_tail_(0), overflow_flag_(false)
{
}

/**
 * @brief Construct a new SerialCommander.
 * @param port Reference to an ISerialPort used for responses/transmit
 */

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

/**
 * @brief Push a received byte into the ISR-safe ring buffer.
 * @param b Byte received from UART ISR
 * @return true if accepted, false if buffer full
 */

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

        // Parse command into structured Command
        ICommandExecutor::Command cmdObj;
        if (!parseCommand(reinterpret_cast<uint8_t*>(cmd), cmd_len, cmdObj)) {
            HAND_DEBUG("CMD ERROR: Invalid syntax");
            const char msg[] = "ERR SYNTAX\n";
            sendResponse(msg, sizeof(msg) - 1);
            continue;
        }

        if (!executor_) {
            const char msg[] = "ERR NOEXEC\n";
            sendResponse(msg, sizeof(msg) - 1);
            continue;
        }

        // Basic validation for GripID and percent ranges
        if (cmdObj.type == ICommandExecutor::CommandType::GripDefault ||
            cmdObj.type == ICommandExecutor::CommandType::GripGlobalPct ||
            cmdObj.type == ICommandExecutor::CommandType::GripPerFingerPct) {
            uint8_t gripCount = static_cast<uint8_t>(GripType::Count);
            uint8_t gripId = static_cast<uint8_t>(cmdObj.grip);
            if (gripId >= gripCount) {
                const char msg[] = "ERR GRIPID\n";
                sendResponse(msg, sizeof(msg) - 1);
                continue;
            }
            if (cmdObj.type == ICommandExecutor::CommandType::GripGlobalPct) {
                if (cmdObj.percent > 100) { const char msg[] = "ERR SPEED\n"; sendResponse(msg, sizeof(msg)-1); continue; }
            }
            if (cmdObj.type == ICommandExecutor::CommandType::GripPerFingerPct) {
                for (size_t i=0;i<cmdObj.perFingerPercent.size();++i) {
                    if (cmdObj.perFingerPercent[i] > 100) { const char msg[] = "ERR SPEED\n"; sendResponse(msg, sizeof(msg)-1); goto next_command; }
                }
            }
        }

        {
            bool ok = executor_->executeCommand(cmdObj);
            if (ok) {
                const char msg[] = "OK\n";
                sendResponse(msg, sizeof(msg) - 1);
            } else {
                const char msg[] = "ERR EXEC\n";
                sendResponse(msg, sizeof(msg) - 1);
            }
        }
next_command: ;
    }
}

/**
 * @brief Parse and execute complete ASCII commands from the buffer.
 *
 * Should be called from non-ISR context (main loop). Returns immediately
 * after processing available complete lines.
 */

void SerialCommander::sendResponse(const char* msg, size_t len) noexcept {
    if (len == 0 || !msg) return;
    (void) port_.transmitDMA(reinterpret_cast<const uint8_t*>(msg), static_cast<uint16_t>(len));
}

/**
 * @brief Send a textual response via the configured port.
 * @param msg Pointer to message bytes
 * @param len Length of message
 */

bool SerialCommander::parseCommand(const uint8_t* data, size_t len,
                                   ICommandExecutor::Command& outCmd) noexcept
{
    if (!data || len == 0) return false;

    // Copy to temporary null-terminated buffer for simple parsing
    char token[RX_BUF_SIZE];
    size_t copy_len = (len < (sizeof(token) - 1)) ? len : (sizeof(token) - 1);
    memcpy(token, data, copy_len);
    token[copy_len] = '\0';

    char* p = token;
    // Skip leading spaces
    while (*p && isspace((unsigned char)*p)) ++p;
    if (!*p) return false;

    // Extract first keyword up to ':'
    char* kw = p;
    while (*p && *p != ':') ++p;
    if (*p) { *p = '\0'; ++p; }

    // Normalize keyword to uppercase for comparisons
    auto equals_icase = [](const char* a, const char* b)->bool {
        while (*a && *b) {
            if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false;
            ++a; ++b;
        }
        return *a==0 && *b==0;
    };

    if (equals_icase(kw, "G")) {
        // Grip command: G:<Side>:<GripID>[:V:<percent>] or [:Vx:<v0>,...]
        if (!p || !*p) return false;
        // Side
        if (!(*p == '0' || *p == '1')) return false;
        outCmd.side = (*p == '0') ? Hand::Side::Left : Hand::Side::Right;
        ++p;
        if (*p != ':') return false;
        ++p;
        // GripID
        if (!isdigit((unsigned char)*p)) return false;
        int val = 0;
        while (*p && isdigit((unsigned char)*p)) { val = val*10 + (*p - '0'); ++p; if (val>255) break; }
        outCmd.grip = static_cast<GripType>(static_cast<uint8_t>(val));

        // Optional modifiers
        if (*p == ':') {
            ++p;
            // token could be V:... or Vx:...
            if (toupper((unsigned char)*p) == 'V') {
                ++p;
                if (*p == 'x' || *p == 'X') {
                    ++p; // Vx
                    if (*p != ':') return false;
                    ++p;
                    // parse 6 comma separated percents
                    for (size_t i=0;i<static_cast<size_t>(Finger::Count);++i) {
                        if (!isdigit((unsigned char)*p)) return false;
                        int num = 0;
                        while (*p && isdigit((unsigned char)*p)) { num = num*10 + (*p - '0'); ++p; if (num>1000) break; }
                        outCmd.perFingerPercent[i] = static_cast<uint16_t>(num);
                        if (i < static_cast<size_t>(Finger::Count)-1) {
                            if (*p != ',') return false;
                            ++p;
                        }
                    }
                    outCmd.type = ICommandExecutor::CommandType::GripPerFingerPct;
                } else {
                    if (*p != ':') return false;
                    ++p;
                    // global percent
                    if (!isdigit((unsigned char)*p)) return false;
                    int num = 0;
                    while (*p && isdigit((unsigned char)*p)) { num = num*10 + (*p - '0'); ++p; if (num>1000) break; }
                    outCmd.percent = static_cast<uint16_t>(num);
                    outCmd.type = ICommandExecutor::CommandType::GripGlobalPct;
                }
            } else {
                // Unknown modifier - reject
                return false;
            }
        } else {
            outCmd.type = ICommandExecutor::CommandType::GripDefault;
        }
        return true;
    }

    if (equals_icase(kw, "F")) {
        // Single finger: F:<Side>:<Finger>:<Pos>[:<Speed>]
        if (!p || !*p) return false;
        if (!(*p == '0' || *p == '1')) return false;
        outCmd.side = (*p == '0') ? Hand::Side::Left : Hand::Side::Right;
        ++p;
        if (*p != ':') return false;
        ++p;
        if (!isdigit((unsigned char)*p)) return false;
        int f = 0;
        while (*p && isdigit((unsigned char)*p)) { f = f*10 + (*p - '0'); ++p; if (f>255) break; }
        outCmd.finger = static_cast<Finger>(static_cast<uint8_t>(f));
        if (*p != ':') return false;
        ++p;
        // Pos
        if (!isdigit((unsigned char)*p)) return false;
        int pos = 0;
        while (*p && isdigit((unsigned char)*p)) { pos = pos*10 + (*p - '0'); ++p; if (pos>10000) break; }
        outCmd.position = static_cast<uint16_t>(pos);
        // Optional :speed
        if (*p == ':') { ++p; if (!isdigit((unsigned char)*p)) return false; int sp=0; while (*p && isdigit((unsigned char)*p)) { sp = sp*10 + (*p - '0'); ++p; if (sp>10000) break; } outCmd.speed_deg_per_s = static_cast<uint16_t>(sp); }
        outCmd.type = ICommandExecutor::CommandType::SingleFinger;
        return true;
    }

    if (equals_icase(kw, "STOP") || equals_icase(kw, "HOLD")) {
        if (!p || !*p) return false;
        // expect side next
        if (!(*p == '0' || *p == '1')) return false;
        outCmd.side = (*p == '0') ? Hand::Side::Left : Hand::Side::Right;
        outCmd.type = equals_icase(kw, "STOP") ? ICommandExecutor::CommandType::Stop : ICommandExecutor::CommandType::Hold;
        return true;
    }

    if (equals_icase(kw, "GET")) {
        // expect :STATUS
        if (!p || !*p) return false;
        char* sub = p;
        if (equals_icase(sub, "STATUS")) {
            outCmd.type = ICommandExecutor::CommandType::GetStatus;
            return true;
        }
    }

    // Unknown command
    return false;
}

/**
 * @brief Parse a grip command of the form "G:<Side>:<GripID>".
 * @param data Input bytes
 * @param len Length of input
 * @param outSide Parsed Hand::Side
 * @param outGripId Parsed numeric Grip ID
 * @return true on success, false on parse error
 */
