#include "hand/as5600.hpp"

namespace {

constexpr uint8_t AS5600_ADDR_7BIT = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE_MSB_REG = 0x0C;
constexpr uint16_t AS5600_RAW_MAX = 4095;

enum class DriverState : uint8_t {
    IDLE = 0,
    SELECT_CHANNEL_TX,
    READ_ANGLE_RX
};

struct EncoderState {
    uint16_t raw_angle;
    uint32_t error_count;
    uint32_t last_update_ms;
    bool online;
    bool fresh;
};

static I2C_HandleTypeDef* g_hi2c = nullptr;
static uint8_t g_tca_addr_7bit = 0x70;
static uint8_t g_channel_map[AS5600_ENCODER_COUNT] = {0, 1, 2};
static EncoderState g_encoders[AS5600_ENCODER_COUNT] = {};

static DriverState g_state = DriverState::IDLE;
static uint8_t g_next_encoder_idx = 0;
static uint8_t g_pending_encoder_idx = 0;
static bool g_initialized = false;
static bool g_polling_enabled = false;
static uint32_t g_poll_period_ms = 10;
static uint32_t g_next_poll_due_ms = 0;

__attribute__((section(".noncacheable"), aligned(32)))
static uint8_t g_tca_select_byte;

__attribute__((section(".noncacheable"), aligned(32)))
static uint8_t g_as5600_rx_buf[2];

static bool is_valid_encoder(as5600_encoder_t encoder)
{
    return (encoder >= AS5600_ENCODER_BASE) && (encoder < AS5600_ENCODER_COUNT);
}

static void mark_transfer_error(uint8_t encoder_idx)
{
    g_encoders[encoder_idx].error_count++;
    g_encoders[encoder_idx].online = false;
    g_encoders[encoder_idx].fresh = false;
    g_next_encoder_idx = (encoder_idx + 1U) % AS5600_ENCODER_COUNT;
    g_state = DriverState::IDLE;
    g_next_poll_due_ms = HAL_GetTick() + g_poll_period_ms;
}

static void mark_transfer_success(uint8_t encoder_idx)
{
    const uint16_t raw_angle = static_cast<uint16_t>(((g_as5600_rx_buf[0] & 0x0FU) << 8U) | g_as5600_rx_buf[1]);
    g_encoders[encoder_idx].raw_angle = raw_angle;
    g_encoders[encoder_idx].online = true;
    g_encoders[encoder_idx].fresh = true;
    g_encoders[encoder_idx].last_update_ms = HAL_GetTick();
    g_next_encoder_idx = (encoder_idx + 1U) % AS5600_ENCODER_COUNT;
    g_state = DriverState::IDLE;
    g_next_poll_due_ms = HAL_GetTick() + g_poll_period_ms;
}

} // namespace

extern "C" {

bool as5600_init(I2C_HandleTypeDef* hi2c, uint8_t tca_addr_7bit)
{
    if (hi2c == nullptr) {
        return false;
    }

    g_hi2c = hi2c;
    g_tca_addr_7bit = tca_addr_7bit;
    g_state = DriverState::IDLE;
    g_next_encoder_idx = 0;
    g_pending_encoder_idx = 0;
    g_poll_period_ms = 10;
    g_next_poll_due_ms = HAL_GetTick();
    g_polling_enabled = false;

    for (uint8_t index = 0; index < AS5600_ENCODER_COUNT; ++index) {
        g_encoders[index].raw_angle = 0;
        g_encoders[index].error_count = 0;
        g_encoders[index].last_update_ms = 0;
        g_encoders[index].online = false;
        g_encoders[index].fresh = false;
    }

    g_initialized = true;
    return true;
}

void as5600_set_channel_map(uint8_t base_channel, uint8_t middle_channel, uint8_t tip_channel)
{
    g_channel_map[AS5600_ENCODER_BASE] = base_channel & 0x07U;
    g_channel_map[AS5600_ENCODER_MIDDLE] = middle_channel & 0x07U;
    g_channel_map[AS5600_ENCODER_TIP] = tip_channel & 0x07U;
}

void as5600_start_polling(uint32_t period_ms)
{
    if (!g_initialized) {
        return;
    }

    if (period_ms == 0U) {
        period_ms = 10U;
    }

    g_poll_period_ms = period_ms;
    g_next_poll_due_ms = HAL_GetTick();
    g_polling_enabled = true;
}

void as5600_stop_polling(void)
{
    g_polling_enabled = false;
}

void as5600_update(void)
{
    if (!g_initialized || !g_polling_enabled || (g_hi2c == nullptr)) {
        return;
    }

    if (g_state != DriverState::IDLE) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_next_poll_due_ms) < 0) {
        return;
    }

    g_pending_encoder_idx = g_next_encoder_idx;
    g_tca_select_byte = static_cast<uint8_t>(1U << g_channel_map[g_pending_encoder_idx]);

    const HAL_StatusTypeDef start_status = HAL_I2C_Master_Transmit_DMA(
        g_hi2c,
        static_cast<uint16_t>(g_tca_addr_7bit << 1U),
        &g_tca_select_byte,
        1U);

    if (start_status != HAL_OK) {
        mark_transfer_error(g_pending_encoder_idx);
        return;
    }

    g_state = DriverState::SELECT_CHANNEL_TX;
}

bool as5600_get_raw_angle(as5600_encoder_t encoder, uint16_t* raw_angle)
{
    if (!is_valid_encoder(encoder) || (raw_angle == nullptr)) {
        return false;
    }

    *raw_angle = g_encoders[encoder].raw_angle;
    g_encoders[encoder].fresh = false;
    return g_encoders[encoder].online;
}

bool as5600_get_degrees(as5600_encoder_t encoder, float* degrees)
{
    if (!is_valid_encoder(encoder) || (degrees == nullptr)) {
        return false;
    }

    *degrees = (static_cast<float>(g_encoders[encoder].raw_angle) * 360.0f) / static_cast<float>(AS5600_RAW_MAX + 1U);
    g_encoders[encoder].fresh = false;
    return g_encoders[encoder].online;
}

bool as5600_is_online(as5600_encoder_t encoder)
{
    if (!is_valid_encoder(encoder)) {
        return false;
    }

    return g_encoders[encoder].online;
}

uint32_t as5600_get_error_count(as5600_encoder_t encoder)
{
    if (!is_valid_encoder(encoder)) {
        return 0;
    }

    return g_encoders[encoder].error_count;
}

bool as5600_has_fresh_sample(as5600_encoder_t encoder)
{
    if (!is_valid_encoder(encoder)) {
        return false;
    }

    return g_encoders[encoder].fresh;
}

void as5600_on_i2c_master_tx_cplt(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != g_hi2c) || (g_state != DriverState::SELECT_CHANNEL_TX)) {
        return;
    }

    const HAL_StatusTypeDef read_status = HAL_I2C_Mem_Read_DMA(
        g_hi2c,
        static_cast<uint16_t>(AS5600_ADDR_7BIT << 1U),
        AS5600_RAW_ANGLE_MSB_REG,
        I2C_MEMADD_SIZE_8BIT,
        g_as5600_rx_buf,
        2U);

    if (read_status != HAL_OK) {
        mark_transfer_error(g_pending_encoder_idx);
        return;
    }

    g_state = DriverState::READ_ANGLE_RX;
}

void as5600_on_i2c_mem_rx_cplt(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != g_hi2c) || (g_state != DriverState::READ_ANGLE_RX)) {
        return;
    }

    mark_transfer_success(g_pending_encoder_idx);
}

void as5600_on_i2c_error(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != g_hi2c) || (g_state == DriverState::IDLE)) {
        return;
    }

    mark_transfer_error(g_pending_encoder_idx);
}

} // extern "C"