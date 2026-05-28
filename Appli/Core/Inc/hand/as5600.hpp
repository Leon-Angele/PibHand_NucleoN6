#ifndef AS5600_HPP
#define AS5600_HPP

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AS5600_ENCODER_BASE = 0,
    AS5600_ENCODER_MIDDLE = 1,
    AS5600_ENCODER_TIP = 2,
    AS5600_ENCODER_COUNT = 3
} as5600_encoder_t;

bool as5600_init(I2C_HandleTypeDef* hi2c, uint8_t tca_addr_7bit);
void as5600_set_channel_map(uint8_t base_channel, uint8_t middle_channel, uint8_t tip_channel);
void as5600_start_polling(uint32_t period_ms);
void as5600_stop_polling(void);
void as5600_update(void);

bool as5600_get_raw_angle(as5600_encoder_t encoder, uint16_t* raw_angle);
bool as5600_get_degrees(as5600_encoder_t encoder, float* degrees);
bool as5600_is_online(as5600_encoder_t encoder);
uint32_t as5600_get_error_count(as5600_encoder_t encoder);
bool as5600_has_fresh_sample(as5600_encoder_t encoder);

void as5600_on_i2c_master_tx_cplt(I2C_HandleTypeDef* hi2c);
void as5600_on_i2c_mem_rx_cplt(I2C_HandleTypeDef* hi2c);
void as5600_on_i2c_error(I2C_HandleTypeDef* hi2c);

#ifdef __cplusplus
}
#endif

#endif