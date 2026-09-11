#ifndef FSR400_HPP
#define FSR400_HPP

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSR400_SENSOR_COUNT 5U

/**
 * @brief Start the FSR400 ADC/DMA acquisition and its TIM6 trigger.
 *
 * ADC DMA is started before TIM6 so that the first trigger can be handled
 * immediately. TIM6 runs with its update interrupt enabled for the hand
 * control tick. The DMA destination is owned by the FSR400 module.
 */
bool FSR_Start(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

/**
 * @brief Process one complete ADC sequence.
 *
 * This function is intended to be called from HAL_ADC_ConvCpltCallback().
 * It performs only five IIR updates and never blocks.
 */
void FSR_Update(void);

/**
 * @brief Return the latest unfiltered ADC sample for one sensor.
 * @param index Sensor index in the range 0..4.
 */
uint16_t FSR_GetRaw(uint8_t index);

/**
 * @brief Return the filtered ADC sample converted to volts.
 * @param index Sensor index in the range 0..4.
 */
float FSR_GetVoltage(uint8_t index);

/**
 * @brief Configure the IIR coefficient.
 * @param alpha Filter coefficient in the range 0.0..1.0.
 */
void FSR_SetFilterAlpha(float alpha);

/**
 * @brief Return the currently configured IIR coefficient.
 */
float FSR_GetFilterAlpha(void);

#ifdef __cplusplus
}
#endif

#endif /* FSR400_HPP */
