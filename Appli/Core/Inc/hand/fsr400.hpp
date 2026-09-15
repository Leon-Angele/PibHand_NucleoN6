#ifndef FSR400_HPP
#define FSR400_HPP

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSR400_SENSOR_COUNT 5U

typedef struct {
    uint32_t sequence;
    uint16_t raw[FSR400_SENSOR_COUNT];
    float filtered[FSR400_SENSOR_COUNT];
    float voltage[FSR400_SENSOR_COUNT];
    float force_newton[FSR400_SENSOR_COUNT];
    bool tared;
    bool saturated;
} FSR_Snapshot;

bool FSR_Start(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);
void FSR_Update(void);
bool FSR_Tare(void);
bool FSR_IsTared(void);
void FSR_GetSnapshot(FSR_Snapshot *snapshot);
uint16_t FSR_GetRaw(uint8_t index);
float FSR_GetVoltage(uint8_t index);
float FSR_GetForceNewton(uint8_t index);
void FSR_SetFilterAlpha(float alpha);
float FSR_GetFilterAlpha(void);

#ifdef __cplusplus
}
#endif

#endif /* FSR400_HPP */
