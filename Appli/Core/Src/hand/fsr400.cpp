#include "hand/fsr400.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr uint16_t FSR_ADC_MAX = 4095U;
constexpr float FSR_VREF = 3.3f;
constexpr float FSR_MEASURING_RESISTOR_OHM = 47000.0f;
constexpr float FSR_DEFAULT_FILTER_ALPHA = 0.2f;
constexpr float FSR_MAX_N = 20.0f;

__attribute__((section(".noncacheable"), aligned(32)))
static volatile uint32_t fsr_raw[FSR400_SENSOR_COUNT] = {};
volatile HAL_StatusTypeDef fsr_dma_last_stop_status = HAL_OK;
volatile HAL_StatusTypeDef fsr_dma_last_start_status = HAL_OK;

static volatile float fsr_filtered[FSR400_SENSOR_COUNT] = {};
static volatile float fsr_force[FSR400_SENSOR_COUNT] = {};
static float fsr_tare_conductance_us[FSR400_SENSOR_COUNT] = {};
static volatile float fsr_filter_alpha = FSR_DEFAULT_FILTER_ALPHA;
static volatile uint32_t fsr_sequence = 0U;
static volatile bool fsr_filter_initialized = false;
static volatile bool fsr_tared = false;
static volatile bool fsr_saturated = false;
static uint16_t startup_tare_samples = 0U;
static float startup_tare_sum[FSR400_SENSOR_COUNT] = {};

bool is_valid_index(uint8_t index)
{
    return index < FSR400_SENSOR_COUNT;
}

float clamp_alpha(float alpha)
{
    return std::clamp(alpha, 0.0f, 1.0f);
}

float adc_to_conductance_us(float adc)
{
    if (adc <= 0.0f) return 0.0f;
    if (adc >= static_cast<float>(FSR_ADC_MAX)) return 1000000.0f;

    /* FSR is connected to 3V3 and the 47 kOhm resistor to ground. */
    const float resistance = FSR_MEASURING_RESISTOR_OHM *
                             (static_cast<float>(FSR_ADC_MAX) - adc) / adc;
    if (resistance <= 0.0f) return 1000000.0f;
    return 1000000.0f / resistance;
}

float effective_conductance_to_force(float conductance_us)
{
    /* Central approximation of the FSR400 datasheet curve. */
    constexpr float conductance[] = {0.0f, 9.0f, 19.0f, 32.0f, 54.0f, 141.0f, 284.0f, 554.0f};
    constexpr float force[]       = {0.0f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f};
    constexpr size_t count = sizeof(force) / sizeof(force[0]);

    if (conductance_us <= conductance[0]) return 0.0f;
    for (size_t i = 1; i < count; ++i) {
        if (conductance_us <= conductance[i]) {
            const float span = conductance[i] - conductance[i - 1];
            const float ratio = span > 0.0f ? (conductance_us - conductance[i - 1]) / span : 0.0f;
            return force[i - 1] + ratio * (force[i] - force[i - 1]);
        }
    }
    return FSR_MAX_N;
}

} // namespace

extern "C" {

bool FSR_Start(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
    if ((hadc == nullptr) || (htim == nullptr)) return false;

    fsr_filter_initialized = false;
    fsr_tared = false;
    fsr_saturated = false;
    fsr_sequence = 0U;
    startup_tare_samples = 0U;
    for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
        fsr_raw[i] = 0U;
        fsr_filtered[i] = 0.0f;
        fsr_force[i] = 0.0f;
        fsr_tare_conductance_us[i] = 0.0f;
        startup_tare_sum[i] = 0.0f;
    }

    if (HAL_ADC_Start_DMA(hadc, reinterpret_cast<uint32_t*>(const_cast<uint32_t*>(fsr_raw)),
                          FSR400_SENSOR_COUNT) != HAL_OK) {
        return false;
    }

    /* TIM6 triggers ADC1 and independently schedules the control loop. */
    if (HAL_TIM_Base_Start_IT(htim) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(hadc);
        return false;
    }
    return true;
}

bool FSR_RestartDMA(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
    if (hadc == nullptr || htim == nullptr) return false;

    if (HAL_TIM_Base_Stop_IT(htim) != HAL_OK) return false;

    fsr_dma_last_stop_status = HAL_ADC_Stop_DMA(hadc);
    if (fsr_dma_last_stop_status != HAL_OK) {
        (void)HAL_TIM_Base_Start_IT(htim);
        return false;
    }

    fsr_dma_last_start_status = HAL_ADC_Start_DMA(
        hadc,
        reinterpret_cast<uint32_t*>(const_cast<uint32_t*>(fsr_raw)),
        FSR400_SENSOR_COUNT);
    if (fsr_dma_last_start_status != HAL_OK) {
        (void)HAL_TIM_Base_Start_IT(htim);
        return false;
    }

    return HAL_TIM_Base_Start_IT(htim) == HAL_OK;
}

void FSR_Update(void)
{
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(const_cast<uint32_t*>(fsr_raw)), 32U);
    const float alpha = clamp_alpha(fsr_filter_alpha);
    if (!fsr_filter_initialized) {
        for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
            fsr_filtered[i] = static_cast<float>(fsr_raw[i]);
        }
        fsr_filter_initialized = true;
    } else {
        for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
            const float sample = static_cast<float>(fsr_raw[i]);
            fsr_filtered[i] += alpha * (sample - fsr_filtered[i]);
        }
    }

    bool saturated = false;
    for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
        const float conductance = adc_to_conductance_us(fsr_filtered[i]);
        if (!fsr_tared && startup_tare_samples < 250U) {
            startup_tare_sum[i] += conductance;
        }
        const float effective = std::max(conductance - fsr_tare_conductance_us[i], 0.0f);
        fsr_force[i] = effective_conductance_to_force(effective);
        saturated = saturated || (fsr_filtered[i] >= 4080.0f);
    }
    if (!fsr_tared && startup_tare_samples < 250U) {
        ++startup_tare_samples;
        if (startup_tare_samples >= 250U) {
            for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
                fsr_tare_conductance_us[i] = startup_tare_sum[i] / 250.0f;
            }
            fsr_tared = true;
        }
    }
    fsr_saturated = saturated;
    ++fsr_sequence;
}

bool FSR_Tare(void)
{
    if (!fsr_filter_initialized) return false;
    for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
        fsr_tare_conductance_us[i] = adc_to_conductance_us(fsr_filtered[i]);
    }
    fsr_tared = true;
    return true;
}

bool FSR_IsTared(void)
{
    return fsr_tared;
}

void FSR_GetSnapshot(FSR_Snapshot *snapshot)
{
    if (snapshot == nullptr) return;
    uint32_t first_sequence;
    uint32_t last_sequence;
    do {
        first_sequence = fsr_sequence;
        snapshot->tared = fsr_tared;
        snapshot->saturated = fsr_saturated;
        for (uint8_t i = 0; i < FSR400_SENSOR_COUNT; ++i) {
            snapshot->raw[i] = fsr_raw[i];
            snapshot->filtered[i] = fsr_filtered[i];
            snapshot->voltage[i] = (fsr_filtered[i] * FSR_VREF) / static_cast<float>(FSR_ADC_MAX);
            snapshot->force_newton[i] = fsr_force[i];
        }
        last_sequence = fsr_sequence;
    } while (first_sequence != last_sequence);
    snapshot->sequence = last_sequence;
}

uint16_t FSR_GetRaw(uint8_t index)
{
    return is_valid_index(index) ? fsr_raw[index] : 0U;
}

float FSR_GetVoltage(uint8_t index)
{
    return is_valid_index(index)
        ? (fsr_filtered[index] * FSR_VREF) / static_cast<float>(FSR_ADC_MAX)
        : 0.0f;
}

float FSR_GetForceNewton(uint8_t index)
{
    return is_valid_index(index) ? fsr_force[index] : 0.0f;
}

void FSR_SetFilterAlpha(float alpha)
{
    fsr_filter_alpha = clamp_alpha(alpha);
}

float FSR_GetFilterAlpha(void)
{
    return fsr_filter_alpha;
}

} // extern "C"
