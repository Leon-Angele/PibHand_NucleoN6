#include "hand/fsr400.hpp"

namespace {

constexpr uint16_t FSR_ADC_MAX = 4095U;
constexpr float FSR_VREF = 3.3f;
constexpr float FSR_DEFAULT_FILTER_ALPHA = 0.2f;

__attribute__((section(".noncacheable"), aligned(32)))
static volatile uint16_t fsr_raw[5] = {};

static volatile float fsr_filtered[5] = {};
static volatile float fsr_filter_alpha = FSR_DEFAULT_FILTER_ALPHA;
static bool fsr_filter_initialized = false;

bool is_valid_index(uint8_t index)
{
    return index < FSR400_SENSOR_COUNT;
}

float clamp_alpha(float alpha)
{
    if (alpha < 0.0f) {
        return 0.0f;
    }
    if (alpha > 1.0f) {
        return 1.0f;
    }
    return alpha;
}

} // namespace

extern "C" {

bool FSR_Start(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
    if ((hadc == nullptr) || (htim == nullptr)) {
        return false;
    }

    fsr_filter_initialized = false;
    for (uint8_t index = 0; index < FSR400_SENSOR_COUNT; ++index) {
        fsr_raw[index] = 0U;
        fsr_filtered[index] = 0.0f;
    }

    // HAL uses a const uint32_t pointer for the DMA destination even though
    // the peripheral writes to it. The buffer remains volatile and halfword-sized.
    const void *dma_address = const_cast<uint16_t *>(fsr_raw);
    const uint32_t *dma_buffer = static_cast<const uint32_t *>(dma_address);
    if (HAL_ADC_Start_DMA(hadc, dma_buffer, FSR400_SENSOR_COUNT) != HAL_OK) {
        return false;
    }

    if (HAL_TIM_Base_Start_IT(htim) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(hadc);
        return false;
    }

    return true;
}

void FSR_Update(void)
{
    const float alpha = clamp_alpha(fsr_filter_alpha);

    if (!fsr_filter_initialized) {
        for (uint8_t index = 0; index < FSR400_SENSOR_COUNT; ++index) {
            fsr_filtered[index] = static_cast<float>(fsr_raw[index]);
        }
        fsr_filter_initialized = true;
        return;
    }

    for (uint8_t index = 0; index < FSR400_SENSOR_COUNT; ++index) {
        const float sample = static_cast<float>(fsr_raw[index]);
        fsr_filtered[index] += alpha * (sample - fsr_filtered[index]);
    }
}

uint16_t FSR_GetRaw(uint8_t index)
{
    if (!is_valid_index(index)) {
        return 0U;
    }

    return fsr_raw[index];
}

float FSR_GetVoltage(uint8_t index)
{
    if (!is_valid_index(index)) {
        return 0.0f;
    }

    return (fsr_filtered[index] * FSR_VREF) / static_cast<float>(FSR_ADC_MAX);
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
