
/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Neural ART Embedded Client API
  *          Re-target of the printf function is out-of-scope.
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\[user_name]\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\7.1.0
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"

#include "ll_aton_debug.h"
#include "ll_aton_rt_user_api.h"
#include "ll_aton_caches_interface.h"

/* USER CODE BEGIN includes */
/* USER CODE END includes */

/* Entry points --------------------------------------------------------------*/

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(blockernn)
uint8_t *buffer_in;
uint8_t *buffer_out;

void set_clk_sleep_mode(void)
{
  /* Leave clocks enabled in Low Power modes */
  // Low-power clock enable misc
#if defined (CPU_IN_SECURE_STATE)
  __HAL_RCC_DBG_CLK_SLEEP_ENABLE();
#endif
  __HAL_RCC_XSPIPHYCOMP_CLK_SLEEP_ENABLE();

  // Low-power clock enable for memories
  __HAL_RCC_AXISRAM1_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM2_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM3_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_FLEXRAM_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_SLEEP_ENABLE();
  // LP clock AHB1: None
  // LP clock AHB2: None
  // LP clock AHB3
#if defined (CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_SLEEP_ENABLE();
  __HAL_RCC_RISAF_CLK_SLEEP_ENABLE();
  __HAL_RCC_IAC_CLK_SLEEP_ENABLE();
#endif
  // LP clock AHB4: None
  // LP clocks AHB5
  __HAL_RCC_XSPI1_CLK_SLEEP_ENABLE();
  __HAL_RCC_XSPI2_CLK_SLEEP_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_SLEEP_ENABLE();
  __HAL_RCC_NPU_CLK_SLEEP_ENABLE();
  // LP clocks APB1: None
  // LP clocks APB2
  __HAL_RCC_USART1_CLK_SLEEP_ENABLE();
  // LP clocks APB4: None
  // LP clocks APB5: None
}

void MX_X_CUBE_AI_Init(void)
{
    __HAL_RCC_AXISRAM2_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
    RAMCFG_SRAM2_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM3_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM4_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM5_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    RAMCFG_SRAM6_AXI->CR &= ~RAMCFG_CR_SRAMSD;
    set_clk_sleep_mode();
    __HAL_RCC_NPU_CLK_ENABLE();
    __HAL_RCC_NPU_FORCE_RESET();
    __HAL_RCC_NPU_RELEASE_RESET();
    npu_cache_init();
    /* USER CODE BEGIN 5 */
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}

/* Simple helpers to run the generated NN from other translation units (e.g. main.c)
   These use the static `NN_Instance_blockernn` / `NN_Interface_blockernn` objects
   declared above in this file via the generator macro. */

void blockernn_model_init(void)
{
  /* LL_ATON_RT_RuntimeInit() MUST be called before any other LL ATON function.
     It is called here ONCE; the polling loop in blockernn_infer() must NOT call
     LL_ATON_RT_Main() (which would call RuntimeInit again → double-init → hang). */
  LL_ATON_RT_RuntimeInit();
  LL_ATON_RT_Init_Network(&NN_Instance_blockernn);
}

int blockernn_infer(float in, float *out)
{
  unsigned len = 0;
  unsigned bits = 0;
  /* Input/output tensor addresses from generated descriptors */
  float *in_buf  = (float *)get_buffer("Input_0_out_0", BUFF_IN,  &len, &bits, &NN_Interface_blockernn);
  float *out_buf = (float *)get_buffer("Gemm_5_out_0",  BUFF_OUT, &len, &bits, &NN_Interface_blockernn);
  if (!in_buf || !out_buf)
    return -1;

  /* Write input and flush D-Cache so the SW operators see the new value */
  *in_buf = in;
  LL_ATON_Cache_MCU_Clean_Range((uintptr_t)in_buf, sizeof(float));

  /* Run all epoch blocks without WFE.
     This model is 100 % pure-SW (wait_mask = 0 for every epoch block):
     LL_ATON_RT_RunEpochBlock will never return LL_ATON_RT_WFE, only
     LL_ATON_RT_NO_WFE or LL_ATON_RT_DONE.
     Using a plain polling loop avoids __WFE() hanging the core. */
  LL_ATON_RT_RetValues_t ret;
  do {
    ret = LL_ATON_RT_RunEpochBlock(&NN_Instance_blockernn);
  } while (ret != LL_ATON_RT_DONE);

  /* Reset instance state so the next call starts from the first epoch */
  LL_ATON_RT_Reset_Network(&NN_Instance_blockernn);

  /* Invalidate D-Cache for output buffer, then read result */
  LL_ATON_Cache_MCU_Invalidate_Range((uintptr_t)out_buf, sizeof(float));
  *out = *out_buf;
  return 0;
}
#ifdef __cplusplus
}
#endif
