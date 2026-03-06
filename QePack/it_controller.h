/**
  * @file       adc_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      ADC 转换驱动，基于 STM32 HAL 库
  * 
  */
#include "project_config.h"

#ifndef _IT_CONTROLLER_H_
#define _IT_CONTROLLER_H_

#if ENCODER_IS_ENABLE
    #include "encoder_device.h"
#endif

#if KEY_IS_ENABLE
    #include "key_device.h"
#endif

#if UART_IS_ENABLE
    #include "uart_device.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
    #include "tim.h"
    void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif

#endif
