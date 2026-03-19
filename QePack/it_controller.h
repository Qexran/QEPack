/**
  * @file       adc_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      ADC 转换驱动，基于 STM32 HAL 库
  * 
  */
#ifndef _IT_CONTROLLER_H_
#define _IT_CONTROLLER_H_

#include "project_config.h"

#if TIMER_CONTROLLER_IS_ENABLE
    #include "timer_controller.h"
#endif

#if ENCODER_IS_ENABLE
    #include "encoder_device.h"
#endif

#if KEY_IS_ENABLE
    #include "key_device.h"
#endif

#if LED_IS_ENABLE
    #include "led_device.h"
#endif


#if UART_IS_ENABLE
    #include "uart_device.h"
#endif

void vDevicePeriodExecute();

#endif
