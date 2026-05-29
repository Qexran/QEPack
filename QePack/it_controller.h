/**
  * @file       it_controller.h
  * @author     Qe_xr
  * @version    V1.0.1
  * @date       2026/5/29
  * @brief      中断管理
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

#if STEP_M_ENABLE
    #include "step_machine.h"
#endif

#if MOTOR_IS_ENABLE
    #include "motor_device.h"
#endif

#if UART_IS_ENABLE
    #include "uart_device.h"
#endif

#if SERVO_IS_ENABLE
    #include "servo_device.h"
#endif

#if HWT101_IS_ENABLE
    #include "hwt101_device.h"
#endif

#if ULTRASONIC_IS_ENABLE
    #include "ultrasonic_device.h"
#endif


#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE
    #include "motor_system_controller.h"
#endif

#if JOYSTICK_IS_ENABLE
    #include "joystick_device.h"
#endif

void vDevicePeriodExecute(void);
void vDevicePeriodExecuteMainLoop(void);

#endif
