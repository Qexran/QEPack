/**
  * @file       adc_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      中断管理
  * 
  */
#include "it_controller.h"

/**
 * @brief       设备周期更新函数
 */
void vDevicePeriodExecute(){
    /* KEY */
    #if KEY_IS_ENABLE
        #ifdef KEY0 
            vKeyDevicePeriodExecute(KEY0); 
        #endif
        #ifdef KEY1 
            vKeyDevicePeriodExecute(KEY1); 
        #endif
        #ifdef KEY2 
            vKeyDevicePeriodExecute(KEY2); 
        #endif
        #ifdef KEY3 
            vKeyDevicePeriodExecute(KEY3); 
        #endif
        #ifdef KEY4 
            vKeyDevicePeriodExecute(KEY4); 
        #endif
        #ifdef KEY5 
            vKeyDevicePeriodExecute(KEY5); 
        #endif
        #ifdef KEY6 
            vKeyDevicePeriodExecute(KEY6); 
        #endif
        #ifdef KEY7 
            vKeyDevicePeriodExecute(KEY7); 
        #endif
    #endif
    /* LED */
    #if LED_IS_ENABLE
        #ifdef LED0 
            vLedDevicePeriodExecute(LED0); 
        #endif
        #ifdef LED1 
            vLedDevicePeriodExecute(LED1); 
        #endif
        #ifdef LED2 
            vLedDevicePeriodExecute(LED2); 
        #endif
        #ifdef LED3 
            vLedDevicePeriodExecute(LED3); 
        #endif
        #ifdef LED4 
            vLedDevicePeriodExecute(LED4); 
        #endif
        #ifdef LED5 
            vLedDevicePeriodExecute(LED5); 
        #endif
        #ifdef LED6 
            vLedDevicePeriodExecute(LED6); 
        #endif
        #ifdef LED7 
            vLedDevicePeriodExecute(LED7); 
        #endif
    #endif
    /* SERVO */
    #if SERVO_IS_ENABLE
        #ifdef SERVO0 
            vServoDevicePeriodExecute(SERVO0); 
        #endif
        #ifdef SERVO1 
            vServoDevicePeriodExecute(SERVO1); 
        #endif
        #ifdef SERVO2 
            vServoDevicePeriodExecute(SERVO2); 
        #endif
        #ifdef SERVO3 
            vServoDevicePeriodExecute(SERVO3); 
        #endif
        #ifdef SERVO4 
            vServoDevicePeriodExecute(SERVO4); 
        #endif
        #ifdef SERVO5 
            vServoDevicePeriodExecute(SERVO5); 
        #endif
        #ifdef SERVO6 
            vServoDevicePeriodExecute(SERVO6); 
        #endif
        #ifdef SERVO7 
            vServoDevicePeriodExecute(SERVO7); 
        #endif
    #endif
    /* UART */
    #if UART_IS_ENABLE
        #ifdef UART0 
            vUartDevicePeriodExecute(UART0); 
        #endif
        #ifdef UART1 
            vUartDevicePeriodExecute(UART1); 
        #endif
        #ifdef UART2 
            vUartDevicePeriodExecute(UART2); 
        #endif
        #ifdef UART3 
            vUartDevicePeriodExecute(UART3); 
        #endif
        #ifdef UART4 
            vUartDevicePeriodExecute(UART4); 
        #endif
        #ifdef UART5 
            vUartDevicePeriodExecute(UART5); 
        #endif
        #ifdef UART6 
            vUartDevicePeriodExecute(UART6); 
        #endif
        #ifdef UART7 
            vUartDevicePeriodExecute(UART7); 
        #endif
    #endif
    /* ULTRASONIC */
    #if ULTRASONIC_IS_ENABLE
        #ifdef ULTRASONIC0 
            vUltrasonicDevicePeriodExecute(ULTRASONIC0); 
        #endif
        #ifdef ULTRASONIC1 
            vUltrasonicDevicePeriodExecute(ULTRASONIC1); 
        #endif
    #endif
}

/**
 * @brief       定时器溢出回调函数
 * @param       htim:定时器句柄指针
 * @note        此函数被定时器中断函数共同调用
 */
#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 1ms定时器 */
    #if TIMER_CONTROLLER_IS_ENABLE
        if(htim == &TIMER_CONTROLLER_TICK_TIM)
            // 1ms 定时器实现
            vTimerTickHandler();
            // 模块循环实现
            vDevicePeriodExecute();
    #endif
    
    
    /* 编码器 */
    #if ENCODER_IS_ENABLE
        #if ENCODER_HANDLE_PLAN // TIM
            vEncoder_Handler(htim);
        #endif
    
        if(htim == &ENCODER_COMPUTE_IT_TIM){
            vEncoderComputeSpeed(ENCODER_0);    // 计算速度
        }
    #endif
}
#endif

/**
 * @brief       GPIO外部中断回调函数
 * @param       GPIO_Pin:GPIO
 * @note        此函数会被GPIO外部中断共同调用
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   /* 编码器 */
    #if ENCODER_IS_ENABLE
        #if !ENCODER_HANDLE_PLAN // GPIO
            vEncoder_Handler(GPIO_Pin);
        #endif
    #endif
}

