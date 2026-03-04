/**
  * @file       adc_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      中断管理
  * 
  */
#include "it_controller.h"


#if ENCODER_IS_ENABLE
    #include "encoder_device.h"
#endif

#if KEY_IS_ENABLE
    #include "key_device.h"
#endif

#if UART_IS_ENABLE
    #include "uart_device.h"
#endif


#include "tim.h"

/**
 * @brief       定时器溢出回调函数
 * @param       htim:定时器句柄指针
 * @note        此函数被定时器中断函数共同调用
 */
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 1ms定时器 */
    if(htim == &htim1){
        vKeyDevicePeriodExecute(KEY0);
        vKeyDevicePeriodExecute(KEY1);
        vKeyDevicePeriodExecute(KEY2);
    
    }
    
    /* 编码器 */
    #if ENCODER_IS_ENABLE
        #if ENCODER_HANDLE_PLAN // TIM
            vEncoder_Handler(htim);
        #endif
    
        if(htim == &ENCODER_COMPUTE_IT_TIM){
            vEncoderComputeSpeed(ENCODER_0);
        }
    #endif
}


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

