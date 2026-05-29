#include "st_interrupt.h"

#if (QEPACK_PLATFORM == ST)

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
        if(htim == &ST_TIMER_CONTROLLER_TICK_TIM)
        {
            // 1ms 定时器实现
            vTimerTickHandler();
            // 模块循环实现
            vDevicePeriodExecute();
        }
    #endif


    /* 编码器 */
    #if ENCODER_IS_ENABLE
        #if (ENCODER_HANDLE_PLAN == TIM)
            vEncoder_Handler(htim);        // 溢出中断
        #endif

        #if ENCODER_IS_USE_PARASITISM
            vEncoderComputeSpeed(htim);    // 计算速度
        #endif

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
        #if (ENCODER_HANDLE_PLAN == GPIO)
            vEncoder_Handler(GPIO_Pin);
        #endif
    #endif
}

#endif

