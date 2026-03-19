#include "ti_interrupt.h"

#if (QEPACK_PLATFORM == TI)

#include "ti_clock.h"

/**
 * @brief SysTick中断处理函数
 * 
 */
void SysTick_Handler(void)
{
    tick_ms++;
}


// 在此处填写定时器实例名
CREATE_1MS_TIMER_HANDLER(TIMER_0)

#endif