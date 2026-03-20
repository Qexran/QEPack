#include "ti_interrupt.h"

#if (QEPACK_PLATFORM == TI)

#include "ti_clock.h"

/**
 * @brief SysTick处理函数
 * 
 */
void SysTick_Handler(void)
{
    tick_ms++;
}


// 在此处填写 1MS定时器 实例名
CREATE_1MS_TIMER_HANDLER(TIMER_0)

// 在此处填写 UART中断实现 实例名
CREATE_UART_IRQ_HANDLER(UART_0, UART_DEVICE_0)

#endif