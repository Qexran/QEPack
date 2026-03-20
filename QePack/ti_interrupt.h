#ifndef _TI_INTERRUPT_H_
#define _TI_INTERRUPT_H_

#include "project_config.h"

#if (QEPACK_PLATFORM == TI)

#include "it_controller.h"

void Interrupt_Init(void);

#if UART_IS_ENABLE
    #include "uart_device.h"
#endif

/**
 * @brief 定时器中断处理函数模板宏
 * @param TIMER_NAME: 定时器实例名
 */
#define CREATE_1MS_TIMER_HANDLER(TIMER_NAME)                                  \
    void TIMER_NAME##_INST_IRQHandler(void) {                         \
            DL_TIMER_IIDX pendingInterrupt =                      \
                DL_TimerA_getPendingInterrupt(TIMER_NAME##_INST);     \
                                                                    \
            switch (pendingInterrupt) {                           \
                case DL_TIMER_IIDX_ZERO:                          \
                    vTimerTickHandler();                          \
                    vDevicePeriodExecute();                       \
                    DL_TimerA_clearInterruptStatus(               \
                        TIMER_NAME##_INST, DL_TIMER_IIDX_ZERO);       \
                    break;                                        \
                default:                                          \
                    break;                                        \
            }                                                                                                    \
        }                                                         


/**
 * @brief UART中断处理函数模板宏
 * @param UART_NAME: UART实例名
 */
#define CREATE_UART_IRQ_HANDLER(UART_NAME, emDevNum)                                    \
    void UART_NAME##_INST_IRQHandler(void) {                                  \
        switch (DL_UART_Main_getPendingInterrupt(UART_NAME##_INST)) {         \
            case DL_UART_MAIN_IIDX_RX:                                        \
                vUartRxCallBackHandler(emDevNum);                             \
                break;                                                        \
            default:                                                          \
                break;                                                        \
        }                                                                     \
    }



#endif

#endif
