#ifndef _TI_INTERRUPT_H_
#define _TI_INTERRUPT_H_

#include "project_config.h"

#if (QEPACK_PLATFORM == TI)

#include "it_controller.h"

void Interrupt_Init(void);

#if UART_IS_ENABLE
    #include "uart_device.h"
#endif

#if LINEAR_CCD_IS_ENABLE
    #include "linear_ccd_device.h"
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
                    DL_Timer_clearInterruptStatus(               \
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

/**
 * @brief 编码器处理数据的定时器中断处理函数模板宏
 * @param TIMER_NAME: 定时器实例名
 */
#define CREATE_ENCODER_HANDLE_TIMER_HANDLER(ENCODER_HANDLE_TIMER, emDevNum)                                  \
    void ENCODER_HANDLE_TIMER##_INST_IRQHandler(void) {                         \
            DL_TIMER_IIDX pendingInterrupt =                                    \
                DL_TimerA_getPendingInterrupt(ENCODER_HANDLE_TIMER##_INST);     \
                                                                    \
            switch (pendingInterrupt) {                           \
                case DL_TIMER_IIDX_ZERO:                          \
                    vEncoderComputeSpeed(emDevNum);               \
                    DL_Timer_clearInterruptStatus(               \
                        ENCODER_HANDLE_TIMER##_INST, DL_TIMER_IIDX_ZERO);       \
                    break;                                        \
                default:                                          \
                    break;                                        \
            }                                                                                                    \
        }      


#define CREATE_ENCODER_COMPARE_TIMER_HANDLER(ENCODER_COMPARE_TIMER_NAME, emDevNum)                                  \
    void ENCODER_COMPARE_TIMER_NAME##_INST_IRQHandler(void) {                         \
            DL_TIMER_IIDX pendingInterrupt =                                    \
                DL_TimerA_getPendingInterrupt(ENCODER_COMPARE_TIMER_NAME##_INST);     \
                                                                    \
            switch (pendingInterrupt) {                           \
                case DL_TIMERA_IIDX_LOAD:                          \
                    vEncoder_Handler(emDevNum);               \
                    DL_Timer_clearInterruptStatus(               \
                        ENCODER_COMPARE_TIMER_NAME##_INST, DL_TIMERA_IIDX_LOAD);       \
                    break;                                        \
                default:                                          \
                    break;                                        \
            }                                                                                                    \
        }      

/* ADC12_0 的中断服务程序 (IRQ Handler) */
/* 当 ADC 产生中断时，CPU 会自动跳转到此函数执行 */
#define CREATE_ADC12_0_IRQ_HANDLER(ADC_NAME, emDevNum)                                  \
    void ADC_NAME##_INST_IRQHandler(void){                                              \
            /* 检查是哪种 ADC 中断源触发了中断 */                                           \
            switch (DL_ADC12_getPendingInterrupt(ADC_NAME##_INST)) {                             \
                /* 当 MEM0 (内存索引0) 的结果加载完成时触发 */                             \
                case DL_ADC12_IIDX_MEM0_RESULT_LOADED:                                  \
                    /* 设置全局标志位，通知主循环 ADC 转换已完成 */                       \
                    vSetAdcConvertFlag(emDevNum, true);                                  \
                    break;                                                        \
                default:                                                          \
                    /* 处理其他未定义的中断情况（此处为空） */                        \
                    break;                                                        \
            }                                                                   \
        }

// void COMPARE_0_INST_IRQHandler(void) 
// {
//   switch (DL_TimerA_getPendingInterrupt(COMPARE_0_INST)) 
//   {
//   case DL_TIMERA_IIDX_LOAD:
//     c++;
//     if (!DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_19_PIN)) 
//     {
//     // 反转
//         F = -1;
//     } 
//     else 
//     {
//     // 正转
//         F = 1;
//     }
//     break;
//   default:
//     break;
//   }
// }

#endif

#endif
