#include "ti_interrupt.h"

#if (QEPACK_PLATFORM == TI)

// 在此处填写 1MS定时器 实例名 (只能定义一次)
CREATE_1MS_TIMER_HANDLER(TIMER_0)

// 在此处填写 UART中断实现 实例名
CREATE_UART_IRQ_HANDLER(UART_0, UART_DEVICE_0)

// CREATE_UART_IRQ_HANDLER(UART_EMM, UART_DEVICE_1)

// CREATE_UART_IRQ_HANDLER(UART_ATK, UART_DEVICE_1)
void UART3_IRQHandler(void) {
  DL_UART_IIDX iidx;
  while ((iidx = DL_UART_getPendingInterrupt(UART3)) !=
         DL_UART_IIDX_NO_INTERRUPT) {
    switch (iidx) {
    case DL_UART_IIDX_RX:
      vUartRxCallBackHandler(emUartDevNum1);
      break;
    case DL_UART_IIDX_OVERRUN_ERROR:
    case DL_UART_IIDX_BREAK_ERROR:
    case DL_UART_IIDX_PARITY_ERROR:
    case DL_UART_IIDX_FRAMING_ERROR:
    case DL_UART_IIDX_RX_TIMEOUT_ERROR:
    case DL_UART_IIDX_NOISE_ERROR:
      while (!DL_UART_isRXFIFOEmpty(UART3)) {
        DL_UART_receiveData(UART3);
      }
      break;
    default:
      break;
    }
  }
}

// 在此处填写 编码器的比较捕获定时器 实例名
// CREATE_ENCODER_COMPARE_TIMER_HANDLER(ZFJ, ENCODER_0)

// 在此处填写 编码器处理数据的定时器 实例名 (若关闭寄生定时器功能则定义)
// CREATE_ENCODER_HANDLE_TIMER_HANDLER(TIMER_2, ENCODER_0)

#endif
