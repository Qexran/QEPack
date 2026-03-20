#ifndef __QEPACK_H__
#define __QEPACK_H__

#include                    "it_controller.h"           // 中断管理器
#include                    "ti_interrupt.h"

// 生成 I2C 配置结构体
#define TI_GET_I2C_STRUCTURE(I2C_NAME) \
    (stI2CTdf){ \
        .i2c_inst         = I2C_NAME##_INST, \
        .pstSdaGpioPort   = GPIO_##I2C_NAME##_SDA_PORT, \
        .usSdaPin         = GPIO_##I2C_NAME##_SDA_PIN, \
        .ulIOMuxSda       = GPIO_##I2C_NAME##_IOMUX_SDA, \
        .ulIOMuxSdaFunc   = GPIO_##I2C_NAME##_IOMUX_SDA_FUNC, \
        .pstSclGpioPort   = GPIO_##I2C_NAME##_SCL_PORT, \
        .usSclPin         = GPIO_##I2C_NAME##_SCL_PIN, \
        .ulIOMuxScl       = GPIO_##I2C_NAME##_IOMUX_SCL, \
        .ulIOMuxSclFunc   = GPIO_##I2C_NAME##_IOMUX_SCL_FUNC, \
        .vI2cInitFunc     = SYSCFG_DL_##I2C_NAME##_init, \
}

/**
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_33_kHZ_9600_BAUD                                         (1)
#define UART_0_FBRD_33_kHZ_9600_BAUD                                         (9)
*/

#define TI_GET_UART_STRUCTURE(UART_NAME) \
    (stUartTdf){ \
        .uart_inst         = UART_NAME##_INST, \
        .int_irqn          = UART_NAME##_INST_INT_IRQN, \
    }


#endif
