#ifndef __QEPACK_H__
#define __QEPACK_H__

#include                    "it_controller.h"           // 中断管理器
#include                    "ti_interrupt.h"

// 生成配置结构体

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

#define TI_GET_TIMER_STRUCTURE(TIMER_NAME) \
    (stTimerTdf){ \
        .timer_inst         = TIMER_NAME##_INST, \
        .clk_freq         = TIMER_NAME##_INST_CLK_FREQ, \
        .timer_irqn         = TIMER_NAME##_INST_INT_IRQN, \
    }

#define TI_GET_UART_STRUCTURE(UART_NAME) \
    (stUartTdf){ \
        .uart_inst         = UART_NAME##_INST, \
        .int_irqn          = UART_NAME##_INST_INT_IRQN, \
    }


#endif