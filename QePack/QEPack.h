#ifndef __QEPACK_H__
#define __QEPACK_H__

#include                    "it_controller.h"           // 中断管理器
#include                    "ti_interrupt.h"

#if LINEAR_CCD_IS_ENABLE
    #include "linear_ccd_device.h"
#endif

#if LED_IS_ENABLE
    #include "led_device.h"
#endif

#if KEY_IS_ENABLE
    #include "key_device.h"
#endif

#if UART_IS_ENABLE
    #include "uart_device.h"
#endif

#if MOTOR_IS_ENABLE
    #include "motor_device.h"
#endif

#if TIMER_IS_ENABLE
    #include "timer_device.h"
#endif

#if STEP_M_ENABLE
    #include "step_machine.h"
#endif

#if OLED_IS_ENABLE
    #include "OLED.h"
#endif

#if PID_IS_ENABLE
    #include "pid_device.h"
#endif

#if MPU6050_IS_ENABLE
    #include "mpu6050_device.h"
#endif

#if ATK_MS901M_IS_ENABLE
    #include "atk_ms901m_device.h"
#endif

#if EMM_V5_IS_ENABLE
    #include "em_v5_device.h"
#endif

#if W25Q64_IS_ENABLE
    #include "w25q64_device.h"
#endif

#if ENCODER_IS_ENABLE
    #include "encoder_device.h"
#endif

#if SERVO_IS_ENABLE
    #include "servo_device.h"
#endif

#if ADC_DEVICE_IS_ENABLE
    #include "adc_device.h"
#endif

#define ADC_CHANNEL_0                      DL_ADC12_MEM_IDX_0
#define ADC_CHANNEL_1                      DL_ADC12_MEM_IDX_1
#define ADC_CHANNEL_2                      DL_ADC12_MEM_IDX_2
#define ADC_CHANNEL_3                      DL_ADC12_MEM_IDX_3
#define ADC_CHANNEL_4                      DL_ADC12_MEM_IDX_4
#define ADC_CHANNEL_5                      DL_ADC12_MEM_IDX_5
#define ADC_CHANNEL_6                      DL_ADC12_MEM_IDX_6
#define ADC_CHANNEL_7                      DL_ADC12_MEM_IDX_7
#define ADC_CHANNEL_8                      DL_ADC12_MEM_IDX_8
#define ADC_CHANNEL_9                      DL_ADC12_MEM_IDX_9
#define ADC_CHANNEL_10                     DL_ADC12_MEM_IDX_10
#define ADC_CHANNEL_11                     DL_ADC12_MEM_IDX_11


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

#define TI_GET_ADC_STRUCTURE(ADC_NAME) \
    (stAdcTdf){ \
        .adc_inst         = ADC_NAME##_INST, \
        .adc_mem_idx      = ADC_NAME##_ADCMEM_0, \
        .voltage          = ADC_NAME##_ADCMEM_0_REF_VOLTAGE_V, \
        .adc_irqn         = ADC_NAME##_INST_INT_IRQN, \
    }

#endif
