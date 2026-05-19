#ifndef __QEPACK_H__
#define __QEPACK_H__

#include                    "it_controller.h"           // 中断管理器
#include                    "ti_interrupt.h"
#include                    "ti_hardfault_helper.h"
#include                    "arithmetic.h"

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

#if GEAR_MOTOR_IS_ENABLE
    #include "gear_motor_device.h"
#endif

#if TIMER_IS_ENABLE
    #include "timer_device.h"
#endif

#if STEP_M_ENABLE
    #include "step_machine.h"
#endif

#if COUNTER_IS_ENABLE
    #include "counter_controller.h"
#endif

#if OLED_IS_ENABLE
    #include "OLED.h"
#endif

#if PID_IS_ENABLE
    #include "pid_controller.h"
#endif

#if MPU6050_IS_ENABLE
    #include "mpu6050_device.h"
#endif

#if BNO08X_IS_ENABLE
    #include "bno08x_device.h"
#endif

#if IMU660RB_IS_ENABLE
    #include "imu660rb_device.h"
#endif

#if HWT101_IS_ENABLE
    #include "hwt101_device.h"
#endif

#if ATK_MS901M_IS_ENABLE
    #include "atk_ms901m_device.h"
#endif

#if EMM_MOTOR_IS_ENABLE
    #include "emm_step_motor_device.h"
#endif

#if W25Q64_IS_ENABLE
    #include "w25q64_device.h"
#endif

#if SENSOR_IS_ENABLE
    #include "sensor_device.h"
#endif

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE
    #include "motor_system_controller.h"
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

#define TI_GET_ENCODER_STRUCTURE(TIMER_NAME) \
    (stTimerTdf){ \
        .timer_inst         = TIMER_NAME##_INST, \
        .timer_irqn         = TIMER_NAME##_INST_INT_IRQN, \
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

#if UART_IS_USE_DMA
/* DMA 版 UART 结构宏 — 需在 SysConfig 中额外配置 DMA 通道（见 uart_device.h 注释） */
#define TI_GET_UART_STRUCTURE_DMA(UART_NAME, DMA_RX_CH_NAME, DMA_TX_CH_NAME) \
    (stUartTdf){ \
        .uart_inst         = UART_NAME##_INST, \
        .int_irqn          = UART_NAME##_INST_INT_IRQN, \
        .dma_rx_channel    = DMA_RX_CH_NAME##_CHAN_ID, \
        .dma_tx_channel    = DMA_TX_CH_NAME##_CHAN_ID, \
        .dma_rx_trigger    = TI_GetUartDmaRxTrigger(UART_NAME##_INST), \
        .dma_tx_trigger    = TI_GetUartDmaTxTrigger(UART_NAME##_INST), \
    }

uint32_t TI_GetUartDmaRxTrigger(UART_Regs *uart_inst);
uint32_t TI_GetUartDmaTxTrigger(UART_Regs *uart_inst);
#endif

#define TI_GET_ADC_STRUCTURE(ADC_NAME) \
    (stAdcTdf){ \
        .adc_inst         = ADC_NAME##_INST, \
        .adc_mem_idx      = ADC_NAME##_ADCMEM_0, \
        .voltage          = ADC_NAME##_ADCMEM_0_REF_VOLTAGE_V, \
        .adc_irqn         = ADC_NAME##_INST_INT_IRQN, \
    }

#define TI_GET_IMU660RB_SPI_STRUCTURE(BASE_NAME) \
    (stImu660rbSpiTdf){ \
        .spi_inst    = SPI_##BASE_NAME##_INST, \
        .pstCsPort   = GPIO_##BASE_NAME##_PIN_##BASE_NAME##_CS_PORT, \
        .ulCsPin     = GPIO_##BASE_NAME##_PIN_##BASE_NAME##_CS_PIN, \
        .pstIntPort  = GPIO_##BASE_NAME##_PIN_##BASE_NAME##_INT1_PORT, \
        .ulIntPin    = GPIO_##BASE_NAME##_PIN_##BASE_NAME##_INT1_PIN, \
    }

#endif
