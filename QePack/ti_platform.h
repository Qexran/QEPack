#ifndef __TI_PLATFORM__
#define __TI_PLATFORM__

#include "ti_clock.h"

#if (QEPACK_PLATFORM == TI)

#include "ti_msp_dl_config.h"

#define TI_MAX_DELAY      0xFFFFFFFFU

extern volatile unsigned long tick_ms;

/**
 * @brief GPIO引脚状态枚举
 */
typedef enum
{
  GPIO_PIN_RESET,
  GPIO_PIN_SET
} GPIO_PinState;

typedef struct {
    I2C_Regs      *i2c_inst;          // I2C 模块寄存器基地址
    GPIO_Regs     *pstSclGpioPort;    // SCL 端口基地址
    uint32_t      usSclPin;           // SCL 引脚号
    GPIO_Regs     *pstSdaGpioPort;    // SDA 端口基地址
    uint32_t      usSdaPin;           // SDA 引脚号

    uint32_t      ulIOMuxScl;         // SCL 的 IOMUX 索引
    uint32_t      ulIOMuxSda;         // SDA 的 IOMUX 索引
    uint32_t      ulIOMuxSclFunc;     // SCL 对应的具体外设功能号
    uint32_t      ulIOMuxSdaFunc;     // SDA 对应的具体外设功能号

    void (*vI2cInitFunc)(void);       // 指向 SYSCFG_DL_I2C_XXX_init 初始化回调函数指针
} stI2CTdf;

typedef struct {
    UART_Regs     *uart_inst;          // UART 模块寄存器基地址
    uint32_t      int_irqn;            // UART 中断向量号
} stUartTdf;

typedef struct {    /** 注意：若config里没有对应的变量，则不填写该字段 */
    GPTIMER_Regs    *timer_inst;          // 定时器寄存器基地址
    uint32_t        clk_freq;             // 定时器时钟频率（Hz）
    IRQn_Type       timer_irqn;           // 定时器中断向量号
} stTimerTdf;

typedef struct {
    ADC12_Regs          *adc_inst;           // ADC 模块寄存器基地址
    DL_ADC12_MEM_IDX    adc_mem_idx;         // ADC 内存索引
    float               voltage;             // ADC 参考电压（V）
    IRQn_Type           adc_irqn;            // ADC 中断向量号
} stAdcTdf;

typedef struct {
    // TODO 定义DMA句柄
} stAdcDmaTdf;

typedef void (*vI2CInitFunc)(void);
void SysTick_Init(void);
void TI_Delay(uint32_t ms);

uint32_t TI_GetTick(void);

void TI_I2C_Mem_Write(
    stI2CTdf *pstIdf,
    uint8_t DevAddress, uint8_t MemAddress,
    uint8_t *pData, uint16_t Size, uint32_t Timeout
);

void TI_UART_Transmit(
    stUartTdf *i2c_inst, const uint8_t *pData, 
    uint16_t Size, uint32_t Timeout
);

GPIO_PinState TI_GPIO_ReadPin(GPIO_Regs *GPIOx, uint32_t GPIO_Pin);

void TI_GPIO_WritePin(GPIO_Regs *GPIOx, uint32_t GPIO_Pin, GPIO_PinState PinState);

void TI_ADC_Start_DMA(stAdcTdf *pstAdcBase, uint32_t *pData, uint32_t Length);

#endif

#endif
