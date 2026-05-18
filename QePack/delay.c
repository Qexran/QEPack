/**
 * @file    delay.c
 * @brief   DWT 微秒延时（仅 STM32 Cortex-M3/4/7）
 * @note    DWT 在 Cortex-M0+ (TI MSPM0) 上不存在，TI 平台请使用 TI_Delay()
 */
#include "project_config.h"
#include "delay.h"

#if (QEPACK_PLATFORM == ST)

/* 默认为 72 MHz 系统时钟 */
#ifndef SYSTEM_CORE_CLOCK
    #define SYSTEM_CORE_CLOCK 72000000U
#endif

/* DWT 寄存器定义（兼容所有 CM3/4/7） */
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define DEM_CR      (*(volatile uint32_t *)0xE000EDFC)

#define DEM_CR_TRCENA    (1 << 24)
#define DWT_CTRL_CYCCNTENA (1 << 0)

static void DWT_Init(void)
{
    static uint8_t inited = 0;
    if (inited == 0) {
        DEM_CR |= DEM_CR_TRCENA;
        DWT_CYCCNT = 0;
        DWT_CTRL |= DWT_CTRL_CYCCNTENA;
        inited = 1;
    }
}

void Delay_us(uint32_t us)
{
    DWT_Init();
    uint32_t cycles = (SYSTEM_CORE_CLOCK / 1000000U) * us;
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles) {
    }
}

void Delay_ms(uint32_t ms)
{
    while (ms--) {
        Delay_us(1000);
    }
}

void Delay_s(uint32_t s)
{
    while (s--) {
        Delay_ms(1000);
    }
}

#endif
