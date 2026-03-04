#include "project_config.h"   // 寄存器基地址定义
#include "Delay.h"

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

/**
 * @brief 自动初始化 DWT CYCCNT
 */
static void DWT_Init(void)
{
    static uint8_t inited = 0;
    if (inited == 0) {
        DEM_CR |= DEM_CR_TRCENA;        /* 使能 DWT  */
        DWT_CYCCNT = 0;                 /* 清计数器  */
        DWT_CTRL |= DWT_CTRL_CYCCNTENA; /* 启动 CYCCNT */
        inited = 1;
    }
}

/**
 * @brief 微秒延时
 * @param us 延时时间（单位：微秒）
 * @note 0 ~ 2^32/72 微秒延时
 * @note 1000000U * us ≈ 59.6 s
 */
void Delay_us(uint32_t us)
{
    DWT_Init();
    uint32_t cycles = (SYSTEM_CORE_CLOCK / 1000000U) * us;
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles) {
        /* 空等待 */
    }
}

/**
 * @brief 毫秒延时
 * @param ms 延时时间（单位：毫秒）
 * @note 0 ~ 2^32/72 毫秒延时
 * @note 1000 次微秒
 */
void Delay_ms(uint32_t ms)
{
    while (ms--) {
        Delay_us(1000);
    }
}

/**
 * @brief 秒级延时
 * @param s 延时时间（单位：秒）
 * @note 0 ~ 2^32/72 秒级延时
 * @note 1000 次毫秒
 */
void Delay_s(uint32_t s)
{
    while (s--) {
        Delay_ms(1000);
    }
}
