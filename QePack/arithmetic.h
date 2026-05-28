/**
  * @file       arithmetic.h
  * @author     Qe_xr
  * @version    V2.0.0
  * @date       2026/5/11
  * @brief      算法库 — 定点数运算 + 工具函数
  */
#ifndef __ARITHMETIC_H
#define __ARITHMETIC_H

#include <stdint.h>
#include <math.h>
#include "project_config.h"

#if (QEPACK_PLATFORM == TI)
    #include "ti_platform.h"
#endif

#define PI					3.141592653					// PI值

/* ==================== Q16.16 定点数类型 ==================== */

typedef int32_t fix32_t;

#define FIX32_FRAC_BITS     16
#define FIX32_ONE           ((fix32_t)(1 << FIX32_FRAC_BITS))   /* 1.0 */
#define FIX32_MAX           ((fix32_t)0x7FFFFFFF)
#define FIX32_MIN           ((fix32_t)0x80000000)
#define FIX32_ZERO          ((fix32_t)0)
#define FIX32_PI            ((fix32_t)205887)   /* 3.14159265 in Q16.16 */

/* 编译期定点常量 */
#define FIX32_2     ((fix32_t)(2  * FIX32_ONE))
#define FIX32_10    ((fix32_t)(10 * FIX32_ONE))
#define FIX32_60    ((fix32_t)(60 * FIX32_ONE))
#define FIX32_360   ((fix32_t)(360 * FIX32_ONE))
#define FIX32_90    ((fix32_t)(90  * FIX32_ONE))
#define FIX32_180   ((fix32_t)(180 * FIX32_ONE))
#define FIX32_PI_V  ((fix32_t)205887)  /* 3.14159265 in Q16.16 */
#define FIX32_HALF  ((fix32_t)(FIX32_ONE / 2))

/* 整数 -> fix32_t */
#define FIX32_FROM_INT(i)   ((fix32_t)((int32_t)(i) << FIX32_FRAC_BITS))

/* fix32_t -> 整数（截断） */
#define FIX32_TO_INT(f)     ((int32_t)((f) >> FIX32_FRAC_BITS))

/* 浮点数 -> fix32_t */
static inline fix32_t fix32_from_float(float f) {
    return (fix32_t)(f * 65536.0f);
}

/* fix32_t -> 浮点数 */
static inline float fix32_to_float(fix32_t x) {
    return (float)x / 65536.0f;
}

/* ==================== 定点数基础运算 ==================== */

fix32_t fix32_mul(fix32_t a, fix32_t b);
fix32_t fix32_div(fix32_t a, fix32_t b);
fix32_t fix32_abs(fix32_t x);
fix32_t fix32_sat(fix32_t x, fix32_t lo, fix32_t hi);
fix32_t fix32_sqrt(fix32_t x);
fix32_t fix32_map(fix32_t fValue, fix32_t fFromLow, fix32_t fFromHigh, fix32_t fToLow, fix32_t fToHigh);

/* ==================== 平台抽象宏 ==================== */

#if (QEPACK_PLATFORM == ST)
    #define QE_DELAY(ms)     HAL_Delay(ms)
    #define QE_GET_TICK()    HAL_GetTick()
#else
    #define QE_DELAY(ms)     TI_Delay(ms)
    #define QE_GET_TICK()    TI_GetTick()
#endif

/* ==================== 工具函数 ==================== */

int32_t lCmToPulse(fix32_t fCm, fix32_t fDiameterCm, fix32_t fPulsePerRev);
char ucBinToHexHigh(uint8_t val);
char ucBinToHexLow(uint8_t val);

#endif
