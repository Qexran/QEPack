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

#define PI					3.141592653					// PI值

/* ==================== Q16.16 定点数类型 ==================== */

typedef int32_t fix32_t;

#define FIX32_FRAC_BITS     16
#define FIX32_ONE           ((fix32_t)(1 << FIX32_FRAC_BITS))   /* 1.0 */
#define FIX32_MAX           ((fix32_t)0x7FFFFFFF)
#define FIX32_MIN           ((fix32_t)0x80000000)
#define FIX32_ZERO          ((fix32_t)0)
#define FIX32_PI            ((fix32_t)205887)   /* 3.14159265 in Q16.16 */

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

/* ==================== 工具函数 ==================== */

char ucBinToHexHigh(uint8_t val);
char ucBinToHexLow(uint8_t val);

#endif
