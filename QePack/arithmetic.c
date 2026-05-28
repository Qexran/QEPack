/**
  * @file       arithmetic.c
  * @author     Qe_xr
  * @version    V2.0.0
  * @date       2026/5/11
  * @brief      算法库实现 — 定点数运算 + 工具函数
  */
#include "arithmetic.h"

/* ==================== Q16.16 定点数运算 ==================== */

/**
 * @brief 定点数乘法（饱和）
 */
fix32_t fix32_mul(fix32_t a, fix32_t b)
{
    int64_t product = (int64_t)a * (int64_t)b;
    int64_t shifted = product >> FIX32_FRAC_BITS;
    if (shifted > (int64_t)FIX32_MAX) return FIX32_MAX;
    if (shifted < (int64_t)FIX32_MIN) return FIX32_MIN;
    return (fix32_t)shifted;
}

/**
 * @brief 定点数除法（饱和，除零保护）
 */
fix32_t fix32_div(fix32_t a, fix32_t b)
{
    if (b == 0) return (a >= 0) ? FIX32_MAX : FIX32_MIN;
    int64_t dividend = (int64_t)a << FIX32_FRAC_BITS;
    int64_t result = dividend / (int64_t)b;
    if (result > (int64_t)FIX32_MAX) return FIX32_MAX;
    if (result < (int64_t)FIX32_MIN) return FIX32_MIN;
    return (fix32_t)result;
}

/**
 * @brief 定点数绝对值
 */
fix32_t fix32_abs(fix32_t x)
{
    if (x == FIX32_MIN) return FIX32_MAX;
    return (x >= 0) ? x : -x;
}

/**
 * @brief 定点数限幅
 */
fix32_t fix32_sat(fix32_t x, fix32_t lo, fix32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief 定点数开方（整数牛顿迭代法）
 * @note  输入 Q16.16，输出 Q16.16
 *        算法：对原始 Q16.16 值取整数开方，再左移 8 位
 *        验证：sqrt(4.0)=2.0, sqrt(100.0)=10.0, sqrt(0.25)=0.5
 */
fix32_t fix32_sqrt(fix32_t x)
{
    if (x <= 0) return 0;

    uint32_t val = (uint32_t)x;
    uint32_t result = 0;
    uint32_t bit = 1U << 30;

    while (bit > val) bit >>= 2;

    while (bit != 0) {
        if (val >= result + bit) {
            val -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }

    /* result = sqrt(x)，左移 8 位得到 Q16.16 格式 */
    return (fix32_t)(result << (FIX32_FRAC_BITS / 2));
}

/**
 * @brief 定点数线性映射，将值从 [fromLow, fromHigh] 映射到 [toLow, toHigh]
 */
fix32_t fix32_map(fix32_t fValue, fix32_t fFromLow, fix32_t fFromHigh, fix32_t fToLow, fix32_t fToHigh)
{
    fix32_t fFromRange = fFromHigh - fFromLow;
    if (fFromRange == 0) return fToLow;
    int64_t llNum = (int64_t)(fValue - fFromLow) * (int64_t)(fToHigh - fToLow);
    return (fix32_t)(llNum / (int64_t)fFromRange) + fToLow;
}

/**
 * @brief cm → 脉冲数转换
 * @param fCm          目标距离(cm), Q16.16
 * @param fDiameterCm  轮子直径(cm), Q16.16
 * @param fPulsePerRev 编码器单圈脉冲数, Q16.16
 * @return 脉冲数（整数），参数为 0 时返回 0
 */
int32_t lCmToPulse(fix32_t fCm, fix32_t fDiameterCm, fix32_t fPulsePerRev)
{
    if (fDiameterCm < FIX32_ONE || fPulsePerRev < FIX32_ONE) return 0;
    fix32_t fCirc = fix32_mul(FIX32_PI, fDiameterCm);
    return FIX32_TO_INT(fix32_mul(fix32_div(fCm, fCirc), fPulsePerRev));
}

/* ==================== 工具函数 ==================== */

/**
 * @brief 二进制转十六进制高4位
 */
char ucBinToHexHigh(uint8_t val){
    uint8_t h = (val >> 4) & 0x0F;
    return h > 9 ? (h - 10 + 'A') : (h + '0');
}

/**
 * @brief 二进制转十六进制低4位
 */
char ucBinToHexLow(uint8_t val){
    uint8_t l = val & 0x0F;
    return l > 9 ? (l - 10 + 'A') : (l + '0');
}

