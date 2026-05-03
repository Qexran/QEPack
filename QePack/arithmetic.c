/**
  * @file       arithmetic.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/4/18
  * @brief      算法库实现
  */
#include "arithmetic.h"

/**
 * @brief 二进制转十六进制高4位
 * @param val 二进制值
 * @return char 高4位
 */
char ucBinToHexHigh(uint8_t val){
    uint8_t h = (val >> 4) & 0x0F;
    return h > 9 ? (h - 10 + 'A') : (h + '0');
}

/**
 * @brief 二进制转十六进制低4位
 * @param val 二进制值
 * @return char 低4位
 */
char ucBinToHexLow(uint8_t val){
    uint8_t l = val & 0x0F;
    return l > 9 ? (l - 10 + 'A') : (l + '0');
}

