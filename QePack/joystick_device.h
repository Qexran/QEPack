/**
  * @file       joystick_device.h
  * @author     QePack
  * @version    V2.0.0
  * @date       2026/05/28
  * @brief      PS2 双轴摇杆驱动（定点数版本）
  *             读取 X/Y 两路 ADC，线性映射到用户指定的输出范围
  */
#include "project_config.h"
#if JOYSTICK_IS_ENABLE

#ifndef _JOYSTICK_DEVICE_H_
#define _JOYSTICK_DEVICE_H_

#include <string.h>
#include "adc_device.h"
#include "arithmetic.h"

/// @brief 摇杆输出范围默认值（Q16.16 定点数）
#define JOYSTICK_DEFAULT_OUTPUT_MIN   fix32_from_float(-100.0f)
#define JOYSTICK_DEFAULT_OUTPUT_MAX   fix32_from_float(100.0f)

/// @brief 摇杆设备号枚举
typedef enum {
    emJoystickDevNum0 = 0,
    emJoystickDevNum1,
} emJoystickDevNumTdf;

/// @brief 摇杆反向枚举
typedef enum {
    emJoystickNonInvert = 0,
    emJoystickInvert    = 1,
} emJoystickInvertTdf;

/// @brief 摇杆静态参数
typedef struct {
    emAdcDevNumTdf      emAdcDevNumX;       // X轴 ADC 设备号（DMA 模式下与 Y 共用同一设备）
    emJoystickInvertTdf emInvertX;          // X轴反向
    emAdcDevNumTdf      emAdcDevNumY;       // Y轴 ADC 设备号（DMA 模式下与 X 共用同一设备）
    emJoystickInvertTdf emInvertY;          // Y轴反向

    fix32_t             fOutputMax;         // 输出值上限
    fix32_t             fOutputMin;         // 输出值下限
#if ADC_IS_USE_DMA
    uint8_t             ucXAdcChannel;      // X轴在 DMA 缓存中的通道索引（0=MEM0, 1=MEM1...）
    uint8_t             ucYAdcChannel;      // Y轴在 DMA 缓存中的通道索引
#endif
} stJoystickStaticParamTdf;

/// @brief 摇杆运行参数
typedef struct {
    uint16_t usXAdcRaw;        // X轴 ADC 原始值
    uint16_t usYAdcRaw;        // Y轴 ADC 原始值
    fix32_t  fXValue;          // X轴映射后的输出值
    fix32_t  fYValue;          // Y轴映射后的输出值
} stJoystickRunningParamTdf;

/// @brief 摇杆设备参数
typedef struct {
    stJoystickStaticParamTdf  stStaticParam;
    stJoystickRunningParamTdf stRunningParam;
} stJoystickDeviceParamTdf;

/* 获取设备参数 */
const stJoystickDeviceParamTdf *c_pstGetJoystickDeviceParam(emJoystickDevNumTdf emDevNum);

/* 初始化（绑定 ADC 设备、配置映射范围及反向） */
void vJoystickDeviceInit(stJoystickStaticParamTdf *pstInit, emJoystickDevNumTdf emDevNum);

/* 周期执行：读取 X/Y 两路 ADC 并计算映射值，建议在主循环中周期性调用 */
void vJoystickPeriodExecute(emJoystickDevNumTdf emDevNum);

/* 读取映射后的输出值（定点数） */
fix32_t fJoystickGetX(emJoystickDevNumTdf emDevNum);
fix32_t fJoystickGetY(emJoystickDevNumTdf emDevNum);

/* 读取 ADC 原始值 */
uint16_t usJoystickGetRawX(emJoystickDevNumTdf emDevNum);
uint16_t usJoystickGetRawY(emJoystickDevNumTdf emDevNum);

#endif
#endif
