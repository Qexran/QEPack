/**
  * @file       led_device.h
  * @author     Qe_xr
  * @version    V1.0.1
  * @date       2026/1/20
  * @brief      LED 驱动，基于 STM32 HAL 库
  * 
  */
#include "project_config.h"
#if LED_IS_ENABLE

#ifndef _LED_DEVICE_H_
#define _LED_DEVICE_H_

#include "string.h"
#include "math.h"

/**
 * @brief          设备号枚举
 * @note 
 */
typedef enum
{
    emLedDevNum0        = 0,
    emLedDevNum1,
    emLedDevNum2,
    emLedDevNum3,
    emLedDevNum4,
    emLedDevNum5,
    emLedDevNum6,
    emLedDevNum7,
    emLedDevNum8,
} 
emLedDevNumTdf;

/**
 * @brief          LED ON 时的电平枚举
 * @note 
 */
typedef enum
{
    emLedOnLevel_Low    = 0,                // 低电平点亮
    emLedOnLevel_High,                      // 高电平点亮
}
emLedOnLevelTdf;

/**
 * @brief          LED 状态枚举
 * @note 
 */
typedef enum
{
    emLedStatus_Off     = 0,                // 状态 OFF
    emLedStatus_On,                         // 状态 ON
}
emLedStatusTdf;

/**
 * @brief          LED 模式枚举
 * @note 
 */
typedef enum
{
    emLedMode_Static        = 0,            // 静态模式
    emLedMode_Blink,                        // 闪烁模式
    emLedMode_Breath,                       // 呼吸灯模式
}
emLedModeTdf;

/**
 * @brief          静态参数定义
 * @note           
 */
typedef struct
{
    GPIO_TypeDef        *pstGpioBase;       // 使用的 GPIOx
    uint16_t            usGpioPin;          // 使用的 GPIO_PIN_x
    emLedOnLevelTdf     emOnLevel;          // LED 点亮时的电平
}
stLedStaticParamTdf;

/**
 * @brief          运行参数定义
 * @note           
 */
typedef struct
{
    emLedStatusTdf      emCurrentStatus;        // 当前状态
    emLedModeTdf        emMode;                 // 模式
	
	
    /* LED闪烁相关函数: vLedDeviceBilnkExecute */
    uint32_t            ulCurrentCount;         // 当前计数
    uint32_t            ulOnCountThreshold;     // ON  计数阈值
    uint32_t            ulOffCountThreshold;    // OFF 计数阈值
    
	/* LED呼吸灯相关函数: vLedDeviceBreathExecute */
    uint32_t            ulBreathPeriod;         // 呼吸周期
	
}
stLedRunningParamTdf;

/**
 * @brief          结构参数定义
 * @note           
 */
typedef struct
{
    stLedStaticParamTdf     stStaticParam;  // 静态参数
    stLedRunningParamTdf    stRunningParam; // 运行参数
}
stLedDeviceParamTdf;


/* 获取当前LED的运行参数 */
const stLedDeviceParamTdf *c_pstGetLedDeviceParam(emLedDevNumTdf emDevNum);

/* 基本控制 */
void vLedOn(emLedDevNumTdf emDevNum);		// 开启LED
void vLedOff(emLedDevNumTdf emDevNum);		// 关闭LED
void vLedToggle(emLedDevNumTdf emDevNum);	// 翻转LED

/* 开始动态执行（应放在while中） */
void vLedDevicePeriodExecute(emLedDevNumTdf emDevNum);

/* 初始化相关 */
void vLedDeviceRunningParamInit(stLedRunningParamTdf *pstInit, emLedDevNumTdf emDevNum);	// 初始化动态参数
void vLedDeviceInit(stLedStaticParamTdf *pstInit, emLedDevNumTdf emDevNum);					// 初始化静态参数

#endif

#endif
