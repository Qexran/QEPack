/**
  * @file       key_device.h
  * @author     Qe_xr
  * @version    V1.0.1
  * @date       2026/1/20
  * @brief      按键驱动，基于 STM32 HAL 库
  * 
  */
#include "project_config.h"
#if KEY_IS_ENABLE

#ifndef _KEY_DEVICE_H_
#define _KEY_DEVICE_H_

#include "string.h"

/// @brief          按键设备号枚举
/// @note 
typedef enum
{
    emKeyDevNum0        = 0,
    emKeyDevNum1,
    emKeyDevNum2,
    emKeyDevNum3,
    emKeyDevNum4,
    emKeyDevNum5,
    emKeyDevNum6,
    emKeyDevNum7,
} 
emKeyDevNumTdf;

/// @brief          按键有效电平枚举
/// @note 
typedef enum
{
    emKeyValidLevel_Low    = 0,                // 低电平有效（按键按下时为低电平）
    emKeyValidLevel_High,                      // 高电平有效（按键按下时为高电平）
}
emKeyValidLevelTdf;

/// @brief          按键当前状态枚举
/// @note 
typedef enum
{
    emKeyStatus_Release     = 0,                // 未按下（释放）
    emKeyStatus_Press,                         // 按下
}
emKeyStatusTdf;

/// @brief          按键事件枚举
/// @note 
typedef enum
{
    emKeyEvent_None     = 0,                   // 无事件
    emKeyEvent_Click,                          // 单击事件
    emKeyEvent_DoubleClick,                    // 双击事件
    emKeyEvent_LongPress,                      // 长按事件
    emKeyEvent_Repeat                          // 连发事件
}
emKeyEventTdf;

/// @brief          按键静态参数定义
/// @note           
typedef struct
{
    GPIO_TypeDef        *pstGpioBase;           // 使用的 GPIOx
    uint16_t            usGpioPin;              // 使用的 GPIO_PIN_x
    emKeyValidLevelTdf  emValidLevel;           // 按键有效电平
    uint32_t            ulDebounceThreshold;    // 消抖计数阈值（单位：周期，如1ms周期则阈值为20表示20ms消抖）
    uint32_t            ulLongPressThreshold;   // 长按计数阈值（单位：周期）
    uint32_t            ulDoubleClickThreshold; // 双击间隔阈值（单位：周期）
}
stKeyStaticParamTdf;


/// @brief          按键运行参数定义
/// @note           
typedef struct
{
    emKeyStatusTdf      emCurrentStatus;        // 当前按键状态
    emKeyEventTdf       emKeyEvent;             // 按键事件标志
    
    uint32_t            ulDebounceCount;        // 消抖计数
    uint32_t            ulLongPressCount;       // 长按计数
    uint32_t            ulDoubleClickCount;     // 双击计时计数
    uint8_t             ucDoubleClickFlag;      // 双击检测中标志
    uint8_t             ucLongPressTrigger;     // 长按触发标志
}
stKeyRunningParamTdf;

/// @brief          按键设备参数定义
/// @note           
typedef struct
{
    stKeyStaticParamTdf     stStaticParam;  // 静态参数
    stKeyRunningParamTdf    stRunningParam; // 运行参数
}
stKeyDeviceParamTdf;

/* 获取当前按键的设备参数 */
const stKeyDeviceParamTdf *c_pstGetKeyDeviceParam(emKeyDevNumTdf emDevNum);

/* 初始化相关 */
void vKeyDeviceRunningParamInit(stKeyRunningParamTdf *pstInit, emKeyDevNumTdf emDevNum); // 初始化运行参数
void vKeyDeviceInit(stKeyStaticParamTdf *pstInit, emKeyDevNumTdf emDevNum);              // 初始化静态参数

/* 周期执行 */
void vKeyDevicePeriodExecute(emKeyDevNumTdf emDevNum);

/* 事件获取与清除 */
emKeyEventTdf emKeyGetEvent(emKeyDevNumTdf emDevNum);   // 获取按键事件
void vKeyClearEvent(emKeyDevNumTdf emDevNum);           // 清除按键事件

#endif

#endif

/*

#include "key_device.h"

// 按键静态参数初始化
void vKeyConfigInit(void)
{
    stKeyStaticParamTdf stKey0Init;
    stKey0Init.pstGpioBase = GPIOB;
    stKey0Init.usGpioPin = GPIO_PIN_0;
    stKey0Init.emValidLevel = emKeyValidLevel_Low;
    stKey0Init.ulDebounceThreshold = 20;        // 20ms消抖
    stKey0Init.ulLongPressThreshold = 1000;     // 1s长按
    stKey0Init.ulDoubleClickThreshold = 300;     // 300ms双击间隔
    
    // 初始化按键0
    vKeyDeviceInit(&stKey0Init, KEY0);
    
    // 初始化运行参数
    stKeyRunningParamTdf stKeyRunInit;
    memset(&stKeyRunInit, 0, sizeof(stKeyRunInit));
    vKeyDeviceRunningParamInit(&stKeyRunInit, KEY0);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    
    vKeyConfigInit();
    
    while (1)
    {
        // 1ms周期执行（需自行配置定时器或延时实现1ms周期）
        vKeyDevicePeriodExecute(KEY0);
        
        // 检测按键事件
        emKeyEventTdf emKeyEvent = emKeyGetEvent(KEY0);
        if(emKeyEvent != emKeyEvent_None)
        {
            switch(emKeyEvent)
            {
                case emKeyEvent_Click:
                    // 单击处理逻辑
                    break;
                case emKeyEvent_DoubleClick:
                    // 双击处理逻辑
                    break;
                case emKeyEvent_LongPress:
                    // 长按处理逻辑
                    break;
                default:
                    break;
            }
            vKeyClearEvent(KEY0); // 清除事件标志
        }
        
        HAL_Delay(1); // 模拟1ms周期
    }
}

*/
