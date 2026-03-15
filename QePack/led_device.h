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

/* 开始动态执行 */
void vLedDevicePeriodExecute(emLedDevNumTdf emDevNum);

/* 初始化相关 */
void vLedDeviceRunningParamInit(stLedRunningParamTdf *pstInit, emLedDevNumTdf emDevNum);	// 初始化动态参数
void vLedDeviceInit(stLedStaticParamTdf *pstInit, emLedDevNumTdf emDevNum);					// 初始化静态参数

#endif

#endif

/*
    LED 驱动模块使用示例
    
    1. 静态模式（直接控制LED亮灭）
    
    // 初始化LED静态参数
    void vLedConfigInit(void)
    {
        stLedStaticParamTdf stLed0Static;
        stLed0Static.pstGpioBase = GPIOC;
        stLed0Static.usGpioPin = GPIO_PIN_13;
        stLed0Static.emOnLevel = emLedOnLevel_Low;  // 低电平点亮
        
        // 初始化LED0
        vLedDeviceInit(&stLed0Static, LED0);
        
        // 初始化运行参数
        stLedRunningParamTdf stLed0Running;
        memset(&stLed0Running, 0, sizeof(stLed0Running));
        stLed0Running.emCurrentStatus = emLedStatus_Off;
        stLed0Running.emMode = emLedMode_Static;
        vLedDeviceRunningParamInit(&stLed0Running, LED0);
    }
    
    // 主循环
    int main(void)
    {
        HAL_Init();
        SystemClock_Config();
        MX_GPIO_Init();
        
        vLedConfigInit();
        
        while(1)
        {
            // 直接控制LED
            vLedOn(LED0);
            HAL_Delay(1000);
            vLedOff(LED0);
            HAL_Delay(1000);
            
            // 翻转LED
            vLedToggle(LED0);
            HAL_Delay(500);
        }
    }
    
    
    2. 闪烁模式
    
    // 配置LED闪烁参数
    void vLedBlinkConfig(void)
    {
        stLedRunningParamTdf stLed0Running;
        memset(&stLed0Running, 0, sizeof(stLed0Running));
        stLed0Running.emCurrentStatus = emLedStatus_Off;
        stLed0Running.emMode = emLedMode_Blink;
        stLed0Running.ulOnCountThreshold = 500;   // 亮500ms
        stLed0Running.ulOffCountThreshold = 500;  // 灭500ms
        stLed0Running.ulCurrentCount = 0;
        vLedDeviceRunningParamInit(&stLed0Running, LED0);
    }
    
    // 主循环
    int main(void)
    {
        HAL_Init();
        SystemClock_Config();
        MX_GPIO_Init();
        vLedConfigInit();  // 先初始化静态参数
        vLedBlinkConfig();  // 再配置运行参数
        
        while(1)
        {
            // 1ms周期调用
            vLedDevicePeriodExecute(LED0);
            HAL_Delay(1);
        }
    }
    
    
    3. 呼吸灯模式
    
    // 配置LED呼吸灯参数
    void vLedBreathConfig(void)
    {
        stLedRunningParamTdf stLed0Running;
        memset(&stLed0Running, 0, sizeof(stLed0Running));
        stLed0Running.emCurrentStatus = emLedStatus_Off;
        stLed0Running.emMode = emLedMode_Breath;
        stLed0Running.ulOnCountThreshold = 5;    // 闪烁周期中亮的时间
        stLed0Running.ulOffCountThreshold = 5;   // 闪烁周期中灭的时间
        stLed0Running.ulBreathPeriod = 2000;     // 呼吸周期2000ms
        stLed0Running.ulCurrentCount = 0;
        vLedDeviceRunningParamInit(&stLed0Running, LED0);
    }
    
    // 主循环
    int main(void)
    {
        HAL_Init();
        SystemClock_Config();
        MX_GPIO_Init();
        vLedConfigInit();      // 先初始化静态参数
        vLedBreathConfig();     // 再配置运行参数
        
        while(1)
        {
            // 1ms周期调用
            vLedDevicePeriodExecute(LED0);
            HAL_Delay(1);
        }
    }
    
    注意：
    - 闪烁模式和呼吸灯模式需要在1ms周期中调用 vLedDevicePeriodExecute()
    - 呼吸灯模式以闪烁模式为基础，会自动调整闪烁的占空比
    - emLedOnLevel_Low 表示低电平点亮，emLedOnLevel_High 表示高电平点亮
*/
