/**
  * @file       timer_controller.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/03/12
  * @brief      定时器控制器，基于 1ms 定时器可自定义任意时间触发回调
  *
  */
#include "project_config.h"
#if TIMER_CONTROLLER_IS_ENABLE

#ifndef _TIMER_CONTROLLER_H_
#define _TIMER_CONTROLLER_H_

#include "string.h"
#include "tim.h"

/**
 * @brief          定时器对象号枚举
 * @note 
 */
typedef enum
{
    emTimerDevNum0        = 0,
    emTimerDevNum1,
    emTimerDevNum2,
    emTimerDevNum3,
    emTimerDevNum4,
    emTimerDevNum5,
    emTimerDevNum6,
    emTimerDevNum7,
}
emTimerDevNumTdf;

/**
 * @brief          定时器模式枚举
 * @note 
 */
typedef enum
{
    emTimerMode_Once        = 0,    // 单次触发模式
    emTimerMode_Periodic,           // 周期触发模式
}
emTimerModeTdf;

/**
 * @brief          定时器回调函数类型
 * @note 
 */
typedef void (*vTimerCallback)(emTimerDevNumTdf emDevNum);

/**
 * @brief          静态参数定义
 * @note           
 */
typedef struct
{
    uint32_t            ulPeriod;        // 定时周期（毫秒）
    emTimerModeTdf      emMode;          // 定时器模式
    vTimerCallback      vCallbackFcn;    // 回调函数指针
}
stTimerStaticParamTdf;

/**
 * @brief          运行参数定义
 * @note           
 */
typedef struct
{
    uint32_t            ulCounter;       // 当前计数器
    uint8_t             ucEnabled;       // 使能标志（0：禁用 1：使能）
    uint8_t             ucTriggered;     // 触发标志（单次模式使用）
}
stTimerRunningParamTdf;

/**
 * @brief          定时器对象参数定义
 * @note           
 */
typedef struct
{
    stTimerStaticParamTdf     stStaticParam;  // 静态参数
    stTimerRunningParamTdf    stRunningParam; // 运行参数
}
stTimerDeviceParamTdf;


/* 获取定时器对象参数 */
const stTimerDeviceParamTdf *c_pstGetTimerDeviceParam(emTimerDevNumTdf emDevNum);

/* 初始化相关 */
void vTimerDeviceInit(stTimerStaticParamTdf *pstInit, emTimerDevNumTdf emDevNum);

/* 定时器控制 */
void vTimerStart(emTimerDevNumTdf emDevNum);
void vTimerStop(emTimerDevNumTdf emDevNum);
void vTimerReset(emTimerDevNumTdf emDevNum);

/* 修改参数 */
void vTimerSetPeriod(emTimerDevNumTdf emDevNum, uint32_t period);
void vTimerSetMode(emTimerDevNumTdf emDevNum, emTimerModeTdf mode);
void vTimerSetCallback(emTimerDevNumTdf emDevNum, vTimerCallback callback);

/* 周期执行*/
void vTimerTickHandler(void);

/* 查询状态 */
uint8_t ucTimerIsEnabled(emTimerDevNumTdf emDevNum);
uint32_t ulTimerGetRemaining(emTimerDevNumTdf emDevNum);

#endif

#endif
/*
    // 定义回调函数
    void vTimer0Callback(emTimerDevNumTdf emDevNum) {
        vLedToggle(LED0); // 翻转 LED
    }

    // 配置静态参数
    stTimerStaticParamTdf stTimer0Static;
    stTimer0Static.ulPeriod = 1000;               // 1000ms 周期
    stTimer0Static.emMode = emTimerMode_Periodic;  // 周期模式
    stTimer0Static.vCallbackFcn = vTimer0Callback;   // 回调函数

    // 初始化定时器对象
    vTimerDeviceInit(&stTimer0Static, TIMER0); 
    
    // 启动定时器
    vTimerStart(TIMER0);
    
    
    //其他常用操作
    
    // 停止定时器
    vTimerStop(TIMER0);

    // 重置定时器
    vTimerReset(TIMER0);

    // 修改周期
    vTimerSetPeriod(TIMER0, 500); // 改为 500ms

    // 修改模式
    vTimerSetMode(TIMER0, emTimerMode_Once); // 改为单次模式

    // 修改回调
    vTimerSetCallback(TIMER0, vNewCallback);

    // 查询状态
    uint8_t enabled = ucTimerIsEnabled(TIMER0);
    uint32_t remaining = ulTimerGetRemaining(TIMER0);
    
    // 1ms 定时器中断回调中必须调用
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim == &TIMER_CONTROLLER_TICK_TIM) {
            vTimerTickHandler();
        }
    }
*/
