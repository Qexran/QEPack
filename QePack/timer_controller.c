/**
  * @file       timer_controller.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/03/11
  * @brief      定时器控制器，基于 1ms 定时器可自定义任意时间触发回调
  *
  */
#include "timer_controller.h"
#if TIMER_CONTROLLER_IS_ENABLE

stTimerDeviceParamTdf astTimerDeviceParam[TIMER_CONTROLLER_NUM];

/**
 * @brief       获取定时器对象参数
 * @param       emDevNum   ：定时器对象号
 * @return      const stTimerDeviceParamTdf * ：定时器对象参数指针
 */
const stTimerDeviceParamTdf *c_pstGetTimerDeviceParam(emTimerDevNumTdf emDevNum)
{
    return &astTimerDeviceParam[emDevNum];
}

/**
 * @brief       定时器对象初始化
 * @param       pstInit     ：初始化参数结构体的首地址
 * @param       emDevNum    ：定时器对象号
 */
void vTimerDeviceInit(stTimerStaticParamTdf *pstInit, emTimerDevNumTdf emDevNum)
{
    if (pstInit == NULL || emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    memcpy(
        &astTimerDeviceParam[emDevNum].stStaticParam,
        pstInit,
        sizeof(stTimerStaticParamTdf)
    );
    
    memset(
        &astTimerDeviceParam[emDevNum].stRunningParam,
        0,
        sizeof(stTimerRunningParamTdf)
    );
}

/**
 * @brief       启动定时器
 * @param       emDevNum   ：定时器对象号
 */
void vTimerStart(emTimerDevNumTdf emDevNum)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    stTimerRunningParamTdf *pstRunning = &astTimerDeviceParam[emDevNum].stRunningParam;
    pstRunning->ucEnabled = 1;
    pstRunning->ulCounter = 0;
    pstRunning->ucTriggered = 0;
}

/**
 * @brief       停止定时器
 * @param       emDevNum   ：定时器对象号
 */
void vTimerStop(emTimerDevNumTdf emDevNum)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    astTimerDeviceParam[emDevNum].stRunningParam.ucEnabled = 0;
}

/**
 * @brief       重置定时器
 * @param       emDevNum   ：定时器对象号
 */
void vTimerReset(emTimerDevNumTdf emDevNum)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    stTimerRunningParamTdf *pstRunning = &astTimerDeviceParam[emDevNum].stRunningParam;
    pstRunning->ulCounter = 0;
    pstRunning->ucTriggered = 0;
}

/**
 * @brief       设置定时器周期
 * @param       emDevNum   ：定时器对象号
 * @param       period     ：定时周期（毫秒）
 */
void vTimerSetPeriod(emTimerDevNumTdf emDevNum, uint32_t period)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    astTimerDeviceParam[emDevNum].stStaticParam.ulPeriod = period;
    astTimerDeviceParam[emDevNum].stRunningParam.ulCounter = 0;
}

/**
 * @brief       设置定时器模式
 * @param       emDevNum   ：定时器对象号
 * @param       mode       ：定时器模式
 */
void vTimerSetMode(emTimerDevNumTdf emDevNum, emTimerModeTdf mode)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    astTimerDeviceParam[emDevNum].stStaticParam.emMode = mode;
}

/**
 * @brief       设置回调函数
 * @param       emDevNum   ：定时器对象号
 * @param       callback   ：回调函数指针
 */
void vTimerSetCallback(emTimerDevNumTdf emDevNum, vTimerCallback callback)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return;
    }
    
    astTimerDeviceParam[emDevNum].stStaticParam.vCallbackFcn = callback;
}

/**
 * @brief       定时器周期处理
 * @param       无
 */
void vTimerTickHandler(void)
{
    for (int i = 0; i < TIMER_CONTROLLER_NUM; i++) {
        stTimerDeviceParamTdf *pstDev = &astTimerDeviceParam[i];
        stTimerStaticParamTdf *pstStatic = &pstDev->stStaticParam;
        stTimerRunningParamTdf *pstRunning = &pstDev->stRunningParam;
        
        if (pstRunning->ucEnabled == 0) {
            continue;
        }
        
        if (pstStatic->emMode == emTimerMode_Once && pstRunning->ucTriggered) {
            continue;
        }
        
        pstRunning->ulCounter++;
        
        if (pstRunning->ulCounter >= pstStatic->ulPeriod) {
            if (pstStatic->vCallbackFcn != NULL) {
                pstStatic->vCallbackFcn((emTimerDevNumTdf)i);
            }
            
            if (pstStatic->emMode == emTimerMode_Once) {
                pstRunning->ucTriggered = 1;
                pstRunning->ucEnabled = 0;
            } else {
                pstRunning->ulCounter = 0;
            }
        }
    }
}

/**
 * @brief       查询定时器是否使能
 * @param       emDevNum   ：定时器对象号
 * @return      uint8_t    ：1：使能 0：禁用
 */
uint8_t ucTimerIsEnabled(emTimerDevNumTdf emDevNum)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return 0;
    }
    
    return astTimerDeviceParam[emDevNum].stRunningParam.ucEnabled;
}

/**
 * @brief       获取剩余时间
 * @param       emDevNum   ：定时器对象号
 * @return      uint32_t   ：剩余毫秒数
 */
uint32_t ulTimerGetRemaining(emTimerDevNumTdf emDevNum)
{
    if (emDevNum >= TIMER_CONTROLLER_NUM) {
        return 0;
    }
    
    stTimerDeviceParamTdf *pstDev = &astTimerDeviceParam[emDevNum];
    if (pstDev->stRunningParam.ucEnabled == 0) {
        return 0;
    }
    
    return pstDev->stStaticParam.ulPeriod - pstDev->stRunningParam.ulCounter;
}

#endif
