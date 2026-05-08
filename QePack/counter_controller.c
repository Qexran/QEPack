/**
  * @file       counter_controller.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/8
  * @brief      计数器控制器实现
  * 
  */
  
#include "counter_controller.h"

#if COUNTER_IS_ENABLE

/* 全局计数器设备数组 */
stCounterRunningParamTdf g_astCounters[emCounterDevMax] = {0};

/**
 * @brief          初始化计数器
 * @param  emDevNum : 计数器设备号
 * @param  ulTargetCount : 目标计数次数
 */
void vCounterInit(emCounterDevNumTdf emDevNum, uint32_t ulTargetCount)
{
    if (emDevNum >= emCounterDevMax) return;
    
    g_astCounters[emDevNum].ulCurrentCount = 0;
    g_astCounters[emDevNum].ulTargetCount = ulTargetCount;
    g_astCounters[emDevNum].emState = emCounterStateIdle;
}

/**
 * @brief          重置计数器
 * @param  emDevNum : 计数器设备号
 */
void vCounterReset(emCounterDevNumTdf emDevNum)
{
    if (emDevNum >= emCounterDevMax) return;
    
    g_astCounters[emDevNum].ulCurrentCount = 0;
    g_astCounters[emDevNum].emState = emCounterStateIdle;
}

/**
 * @brief          增加计数
 * @param  emDevNum : 计数器设备号
 * @param  ulIncrement : 增加的计数值(默认1)
 */
void vCounterIncrement(emCounterDevNumTdf emDevNum, uint32_t ulIncrement)
{
    if (emDevNum >= emCounterDevMax) return;
    
    stCounterRunningParamTdf *pstCounter = &g_astCounters[emDevNum];
    
    pstCounter->ulCurrentCount += ulIncrement;
    pstCounter->emState = emCounterStateRunning;
}

/**
 * @brief          减少计数
 * @param  emDevNum : 计数器设备号
 * @param  ulDecrement : 减少的计数值(默认1)
 */
void vCounterDecrement(emCounterDevNumTdf emDevNum, uint32_t ulDecrement)
{
    if (emDevNum >= emCounterDevMax) return;
    
    stCounterRunningParamTdf *pstCounter = &g_astCounters[emDevNum];
    
    if (pstCounter->ulCurrentCount >= ulDecrement) {
        pstCounter->ulCurrentCount -= ulDecrement;
    } else {
        pstCounter->ulCurrentCount = 0;
    }
    
    if (pstCounter->emState == emCounterStateReached && pstCounter->ulCurrentCount < pstCounter->ulTargetCount) {
        pstCounter->emState = emCounterStateRunning;
    }
}

/**
 * @brief          设置计数值
 * @param  emDevNum : 计数器设备号
 * @param  ulCount : 设置的计数值
 */
void vCounterSetCount(emCounterDevNumTdf emDevNum, uint32_t ulCount)
{
    if (emDevNum >= emCounterDevMax) return;
    
    g_astCounters[emDevNum].ulCurrentCount = ulCount;
    g_astCounters[emDevNum].emState = emCounterStateRunning;
}

/**
 * @brief          设置目标次数
 * @param  emDevNum : 计数器设备号
 * @param  ulTarget : 目标计数值
 */
void vCounterSetTarget(emCounterDevNumTdf emDevNum, uint32_t ulTarget)
{
    if (emDevNum >= emCounterDevMax) return;
    
    g_astCounters[emDevNum].ulTargetCount = ulTarget;
    g_astCounters[emDevNum].emState = emCounterStateRunning;
}

/**
 * @brief          获取当前计数值
 * @param  emDevNum : 计数器设备号
 * @return         当前计数值
 */
uint32_t ulCounterGetCurrent(emCounterDevNumTdf emDevNum)
{
    if (emDevNum >= emCounterDevMax) return 0;
    
    return g_astCounters[emDevNum].ulCurrentCount;
}

/**
 * @brief          获取目标计数值
 * @param  emDevNum : 计数器设备号
 * @return         目标计数值
 */
uint32_t ulCounterGetTarget(emCounterDevNumTdf emDevNum)
{
    if (emDevNum >= emCounterDevMax) return 0;
    
    return g_astCounters[emDevNum].ulTargetCount;
}

/**
 * @brief          获取计数器状态
 * @param  emDevNum : 计数器设备号
 * @return         计数器状态枚举
 */
emCounterStateTdf emCounterGetState(emCounterDevNumTdf emDevNum)
{
    if (emDevNum >= emCounterDevMax) return emCounterStateIdle;
    
    return g_astCounters[emDevNum].emState;
}

/**
 * @brief          检查是否达到目标次数
 * @param  emDevNum : 计数器设备号
 * @return         0=未达到,1=已达到
 */
uint8_t u8CounterIsReached(emCounterDevNumTdf emDevNum)
{
    if (emDevNum >= emCounterDevMax) return 0;
    
    stCounterRunningParamTdf *pstCounter = &g_astCounters[emDevNum];
    
    if (pstCounter->ulCurrentCount >= pstCounter->ulTargetCount) {
        if (pstCounter->ulTargetCount > 0) {
            pstCounter->emState = emCounterStateReached;
        }
        return 1;
    }
    
    return 0;
}

/**
 * @brief          周期执行
 * @param  emDevNum : 计数器设备号
 */
void vCounterPeriodExecute(emCounterDevNumTdf emDevNum)
{
    (void)emDevNum;
}

#endif
