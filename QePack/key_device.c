/**
  * @file       key_device.c
  * @author     Qe_xr
  * @version    V1.0.5
  * @date       2026/1/29
  * @brief      按键驱动，基于 STM32 HAL 库
  *
  */

#include "key_device.h"
#if KEY_IS_ENABLE


// 需在project_config.h中定义KEY_DEV_NUM，例如：#define KEY_DEV_NUM 9
stKeyDeviceParamTdf astKeyDeviceParam[KEY_DEV_NUM];

/**
 * @brief      获取按键设备参数
 * @param      emDevNum   ：设备号
 * @note       返回值是只读的stKeyDeviceParamTdf型指针
 * @return     const stKeyDeviceParamTdf * ：按键设备参数指针
 */
const stKeyDeviceParamTdf *c_pstGetKeyDeviceParam(emKeyDevNumTdf emDevNum)
{
    return &astKeyDeviceParam[emDevNum];
}

/**
 * @brief      初始化按键运行参数
 * @param      pstInit    ：初始化参数指针
 * @param      emDevNum   ：设备号
 */
void vKeyDeviceRunningParamInit(stKeyRunningParamTdf *pstInit, emKeyDevNumTdf emDevNum)
{
    memcpy(&astKeyDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stKeyRunningParamTdf));
}

/**
 * @brief      初始化按键静态参数
 * @param      pstInit    ：初始化参数指针
 * @param      emDevNum   ：设备号
 */
void vKeyDeviceInit(stKeyStaticParamTdf *pstInit, emKeyDevNumTdf emDevNum)
{
    memcpy(&astKeyDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stKeyStaticParamTdf));
	
	/* 运行参数默认初始化 */
	memset(&astKeyDeviceParam[emDevNum].stRunningParam, 0, sizeof(stKeyRunningParamTdf));
}

/**
 * @brief      读取按键原始电平状态
 * @param      emDevNum   ：设备号
 * @return     1：按下状态，0：释放状态
 */
static __INLINE uint8_t ucKeyReadRawState(emKeyDevNumTdf emDevNum)
{
    stKeyStaticParamTdf *pstStaticParam = &astKeyDeviceParam[emDevNum].stStaticParam;
    uint8_t ucRawLevel;
    
    // 读取GPIO引脚电平
    ucRawLevel = HAL_GPIO_ReadPin(pstStaticParam->pstGpioBase, pstStaticParam->usGpioPin);
    
    // 根据有效电平判断按键是否按下
    if(pstStaticParam->emValidLevel == emKeyValidLevel_Low)
    {
        return (ucRawLevel == GPIO_PIN_RESET) ? 1 : 0;
    }
    else
    {
        return (ucRawLevel == GPIO_PIN_SET) ? 1 : 0;
    }
}

/**
 * @brief      清除按键运行参数
 * @param      emDevNum   ：设备号
 */
static __INLINE void vKeyClearRunningState(emKeyDevNumTdf emDevNum)
{
    stKeyRunningParamTdf *pstRunParam = &astKeyDeviceParam[emDevNum].stRunningParam;
    pstRunParam->ulDebounceCount = 0;
    pstRunParam->ulLongPressCount = 0;
    pstRunParam->ulDoubleClickCount = 0;
    pstRunParam->ucDoubleClickFlag = 0;
    pstRunParam->ucLongPressTrigger = 0;
}

/**
 * @brief      按键周期执行
 * @param      emDevNum   ：设备号
 * @note       需放在1ms周期的while循环中执行
 */
void vKeyDevicePeriodExecute(emKeyDevNumTdf emDevNum)
{
    uint8_t ucCurPressState = ucKeyReadRawState(emDevNum);
    stKeyRunningParamTdf *pstRunParam = &astKeyDeviceParam[emDevNum].stRunningParam;
    stKeyStaticParamTdf *pstStaticParam = &astKeyDeviceParam[emDevNum].stStaticParam;
    
    // 1. 消抖处理
    if(ucCurPressState != pstRunParam->emCurrentStatus)
    {
        pstRunParam->ulDebounceCount++;
        if(pstRunParam->ulDebounceCount >= pstStaticParam->ulDebounceThreshold)
        {
            // 消抖完成，更新按键状态
            pstRunParam->emCurrentStatus = (emKeyStatusTdf)ucCurPressState;
            pstRunParam->ulDebounceCount = 0;
        }
    }
    else
    {
        pstRunParam->ulDebounceCount = 0;
    }
    
    // 2. 按键状态处理
    if(pstRunParam->emCurrentStatus == emKeyStatus_Press)
    {
        // 2.1 按键按下状态 - 先累加长按计数
        // （无论长按功能是否启用，确保单击能检测到按下）
        pstRunParam->ulLongPressCount++;
        
        // 仅当长按阈值>0时，才处理长按触发逻辑
        if((pstStaticParam->ulLongPressThreshold > 0) && (pstRunParam->ucLongPressTrigger == 0))
        {
            if(pstRunParam->ulLongPressCount >= pstStaticParam->ulLongPressThreshold)
            {
                // 触发长按事件
                pstRunParam->emKeyEvent = emKeyEvent_LongPress;
                pstRunParam->ucLongPressTrigger = 1;
                pstRunParam->ucDoubleClickFlag = 0; // 长按后取消双击检测
            }
        }
    }
    else
    {
        // 2.2 按键释放状态
        if(pstRunParam->ucLongPressTrigger == 1)
        {
            // 长按后释放，清除状态
            vKeyClearRunningState(emDevNum);
        }
        else if(pstRunParam->ulLongPressCount > 0)
        {
            // 短按释放，处理单击/双击逻辑
            if(pstStaticParam->ulDoubleClickThreshold > 0)
            {
                // 双击功能启用时的逻辑
                if(pstRunParam->ucDoubleClickFlag == 0)
                {
                    // 第一次按下释放，进入双击检测
                    pstRunParam->ucDoubleClickFlag = 1;
                    pstRunParam->ulDoubleClickCount = 0;
                }
                else
                {
                    // 第二次按下释放，触发双击事件
                    pstRunParam->emKeyEvent = emKeyEvent_DoubleClick;
                    pstRunParam->ucDoubleClickFlag = 0;
                    vKeyClearRunningState(emDevNum);
                }
            }
            else
            {
                // 双击功能禁用，直接触发单击事件
                pstRunParam->emKeyEvent = emKeyEvent_Click;
                vKeyClearRunningState(emDevNum);
            }
            pstRunParam->ulLongPressCount = 0;
        }
        else if(pstRunParam->ucDoubleClickFlag == 1)
        {
            // 双击检测中，累计计时
            if(pstStaticParam->ulDoubleClickThreshold > 0)
            {
                pstRunParam->ulDoubleClickCount++;
                if(pstRunParam->ulDoubleClickCount >= pstStaticParam->ulDoubleClickThreshold)
                {
                    // 双击超时，触发单击事件
                    pstRunParam->emKeyEvent = emKeyEvent_Click;
                    pstRunParam->ucDoubleClickFlag = 0;
                    vKeyClearRunningState(emDevNum);
                }
            }
        }
    }
}

/**
 * @brief      获取按键事件
 * @param      emDevNum   ：设备号
 * @return     按键事件枚举值
 */
emKeyEventTdf emKeyGetEvent(emKeyDevNumTdf emDevNum)
{
    return astKeyDeviceParam[emDevNum].stRunningParam.emKeyEvent;
}

/**
 * @brief      清除按键事件
 * @param      emDevNum   ：设备号
 */
void vKeyClearEvent(emKeyDevNumTdf emDevNum)
{
    astKeyDeviceParam[emDevNum].stRunningParam.emKeyEvent = emKeyEvent_None;
}

#endif
