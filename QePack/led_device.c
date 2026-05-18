/**
  * @file       led_device.c
  * @author     Qe_xr
  * @version    V1.0.2
  * @date       2026/1/20
  * @brief      LED 驱动，基于 STM32 HAL 库
  *
  */
#include "led_device.h"
#if LED_IS_ENABLE

stLedDeviceParamTdf astLedDeviceParam[LED_DEV_NUM];

#if (QEPACK_PLATFORM == TI)
    #include "ti_platform.h"
#endif

/**
 * @brief       获取 LED 设备参数
 * @param       emDevNum   ：设备号
 * @return      const stLedDeviceParamTdf * ：LED 设备参数指针
 */
const stLedDeviceParamTdf *c_pstGetLedDeviceParam(emLedDevNumTdf emDevNum)
{
    return &astLedDeviceParam[emDevNum];
}

/**
 * @brief       拷贝运行参数
 * @param       emDevNum   ：设备号
 * @param       pstInit    ：初始化参数指针
 */
void vLedDeviceRunningParamInit(stLedRunningParamTdf *pstInit, emLedDevNumTdf emDevNum)
{
    memcpy(&astLedDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stLedRunningParamTdf));
}

/**
 * @brief       LED 更新引脚电平
 * @param       emDevNum   ：设备号
 * @note           点亮电平                当前状态                引脚输出电平
 *                 emOnLevel               emCurrentStatus         GPIO_PinState
 *             ------------------------------------------------------------------------
 *                 emLedOnLevel_Low(0)     emLedStatus_Off(0)      GPIO_PIN_SET(1)
 *                 emLedOnLevel_Low(0)     emLedStatus_On(1)       GPIO_PIN_RESET(0)
 *                 emLedOnLevel_High(1)    emLedStatus_Off(0)      GPIO_PIN_RESET(0)
 *                 emLedOnLevel_High(1)    emLedStatus_On(1)       GPIO_PIN_SET(1)
 *             ------------------------------------------------------------------------
 *             由以上真值表，有 GPIO_PinState = !(emOnLevel ^ emCurrentStatus)
 */
void vLedUpdatePinLevel(emLedDevNumTdf emDevNum)
{
    if(emDevNum >= LED_DEV_NUM) return;

    uint8_t ucOutput;
    ucOutput = !(astLedDeviceParam[emDevNum].stStaticParam.emOnLevel ^ astLedDeviceParam[emDevNum].stRunningParam.emCurrentStatus);

    #if (QEPACK_PLATFORM == TI)
        TI_GPIO_WritePin(astLedDeviceParam[emDevNum].stStaticParam.pstGpioPort, astLedDeviceParam[emDevNum].stStaticParam.usGpioPin, (GPIO_PinState)ucOutput);
    #else
        HAL_GPIO_WritePin(astLedDeviceParam[emDevNum].stStaticParam.pstGpioBase, astLedDeviceParam[emDevNum].stStaticParam.usGpioPin, (GPIO_PinState)ucOutput);
    #endif
}



/**
 * @brief       LED 点亮
 * @param       emDevNum   ：设备号
 * @note
 */
void vLedOn(emLedDevNumTdf emDevNum)
{
    if(emDevNum >= LED_DEV_NUM) return;
    astLedDeviceParam[emDevNum].stRunningParam.emCurrentStatus = emLedStatus_On;
    vLedUpdatePinLevel(emDevNum);
}

void vLedOff(emLedDevNumTdf emDevNum)
{
    if(emDevNum >= LED_DEV_NUM) return;
    astLedDeviceParam[emDevNum].stRunningParam.emCurrentStatus = emLedStatus_Off;
    vLedUpdatePinLevel(emDevNum);
}

void vLedToggle(emLedDevNumTdf emDevNum)
{
    if(emDevNum >= LED_DEV_NUM) return;
    astLedDeviceParam[emDevNum].stRunningParam.emCurrentStatus = (emLedStatusTdf)(astLedDeviceParam[emDevNum].stRunningParam.emCurrentStatus ^ 1);
    vLedUpdatePinLevel(emDevNum);
}

/**
 * @brief       LED 闪烁执行
 * @param       emDevNum    ：设备编号
 * @note 		每循环一次，计数+1，当<=ON值时点亮，当<=ON+OFF值时熄灭
 */
void vLedDeviceBilnkExecute(emLedDevNumTdf emDevNum)
{
    // 1. 根据当前计数，更新状态和引脚输出
    if(astLedDeviceParam[emDevNum].stRunningParam.ulCurrentCount < astLedDeviceParam[emDevNum].stRunningParam.ulOnCountThreshold)
    {
        vLedOn(emDevNum);
    }
    else if(astLedDeviceParam[emDevNum].stRunningParam.ulCurrentCount <= astLedDeviceParam[emDevNum].stRunningParam.ulOnCountThreshold + 
                                                                         astLedDeviceParam[emDevNum].stRunningParam.ulOffCountThreshold)
    {
        vLedOff(emDevNum);
    }
    
    // 2. 更新当前计数
    astLedDeviceParam[emDevNum].stRunningParam.ulCurrentCount++;
    if(astLedDeviceParam[emDevNum].stRunningParam.ulCurrentCount >= astLedDeviceParam[emDevNum].stRunningParam.ulOnCountThreshold + 
                                                                         astLedDeviceParam[emDevNum].stRunningParam.ulOffCountThreshold)
    {
        astLedDeviceParam[emDevNum].stRunningParam.ulCurrentCount = 0;
    }
}

/**
 * @brief       LED 呼吸灯执行
 * @param       emDevNum    ：设备编号
 * @note       呼吸灯以 LED 闪烁函数为基础，增加了【呼吸周期】
 *
 *             【闪烁周期】 = 【ON 持续时间（计数阈值）】+【OFF 持续时间（计数阈值）】
 *             【最大计数】 = 【呼吸周期】 / 【闪烁周期】
 *             【ON 持续时间（计数阈值）】和【OFF 持续时间（计数阈值）】，应该在每次【闪烁周期】开始前，重新计算
 *             目前使用 y = sin(x) * sin(x) 函数作为计算公式
 *
 *             举例：
 *
 *             上层每 1 ms 调用一次本函数
 *
 *             【闪烁周期】 = 10，【ON 持续时间（计数阈值）】 = 5，【OFF 持续时间（计数阈值）】 = 5，
 *             则 LED 点亮 5 ms，熄灭 5 ms，如此循环
 *
 *             【呼吸周期】 = 1000，则 LED 应在 1000 ms 内完成一次呼吸
 *             每个【呼吸周期】内包含 1000 / 10 = 100 个【闪烁周期】，即【最大计数】为 100
 *             
 *             在【当前呼吸计数值】为 3 时，【ON 持续时间（计数阈值）】 = sin(【当前呼吸计数值】 / 【最大计数】 * 3.1415926) * sin(【当前呼吸计数值】 / 【最大计数】 * 3.1415926) * 【闪烁周期】
 *                                                                         = sin(3 / 10 * 3.1415926) * sin(3 / 10 * 3.1415926) * 10
 *                                                                        = 4.999999999999999
 */
void vLedDeviceBreathExecute(emLedDevNumTdf emDevNum)
{
    if(emDevNum >= LED_DEV_NUM) return;

    stLedRunningParamTdf *pstRunning = &astLedDeviceParam[emDevNum].stRunningParam;
    uint32_t ulBreathCountMax;
    uint32_t ulBlinkPeriod;

    vLedDeviceBilnkExecute(emDevNum);

    ulBlinkPeriod = pstRunning->ulOnCountThreshold + pstRunning->ulOffCountThreshold;
    if (ulBlinkPeriod == 0) return;

    ulBreathCountMax = pstRunning->ulBreathPeriod / ulBlinkPeriod;
    if (ulBreathCountMax == 0) return;

    if(pstRunning->ulCurrentCount == 0)
    {
        float fPhase = PI * pstRunning->ulBreathCount / (float)ulBreathCountMax;
        float fSinVal = sin(fPhase);
        pstRunning->ulOnCountThreshold = ulBlinkPeriod * fSinVal * fSinVal;
        pstRunning->ulOffCountThreshold = ulBlinkPeriod - pstRunning->ulOnCountThreshold;

        pstRunning->ulBreathCount++;
        if(pstRunning->ulBreathCount >= ulBreathCountMax)
        {
            pstRunning->ulBreathCount = 0;
        }
    }
}

/**
 * @brief       LED 周期执行
 * @param       emDevNum    ：设备编号
 * @note       根据模式执行不同操作
 */
void vLedDevicePeriodExecute(emLedDevNumTdf emDevNum)
{
    if(emDevNum >= LED_DEV_NUM) return;

    switch(astLedDeviceParam[emDevNum].stRunningParam.emMode)
    {
        case emLedMode_Blink:
        {
            vLedDeviceBilnkExecute(emDevNum);
            break;
        }
        case emLedMode_Breath:
        {
            vLedDeviceBreathExecute(emDevNum);
            break;
        }
        default:
        {
            return;
        }
    }
}

/**
 * @brief       LED 设备初始化
 * @param       pstInit     ：初始化参数结构体的首地址
 * @param       emDevNum    ：设备编号
 * @note
 */
void vLedDeviceInit(stLedStaticParamTdf *pstInit, emLedDevNumTdf emDevNum)
{
    // 1. 初始化静态参数
    memcpy(&astLedDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stLedStaticParamTdf));
}

#endif
