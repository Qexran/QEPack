/**
  * @file       ultrasonic_device.c
  * @author     Qe_xr
  * @version    V1.0.2
  * @date       2026/2/11
  * @brief      超声波测量驱动，基于 STM32 HAL 库
  * @note		思路: 通过占用定时器两个输入捕获通道(直接+间接)
  * @note		分别接收 Echo端 的上升沿和下降沿，这个定时器的计数值即为时间
  */

#include "ultrasonic_device.h"
#if ULTRASONIC_IS_ENABLE

#include "delay.h"  // 需确保存在us级延时函数


// 超声波设备参数数组
stUltrasonicDeviceParamTdf astUltrasonicDeviceParam[ULTRASONIC_DEV_NUM];

/// @brief      获取超声波设备参数（只读）
/// @param      emDevNum   ：设备号
const stUltrasonicDeviceParamTdf *c_pstGetUltrasonicDeviceParam(emUltrasonicDevNumTdf emDevNum)
{
    return &astUltrasonicDeviceParam[emDevNum];
}

/// @brief      初始化超声波运行参数
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
void vUltrasonicDeviceRunningParamInit(stUltrasonicRunningParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum)
{
    memcpy(&astUltrasonicDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stUltrasonicRunningParamTdf) / sizeof(uint8_t));
}

/// @brief      初始化超声波静态参数（硬件配置）
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
void vUltrasonicDeviceInit(stUltrasonicStaticParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum)
{
    // 拷贝静态参数（硬件配置）
    memcpy(&astUltrasonicDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stUltrasonicStaticParamTdf) / sizeof(uint8_t));
    
    // 默认运行参数初始化
    stUltrasonicRunningParamTdf stDefaultRunning = {
        .emCurrentStatus = emUltrasonicStatus_Idle,
        .ucIsSuccess = 0,
        .ulCCR1 = 0,
        .ulCCR2 = 0,
        .ulTimeoutMs = 50,    // 默认超时50ms
        .fDistance = 0.0f,
        .ulExpireTime = 0,
		.fTemperature = ULTRASONIC_DEFAULT_ENV_TEMP
    };
    vUltrasonicDeviceRunningParamInit(&stDefaultRunning, emDevNum);
}

/// @brief      启动超声波单次测量
/// @param      emDevNum   ：设备号
void vUltrasonicStartMeasure(emUltrasonicDevNumTdf emDevNum)
{		
    stUltrasonicStaticParamTdf *pstStatic = &astUltrasonicDeviceParam[emDevNum].stStaticParam;
    stUltrasonicRunningParamTdf *pstRunning = &astUltrasonicDeviceParam[emDevNum].stRunningParam;
	
	// 状态检查
	if (pstRunning->emCurrentStatus == emUltrasonicStatus_Measuring) {
		return;
	}
    
    // 1. 置为测量中状态
    pstRunning->emCurrentStatus = emUltrasonicStatus_Measuring;
    pstRunning->ucIsSuccess = 0;
    pstRunning->fDistance = 0.0f;
    
    // 2. 重置定时器状态
    __HAL_TIM_SET_COUNTER(pstStatic->pstTimHandle, 0);
    __HAL_TIM_CLEAR_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC2);
    
    // 3. 启动输入捕获
    HAL_TIM_IC_Start(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
    HAL_TIM_IC_Start(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
    
    // 4. 发送触发信号（10us高电平）
    HAL_GPIO_WritePin(pstStatic->pstTrigGpioBase, pstStatic->usTrigGpioPin, GPIO_PIN_SET);
    Delay_us(10);  // 10us高电平触发
    HAL_GPIO_WritePin(pstStatic->pstTrigGpioBase, pstStatic->usTrigGpioPin, GPIO_PIN_RESET);
    
    // 5. 设置超时时间戳
    pstRunning->ulExpireTime = HAL_GetTick() + pstRunning->ulTimeoutMs;
}

/// @brief      计算测量距离（内部函数）
/// @param      emDevNum   ：设备号
static void vUltrasonicCalcDistance(emUltrasonicDevNumTdf emDevNum)
{
	stUltrasonicStaticParamTdf *pstStatic = &astUltrasonicDeviceParam[emDevNum].stStaticParam;
    stUltrasonicRunningParamTdf *pstRunning = &astUltrasonicDeviceParam[emDevNum].stRunningParam;
    
    // 脉宽 = (CCR2 - CCR1) * 定时器计数周期
    float fPulseWidth = (pstRunning->ulCCR2 - pstRunning->ulCCR1) * pstStatic->fTimerPeriod;
    // 距离 = 声速(340m/s) * 脉宽 / 2（往返路程）
    // 旧方案:pstRunning->fDistance = 340.0f * fPulseWidth * 0.5f;
	// 新方案: 声速 = 声速基准值 + 温度系数 (声速随温度变化率, m/s/°C) * 默认环境温度 (用于无温度传感器时的默认值, °C)
	float fSpeedOfSound = 	ULTRASONIC_SOUND_SPEED_BASE 
							+ ULTRASONIC_SOUND_SPEED_TEMP_COEF * pstRunning->fTemperature;
	pstRunning->fDistance = fSpeedOfSound * fPulseWidth * 0.5f;
}

/// @brief      超声波周期执行
/// @param      emDevNum   ：设备号
void vUltrasonicDevicePeriodExecute(emUltrasonicDevNumTdf emDevNum)
{
    stUltrasonicStaticParamTdf *pstStatic = &astUltrasonicDeviceParam[emDevNum].stStaticParam;
    stUltrasonicRunningParamTdf *pstRunning = &astUltrasonicDeviceParam[emDevNum].stRunningParam;
    
    // 仅处理「测量中」状态
    if (pstRunning->emCurrentStatus != emUltrasonicStatus_Measuring)
    {
        return;
    }
    
    // 1. 检查是否超时
    if (HAL_GetTick() >= pstRunning->ulExpireTime)
    {
        // 停止输入捕获
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
        
        // 更新状态
        pstRunning->emCurrentStatus = emUltrasonicStatus_Timeout;
        pstRunning->ucIsSuccess = 0;
        return;
    }
    
    // 2. 检查捕获标志（CH1和CH2均捕获到）
    uint32_t ulCC1Flag = __HAL_TIM_GET_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC1);
    uint32_t ulCC2Flag = __HAL_TIM_GET_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC2);
    if (ulCC1Flag && ulCC2Flag)
    {
        // 读取捕获值
        pstRunning->ulCCR1 = HAL_TIM_ReadCapturedValue(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        pstRunning->ulCCR2 = HAL_TIM_ReadCapturedValue(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
        
        // 停止输入捕获
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
        
        // 计算距离
        vUltrasonicCalcDistance(emDevNum);
        
        // 更新状态
        pstRunning->emCurrentStatus = emUltrasonicStatus_Completed;
        pstRunning->ucIsSuccess = 1;
    }
}

/// @brief      获取超声波测量距离
/// @param      emDevNum   ：设备号
/// @return     测量距离（单位：米）
float fUltrasonicGetDistance(emUltrasonicDevNumTdf emDevNum)
{
    return astUltrasonicDeviceParam[emDevNum].stRunningParam.fDistance;
}

/// @brief      获取测量是否成功
/// @param      emDevNum   ：设备号
/// @return     0-失败 1-成功
uint8_t ucUltrasonicIsMeasureSuccess(emUltrasonicDevNumTdf emDevNum)
{
    return astUltrasonicDeviceParam[emDevNum].stRunningParam.ucIsSuccess;
}

#endif
