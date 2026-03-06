/**
  * @file       servo_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/02/11
  * @brief      舵机驱动（兼容180°角度型/360°旋转型），基于 STM32 HAL 库
  */
#include "servo_device.h"
#if SERVO_IS_ENABLE

static stServoDeviceParamTdf astServoDeviceParam[SERVO_DEV_NUM];

/// @brief      获取舵机设备参数（只读）
/// @param      emDevNum   ：设备号
/// @note       返回值为只读指针，避免参数被意外修改
const stServoDeviceParamTdf *c_pstGetServoDeviceParam(emServoDevNumTdf emDevNum)
{
    // 设备号边界检查
    if(emDevNum >= SERVO_DEV_NUM) return NULL;
    return &astServoDeviceParam[emDevNum];
}

/// @brief      初始化舵机运行参数（自定义参数）
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
void vServoDeviceRunningParamInit(stServoRunningParamTdf *pstInit, emServoDevNumTdf emDevNum)
{
    if(emDevNum >= SERVO_DEV_NUM || pstInit == NULL) return;
    memcpy(&astServoDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stServoRunningParamTdf));
}

/// @brief      初始化舵机静态参数（自定义参数，硬件配置）
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
/// @note       初始化后启动PWM输出
void vServoDeviceInit(stServoStaticParamTdf *pstInit, emServoDevNumTdf emDevNum)
{
    if(emDevNum >= SERVO_DEV_NUM || pstInit == NULL) return;
    
    // 1. 拷贝静态参数
    memcpy(&astServoDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stServoStaticParamTdf));
    
    // 2. 启动PWM输出
    HAL_TIM_PWM_Start(astServoDeviceParam[emDevNum].stStaticParam.pstTimHandle,
                      astServoDeviceParam[emDevNum].stStaticParam.ulChannel);
}

/// @brief      值转脉冲宽度（内部函数）
/// @param      emDevNum   ：设备号
/// @param      fValue     ：目标值（180°=角度，360°=速度）
/// @return     脉冲宽度(us)
static float fServoValueToPulse(emServoDevNumTdf emDevNum, float fValue)
{
    if(emDevNum >= SERVO_DEV_NUM) return 0.0f;
    
    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;
    
    // 边界限制
    if(fValue < pstStatic->fValueMin) fValue = pstStatic->fValueMin;
    if(fValue > pstStatic->fValueMax) fValue = pstStatic->fValueMax;
    
    // 180°角度型舵机：角度 -> 脉冲宽度（线性映射）
    if(pstStatic->emType == emServoType_Angle)
    {
        return pstStatic->fPulseMin + (fValue - pstStatic->fValueMin) * 
               (pstStatic->fPulseMax - pstStatic->fPulseMin) / (pstStatic->fValueMax - pstStatic->fValueMin);
    }
    // 360°连续旋转型舵机：速度 -> 脉冲宽度
    else if(pstStatic->emType == emServoType_360)
    {
        if(fValue == 0) return pstStatic->fPulseMid; // 0速度=停转（中位脉冲）
        
        // 正转（0~100）：中位脉冲 -> 最大脉冲
        else if(fValue > 0)
        {
            return pstStatic->fPulseMid + (fValue / SERVO_360_SPEED_MAX) * 
                   (pstStatic->fPulseMax - pstStatic->fPulseMid);
        }
        // 反转（-100~0）：中位脉冲 -> 最小脉冲
        else
        {
            return pstStatic->fPulseMid - (fabs(fValue) / fabs(SERVO_360_SPEED_MIN)) * 
                   (pstStatic->fPulseMid - pstStatic->fPulseMin);
        }
    }
    
    return pstStatic->fPulseMid; // 默认返回中位脉冲（停转）
}

/// @brief      更新PWM占空比（内部函数）
/// @param      emDevNum   ：设备号
/// @param      fPulseUs   ：脉冲宽度(us)
static void vServoUpdatePwm(emServoDevNumTdf emDevNum, float fPulseUs)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    
    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;
    
    // 计算ARR值（定时器自动重装值）
    uint32_t ulArr = (SystemCoreClock / (pstStatic->fPwmFreq * (pstStatic->pstTimHandle->Init.Prescaler + 1))) - 1;
    // 计算CCR值（比较值）：CCR = (脉冲宽度/1秒) * PWM频率 * ARR
    uint32_t ulCcr = (uint32_t)((fPulseUs / 1000000.0f) * pstStatic->fPwmFreq * ulArr);
    
    // 设置PWM比较值
    __HAL_TIM_SET_COMPARE(pstStatic->pstTimHandle, pstStatic->ulChannel, ulCcr);
}

/// @brief      直接设置舵机值（静态模式）
/// @param      emDevNum   ：设备号
/// @param      fValue     ：目标值（180°=角度(°)；360°=速度(-100~100)）
/// @note       立即生效，无平滑过渡
void vServoSetValue(emServoDevNumTdf emDevNum, float fValue)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    
    // 1. 更新当前值
    astServoDeviceParam[emDevNum].stRunningParam.fCurrentValue = fValue;
    
    // 2. 值转脉冲并更新PWM
    vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fValue));
}

/// @brief      设置舵机目标值（平滑模式）
/// @param      emDevNum   ：设备号
/// @param      fValue     ：目标值（180°=角度(°)；360°=速度(-100~100)）
/// @note       需配合 vServoDevicePeriodExecute 实现平滑调速
void vServoSetTargetValue(emServoDevNumTdf emDevNum, float fValue)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    
    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;
    
    // 边界限制
    if(fValue < pstStatic->fValueMin) fValue = pstStatic->fValueMin;
    if(fValue > pstStatic->fValueMax) fValue = pstStatic->fValueMax;
    
    // 更新目标值
    astServoDeviceParam[emDevNum].stRunningParam.fTargetValue = fValue;
}

/// @brief      舵机周期执行（平滑调速核心）
/// @param      emDevNum   ：设备号
/// @note       建议1ms调用一次，仅在平滑模式下生效
void vServoDevicePeriodExecute(emServoDevNumTdf emDevNum)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    
    stServoRunningParamTdf *pstRun = &astServoDeviceParam[emDevNum].stRunningParam;
    float fDelta = 0.0f;
    
    // 仅处理平滑模式
    if(pstRun->emMode != emServoMode_Smooth) return;
    
    // 计算当前值与目标值的差值
    fDelta = pstRun->fTargetValue - pstRun->fCurrentValue;
    
    // 差值小于0.1时，认为已到位（避免抖动）
    if(fabs(fDelta) < 0.1f)
    {
        pstRun->fCurrentValue = pstRun->fTargetValue;
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, pstRun->fCurrentValue));
        return;
    }
    
    // 按速度调整当前值（1ms调用一次，速度单位：值/ms）
    if(fDelta > 0)
    {
        pstRun->fCurrentValue += pstRun->fSpeed;
        if(pstRun->fCurrentValue > pstRun->fTargetValue)
        {
            pstRun->fCurrentValue = pstRun->fTargetValue;
        }
    }
    else
    {
        pstRun->fCurrentValue -= pstRun->fSpeed;
        if(pstRun->fCurrentValue < pstRun->fTargetValue)
        {
            pstRun->fCurrentValue = pstRun->fTargetValue;
        }
    }
    
    // 更新PWM输出
    vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, pstRun->fCurrentValue));
}

/// @brief      180°角度型舵机默认参数初始化（简化版）
/// @param      emDevNum   ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel ：PWM通道
void vServoDeviceDefaultInit_Angle(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel)
{
    if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;
    
    // 1. 静态参数（使用project_config.h的默认宏定义）
    stServoStaticParamTdf stStaticInit = {
        .pstTimHandle = pstTimHandle,
        .ulChannel = ulChannel,
        .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
        .emType = emServoType_Angle,
        .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
        .fValueMax = SERVO_DEFAULT_ANGLE_MAX,
        .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
        .fPulseMid = 0.0f, // 180°舵机无需中位脉冲
        .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
    };
    
    // 2. 运行参数（默认配置）
    stServoRunningParamTdf stRunInit = {
        .emMode = emServoMode_Static,
        .fCurrentValue = (SERVO_DEFAULT_ANGLE_MIN + SERVO_DEFAULT_ANGLE_MAX) / 2, // 初始90°
        .fTargetValue = (SERVO_DEFAULT_ANGLE_MIN + SERVO_DEFAULT_ANGLE_MAX) / 2,
        .fSpeed = SERVO_DEFAULT_SPEED,
    };
    
    // 3. 执行初始化
    vServoDeviceInit(&stStaticInit, emDevNum);
    vServoDeviceRunningParamInit(&stRunInit, emDevNum);
    
    // 4. 设置初始角度（90°）
    vServoSetValue(emDevNum, stRunInit.fCurrentValue);
}

/// @brief      360°旋转型舵机默认参数初始化（简化版）
/// @param      emDevNum   ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel ：PWM通道
void vServoDeviceDefaultInit_360(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel)
{
    if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;
    
    // 1. 静态参数（使用project_config.h的默认宏定义）
    stServoStaticParamTdf stStaticInit = {
        .pstTimHandle = pstTimHandle,
        .ulChannel = ulChannel,
        .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
        .emType = emServoType_360,
        .fValueMin = SERVO_360_SPEED_MIN,    // -100
        .fValueMax = SERVO_360_SPEED_MAX,    // 100
        .fPulseMin = SERVO_DEFAULT_PULSE_MIN,// 500us
        .fPulseMid = SERVO_360_PULSE_MID,    // 1500us（停转）
        .fPulseMax = SERVO_DEFAULT_PULSE_MAX,// 2500us
    };
    
    // 2. 运行参数（默认配置）
    stServoRunningParamTdf stRunInit = {
        .emMode = emServoMode_Static,
        .fCurrentValue = 0.0f, // 初始停转
        .fTargetValue = 0.0f,
        .fSpeed = SERVO_DEFAULT_SPEED,
    };
    
    // 3. 执行初始化
    vServoDeviceInit(&stStaticInit, emDevNum);
    vServoDeviceRunningParamInit(&stRunInit, emDevNum);
    
    // 4. 设置初始速度（停转）
    vServoSetValue(emDevNum, stRunInit.fCurrentValue);
}


/// @brief      设置舵机模式
/// @param      emDevNum   ：设备号
/// @param      newemMode ：新的模式
void vServoSetMode(emServoDevNumTdf emDevNum, emServoModeTdf newemMode)
{
    astServoDeviceParam[emDevNum].stRunningParam.emMode = newemMode;
}


#endif /* SERVO_IS_ENABLE */
