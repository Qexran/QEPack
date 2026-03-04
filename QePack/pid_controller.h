/** 
 * @file    pid_controller.h
 * @author  Qe_xr
 * @version V1.3.0
 * @date    2026/2/26
 * @brief   PID控制器模块，基于 STM32 HAL 库
 * 
 * 本模块实现了标准的PID控制器，支持位置式和速度式两种计算模式，
 * 支持变速积分、积分分离和不完全微分功能，包括比例、积分、微分控制，
 * 以及积分限幅和输出限幅功能。
 */
#include "project_config.h"
#include "string.h"
#if PID_IS_ENABLE

#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

/** @brief PID设备号枚举 */
typedef enum {
    emPidDevNum0 = 0,
    emPidDevNum1,
    emPidDevNum2,
    emPidDevNum3,
} emPidDevNumTdf;

/** @brief PID计算模式枚举 */
typedef enum {
    emPidModePosition = 0,
    emPidModeIncremental,
} emPidModeTdf;

/** @brief PID静态参数定义 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxIntegral;
    float MaxOutput;
    emPidModeTdf Mode;
    uint8_t EnableIntegralSeparation;
    float IntegralSeparationThreshold;
    uint8_t EnableVariableIntegral;
    float VariableIntegralBeta;
    uint8_t EnableIncompleteDerivative;
    float IncompleteDerivativeAlpha;
} stPidStaticParamTdf;

/** @brief PID运行参数定义 */
typedef struct {
    float Error;
    float LastError;
    float LastLastError;
    float Integral;
    float Output;
    float LastOutput;
    float LastDerivative;
} stPidRunningParamTdf;

/** @brief PID设备参数总结构体 */
typedef struct {
    stPidStaticParamTdf stStaticParam;
    stPidRunningParamTdf stRunningParam;
} stPidDeviceParamTdf;

const stPidDeviceParamTdf *c_pstGetPidDeviceParam(emPidDevNumTdf emDevNum);
void vPidDeviceInit(stPidStaticParamTdf *pstInit, emPidDevNumTdf emDevNum);
void vPidReset(emPidDevNumTdf emDevNum);
void vPidSetMode(emPidDevNumTdf emDevNum, emPidModeTdf emMode);
void vPidCalc(emPidDevNumTdf emDevNum, float fReference, float fFeedback);
float fPidGetOutput(emPidDevNumTdf emDevNum);

#endif

#endif

/*
    使用示例：
    // 初始化PID参数（位置式 + 变速积分 + 积分分离 + 不完全微分）
    stPidStaticParamTdf stPidStaticInit;
    stPidStaticInit.Kp = 10.0f;
    stPidStaticInit.Ki = 1.0f;
    stPidStaticInit.Kd = 5.0f;
    stPidStaticInit.MaxIntegral = 800.0f;
    stPidStaticInit.MaxOutput = 1000.0f;
    stPidStaticInit.Mode = emPidModePosition;
    
    // 积分分离配置
    stPidStaticInit.EnableIntegralSeparation = 1;
    stPidStaticInit.IntegralSeparationThreshold = 100.0f;
    
    // 变速积分配置
    stPidStaticInit.EnableVariableIntegral = 1;
    stPidStaticInit.VariableIntegralBeta = 0.01f;  // 调节因子，公式：f = 1 / (1 + beta * abs(e))
    
    // 不完全微分配置
    stPidStaticInit.EnableIncompleteDerivative = 1;
    stPidStaticInit.IncompleteDerivativeAlpha = 0.1f;  // 滤波系数，范围0-1

    // 初始化PID设备
    vPidDeviceInit(&stPidStaticInit, PID0);

    // 在主循环中进行PID控制
    while (1) {
        float fFeedbackValue = ...;  // 获取被控对象的反馈值
        float fTargetValue = ...;    // 获取目标值
        vPidCalc(PID0, fTargetValue, fFeedbackValue);  // 进行PID计算
        float fOutput = fPidGetOutput(PID0);            // 获取PID输出
        // 设定执行器输出大小
        delay(10);
    }
*/
