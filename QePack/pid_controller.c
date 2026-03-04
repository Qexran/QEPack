/** 
 * @file    pid_controller.c
 * @author  Qe_xr
 * @version V1.3.0
 * @date    2026/2/26
 * @brief   PID控制器模块，基于 STM32 HAL 库
 */
#include "pid_controller.h"
#include <math.h>

#if PID_IS_ENABLE


stPidDeviceParamTdf astPidDeviceParam[PID_DEV_NUM];

/**
 * @brief 计算积分系数
 * @param pstStatic PID静态参数指针
 * @param fError 当前误差
 * @return float 积分系数
 */
static float fCalculateIntegralCoefficient(stPidStaticParamTdf *pstStatic, float fError)
{
    float fAbsError = fabsf(fError);
    float fCoefficient = 1.0f;

    if (pstStatic->EnableIntegralSeparation) {
        if (fAbsError > pstStatic->IntegralSeparationThreshold) {
            return 0.0f;
        }
    }

    if (pstStatic->EnableVariableIntegral) {
        float fBeta = pstStatic->VariableIntegralBeta;
        fCoefficient = 1.0f / (1.0f + fBeta * fAbsError);
    }

    return fCoefficient;
}

/**
 * @brief 计算微分项
 * @param pstStatic PID静态参数指针
 * @param pstRunning PID运行参数指针
 * @param fError 当前误差
 * @param fLastError 上一次误差
 * @return float 微分项
 */
static float fCalculateDerivative(stPidStaticParamTdf *pstStatic, stPidRunningParamTdf *pstRunning, float fError, float fLastError)
{
    float fDerivative = (fError - fLastError) * pstStatic->Kd;

    if (pstStatic->EnableIncompleteDerivative) {
        float fAlpha = pstStatic->IncompleteDerivativeAlpha;
        fDerivative = fAlpha * fDerivative + (1.0f - fAlpha) * pstRunning->LastDerivative;
        pstRunning->LastDerivative = fDerivative;
    }

    return fDerivative;
}

/**
 * @brief 计算位置式PID输出
 * @param pstStatic PID静态参数指针
 * @param pstRunning PID运行参数指针
 * @param fReference 参考值
 * @param fFeedback 反馈值
 */
static void vCalcPositionPID(stPidStaticParamTdf *pstStatic, stPidRunningParamTdf *pstRunning, float fReference, float fFeedback)
{
    pstRunning->LastError = pstRunning->Error;
    pstRunning->Error = fReference - fFeedback;

    float fDout = fCalculateDerivative(pstStatic, pstRunning, pstRunning->Error, pstRunning->LastError);
    float fPout = pstRunning->Error * pstStatic->Kp;
    float fIntegralCoeff = fCalculateIntegralCoefficient(pstStatic, pstRunning->Error);
    pstRunning->Integral += pstRunning->Error * pstStatic->Ki * fIntegralCoeff;

    if (pstRunning->Integral > pstStatic->MaxIntegral) {
        pstRunning->Integral = pstStatic->MaxIntegral;
    } else if (pstRunning->Integral < -pstStatic->MaxIntegral) {
        pstRunning->Integral = -pstStatic->MaxIntegral;
    }

    pstRunning->Output = fPout + fDout + pstRunning->Integral;

    if (pstRunning->Output > pstStatic->MaxOutput) {
        pstRunning->Output = pstStatic->MaxOutput;
    } else if (pstRunning->Output < -pstStatic->MaxOutput) {
        pstRunning->Output = -pstStatic->MaxOutput;
    }
}

/**
 * @brief 计算增量式PID输出
 * @param pstStatic PID静态参数指针
 * @param pstRunning PID运行参数指针
 * @param fReference 参考值
 * @param fFeedback 反馈值
 */
static void vCalcIncrementalPID(stPidStaticParamTdf *pstStatic, stPidRunningParamTdf *pstRunning, float fReference, float fFeedback)
{
    pstRunning->LastLastError = pstRunning->LastError;
    pstRunning->LastError = pstRunning->Error;
    pstRunning->Error = fReference - fFeedback;

    float fIntegralCoeff = fCalculateIntegralCoefficient(pstStatic, pstRunning->Error);
    float fDeltaP = pstStatic->Kp * (pstRunning->Error - pstRunning->LastError);
    float fDeltaI = pstStatic->Ki * pstRunning->Error * fIntegralCoeff;
    
    float fCurrentDerivative = pstRunning->Error - 2.0f * pstRunning->LastError + pstRunning->LastLastError;
    float fDeltaD = pstStatic->Kd * fCurrentDerivative;
    
    if (pstStatic->EnableIncompleteDerivative) {
        float fAlpha = pstStatic->IncompleteDerivativeAlpha;
        fDeltaD = fAlpha * fDeltaD + (1.0f - fAlpha) * pstRunning->LastDerivative;
        pstRunning->LastDerivative = fDeltaD;
    }

    float fDeltaOutput = fDeltaP + fDeltaI + fDeltaD;

    pstRunning->Output += fDeltaOutput;

    if (pstRunning->Output > pstStatic->MaxOutput) {
        pstRunning->Output = pstStatic->MaxOutput;
    } else if (pstRunning->Output < -pstStatic->MaxOutput) {
        pstRunning->Output = -pstStatic->MaxOutput;
    }
}

/**
 * @brief 获取PID设备参数
 * @param emDevNum PID设备号
 * @return const stPidDeviceParamTdf* PID设备参数指针
 */
const stPidDeviceParamTdf *c_pstGetPidDeviceParam(emPidDevNumTdf emDevNum)
{
    if (emDevNum >= PID_DEV_NUM) {
        return NULL;
    }
    return &astPidDeviceParam[emDevNum];
}

/**
 * @brief 初始化PID静态参数
 * @param pstInit PID静态参数指针
 * @param emDevNum PID设备号
 */
void vPidDeviceInit(stPidStaticParamTdf *pstInit, emPidDevNumTdf emDevNum)
{
    if (emDevNum >= PID_DEV_NUM || pstInit == NULL) {
        return;
    }

    memcpy(&astPidDeviceParam[emDevNum].stStaticParam, 
           pstInit, 
           sizeof(stPidStaticParamTdf));

    memset(&astPidDeviceParam[emDevNum].stRunningParam, 
           0, 
           sizeof(stPidRunningParamTdf));
}

/**
 * @brief 重置PID运行参数
 * @param emDevNum PID设备号
 */
void vPidReset(emPidDevNumTdf emDevNum)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }

    memset(&astPidDeviceParam[emDevNum].stRunningParam, 
           0, 
           sizeof(stPidRunningParamTdf));
}

/**
 * @brief 设置PID模式
 * @param emDevNum PID设备号
 * @param emMode PID模式
 */
void vPidSetMode(emPidDevNumTdf emDevNum, emPidModeTdf emMode)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }

    astPidDeviceParam[emDevNum].stStaticParam.Mode = emMode;
    vPidReset(emDevNum);
}

/**
 * @brief 计算PID输出
 * @param emDevNum PID设备号
 * @param fReference 参考值
 * @param fFeedback 反馈值
 */
void vPidCalc(emPidDevNumTdf emDevNum, float fReference, float fFeedback)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }

    stPidRunningParamTdf *pstRunning = &astPidDeviceParam[emDevNum].stRunningParam;
    stPidStaticParamTdf *pstStatic = &astPidDeviceParam[emDevNum].stStaticParam;

    if (pstStatic->Mode == emPidModePosition) {
        vCalcPositionPID(pstStatic, pstRunning, fReference, fFeedback);
    } else {
        vCalcIncrementalPID(pstStatic, pstRunning, fReference, fFeedback);
    }
}

/**
 * @brief 获取PID输出
 * @param emDevNum PID设备号
 * @return float PID输出
 */
float fPidGetOutput(emPidDevNumTdf emDevNum)
{
    if (emDevNum >= PID_DEV_NUM) {
        return 0.0f;
    }
    return astPidDeviceParam[emDevNum].stRunningParam.Output;
}

#endif
