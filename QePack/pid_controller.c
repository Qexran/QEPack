/**
 * @file    pid_controller.c
 * @author  Qe_xr
 * @version V2.0.0
 * @date    2026/5/11
 * @brief   PID控制器模块（定点数版本）
 */
#include "pid_controller.h"

#if PID_IS_ENABLE

stPidDeviceParamTdf astPidDeviceParam[PID_DEV_NUM];

/**
 * @brief 计算积分系数
 */
static fix32_t fCalculateIntegralCoefficient(stPidStaticParamTdf *pstStatic, fix32_t fError)
{
    fix32_t fAbsError = fix32_abs(fError);
    fix32_t fCoefficient = FIX32_ONE;

    if (pstStatic->EnableIntegralSeparation) {
        if (fAbsError > pstStatic->IntegralSeparationThreshold) {
            return FIX32_ZERO;
        }
    }

    if (pstStatic->EnableVariableIntegral) {
        fix32_t fBeta = pstStatic->VariableIntegralBeta;
        fCoefficient = fix32_div(FIX32_ONE, FIX32_ONE + fix32_mul(fBeta, fAbsError));
    }

    return fCoefficient;
}

/**
 * @brief 计算微分项
 */
static fix32_t fCalculateDerivative(stPidStaticParamTdf *pstStatic, stPidRunningParamTdf *pstRunning, fix32_t fError, fix32_t fFeedback, fix32_t fLastError)
{
    fix32_t fDerivative;

    if (pstStatic->EnableDerivativeOnFeedback) {
        fDerivative = fix32_mul(fFeedback - pstRunning->LastFeedback, pstStatic->Kd);
    } else {
        fDerivative = fix32_mul(fError - fLastError, pstStatic->Kd);
    }

    if (pstStatic->EnableIncompleteDerivative) {
        fix32_t fAlpha = pstStatic->IncompleteDerivativeAlpha;
        fDerivative = fix32_mul(fAlpha, fDerivative) + fix32_mul(FIX32_ONE - fAlpha, pstRunning->LastDerivative);
        pstRunning->LastDerivative = fDerivative;
    }

    return fDerivative;
}

/**
 * @brief 计算位置式PID输出
 */
static void vCalcPositionPID(stPidStaticParamTdf *pstStatic, stPidRunningParamTdf *pstRunning, fix32_t fReference, fix32_t fFeedback)
{
    pstRunning->LastError = pstRunning->Error;
    pstRunning->Error = fReference - fFeedback;

    fix32_t fPout = fix32_mul(pstRunning->Error, pstStatic->Kp);
    fix32_t fIntegralCoeff = fCalculateIntegralCoefficient(pstStatic, pstRunning->Error);

    /* 积分累加（饱和加法防溢出） */
    fix32_t fErrKiCoeff = fix32_mul(fix32_mul(pstRunning->Error, pstStatic->Ki), fIntegralCoeff);
    int64_t newIntegral = (int64_t)pstRunning->Integral + (int64_t)fErrKiCoeff;
    if (newIntegral > (int64_t)pstStatic->MaxIntegral) {
        newIntegral = (int64_t)pstStatic->MaxIntegral;
    } else if (newIntegral < -(int64_t)pstStatic->MaxIntegral) {
        newIntegral = -(int64_t)pstStatic->MaxIntegral;
    }
    pstRunning->Integral = (fix32_t)newIntegral;

    fix32_t fDout = fCalculateDerivative(pstStatic, pstRunning, pstRunning->Error, fFeedback, pstRunning->LastError);

    int64_t llOutput = (int64_t)fPout + (int64_t)pstRunning->Integral + (int64_t)fDout;
    if (llOutput > (int64_t)pstStatic->MaxOutput) {
        pstRunning->Output = pstStatic->MaxOutput;
    } else if (llOutput < -(int64_t)pstStatic->MaxOutput) {
        pstRunning->Output = -pstStatic->MaxOutput;
    } else {
        pstRunning->Output = (fix32_t)llOutput;
    }
    pstRunning->LastFeedback = fFeedback;
}

/**
 * @brief 计算增量式PID输出
 */
static void vCalcIncrementalPID(stPidStaticParamTdf *pstStatic, stPidRunningParamTdf *pstRunning, fix32_t fReference, fix32_t fFeedback)
{
    pstRunning->LastLastError = pstRunning->LastError;
    pstRunning->LastError = pstRunning->Error;
    pstRunning->Error = fReference - fFeedback;

    fix32_t fDeltaP = fix32_mul(pstStatic->Kp, pstRunning->Error - pstRunning->LastError);
    fix32_t fDeltaI = fix32_mul(pstStatic->Ki, pstRunning->Error);

    if (pstStatic->EnableIntegralSeparation) {
        fix32_t fAbsError = fix32_abs(pstRunning->Error);
        if (fAbsError > pstStatic->IntegralSeparationThreshold) {
            fDeltaI = FIX32_ZERO;
        }
    }

    fix32_t fCurrentDerivative = pstRunning->Error - (pstRunning->LastError * 2) + pstRunning->LastLastError;
    fix32_t fDeltaD = fix32_mul(pstStatic->Kd, fCurrentDerivative);

    if (pstStatic->EnableIncompleteDerivative) {
        fix32_t fAlpha = pstStatic->IncompleteDerivativeAlpha;
        fDeltaD = fix32_mul(fAlpha, fDeltaD) + fix32_mul(FIX32_ONE - fAlpha, pstRunning->LastDerivative);
        pstRunning->LastDerivative = fDeltaD;
    }

    int64_t llNewOutput = (int64_t)pstRunning->Output + (int64_t)fDeltaP + (int64_t)fDeltaI + (int64_t)fDeltaD;
    if (llNewOutput > (int64_t)pstStatic->MaxOutput) {
        pstRunning->Output = pstStatic->MaxOutput;
    } else if (llNewOutput < -(int64_t)pstStatic->MaxOutput) {
        pstRunning->Output = -pstStatic->MaxOutput;
    } else {
        pstRunning->Output = (fix32_t)llNewOutput;
    }
    pstRunning->LastFeedback = fFeedback;
}

/**
 * @brief 获取PID设备参数
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

    astPidDeviceParam[emDevNum].stRunningParam.Enable = 1;
}

/**
 * @brief 重置PID运行参数
 */
void vPidReset(emPidDevNumTdf emDevNum)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }

    memset(&astPidDeviceParam[emDevNum].stRunningParam,
           0,
           sizeof(stPidRunningParamTdf));
    astPidDeviceParam[emDevNum].stRunningParam.Enable = 1;
}

/**
 * @brief 设置PID模式
 */
void vPidSetMode(emPidDevNumTdf emDevNum, emPidModeTdf emMode, uint8_t bReset)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }

    astPidDeviceParam[emDevNum].stStaticParam.Mode = emMode;
    if (bReset) {
        vPidReset(emDevNum);
    }
}

/**
 * @brief 计算PID输出
 */
void vPidCalc(emPidDevNumTdf emDevNum, fix32_t fReference, fix32_t fFeedback)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }

    stPidRunningParamTdf *pstRunning = &astPidDeviceParam[emDevNum].stRunningParam;
    stPidStaticParamTdf *pstStatic = &astPidDeviceParam[emDevNum].stStaticParam;

    if (!pstRunning->Enable) {
        return;
    }

    if (pstStatic->Mode == emPidModePosition) {
        vCalcPositionPID(pstStatic, pstRunning, fReference, fFeedback);
    } else {
        vCalcIncrementalPID(pstStatic, pstRunning, fReference, fFeedback);
    }
}

/**
 * @brief 获取PID输出
 */
QE_StatusTypeDef ePidGetOutput(emPidDevNumTdf emDevNum, fix32_t *pfOutput)
{
    if (emDevNum >= PID_DEV_NUM || pfOutput == NULL) {
        return QE_ERROR;
    }
    *pfOutput = astPidDeviceParam[emDevNum].stRunningParam.Output;
    return QE_OK;
}

/**
 * @brief 运行时修改 PID 参数
 */
void vPidSetParam(emPidDevNumTdf emDevNum, fix32_t fKp, fix32_t fKi, fix32_t fKd)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }
    astPidDeviceParam[emDevNum].stStaticParam.Kp = fKp;
    astPidDeviceParam[emDevNum].stStaticParam.Ki = fKi;
    astPidDeviceParam[emDevNum].stStaticParam.Kd = fKd;
}

/**
 * @brief 设置 PID 目标值
 */
void vPidSetTarget(emPidDevNumTdf emDevNum, fix32_t fTarget)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }
    astPidDeviceParam[emDevNum].stRunningParam.Target = fTarget;
}

/**
 * @brief 使能/禁用 PID
 */
void vPidSetEnable(emPidDevNumTdf emDevNum, uint8_t bEnable)
{
    if (emDevNum >= PID_DEV_NUM) {
        return;
    }
    astPidDeviceParam[emDevNum].stRunningParam.Enable = bEnable;
}

/**
 * @brief 获取 PID 目标值
 */
fix32_t fPidGetTarget(emPidDevNumTdf emDevNum)
{
    if (emDevNum >= PID_DEV_NUM) {
        return FIX32_ZERO;
    }
    return astPidDeviceParam[emDevNum].stRunningParam.Target;
}

#endif
