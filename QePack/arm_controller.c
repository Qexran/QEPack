/**
  * @file       arm_controller.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/06/02
  * @brief      多轴协调运动控制器实现
  */

#include "arm_controller.h"

#if ARM_CONTROLLER_IS_ENABLE

#include <string.h>
#include <math.h>

/* ==================== 全局变量 ==================== */

static stArmControllerDeviceParamTdf g_stArmController;

/* ==================== 内部函数 ==================== */

/**
 * @brief  估算电机运动时间（梯形/三角形速度曲线）
 * @param  fAbsDistance  : 运动距离的绝对值（脉冲数）
 * @param  usVel        : 最大速度
 * @param  ucAcc        : 加速度系数
 * @return 预估时间(ms)
 * @note   速度/加速度的单位取决于底层电机驱动实现
 */
static uint32_t ulEstimateMotorTimeMs(float fAbsDistance, uint16_t usVel, uint8_t ucAcc)
{
    if(usVel == 0 || ucAcc == 0 || fAbsDistance <= 0.0f) return 0;

    /*
     * 简化估算模型：
     * 假设速度 usVel 的单位为 RPM（或类似线性单位），
     * 加速度 ucAcc 为 0~255 的系数。
     *
     * 加速段时间: t_acc = usVel / (ucAcc * K)
     * 加速段距离: d_acc = 0.5 * usVel * t_acc
     *
     * 若 d_acc * 2 >= 总距离 → 三角形（达不到最大速度）
     * 否则 → 梯形
     *
     * K 为经验系数，通过 fMotorStepsPerUnit 或默认值校准
     */
    float fVel = (float)usVel;
    float fAcc = (float)ucAcc;

    /* 简化：假设加速度 = ucAcc * 10 步/秒² */
    float fAccelRate = fAcc * 10.0f;

    float fAccelTime = fVel / fAccelRate;
    float fAccelDist = 0.5f * fVel * fAccelTime;

    float fTotalTime;
    if(fAbsDistance <= 2.0f * fAccelDist)
    {
        /* 三角形速度曲线 */
        fTotalTime = 2.0f * sqrtf(fAbsDistance / fAccelRate);
    }
    else
    {
        /* 梯形速度曲线 */
        float fConstDist = fAbsDistance - 2.0f * fAccelDist;
        fTotalTime = 2.0f * fAccelTime + fConstDist / fVel;
    }

    return (uint32_t)(fTotalTime * 1000.0f);
}

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief  初始化协调运动控制器
 */
void vArmControllerInit(const stArmControllerStaticParamTdf *pstInit)
{
    if(pstInit == NULL) return;

    memset(&g_stArmController, 0, sizeof(stArmControllerDeviceParamTdf));
    g_stArmController.stStaticParam = *pstInit;
    g_stArmController.stRunningParam.ucMotionDone = 1;
}

/**
 * @brief  发起协调运动
 * @param  pstCmds     : 轴命令数组
 * @param  ucCmdCount  : 命令数量
 */
void vArmControllerMove(const stArmAxisCommandTdf *pstCmds, uint8_t ucCmdCount)
{
    if(pstCmds == NULL || ucCmdCount == 0) return;

    stArmControllerDeviceParamTdf *pstCtrl = &g_stArmController;
    if(pstCtrl->stStaticParam.pstAxisConfigs == NULL) return;

    uint32_t ulMaxTimeMs = 0;

    for(uint8_t i = 0; i < ucCmdCount; i++)
    {
        const stArmAxisCommandTdf *pstCmd = &pstCmds[i];
        if(pstCmd->ucAxisIdx >= pstCtrl->stStaticParam.ucAxisCount) continue;

        const stArmAxisConfigTdf *pstAxis = &pstCtrl->stStaticParam.pstAxisConfigs[pstCmd->ucAxisIdx];

        if(pstAxis->emType == emArmAxisType_Servo)
        {
            /* 舵机轴：使用 S 曲线插值 */
            vServoSetTargetValueTimed((emServoDevNumTdf)pstAxis->ucDevIndex,
                                      pstCmd->fTarget, pstCmd->ulDurationMs);
            if(pstCmd->ulDurationMs > ulMaxTimeMs)
                ulMaxTimeMs = pstCmd->ulDurationMs;
        }
        else if(pstAxis->emType == emArmAxisType_Motor)
        {
            /* 电机轴：发送位置控制命令 */
            emMotorDirTdf emDir = (pstCmd->fTarget >= 0) ? emMotorDir_Forward : emMotorDir_Backward;
            uint32_t ulClk = (uint32_t)fabsf(pstCmd->fTarget);

            vMotorPosControl((emMotorDevNumTdf)pstAxis->ucDevIndex,
                             emDir, pstAxis->usMotorVel, pstAxis->ucMotorAcc,
                             ulClk, 0, 0);

            /* 计算预估时间 */
            uint32_t ulMotorTime;
            if(pstCmd->ulDurationMs > 0)
            {
                /* 用户指定了时长 */
                ulMotorTime = pstCmd->ulDurationMs;
            }
            else
            {
                /* 自动估算 */
                ulMotorTime = ulEstimateMotorTimeMs((float)ulClk, pstAxis->usMotorVel, pstAxis->ucMotorAcc);
            }

            if(ulMotorTime > ulMaxTimeMs)
                ulMaxTimeMs = ulMotorTime;
        }
    }

    pstCtrl->stRunningParam.ulTotalTimeMs = ulMaxTimeMs;
    pstCtrl->stRunningParam.ulElapsedMs = 0;
    pstCtrl->stRunningParam.ucMotionDone = 0;
}

/**
 * @brief  查询运动是否完成
 * @return 1=完成或空闲，0=运动中
 */
uint8_t ucArmControllerIsMotionDone(void)
{
    return g_stArmController.stRunningParam.ucMotionDone;
}

/**
 * @brief  周期执行（主循环调用）
 * @note   检查所有舵机轴是否完成运动，超时保护
 */
void vArmControllerPeriodExecute(void)
{
    stArmControllerDeviceParamTdf *pstCtrl = &g_stArmController;
    if(pstCtrl->stRunningParam.ucMotionDone) return;

    pstCtrl->stRunningParam.ulElapsedMs++;

    /* 检查所有舵机轴是否完成 */
    uint8_t ucAllDone = 1;
    for(uint8_t i = 0; i < pstCtrl->stStaticParam.ucAxisCount; i++)
    {
        const stArmAxisConfigTdf *pstAxis = &pstCtrl->stStaticParam.pstAxisConfigs[i];
        if(pstAxis->emType == emArmAxisType_Servo)
        {
            if(!ucServoIsSmoothStepDone((emServoDevNumTdf)pstAxis->ucDevIndex))
            {
                ucAllDone = 0;
                break;
            }
        }
    }

    /* 超时保护：总时长 + 500ms 余量 */
    if(pstCtrl->stRunningParam.ulTotalTimeMs > 0 &&
       pstCtrl->stRunningParam.ulElapsedMs >= pstCtrl->stRunningParam.ulTotalTimeMs + 500)
    {
        ucAllDone = 1;
    }

    if(ucAllDone)
    {
        pstCtrl->stRunningParam.ucMotionDone = 1;
    }
}

/**
 * @brief  紧急停止所有轴
 */
void vArmControllerStop(void)
{
    stArmControllerDeviceParamTdf *pstCtrl = &g_stArmController;

    for(uint8_t i = 0; i < pstCtrl->stStaticParam.ucAxisCount; i++)
    {
        const stArmAxisConfigTdf *pstAxis = &pstCtrl->stStaticParam.pstAxisConfigs[i];
        if(pstAxis->emType == emArmAxisType_Servo)
        {
            emServoDevNumTdf emServo = (emServoDevNumTdf)pstAxis->ucDevIndex;
            vServoSetValue(emServo,
                           c_pstGetServoDeviceParam(emServo)->stRunningParam.fCurrentValue);
            vServoSetMode(emServo, emServoMode_Static);
        }
        else if(pstAxis->emType == emArmAxisType_Motor)
        {
            vMotorStop((emMotorDevNumTdf)pstAxis->ucDevIndex, 0);
        }
    }

    pstCtrl->stRunningParam.ucMotionDone = 1;
}

#endif /* ARM_CONTROLLER_IS_ENABLE */
