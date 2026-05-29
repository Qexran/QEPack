/**
 * @file       gear_motor_device.c
 * @author     Qe_xr
 * @version    V2.0.0
 * @date       2026/5/11
 * @brief      直流减速电机控制驱动（定点数版本）
 *             支持 PID 绑定、梯形加速度规划、速度模式控制
 */

/* ==============================================
   包含头文件
   ============================================== */
#include "gear_motor_device.h"

/* ==============================================
   条件编译：当GEAR_MOTOR_IS_ENABLE宏定义时才编译此文件
   ============================================== */
#if GEAR_MOTOR_IS_ENABLE

/* ==============================================
   全局变量：减速电机设备参数数组
   ============================================== */
stGearMotorDeviceParamTdf astGearMotorDeviceParam[GEAR_MOTOR_DEV_NUM];

/* ==============================================
   编译期常量：0.01 的 Q16.16 定点数表示 ≈ 655
   用于判断一个值是否接近于0
   ============================================== */
#define FIX32_0_01  ((fix32_t)655)

/* ==============================================
   静态辅助函数：获取编码器当前速度(RPM)
   ============================================== */
/**
 * @brief 获取编码器当前速度(RPM)
 * @param pstMotor 减速电机设备指针
 * @return 编码器当前速度（RPM，定点数格式）
 */
static fix32_t fGearMotorGetEncoderSpeedRPM(stGearMotorDeviceParamTdf *pstMotor)
{
    /* 直接调用编码器设备的速度获取函数 */
    return fEncoderGetSpeed(pstMotor->stStaticParam.emEncoderDevNum);
}

/* ==============================================
   静态辅助函数：计算速度模式预估时间(ms)
   ============================================== */
/**
 * @brief 计算速度模式预估时间(ms)
 * @param fStartRPM 起始速度（RPM，定点数）
 * @param fTargetRPM 目标速度（RPM，定点数）
 * @param fAccelRPMpS 加速度（RPM/s，定点数）
 * @return 预估完成时间（ms，定点数）
 */
static fix32_t fCalcVelProfileTime(fix32_t fStartRPM, fix32_t fTargetRPM, fix32_t fAccelRPMpS)
{
    /* 参数有效性检查 */
    if (fAccelRPMpS < FIX32_0_01) {
        return FIX32_ZERO;
    }
    
    /* 计算速度差的绝对值 */
    fix32_t fDelta = fTargetRPM - fStartRPM;
    if (fDelta < FIX32_ZERO) fDelta = -fDelta;
    
    /* 预估时间 = (速度差 / 加速度) * 1000（转换为ms） */
    return fix32_mul(fix32_div(fDelta, fAccelRPMpS), ((fix32_t)(1000 * 65536)));
}

/* ==============================================
   静态辅助函数：模式切换时重置 PID 并设置目标
   ============================================== */
/**
 * @brief 在非速度模式切换到速度模式时重置 PID 并设定新目标
 * @param emPidDevNum PID 设备号（emNoPid 则不操作）
 * @param emPrevMode 之前的控制模式
 * @param fTarget PID 目标值
 */
static void vGearMotorPidSwitchToVel(emPidDevNumTdf emPidDevNum,
                                     emGearMotorCtrlModeTdf emPrevMode,
                                     fix32_t fTarget)
{
    if (emPidDevNum != emNoPid) {
        if (emPrevMode != emGearMotorCtrlMode_Vel) {
            vPidReset(emPidDevNum);
        }
        vPidSetTarget(emPidDevNum, fTarget);
    }
}

/* ==============================================
   虚方法表（VTable）：定义减速电机设备的虚函数接口
   ============================================== */
static stMotorVTableTdf g_stGearMotorVTable = {
    vGearMotorInit,              /* 初始化函数指针 */
    vGearMotorPeriodExecute,     /* 周期性执行函数指针 */
    vGearMotorStop,              /* 停止函数指针 */
    vGearMotorEnable,            /* 使能函数指针 */
    emGetGearMotorState,         /* 获取状态函数指针 */
    vGearMotorPosControl,        /* 位置控制函数指针 */
    vGearMotorVelControl,        /* 速度控制函数指针 */
    vGearMotorSynchronousMotion, /* 多机同步触发函数指针 */
};

/* ==============================================
   虚方法实现：初始化函数
   ============================================== */
/**
 * @brief 减速电机初始化函数
 * @param pstInit 减速电机设备指针
 */
void vGearMotorInit(void *pstInit) {
    /* 将void指针转换为减速电机设备参数指针 */
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstInit;
    
    /* 空指针检查 */
    if (pstGearMotor == NULL) {
        return;
    }

    /* 初始化默认方向为正向 */
    pstGearMotor->stRunningParam.emCurrentDir = emMotorDir_Forward;

    /* 根据不同平台初始化PWM */
    #if (QEPACK_PLATFORM == TI)
        /* TI平台：启动定时器计数器 */
        DL_Timer_startCounter(
            pstGearMotor->stStaticParam.stTimer->timer_inst
        );
    #else
        /* STM32平台：启动PWM输出，失败则死循环 */
        if (
            HAL_TIM_PWM_Start(
                pstGearMotor->stStaticParam.pstPWM_htim,
                pstGearMotor->stStaticParam.u32PWM_Channel
            )
            != HAL_OK) {
            while(1);  /* PWM启动失败，进入死循环 */
        }
    #endif
}

/* ==============================================
   位置模式周期执行：梯形速度规划 + 串级PID（位置环 → 速度环）
   ============================================== */
static void vGearMotorPosPeriodExecute(stGearMotorDeviceParamTdf *pstMotorDev)
{
    stGearMotorStaticParamTdf  *pstStatic = &pstMotorDev->stStaticParam;
    stGearMotorRunningParamTdf *pstRun    = &pstMotorDev->stRunningParam;

    int32_t lEncoderNow = (int32_t)ulEncoderGetCount(pstStatic->emEncoderDevNum);
    fix32_t fCurrentSpd = fEncoderGetSpeed(pstStatic->emEncoderDevNum);

    const stEncoderDeviceParamTdf *pstEncoder =
        c_pstGetEncoderDeviceParam(pstStatic->emEncoderDevNum);
    if (pstEncoder == NULL) return;
    int32_t lCountsPerRev = (int32_t)pstEncoder->stStaticParam.A_Round_Count
                          * (int32_t)pstEncoder->stStaticParam.Roto_Ratio;
    if (lCountsPerRev == 0) return;

    int32_t lRemaining    = pstRun->lTargetPos - lEncoderNow;
    int32_t lAbsRemaining = (lRemaining >= 0) ? lRemaining : -lRemaining;
    int8_t  bDirSign      = (lRemaining >= 0) ? 1 : -1;

    fix32_t fProfileSpd = pstRun->fProfileSpd;
    fix32_t fMaxVel     = (pstRun->fPosMaxVelRPM > FIX32_ZERO)
                          ? pstRun->fPosMaxVelRPM
                          : pstStatic->fMaxVelRPM;

    /* ─── 1. 梯形速度规划 ─── */
    int32_t lDecelPulses = 0;
    if (pstRun->ucAccelEn) {
        fix32_t fAccStep = pstRun->fAccStep;
        fix32_t fDecStep = pstRun->fDecStep;

        int64_t llNum = (int64_t)fProfileSpd * (int64_t)fProfileSpd
                      * (int64_t)lCountsPerRev;
        int64_t llDen = (int64_t)120000 * (int64_t)fDecStep;
        if (llDen > 0) {
            lDecelPulses = (int32_t)((llNum >> FIX32_FRAC_BITS) / llDen);
        } else {
            lDecelPulses = 1;
        }
        if (lDecelPulses < 1) lDecelPulses = 1;

        if (lDecelPulses >= lAbsRemaining || lAbsRemaining < 10) {
            fProfileSpd -= fDecStep;
            if (fProfileSpd < FIX32_ZERO) fProfileSpd = FIX32_ZERO;
        } else if (fProfileSpd < fMaxVel) {
            fProfileSpd += fAccStep;
            if (fProfileSpd > fMaxVel) fProfileSpd = fMaxVel;
        }
    } else {
        fProfileSpd = fMaxVel;
    }
    pstRun->fProfileSpd = fProfileSpd;

    /* ─── 2. 位置环 PID ─── */
    fix32_t fSignedProfileSpd = (bDirSign > 0) ? fProfileSpd : -fProfileSpd;
    fix32_t fVelCmd = fSignedProfileSpd;

    if (pstStatic->emPosPidDevNum != emNoPid) {
        int32_t lPosErr = pstRun->lTargetPos - lEncoderNow;
        const stPidDeviceParamTdf *pstPosPid =
            c_pstGetPidDeviceParam(pstStatic->emPosPidDevNum);
        if (pstPosPid != NULL) {
            fix32_t fKpPos = pstPosPid->stStaticParam.Kp;
            int64_t llCorrection = (int64_t)fKpPos * (int64_t)lPosErr;
            fix32_t fPosCorrection;
            if (llCorrection > (int64_t)FIX32_MAX)
                fPosCorrection = FIX32_MAX;
            else if (llCorrection < (int64_t)FIX32_MIN)
                fPosCorrection = FIX32_MIN;
            else
                fPosCorrection = (fix32_t)llCorrection;
            fPosCorrection = fix32_sat(fPosCorrection,
                (fix32_t)(-20 * 65536), (fix32_t)(20 * 65536));
            fVelCmd += fPosCorrection;
        }
    }

    /* 速度限幅 */
    fix32_t fAbsVelCmd = fix32_abs(fVelCmd);
    {
        fix32_t fLimit = (pstRun->fPosMaxVelRPM > FIX32_ZERO)
                         ? pstRun->fPosMaxVelRPM
                         : pstStatic->fMaxVelRPM;
        if (fLimit > FIX32_ZERO) {
            fix32_t fAbsMax = fLimit
                            + fix32_div(fLimit, ((fix32_t)(2 * 65536)));
            fAbsVelCmd = fix32_sat(fAbsVelCmd, FIX32_ZERO, fAbsMax);
        }
    }
    emMotorDirTdf emPosDir = (fVelCmd >= FIX32_ZERO)
                             ? emMotorDir_Forward : emMotorDir_Backward;
    fix32_t fAbsCurrentSpd = fix32_abs(fCurrentSpd);

    /* ─── 3. 速度环 PID ─── */
    fix32_t fPwmOutput = fAbsVelCmd;
    if (pstStatic->emPidDevNum != emNoPid) {
        uint32_t ulNow = QE_GET_TICK();
        if (ulNow - pstRun->ulPidLastTickMs >= pstStatic->usPidPeriodMs) {
            pstRun->ulPidLastTickMs = ulNow;
            vPidCalc(pstStatic->emPidDevNum, fAbsVelCmd, fAbsCurrentSpd);
        }
        fix32_t fPidOut;
        if (ePidGetOutput(pstStatic->emPidDevNum, &fPidOut) == QE_OK) {
            if (lDecelPulses >= lAbsRemaining) {
                fPidOut = fix32_sat(fPidOut,
                    (fix32_t)(-10 * 65536), (fix32_t)(10 * 65536));
            }
            fPwmOutput += fPidOut;
        }
    }
    if (fPwmOutput < FIX32_ZERO) fPwmOutput = FIX32_ZERO;

    int16_t sFinalSpeed = (int16_t)FIX32_TO_INT(fPwmOutput);
    if (emPosDir == emMotorDir_Backward) sFinalSpeed = -sFinalSpeed;
    vGearMotorSetSpeed(pstMotorDev, sFinalSpeed);

    /* ─── 4. 到位检测 ─── */
    if (lAbsRemaining <= 10 && FIX32_TO_INT(fAbsCurrentSpd) < 10) {
        pstRun->ucInPosition   = 1;
        pstRun->ucMotionEnable = 0;
        if (pstStatic->emPidDevNum != emNoPid)
            vPidReset(pstStatic->emPidDevNum);
        if (pstStatic->emPosPidDevNum != emNoPid)
            vPidReset(pstStatic->emPosPidDevNum);
        vGearMotorSetSpeed(pstMotorDev, 0);
        pstMotorDev->stBase.emMotorState = emMotorStateStop;
    }
}

/* ==============================================
   虚方法实现：周期性执行函数（核心控制循环）
   ============================================== */
/**
 * @brief 减速电机周期性执行函数（每1ms调用一次）
 * @param pstMotor 减速电机设备指针
 */
void vGearMotorPeriodExecute(void *pstMotor) {
    /* 类型转换 */
    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) {
        return;
    }

    /* 获取静态参数和运行参数指针 */
    stGearMotorStaticParamTdf  *pstStatic = &pstMotorDev->stStaticParam;
    stGearMotorRunningParamTdf *pstRun = &pstMotorDev->stRunningParam;
    const fix32_t fDtMs = FIX32_ONE;  /* 调用周期固定为1ms */

    /* 如果电机不在运行状态，直接返回 */
    if (pstMotorDev->stBase.emMotorState != emMotorStateRunning) {
        return;
    }

    /* 位置模式：梯形速度规划 + 串级PID */
    if (pstRun->emCtrlMode == emGearMotorCtrlMode_Pos && pstRun->ucMotionEnable) {
        vGearMotorPosPeriodExecute(pstMotorDev);
        return;
    }

    /* 获取编码器当前速度和绝对值 */
    fix32_t fEncoderSpeed = fGearMotorGetEncoderSpeedRPM(pstMotorDev);
    fix32_t fAbsEncoderSpeed = (fEncoderSpeed >= FIX32_ZERO) ? fEncoderSpeed : -fEncoderSpeed;
    fix32_t fSetpointSpeed = pstRun->fCurrentSetpointRPM;  /* 当前设定速度 */
    fix32_t fOutputSpeed = FIX32_ZERO;  /* 最终输出速度 */


    /* ==============================================
        速度模式：加速度规划更新
        ============================================== */
    if (pstRun->ucAccProfileActive) {
        pstRun->fElapsedTimeMs += fDtMs;  /* 更新已用时间 */

        /* 根据当前加速度阶段执行不同的处理 */
        switch (pstRun->emAccPhase) {
            case emGearMotorAccPhase_Accel: {  /* 加速阶段 */
                fix32_t fAccelRPM = pstRun->fAccelRPMpS;
                /* 计算每个周期的速度增量：步长 = (加速度 * 时间间隔) / 1000 */
                fix32_t fStep = fix32_div(fix32_mul(fAccelRPM, fDtMs), ((fix32_t)(1000 * 65536)));

                /* 智能双向斜坡：根据起始速度和目标速度的关系确定斜坡方向 */
                fix32_t fRampEnd;

                /* 速度模式：目标速度 */
                fRampEnd = (fix32_t)((int32_t)pstRun->sTargetSpeedRPM * 65536);

                /* 双向斜坡处理 */
                if (pstRun->fRampStartSpeedRPM <= fRampEnd) {
                    /* 起始速度小于等于终点速度：正向加速 */
                    fSetpointSpeed += fStep;
                    if (fSetpointSpeed >= fRampEnd) {
                        fSetpointSpeed = fRampEnd;  /* 达到终点，锁定 */
                    }
                } else {
                    /* 起始速度大于终点速度：反向减速 */
                    fSetpointSpeed -= fStep;
                    if (fSetpointSpeed <= fRampEnd) {
                        fSetpointSpeed = fRampEnd;  /* 达到终点，锁定 */
                    }
                }

                /* 速度模式：检查是否达到目标速度 */
                fix32_t fTarget = (fix32_t)((int32_t)pstRun->sTargetSpeedRPM * 65536);
                int bArrived = 0;

                /* 判断是否到达目标速度（考虑过冲情况） */
                if ((pstRun->fRampStartSpeedRPM <= fTarget && fSetpointSpeed >= fTarget) ||
                    (pstRun->fRampStartSpeedRPM > fTarget && fSetpointSpeed <= fTarget)) {
                    bArrived = 1;
                }

                if (bArrived) {
                    fSetpointSpeed = fTarget;  /* 锁定目标速度 */
                    pstRun->emAccPhase = emGearMotorAccPhase_Idle;  /* 进入空闲阶段 */
                    pstRun->ucAccProfileActive = 0;  /* 关闭加速度规划 */
                    /* 保持电机在运行状态，PID继续维持目标速度 */
                }
                break;
            }

            case emGearMotorAccPhase_Decel: {  /* 减速阶段 */
                fix32_t fAccelRPM = pstRun->fAccelRPMpS;
                /* 计算每个周期的速度减量 */
                fix32_t fStep = fix32_div(fix32_mul(fAccelRPM, fDtMs), ((fix32_t)(1000 * 65536)));

                if (pstRun->emCtrlMode == emGearMotorCtrlMode_Vel) {
                    /* 速度模式：从当前设定点向目标速度移动 */
                    fix32_t fTarget = (fix32_t)((int32_t)pstRun->sTargetSpeedRPM * 65536);

                    if (fSetpointSpeed > fTarget) {
                        /* 当前设定点大于目标：减速 */
                        fSetpointSpeed -= fStep;
                        if (fSetpointSpeed <= fTarget) {
                            fSetpointSpeed = fTarget;
                        }
                    } else if (fSetpointSpeed < fTarget) {
                        /* 当前设定点小于目标：加速 */
                        fSetpointSpeed += fStep;
                        if (fSetpointSpeed >= fTarget) {
                            fSetpointSpeed = fTarget;
                        }
                    }

                    /* 检查是否到达目标速度 */
                    int bArrived = 0;
                    if ((pstRun->fRampStartSpeedRPM <= fTarget && fSetpointSpeed >= fTarget) ||
                        (pstRun->fRampStartSpeedRPM > fTarget && fSetpointSpeed <= fTarget)) {
                        bArrived = 1;
                    }

                    if (bArrived) {
                        fSetpointSpeed = fTarget;

                        /* 方向切换处理：减速到0后翻转方向，立即开始加速，消除0点停顿 */
                        if (fTarget <= FIX32_0_01 && pstRun->emCurrentDir != pstRun->emTargetDir) {
                            pstRun->emCurrentDir = pstRun->emTargetDir;  /* 切换当前方向 */
                            fix32_t fFinalTarget = pstRun->fRampPeakSpeedRPM;  /* 最终目标速度 */
                            pstRun->sTargetSpeedRPM = (int16_t)FIX32_TO_INT(fFinalTarget);
                            pstRun->fRampStartSpeedRPM = FIX32_ZERO;
                            pstRun->emAccPhase = emGearMotorAccPhase_Accel;  /* 立即进入加速阶段 */

                            /* 立即执行Accel第一步，避免在0点停顿一个周期 */
                            fSetpointSpeed = fix32_div(fix32_mul(pstRun->fAccelRPMpS, fDtMs), ((fix32_t)(1000 * 65536)));
                            pstRun->fCurrentSetpointRPM = fSetpointSpeed;

                            /* 更新PID目标为最终目标速度 */
                            if (pstStatic->emPidDevNum != emNoPid) {
                                vPidSetTarget(pstStatic->emPidDevNum, fFinalTarget);
                            }
                        } else {
                            /* 普通到达目标：进入空闲阶段 */
                            pstRun->emAccPhase = emGearMotorAccPhase_Idle;
                            pstRun->ucAccProfileActive = 0;
                            /* 保持电机在运行状态，PID继续维持目标速度 */
                        }
                    }
                }
                break;
            }

            default:  /* 未知阶段：不做处理 */
                break;
        }

        pstRun->fCurrentSetpointRPM = fSetpointSpeed;  /* 更新当前设定速度 */
    }

    /* ==============================================
        速度模式：PID 计算
        ============================================== */
    if (pstStatic->emPidDevNum != emNoPid) {
        uint32_t ulNow = QE_GET_TICK();
        uint32_t ulElapsed = ulNow - pstRun->ulPidLastTickMs;

        if (ulElapsed >= pstStatic->usPidPeriodMs) {
            pstRun->ulPidLastTickMs = ulNow;
            vPidCalc(pstStatic->emPidDevNum, fSetpointSpeed, fAbsEncoderSpeed);
        }

        fix32_t fPidOutput;
        if (ePidGetOutput(pstStatic->emPidDevNum, &fPidOutput) == QE_OK) {
            fOutputSpeed = fPidOutput + fSetpointSpeed;
        } else {
            fOutputSpeed = fSetpointSpeed;
        }
    } else {
        fOutputSpeed = fSetpointSpeed;
    }

    /* ==============================================
       速度限幅（确保速度非负，方向由 emCurrentDir 控制）
       ============================================== */
    if (fOutputSpeed < FIX32_ZERO) {
        fOutputSpeed = FIX32_ZERO;  /* 速度不能为负 */
    }
    if (pstStatic->fMaxVelRPM > FIX32_ZERO) {
        fOutputSpeed = fix32_sat(fOutputSpeed, FIX32_ZERO, pstStatic->fMaxVelRPM);
    }

    /* ==============================================
       方向处理
       ============================================== */
    int16_t sFinalSpeed;
    /* 统一处理：根据当前方向调整速度符号 */
    sFinalSpeed = (int16_t)FIX32_TO_INT(fOutputSpeed);
    if (pstRun->emCurrentDir == emMotorDir_Backward) {
        sFinalSpeed = -sFinalSpeed;
    }

    /* 设置电机最终速度 */
    vGearMotorSetSpeed(pstMotorDev, sFinalSpeed);
}

/* ==============================================
   虚方法实现：停止函数
   ============================================== */
/**
 * @brief 减速电机停止函数
 * @param pstMotor 减速电机设备指针
 * @param bSyncFlag 同步标志（未使用）
 */
void vGearMotorStop(void *pstMotor, uint8_t bSyncFlag)
{
    (void)bSyncFlag;  /* 未使用参数，消除编译警告 */
    
    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) {
        return;
    }
    
    /* 清除加速度规划标志 */
    pstMotorDev->stRunningParam.ucAccProfileActive = 0;
    pstMotorDev->stRunningParam.emAccPhase = emGearMotorAccPhase_Idle;

    /* 清除位置控制状态 */
    pstMotorDev->stRunningParam.ucMotionEnable = 0;
    pstMotorDev->stRunningParam.ucInPosition  = 0;

    /* 重置PID积分 */
    if (pstMotorDev->stStaticParam.emPidDevNum != emNoPid) {
        vPidReset(pstMotorDev->stStaticParam.emPidDevNum);
    }
    if (pstMotorDev->stStaticParam.emPosPidDevNum != emNoPid) {
        vPidReset(pstMotorDev->stStaticParam.emPosPidDevNum);
    }

    /* 设置电机速度为0 */
    vGearMotorSetSpeed(pstMotor, 0);
}

/* ==============================================
   虚方法实现：使能函数
   ============================================== */
/**
 * @brief 减速电机使能函数（控制STBY引脚）
 * @param pstMotor 减速电机设备指针
 * @param bEnable 使能标志：1-使能，0-禁用
 * @param bSyncFlag 同步标志（未使用）
 */
void vGearMotorEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag)
{
    (void)bSyncFlag;  /* 未使用参数，消除编译警告 */
    
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstMotor;
    stGearMotorStaticParamTdf *pstStatic = &pstGearMotor->stStaticParam;

    /* STBY引脚配置为空，直接返回 */
    if(pstStatic->pstStbyGpioBase == NULL) {
        return;
    }

    /* 根据平台控制STBY引脚 */
    if (!bEnable) {
        /* 禁用：拉低STBY引脚 */
        #if (QEPACK_PLATFORM == TI)
            TI_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_RESET);
        #else
            HAL_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_RESET);
        #endif
    } else {
        /* 使能：拉高STBY引脚 */
        #if (QEPACK_PLATFORM == TI)
            TI_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_SET);
        #else
            HAL_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_SET);
        #endif
    }
}

/* ==============================================
   虚方法实现：获取电机状态函数
   ============================================== */
/**
 * @brief 获取减速电机当前状态
 * @param pstMotor 减速电机设备指针
 * @return 电机当前状态
 */
emMotorStateTdf emGetGearMotorState(void *pstMotor) {
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstMotor;
    return pstGearMotor->stBase.emMotorState;
}

/**
 * @brief 减速电机多机同步触发（空操作）
 * @param pstMotor 减速电机设备指针
 * @note  减速电机由本地 PWM 直接控制，不支持硬件多机同步，此函数为空操作
 */
void vGearMotorSynchronousMotion(void *pstMotor)
{
    (void)pstMotor;
}

/**
 * @brief 减速电机位置控制（串级PID + 梯形速度规划）
 * @param pstMotor 电机指针
 * @param emDir 电机方向
 * @param usVel 最大速度 (RPM)
 * @param ucAcc 加速度 (RPM/s)
 * @param ulClk 目标脉冲数
 * @param bAbsFlag 是否绝对位置：1-绝对位置，0-相对位置
 * @param bSyncFlag 是否同步执行（未使用）
 */
void vGearMotorPosControl(
    void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc,
    uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag)
{
    (void)bSyncFlag;

    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) return;

    stGearMotorStaticParamTdf  *pstStatic = &pstMotorDev->stStaticParam;
    stGearMotorRunningParamTdf *pstRun   = &pstMotorDev->stRunningParam;

    int32_t lEncoderNow = (int32_t)ulEncoderGetCount(pstStatic->emEncoderDevNum);

    /* 方向处理 */
    int32_t lSignedClk = (int32_t)ulClk;
    if (emDir == emMotorDir_Backward) lSignedClk = -lSignedClk;

    /* 计算目标位置 */
    int32_t lNewTarget;
    if (bAbsFlag) {
        lNewTarget = lSignedClk;
    } else {
        lNewTarget = lEncoderNow + lSignedClk;
    }

    /* 已在目标位置 */
    if (lNewTarget == lEncoderNow) {
        pstRun->ucInPosition   = 1;
        pstRun->ucMotionEnable = 0;
        return;
    }

    /* 速度限幅：存入运行参数，不污染静态参数 fMaxVelRPM
       避免位置模式覆写后，切回速度模式时被错误钳位 */
    if (usVel > 0) {
        pstRun->fPosMaxVelRPM = (fix32_t)((int64_t)(usVel) * 65536);
    } else {
        pstRun->fPosMaxVelRPM = FIX32_ZERO;  /* 0 = 沿用静态参数 fMaxVelRPM */
    }

    /* 若已在位置模式中运行，忽略重复调用 */
    if ((pstRun->emCtrlMode == emGearMotorCtrlMode_Pos)
        && pstRun->ucMotionEnable) {
        return;
    }

    /* 首次进入位置模式：设置目标并初始化 */
    pstRun->lTargetPos    = lNewTarget;
    pstRun->ucInPosition   = 0;
    pstRun->ucMotionEnable = 1;
    pstRun->fProfileSpd   = FIX32_ZERO;

    /* 梯形规划参数：ucAcc = 0 则关闭规划，直接跳变速度 */
    if (ucAcc > 0) {
        pstRun->ucAccelEn = 1;
        pstRun->fAccStep  = fix32_div((fix32_t)((int32_t)(ucAcc) * 65536), ((fix32_t)(1000 * 65536)));
        pstRun->fDecStep  = pstRun->fAccStep;  /* 加减速对称 */
    } else {
        pstRun->ucAccelEn = 0;
        pstRun->fAccStep  = FIX32_ZERO;
        pstRun->fDecStep  = FIX32_ZERO;
    }

    if (pstStatic->emPidDevNum != emNoPid)
        vPidReset(pstStatic->emPidDevNum);
    if (pstStatic->emPosPidDevNum != emNoPid)
        vPidReset(pstStatic->emPosPidDevNum);

    pstRun->emCtrlMode = emGearMotorCtrlMode_Pos;
    pstMotorDev->stBase.emMotorState = emMotorStateRunning;
}

/* ==============================================
   虚方法实现：速度控制函数
   ============================================== */
/**
 * @brief 减速电机速度控制（带梯形加速度规划）
 * @param pstMotor 电机指针
 * @param emDir 电机方向
 * @param usVel 目标速度 (RPM)
 * @param ucAcc 加速度 (RPM/s)
 * @param bSyncFlag 是否同步执行（未使用）
 */
void vGearMotorVelControl(
    void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc,
    uint8_t bSyncFlag)
{
    
    (void)bSyncFlag;  /* 未使用参数，消除编译警告 */

    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) {
        return;
    }

    stGearMotorStaticParamTdf  *pstStatic = &pstMotorDev->stStaticParam;
    stGearMotorRunningParamTdf *pstRun = &pstMotorDev->stRunningParam;

    /* 保存之前的控制模式 */
    emGearMotorCtrlModeTdf emPrevMode = pstRun->emCtrlMode;

    /* 从位置模式切换过来时，清除位置模式状态 */
    if (emPrevMode == emGearMotorCtrlMode_Pos) {
        pstRun->ucMotionEnable = 0;
        pstRun->ucInPosition  = 0;
    }

    /* 设置控制模式为速度模式 */
    pstRun->emCtrlMode = emGearMotorCtrlMode_Vel;
    pstRun->emTargetDir = emDir;

    /* 处理目标速度 */
    fix32_t fTargetRPM = (fix32_t)((int64_t)(usVel) * 65536);
    
    /* 速度限制：不超过硬件配置的最大速度 */
    if (pstStatic->fMaxVelRPM > FIX32_ZERO && fTargetRPM > pstStatic->fMaxVelRPM) {
        fTargetRPM = pstStatic->fMaxVelRPM;
    }
    pstRun->sTargetSpeedRPM = (int16_t)FIX32_TO_INT(fTargetRPM);

    /* 加速度=0 → 直接启动，不进行加速度规划 */
    if (ucAcc == 0) {
        pstRun->ucAccProfileActive = 0;
        pstRun->emAccPhase = emGearMotorAccPhase_Idle;
        pstRun->fCurrentSetpointRPM = fTargetRPM;
        pstRun->fEstimatedTimeMs = FIX32_ZERO;
        pstRun->fElapsedTimeMs = FIX32_ZERO;
        pstRun->emCurrentDir = emDir;

        vGearMotorPidSwitchToVel(pstStatic->emPidDevNum, emPrevMode, fTargetRPM);

        /* 直接设置速度 */
        int16_t sSpeed = (int16_t)FIX32_TO_INT(fTargetRPM);
        if (emDir == emMotorDir_Backward) sSpeed = -sSpeed;
        vGearMotorSetSpeed(pstMotorDev, sSpeed);

        /* 设置电机状态为运行 */
        pstMotorDev->stBase.emMotorState = emMotorStateRunning;
        return;
    }

    /* 加速度>0 → 进行加速度规划 */
    fix32_t fAccelRPMpS = (fix32_t)((int32_t)(ucAcc) * 65536);
    pstRun->fAccelRPMpS = fAccelRPMpS;

    /* 从位置模式切换过来时，用实际编码器速度修正加速度规划起点，
       避免 fCurrentSetpointRPM 过期（位置模式不维护此变量）导致从 0 缓慢爬升 */
    if (emPrevMode == emGearMotorCtrlMode_Pos) {
        fix32_t fActualSpeed = fEncoderGetSpeed(pstStatic->emEncoderDevNum);
        if (fActualSpeed < FIX32_ZERO) fActualSpeed = -fActualSpeed;
        /* 根据目标方向设置速度符号，确保方向和速度符号一致 */
        pstRun->fCurrentSetpointRPM = (emDir == emMotorDir_Backward) ? -fActualSpeed : fActualSpeed;
    }

    /* 获取当前设定点的绝对值 */
    fix32_t fStartFromSetpoint = pstRun->fCurrentSetpointRPM;
    fix32_t fCurrentAbsSetpoint = (fStartFromSetpoint >= FIX32_ZERO) ? fStartFromSetpoint : -fStartFromSetpoint;

    /* 检测方向是否发生变化：
     * 条件1：当前方向与目标方向不同
     * 条件2：当前设定速度大于0（不是从静止开始）
     * 条件3：之前已经在速度模式下 */
    int bDirChanged = (pstRun->emCurrentDir != emDir) &&
                      (fCurrentAbsSetpoint > FIX32_0_01) &&
                      (emPrevMode == emGearMotorCtrlMode_Vel);

    /* 初始化加速度规划 */
    pstRun->ucAccProfileActive = 1;
    pstRun->fElapsedTimeMs = FIX32_ZERO;

    if (bDirChanged) {
        /* 方向切换处理：先减速到0，到达0后在PeriodExecute中翻转方向再加速到目标 */
        pstRun->fRampStartSpeedRPM = fCurrentAbsSetpoint;
        pstRun->fCurrentSetpointRPM = fCurrentAbsSetpoint;
        pstRun->sTargetSpeedRPM = 0;              /* 中间目标：减速到0 */
        pstRun->fRampPeakSpeedRPM = fTargetRPM;   /* 最终目标：保存供翻转后使用 */
        pstRun->emAccPhase = emGearMotorAccPhase_Decel;
        
        /* 预估时间 = 减速到0的时间 + 从0加速到目标的时间 */
        pstRun->fEstimatedTimeMs = fCalcVelProfileTime(fCurrentAbsSetpoint, FIX32_ZERO, fAccelRPMpS)
                                 + fCalcVelProfileTime(FIX32_ZERO, fTargetRPM, fAccelRPMpS);
        
        vGearMotorPidSwitchToVel(pstStatic->emPidDevNum, emPrevMode, FIX32_ZERO);
    } else {
        /* 方向未变：直接从当前速度向目标速度过渡 */
        pstRun->emCurrentDir = emDir;
        pstRun->fRampStartSpeedRPM = fCurrentAbsSetpoint;
        pstRun->fCurrentSetpointRPM = fCurrentAbsSetpoint;
        pstRun->fRampPeakSpeedRPM = fTargetRPM;

        /* 根据当前速度与目标速度的关系确定阶段 */
        if (fCurrentAbsSetpoint < fTargetRPM) {
            pstRun->emAccPhase = emGearMotorAccPhase_Accel;  /* 当前速度小于目标：加速阶段 */
        } else if (fCurrentAbsSetpoint > fTargetRPM) {
            pstRun->emAccPhase = emGearMotorAccPhase_Decel;  /* 当前速度大于目标：减速阶段 */
        } else {
            /* 当前速度等于目标：无需规划 */
            pstRun->ucAccProfileActive = 0;
            pstRun->emAccPhase = emGearMotorAccPhase_Idle;
        }

        /* 计算预估完成时间 */
        pstRun->fEstimatedTimeMs = fCalcVelProfileTime(fCurrentAbsSetpoint, fTargetRPM, fAccelRPMpS);

        vGearMotorPidSwitchToVel(pstStatic->emPidDevNum, emPrevMode, fTargetRPM);
    }

    /* 设置电机状态为运行 */
    pstMotorDev->stBase.emMotorState = emMotorStateRunning;
}

/* ==============================================
   速度输出：设置电机PWM和方向
   ============================================== */
/**
 * @brief 设置减速电机速度（直接控制PWM和方向引脚）
 * @param pstMotor 减速电机设备指针
 * @param speed 速度值：正数-正转，负数-反转，0-停止
 */
void vGearMotorSetSpeed(void *pstMotor, int16_t speed)
{
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstGearMotor == NULL) {
        return;
    }

    stGearMotorStaticParamTdf *pstStatic = &pstGearMotor->stStaticParam;

    /* 计算速度的绝对值，用于PWM占空比 */
    uint16_t absSpeed = (speed < 0) ? (uint16_t)(-(int32_t)speed) : (uint16_t)speed;

    /* 占空比死区处理：克服摩擦力的最小驱动 */
    if (absSpeed > 0 && absSpeed < pstStatic->u16MinDuty) {
        absSpeed = pstStatic->u16MinDuty;
    }

    /* ==============================================
       控制电机方向引脚
       ============================================== */
    #if (QEPACK_PLATFORM == TI)
        /* TI平台 */
        if (speed > 0) {
            /* 正转：DIR1=1, DIR2=0 */
            TI_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_SET);
            TI_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        } else if (speed < 0) {
            /* 反转：DIR1=0, DIR2=1 */
            TI_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            TI_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_SET);
        } else {
            /* 停止：DIR1=0, DIR2=0 */
            TI_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            TI_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        }
    #else
        /* STM32平台 */
        if (speed > 0) {
            /* 正转：DIR1=1, DIR2=0 */
            HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        } else if (speed < 0) {
            /* 反转：DIR1=0, DIR2=1 */
            HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_SET);
        } else {
            /* 停止：DIR1=0, DIR2=0 */
            HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        }
    #endif

    /* ==============================================
       修改PWM占空比
       ============================================== */
    #if (QEPACK_PLATFORM == TI)
        /* TI平台：设置定时器捕获比较值 */
        DL_Timer_setCaptureCompareValue(
            pstStatic->stTimer->timer_inst,
            absSpeed,
            pstStatic->emChannel
        );
    #else
        /* STM32平台：设置定时器比较值 */
        __HAL_TIM_SET_COMPARE(pstStatic->pstPWM_htim, pstStatic->u32PWM_Channel, absSpeed);
    #endif
}

/* ==============================================
   设备注册：将减速电机设备注册到电机系统
   ============================================== */
/**
 * @brief 注册减速电机设备
 * @param emDevNum 电机设备编号
 * @param pstInit 静态初始化参数指针
 */
void vGearMotorRegister(emMotorDevNumTdf emDevNum, stGearMotorStaticParamTdf *pstInit)
{
    /* 计算设备偏移量：相对于第一个减速电机设备的偏移 */
    emMotorDevNumTdf offsetDevNum = (emMotorDevNumTdf)(emDevNum - emGearMotorDevNum0);

    /* 参数有效性检查 */
    if (offsetDevNum < GEAR_MOTOR_DEV_NUM && pstInit != NULL) {

        /* 初始化基类参数 */
        astGearMotorDeviceParam[offsetDevNum].stBase.emType = emMotorType_Gear;  /* 设置电机类型为减速电机 */
        astGearMotorDeviceParam[offsetDevNum].stBase.pstVTable = &g_stGearMotorVTable;  /* 设置虚函数表指针 */

        /* 复制静态参数 */
        memcpy(&astGearMotorDeviceParam[offsetDevNum].stStaticParam,
           pstInit,
           sizeof(stGearMotorStaticParamTdf));

        /* 清零运行参数 */
        memset(&astGearMotorDeviceParam[offsetDevNum].stRunningParam,
            0,
            sizeof(stGearMotorRunningParamTdf));

        /* 注册到电机基类系统 */
        vMotorRegisterDevice(emDevNum, &astGearMotorDeviceParam[offsetDevNum].stBase);
    }
}

/* ==============================================
   新增便捷接口：简化常用操作
   ============================================== */

/**
 * @brief 设置目标速度（便捷接口，直接调用 PID 目标设置）
 * @param pstMotor 减速电机设备指针
 * @param fTargetRPM 目标速度（RPM，定点数）：正数-正转，负数-反转
 */
void vGearMotorSetTargetSpeed(void *pstMotor, fix32_t fTargetRPM)
{
    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) return;

    stGearMotorStaticParamTdf *pstStatic = &pstMotorDev->stStaticParam;

    /* 根据目标速度符号确定方向 */
    emMotorDirTdf emDir = (fTargetRPM >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward;
    fix32_t fAbsRPM = (fTargetRPM >= FIX32_ZERO) ? fTargetRPM : -fTargetRPM;

    /* 速度限制 */
    if (pstStatic->fMaxVelRPM > FIX32_ZERO && fAbsRPM > pstStatic->fMaxVelRPM) {
        fAbsRPM = pstStatic->fMaxVelRPM;
    }

    /* 如果绑定了PID，直接设置PID目标速度 */
    if (pstStatic->emPidDevNum != emNoPid) {
        vPidSetTarget(pstStatic->emPidDevNum, fAbsRPM);
    }

    /* 调用速度控制函数，加速度设为0（直接设置） */
    vGearMotorVelControl(pstMotor, emDir, (uint16_t)FIX32_TO_INT(fAbsRPM), 0, 0);
}

/**
 * @brief 获取本次运动预估完成时间
 * @param pstMotor 减速电机设备指针
 * @return 预估完成时间（ms，定点数）
 */
fix32_t fGearMotorGetEstimatedTime(void *pstMotor)
{
    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) return FIX32_ZERO;
    return pstMotorDev->stRunningParam.fEstimatedTimeMs;
}

/**
 * @brief 获取当前实际速度
 * @param pstMotor 减速电机设备指针
 * @return 当前实际速度（RPM，定点数）
 */
fix32_t fGearMotorGetCurrentSpeed(void *pstMotor)
{
    stGearMotorDeviceParamTdf *pstMotorDev = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstMotorDev == NULL) return FIX32_ZERO;
    return fGearMotorGetEncoderSpeedRPM(pstMotorDev);
}

/* ==============================================
   调试接口：用于调试和监控
   ============================================== */

/**
 * @brief 获取当前设定速度（RPM）
 * @param emDevNum 电机设备编号
 * @return 当前设定速度（RPM，定点数）
 */
fix32_t fGearMotorGetSetpointRPM(emMotorDevNumTdf emDevNum)
{
    emMotorDevNumTdf offset = (emMotorDevNumTdf)(emDevNum - emGearMotorDevNum0);
    if (offset >= GEAR_MOTOR_DEV_NUM) return FIX32_ZERO;
    return astGearMotorDeviceParam[offset].stRunningParam.fCurrentSetpointRPM;
}

/**
 * @brief 获取当前加速度阶段
 * @param emDevNum 电机设备编号
 * @return 当前加速度阶段
 */
uint8_t ucGearMotorGetAccPhase(emMotorDevNumTdf emDevNum)
{
    emMotorDevNumTdf offset = (emMotorDevNumTdf)(emDevNum - emGearMotorDevNum0);
    if (offset >= GEAR_MOTOR_DEV_NUM) return 0;
    return (uint8_t)astGearMotorDeviceParam[offset].stRunningParam.emAccPhase;
}

#endif
