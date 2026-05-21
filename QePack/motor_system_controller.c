/**
  * @file       motor_system_controller.c
  * @author     QePack
  * @version    V3.0.0
  * @date       2026/5/22
  * @brief      电机系统控制器实现
  *             支持差速/麦轮底盘，传感器闭环控制，转弯/矫正功能
  */

#include "motor_system_controller.h"

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

stMotorSystemParamTdf g_stMotorSystemController;

/* 基准速度（用于传感器差速修正叠加） */
static fix32_t g_fBaseLeftSpeed  = FIX32_ZERO;
static fix32_t g_fBaseRightSpeed = FIX32_ZERO;

/* 保存的原始基准速度（修正始终以此为基础） */
static fix32_t g_fSavedBaseLeftSpeed  = FIX32_ZERO;
static fix32_t g_fSavedBaseRightSpeed = FIX32_ZERO;

/* 编译期定点常量 */
#define FIX32_2     ((fix32_t)(2  * FIX32_ONE))
#define FIX32_10    ((fix32_t)(10 * FIX32_ONE))
#define FIX32_60    ((fix32_t)(60 * FIX32_ONE))
#define FIX32_360   ((fix32_t)(360 * FIX32_ONE))
#define FIX32_90    ((fix32_t)(90  * FIX32_ONE))
#define FIX32_180   ((fix32_t)(180 * FIX32_ONE))
#define FIX32_PI_V  ((fix32_t)205887)  /* 3.14159265 in Q16.16 */
#define FIX32_HALF  ((fix32_t)(FIX32_ONE / 2))

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief 麦克纳姆轮速度解算
 */
static void vMecanumKinematics(int vx, int vy, int w, int *speed)
{
    speed[0] = +vx + vy - w;
    speed[1] = -vx + vy + w;
    speed[2] = -vx + vy - w;
    speed[3] = +vx + vy + w;
}

/**
 * @brief 根据反转标志获取实际方向
 */
static emMotorDirTdf emGetActualDir(emMotorDirTdf emDir, uint8_t ucReversed)
{
    if (ucReversed) {
        return (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;
    }
    return emDir;
}

/**
 * @brief 判断电机是否已停止
 */
static uint8_t ucMotorIsStopped(emMotorDevNumTdf emDev)
{
    emMotorStateTdf e = emGetMotorState(emDev);
    return (e == emMotorStateStop || e == emMotorStateIdle || e == emMotorStateNULL);
}

/**
 * @brief cm → 脉冲数（定点运算）
 */
static int32_t lCmToPulse(fix32_t fCm, fix32_t fCirc, fix32_t fPpr)
{
    if (fCirc < FIX32_ONE || fPpr < FIX32_ONE) return 0;
    return FIX32_TO_INT(fix32_mul(fix32_div(fCm, fCirc), fPpr));
}

/**
 * @brief 脉冲数 → cm（定点运算）
 */
static fix32_t fPulseToCm(int32_t lPulse, fix32_t fCirc, fix32_t fPpr)
{
    if (fCirc < FIX32_ONE || fPpr < FIX32_ONE) return FIX32_ZERO;
    return fix32_div(fix32_mul(FIX32_FROM_INT(lPulse), fCirc), fPpr);
}

/**
 * @brief 根据转弯类型获取目标角度偏移（定点）
 */
static fix32_t fGetTurnAngleOffset(emTurnTypeTdf emTurnType)
{
    switch (emTurnType) {
        case emTurnLeft90:   return  FIX32_90;
        case emTurnRight90:  return -FIX32_90;
        case emTurnLeft180:  return  FIX32_180;
        case emTurnRight180: return -FIX32_180;
        default:             return FIX32_ZERO;
    }
}

/**
 * @brief 获取位置模式默认速度（RPM），返回 fix32_t
 */
static fix32_t fGetDefaultPosSpeed(stMotorSystemStaticParamTdf *pstStatic)
{
    return (pstStatic->fDefaultPosSpeedRPM > FIX32_ZERO)
           ? pstStatic->fDefaultPosSpeedRPM : FIX32_60;
}

/**
 * @brief 左右差速设置 + 保存基准速度
 */
static void vSetDiffSpeedAndSave(fix32_t fLeft, fix32_t fRight)
{
    vMotorSystemSetSpeed(fLeft, fRight, fLeft, fRight);
    g_fSavedBaseLeftSpeed  = g_fBaseLeftSpeed;
    g_fSavedBaseRightSpeed = g_fBaseRightSpeed;
}

/**
 * @brief 4轮/2轮统一位置控制
 */
static void vAllPosControl(stMotorSystemStaticParamTdf *pstStatic,
                           emMotorDirTdf emDir, uint16_t usVel, uint32_t ulPulse)
{
    emMotorDirTdf emLfDir = emGetActualDir(emDir, pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(emDir, pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(emDir, pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(emDir, pstStatic->ucRightBackMotorReversed);

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
        case emChassisMecanum4:
            vMotorPosControl(pstStatic->emLeftBackMotorDevNum,  emLbDir, usVel, 0, ulPulse, 0, 1);
            vMotorPosControl(pstStatic->emRightBackMotorDevNum, emRbDir, usVel, 0, ulPulse, 0, 1);
            /* fall through */
        case emChassisDiff2:
            if (pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorPosControl(pstStatic->emLeftFrontMotorDevNum,  emLfDir, usVel, 0, ulPulse, 0, 1);
            if (pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulPulse, 0, 1);
            break;
        default:
            break;
    }
}

/**
 * @brief 传感器 PID 修正 → 左右差速应用
 * @return QE_OK 表示修正成功
 */
static QE_StatusTypeDef emApplySensorCorrection(stMotorSystemStaticParamTdf *pstStatic,
                                                 stMotorSystemRunningParamTdf *pstRunning,
                                                 fix32_t fTarget, fix32_t fSensorValue)
{
    vPidCalc(pstStatic->emSensorPidDevNum, fTarget, fSensorValue);

    fix32_t fCorrection;
    if (ePidGetOutput(pstStatic->emSensorPidDevNum, &fCorrection) != QE_OK) {
        return QE_ERROR;
    }

    fix32_t fLeftAdj  = g_fSavedBaseLeftSpeed  - fCorrection;
    fix32_t fRightAdj = g_fSavedBaseRightSpeed + fCorrection;
    vMotorSystemSetSpeed(fLeftAdj, fRightAdj, fLeftAdj, fRightAdj);
    return QE_OK;
}

/* ==================== 初始化 ==================== */

void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit)
{
    if (pstInit != NULL) {
        g_stMotorSystemController.stStaticParam = pstInit;
    }
    memset(&g_stMotorSystemController.stRunningParam, 0, sizeof(stMotorSystemRunningParamTdf));
}

const stMotorSystemParamTdf* c_pstGetMotorSystemControllerParam(void)
{
    return &g_stMotorSystemController;
}

emMotorStateTdf emGetMotorSystemState(void)
{
    return g_stMotorSystemController.stRunningParam.emMotorSystemState;
}

static void vMotorSystemSetState(emMotorStateTdf emState)
{
    g_stMotorSystemController.stRunningParam.emMotorSystemState = emState;
}

/* ==================== 使能/停止 ==================== */

void vMotorSystemEnable(uint8_t bEnable)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    if (pstStatic == NULL) return;

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
        case emChassisMecanum4:
            vMotorEnable(pstStatic->emLeftBackMotorDevNum, bEnable, 0);
            vMotorEnable(pstStatic->emRightBackMotorDevNum, bEnable, 0);
            /* fall through */
        case emChassisDiff2:
            if (pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorEnable(pstStatic->emLeftFrontMotorDevNum, bEnable, 0);
            if (pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorEnable(pstStatic->emRightFrontMotorDevNum, bEnable, 0);
            break;
        default:
            break;
    }
}

void vMotorSystemStop(void)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    vMotorSystemSetState(emMotorStateStop);
    pstRunning->ucTurningActive = 0;
    pstRunning->ucRectifyingActive = 0;
    pstRunning->ucSensorInfluenceActive = 0;

    g_fBaseLeftSpeed  = FIX32_ZERO;
    g_fBaseRightSpeed = FIX32_ZERO;
    g_fSavedBaseLeftSpeed  = FIX32_ZERO;
    g_fSavedBaseRightSpeed = FIX32_ZERO;

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
        case emChassisMecanum4:
            vMotorStop(pstStatic->emLeftBackMotorDevNum, 0);
            vMotorStop(pstStatic->emRightBackMotorDevNum, 0);
            /* fall through */
        case emChassisDiff2:
            vMotorStop(pstStatic->emLeftFrontMotorDevNum, 0);
            vMotorStop(pstStatic->emRightFrontMotorDevNum, 0);
            break;
        default:
            break;
    }
}

/* ==================== 速度控制 ==================== */

void vMotorSystemSetSpeed(fix32_t fLeftFrontSpeed, fix32_t fRightFrontSpeed,
                          fix32_t fLeftBackSpeed,  fix32_t fRightBackSpeed)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    if (pstStatic == NULL) return;

    /* 存储基准速度（取前后轮平均） */
    g_fBaseLeftSpeed  = (fix32_t)(((int64_t)fLeftFrontSpeed  + (int64_t)fLeftBackSpeed)  / 2);
    g_fBaseRightSpeed = (fix32_t)(((int64_t)fRightFrontSpeed + (int64_t)fRightBackSpeed) / 2);

    emMotorDirTdf emLfDir = emGetActualDir(
        (fLeftFrontSpeed >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward,
        pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(
        (fRightFrontSpeed >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward,
        pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(
        (fLeftBackSpeed >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward,
        pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(
        (fRightBackSpeed >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward,
        pstStatic->ucRightBackMotorReversed);

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
            vMotorVelControl(pstStatic->emLeftBackMotorDevNum, emLbDir,
                (uint16_t)FIX32_TO_INT(fix32_abs(fLeftBackSpeed)), 0, 1);
            vMotorVelControl(pstStatic->emRightBackMotorDevNum, emRbDir,
                (uint16_t)FIX32_TO_INT(fix32_abs(fRightBackSpeed)), 0, 1);
            /* fall through */
        case emChassisDiff2:
            if (pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorVelControl(pstStatic->emLeftFrontMotorDevNum, emLfDir,
                    (uint16_t)FIX32_TO_INT(fix32_abs(fLeftFrontSpeed)), 0, 1);
            if (pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorVelControl(pstStatic->emRightFrontMotorDevNum, emRfDir,
                    (uint16_t)FIX32_TO_INT(fix32_abs(fRightFrontSpeed)), 0, 1);
            break;
        default:
            break;
    }
}

void vMotorSystemVelControl(
    emMotorDirTdf emDir, uint8_t ucAcc,
    uint16_t usLeftFrontSpeed, uint16_t usRightFrontSpeed,
    uint16_t usLeftBackSpeed,  uint16_t usRightBackSpeed)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    if (pstStatic == NULL) return;

    /* 存储基准速度 */
    g_fBaseLeftSpeed  = (fix32_t)(((int32_t)usLeftFrontSpeed  + (int32_t)usLeftBackSpeed)  * (FIX32_ONE / 2));
    g_fBaseRightSpeed = (fix32_t)(((int32_t)usRightFrontSpeed + (int32_t)usRightBackSpeed) * (FIX32_ONE / 2));

    emMotorDirTdf emLfDir = emGetActualDir(emDir, pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(emDir, pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(emDir, pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(emDir, pstStatic->ucRightBackMotorReversed);

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
            vMotorVelControl(pstStatic->emLeftBackMotorDevNum, emLbDir, usLeftBackSpeed, ucAcc, 1);
            vMotorVelControl(pstStatic->emRightBackMotorDevNum, emRbDir, usRightBackSpeed, ucAcc, 1);
            /* fall through */
        case emChassisDiff2:
            vMotorVelControl(pstStatic->emLeftFrontMotorDevNum, emLfDir, usLeftFrontSpeed, ucAcc, 1);
            vMotorVelControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usRightFrontSpeed, ucAcc, 1);
            break;
        case emChassisMecanum4: {
            int speed[4];
            vMecanumKinematics(0, (int)usLeftFrontSpeed, 0, speed);
            vMotorVelControl(pstStatic->emLeftFrontMotorDevNum,  emGetActualDir(emLfDir, speed[0] < 0), (uint16_t)((speed[0] > 0) ? speed[0] : -speed[0]), ucAcc, 1);
            vMotorVelControl(pstStatic->emRightFrontMotorDevNum, emGetActualDir(emRfDir, speed[1] < 0), (uint16_t)((speed[1] > 0) ? speed[1] : -speed[1]), ucAcc, 1);
            vMotorVelControl(pstStatic->emLeftBackMotorDevNum,   emGetActualDir(emLbDir, speed[2] < 0), (uint16_t)((speed[2] > 0) ? speed[2] : -speed[2]), ucAcc, 1);
            vMotorVelControl(pstStatic->emRightBackMotorDevNum,  emGetActualDir(emRbDir, speed[3] < 0), (uint16_t)((speed[3] > 0) ? speed[3] : -speed[3]), ucAcc, 1);
            break;
        }
        default:
            break;
    }
}

/* ==================== 位置控制（直走） ==================== */

void vMotorSystemSetPosition(fix32_t fTargetCm)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    int32_t lPulse = lCmToPulse(fTargetCm, pstStatic->fWheelCircumferenceCm, pstStatic->fEncoderPulsePerRev);
    emMotorDirTdf emDir = (fTargetCm >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward;
    uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
    fix32_t fVelFix = fGetDefaultPosSpeed(pstStatic);
    uint16_t usVel = (uint16_t)FIX32_TO_INT(fVelFix);

    pstRunning->fTargetPositionCm = fTargetCm;
    pstRunning->ucSensorSuppressed = 1;
    vMotorSystemSetState(emMotorStateRunning);
    vAllPosControl(pstStatic, emDir, usVel, ulAbsPulse);
}

void vMotorSystemSetPositionX(fix32_t fTargetXCm)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;
    if (pstStatic->emChassisType != emChassisMecanum4) return;

    int32_t lPulse = lCmToPulse(fTargetXCm, pstStatic->fWheelCircumferenceCm, pstStatic->fEncoderPulsePerRev);
    uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
    emMotorDirTdf emDir = (fTargetXCm >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward;
    emMotorDirTdf emDirOpp = (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;
    fix32_t fVelFix = fGetDefaultPosSpeed(pstStatic);
    uint16_t usVel = (uint16_t)FIX32_TO_INT(fVelFix);

    pstRunning->fTargetPositionXCm = fTargetXCm;
    pstRunning->ucSensorSuppressed = 1;
    vMotorSystemSetState(emMotorStateRunning);

    /* 麦轮 X 轴：LF/RB 同向，RF/LB 反向 */
    emMotorDirTdf emLfDir = emGetActualDir(emDir,    pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(emDirOpp, pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(emDirOpp, pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(emDir,    pstStatic->ucRightBackMotorReversed);

    vMotorPosControl(pstStatic->emLeftFrontMotorDevNum,  emLfDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emLeftBackMotorDevNum,   emLbDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emRightBackMotorDevNum,  emRbDir, usVel, 0, ulAbsPulse, 0, 1);
}

void vMotorSystemSetPositionY(fix32_t fTargetYCm)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;
    if (pstStatic->emChassisType != emChassisMecanum4) return;

    int32_t lPulse = lCmToPulse(fTargetYCm, pstStatic->fWheelCircumferenceCm, pstStatic->fEncoderPulsePerRev);
    uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
    emMotorDirTdf emDir = (fTargetYCm >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward;
    fix32_t fVelFix = fGetDefaultPosSpeed(pstStatic);
    uint16_t usVel = (uint16_t)FIX32_TO_INT(fVelFix);

    pstRunning->fTargetPositionYCm = fTargetYCm;
    pstRunning->ucSensorSuppressed = 1;
    vMotorSystemSetState(emMotorStateRunning);
    vAllPosControl(pstStatic, emDir, usVel, ulAbsPulse);
}

/* ==================== 姿态控制 ==================== */

void vMotorSystemSetPose(fix32_t fTargetYawDeg, fix32_t fOmegaRadS)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    pstRunning->fTargetYaw = fTargetYawDeg;

#if SENSOR_IS_ENABLE
    if (pstStatic->emSensorDevNum != emNoSensor && pstStatic->emSensorPidDevNum != emNoPid) {
        vSensorSetTarget(pstStatic->emSensorDevNum, fTargetYawDeg);
        vPidSetTarget(pstStatic->emSensorPidDevNum, fTargetYawDeg);
    }
#endif

    /* fTurnSpeed = fOmegaRadS * fWheelBaseCm * 60 / (2 * fWheelCircumferenceCm) */
    fix32_t fDenom = fix32_mul(pstStatic->fWheelCircumferenceCm, FIX32_2);
    fix32_t fFactor = fix32_div(fix32_mul(pstStatic->fWheelBaseCm, FIX32_60), fDenom);
    fix32_t fTurnSpeed = fix32_mul(fOmegaRadS, fFactor);

    if (fTargetYawDeg > FIX32_ZERO) {
        vSetDiffSpeedAndSave(-fTurnSpeed, fTurnSpeed);
    } else {
        vSetDiffSpeedAndSave(fTurnSpeed, -fTurnSpeed);
    }

    vMotorSystemSetState(emMotorStateRunning);
}

/* ==================== 转弯 ==================== */

void vMotorSystemTurn(emTurnTypeTdf emTurnType, uint8_t bClosedLoop)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    if (pstStatic == NULL) return;

#if SENSOR_IS_ENABLE
    if (bClosedLoop && pstStatic->emSensorDevNum == emNoSensor) {
        vMotorSystemTurnOpenLoop(emTurnType, pstStatic->fOpenLoopTurnK);
        return;
    }
#endif

    if (!bClosedLoop) {
        vMotorSystemTurnOpenLoop(emTurnType, pstStatic->fOpenLoopTurnK);
        return;
    }

#if SENSOR_IS_ENABLE
    {
        stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
        fix32_t fAngleOffset = fGetTurnAngleOffset(emTurnType);
        pstRunning->fAccumulatedYaw += fAngleOffset;
        pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;
        pstRunning->ucTurningActive = 1;
        pstRunning->ucSensorSuppressed = 0;

        vSensorSetTarget(pstStatic->emSensorDevNum, pstRunning->fTargetYaw);
        if (pstStatic->emSensorPidDevNum != emNoPid) {
            vPidReset(pstStatic->emSensorPidDevNum);
            vPidSetTarget(pstStatic->emSensorPidDevNum, pstRunning->fTargetYaw);
        }

        vMotorSystemSetPose(pstRunning->fTargetYaw, FIX32_ONE);
        vMotorSystemSetState(emMotorStateRunning);
    }
#endif
}

void vMotorSystemTurnOpenLoop(emTurnTypeTdf emTurnType, fix32_t fK)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    fix32_t fAngleOffset = fGetTurnAngleOffset(emTurnType);
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;
    pstRunning->ucTurningActive = 1;
    pstRunning->ucSensorSuppressed = 1;

    fix32_t fTurnSpeed = fix32_mul(fK, FIX32_10);

    if (fAngleOffset > FIX32_ZERO) {
        vSetDiffSpeedAndSave(-fTurnSpeed, fTurnSpeed);
    } else {
        vSetDiffSpeedAndSave(fTurnSpeed, -fTurnSpeed);
    }

    vMotorSystemSetState(emMotorStateRunning);
}

void vMotorSystemTurnWithScheme(emTurnTypeTdf emTurnType, emTurnSchemeTdf emScheme,
                                 uint8_t bClosedLoop, fix32_t fK)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    fix32_t fAngleOffset = fGetTurnAngleOffset(emTurnType);
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;
    pstRunning->ucTurningActive = 1;

    if (!bClosedLoop || emScheme == emTurnScheme_Drift) {
        pstRunning->ucSensorSuppressed = 1;
    }

    fix32_t fTurnSpeed = fix32_mul(fK, FIX32_10);

    switch (emScheme) {
        case emTurnScheme_DiffSpin:
            if (fAngleOffset > FIX32_ZERO) {
                vSetDiffSpeedAndSave(-fTurnSpeed, fTurnSpeed);
            } else {
                vSetDiffSpeedAndSave(fTurnSpeed, -fTurnSpeed);
            }
            break;

        case emTurnScheme_OneSideStop:
            if (fAngleOffset > FIX32_ZERO) {
                vSetDiffSpeedAndSave(FIX32_ZERO, fTurnSpeed);
            } else {
                vSetDiffSpeedAndSave(fTurnSpeed, FIX32_ZERO);
            }
            break;

        case emTurnScheme_Drift: {
            /* fArcLength = fWheelBaseCm * fAngleOffset * PI / 360 */
            fix32_t fFactor = fix32_div(fix32_mul(pstStatic->fWheelBaseCm, FIX32_PI_V), FIX32_360);
            fix32_t fArcLength = fix32_mul(fFactor, fAngleOffset);
            int32_t lPulse = lCmToPulse(fArcLength, pstStatic->fWheelCircumferenceCm, pstStatic->fEncoderPulsePerRev);
            uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
            fix32_t fVelFix = fGetDefaultPosSpeed(pstStatic);
            uint16_t usVel = (uint16_t)FIX32_TO_INT(fVelFix);

            emMotorDirTdf emLfDir = emGetActualDir(emMotorDir_Forward, pstStatic->ucLeftFrontMotorReversed);
            emMotorDirTdf emRfDir = emGetActualDir(emMotorDir_Forward, pstStatic->ucRightFrontMotorReversed);

            if (fAngleOffset > FIX32_ZERO) {
                vMotorPosControl(pstStatic->emLeftFrontMotorDevNum,  emLfDir, usVel, 0, 0, 0, 1);
                vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulAbsPulse * 2, 0, 1);
            } else {
                vMotorPosControl(pstStatic->emLeftFrontMotorDevNum,  emLfDir, usVel, 0, ulAbsPulse * 2, 0, 1);
                vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, 0, 0, 1);
            }
            break;
        }
    }

    vMotorSystemSetState(emMotorStateRunning);

#if SENSOR_IS_ENABLE
    if (bClosedLoop && pstStatic->emSensorDevNum != emNoSensor) {
        vSensorSetTarget(pstStatic->emSensorDevNum, pstRunning->fTargetYaw);
        if (pstStatic->emSensorPidDevNum != emNoPid) {
            vPidSetTarget(pstStatic->emSensorPidDevNum, pstRunning->fTargetYaw);
        }
    }
#endif
}

/* ==================== 矫正 ==================== */

void vMotorSystemRectifyAuto(fix32_t fTargetTheta, uint32_t ulTimeoutMs)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

#if SENSOR_IS_ENABLE
    if (pstStatic->emSensorDevNum == emNoSensor) return;

    pstRunning->ucRectifyingActive = 1;
    pstRunning->ulRectifyTimeoutMs = ulTimeoutMs;
    pstRunning->fRectifyElapsedMs = FIX32_ZERO;
    pstRunning->ucSensorSuppressed = 0;

    vSensorSetTarget(pstStatic->emSensorDevNum, fTargetTheta);
    if (pstStatic->emSensorPidDevNum != emNoPid) {
        vPidReset(pstStatic->emSensorPidDevNum);
        vPidSetTarget(pstStatic->emSensorPidDevNum, fTargetTheta);
    }

    /* 启动低速差速旋转，由传感器 PID 修正 */
    {
        fix32_t fTurnSpeed = FIX32_2;
        if (fTargetTheta > FIX32_ZERO) {
            vSetDiffSpeedAndSave(-fTurnSpeed, fTurnSpeed);
        } else {
            vSetDiffSpeedAndSave(fTurnSpeed, -fTurnSpeed);
        }
    }

    vMotorSystemSetState(emMotorStateRunning);
#else
    (void)fTargetTheta;
    (void)ulTimeoutMs;
#endif
}

void vMotorSystemRectifyManual(fix32_t fOffset, uint32_t ulTimeoutMs)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    pstRunning->ucRectifyingActive = 1;
    pstRunning->ulRectifyTimeoutMs = ulTimeoutMs;
    pstRunning->fRectifyElapsedMs = FIX32_ZERO;
    pstRunning->ucSensorSuppressed = 1;

    fix32_t fTurnSpeed = fix32_mul(fOffset, pstStatic->fOpenLoopTurnK);

    if (fOffset > FIX32_ZERO) {
        vSetDiffSpeedAndSave(-fTurnSpeed, fTurnSpeed);
    } else {
        vSetDiffSpeedAndSave(fTurnSpeed, -fTurnSpeed);
    }

    vMotorSystemSetState(emMotorStateRunning);
}

/* ==================== 传感器管理 ==================== */

#if SENSOR_IS_ENABLE

void vMotorSystemSetSensor(emSensorDevNumTdf emSensorDevNum)
{
    g_stMotorSystemController.stStaticParam->emSensorDevNum = emSensorDevNum;
}

void vMotorSystemSetSensorWeight(fix32_t fWeight)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    pstStatic->fSensorWeight = fWeight;
    if (pstStatic->emSensorDevNum != emNoSensor) {
        vSensorSetWeight(pstStatic->emSensorDevNum, fWeight);
    }
    if (pstStatic->emSensorDevNum2 != emNoSensor) {
        vSensorSetWeight(pstStatic->emSensorDevNum2, FIX32_ONE - fWeight);
    }
}

void vMotorSystemSetSensorInfluenceMode(emSensorInfluenceModeTdf emMode)
{
    g_stMotorSystemController.stRunningParam.ucSensorInfluenceActive = (uint8_t)emMode;
}

#endif

/* ==================== 周期执行（核心） ==================== */

void vMotorSystemPeriodExecute(void)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    /* 检查所有电机是否停止，若停止则解除传感器抑制 */
    if (pstRunning->ucSensorSuppressed) {
        uint8_t bAllStopped = ucMotorIsStopped(pstStatic->emLeftFrontMotorDevNum)
                           && ucMotorIsStopped(pstStatic->emRightFrontMotorDevNum);
        if (bAllStopped && (pstStatic->emChassisType == emChassisDiff4 || pstStatic->emChassisType == emChassisMecanum4)) {
            bAllStopped = ucMotorIsStopped(pstStatic->emLeftBackMotorDevNum)
                       && ucMotorIsStopped(pstStatic->emRightBackMotorDevNum);
        }
        if (bAllStopped) {
            pstRunning->ucSensorSuppressed = 0;
        }
    }

    /* 手动矫正超时检查（独立于传感器） */
    if (pstRunning->ucRectifyingActive && pstRunning->ucSensorSuppressed) {
        pstRunning->fRectifyElapsedMs += (fix32_t)((int32_t)MOTOR_SYSTEM_CONTROLLER_PERIOD_MS * FIX32_ONE);
        if (pstRunning->fRectifyElapsedMs >= (fix32_t)((int64_t)pstRunning->ulRectifyTimeoutMs * FIX32_ONE)) {
            pstRunning->ucRectifyingActive = 0;
            pstRunning->fRectifyElapsedMs = FIX32_ZERO;
            pstRunning->ucSensorSuppressed = 0;
            vMotorSystemStop();
        }
    }

#if SENSOR_IS_ENABLE
    /* 传感器周期执行 */
    if (pstStatic->emSensorDevNum != emNoSensor) {
        vSensorPeriodExecute(pstStatic->emSensorDevNum);
    }
    if (pstStatic->emSensorDevNum2 != emNoSensor) {
        vSensorPeriodExecute(pstStatic->emSensorDevNum2);
    }

    /* 传感器闭环控制 */
    if (!pstRunning->ucSensorSuppressed && pstStatic->emSensorDevNum != emNoSensor
        && pstStatic->emSensorPidDevNum != emNoPid) {

        fix32_t fSensorValue = (pstStatic->emSensorDevNum2 != emNoSensor)
            ? fSensorFuseValue(pstStatic->emSensorDevNum, pstStatic->emSensorDevNum2)
            : fSensorGetValue(pstStatic->emSensorDevNum);

        fix32_t fTarget = fSensorGetTarget(pstStatic->emSensorDevNum);

        vPidCalc(pstStatic->emSensorPidDevNum, fTarget, fSensorValue);

        fix32_t fCorrection;
        if (ePidGetOutput(pstStatic->emSensorPidDevNum, &fCorrection) == QE_OK) {

            if (pstRunning->ucTurningActive) {
                /* 转弯模式：差速修正旋转 */
                fix32_t fLeftAdj  = g_fSavedBaseLeftSpeed  - fCorrection;
                fix32_t fRightAdj = g_fSavedBaseRightSpeed + fCorrection;
                vMotorSystemSetSpeed(fLeftAdj, fRightAdj, fLeftAdj, fRightAdj);

                /* 检查是否到达目标角度（2° 容差） */
                fix32_t fError = fix32_abs(fTarget - fSensorValue);
                if (fError < FIX32_2) {
                    pstRunning->ucTurningActive = 0;
                    vPidReset(pstStatic->emSensorPidDevNum);
                    vMotorSystemStop();
                }
            }
            else if (pstRunning->ucSensorInfluenceActive == (uint8_t)emSensorInfluence_DiffCorrect
                  || pstRunning->ucRectifyingActive) {
                /* 直走差速修正 / 自动矫正 */
                fix32_t fLeftAdj  = g_fSavedBaseLeftSpeed  - fCorrection;
                fix32_t fRightAdj = g_fSavedBaseRightSpeed + fCorrection;
                vMotorSystemSetSpeed(fLeftAdj, fRightAdj, fLeftAdj, fRightAdj);
            }
        }

        /* 自动矫正超时/收敛检查 */
        if (pstRunning->ucRectifyingActive) {
            pstRunning->fRectifyElapsedMs += (fix32_t)((int32_t)MOTOR_SYSTEM_CONTROLLER_PERIOD_MS * FIX32_ONE);

            fix32_t fError = fix32_abs(fTarget - fSensorValue);
            if (fError < FIX32_ONE
                || pstRunning->fRectifyElapsedMs >= (fix32_t)((int64_t)pstRunning->ulRectifyTimeoutMs * FIX32_ONE)) {
                pstRunning->ucRectifyingActive = 0;
                pstRunning->fRectifyElapsedMs = FIX32_ZERO;
                vPidReset(pstStatic->emSensorPidDevNum);
                vSensorReset(pstStatic->emSensorDevNum);
                vMotorSystemStop();
            }
        }
    }
#endif
}

#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */
