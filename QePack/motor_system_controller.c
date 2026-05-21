/**
  * @file       motor_system_controller.c
  * @author     QePack
  * @version    V2.0.0
  * @date       2026/5/10
  * @brief      电机系统控制器实现
  *             支持差速/麦轮底盘，传感器闭环控制，转弯/矫正功能
  */

#include "motor_system_controller.h"

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

stMotorSystemParamTdf g_stMotorSystemController;

/* 基准速度（用于传感器差速修正叠加） */
static fix32_t g_fBaseLeftSpeed = FIX32_ZERO;
static fix32_t g_fBaseRightSpeed = FIX32_ZERO;

/* 保存的原始基准速度（不会被修正回调覆盖，修正始终以此为基础） */
static fix32_t g_fSavedBaseLeftSpeed = FIX32_ZERO;
static fix32_t g_fSavedBaseRightSpeed = FIX32_ZERO;

/**
 * @brief 麦克纳姆轮速度解算
 * @param vx 横向速度（正值=右）
 * @param vy 前后速度（正值=前）
 * @param w  旋转角速度（正值=逆时针）
 * @param speed 输出数组[0]=左前, [1]=右前, [2]=左后, [3]=右后
 */
static void vMecanumKinematics(int vx, int vy, int w, int *speed)
{
    speed[0] = +vx + vy - w;   /* 左前轮 */
    speed[1] = -vx + vy + w;   /* 右前轮 */
    speed[2] = -vx + vy - w;   /* 左后轮 */
    speed[3] = +vx + vy + w;   /* 右后轮 */
}

/**
 * @brief 根据电机反转标志获取实际方向
 * @param emDir 原始方向
 * @param ucReversed 是否反转
 * @return emMotorDirTdf 实际方向
 */
static emMotorDirTdf emGetActualDir(emMotorDirTdf emDir, uint8_t ucReversed)
{
    if (ucReversed) {
        return (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;
    }
    return emDir;
}

/**
 * @brief cm 转 脉冲数
 */
static int32_t lCmToPulse(fix32_t fCm, fix32_t fWheelCircumferenceCm, fix32_t fPulsePerRev)
{
    float fCmF = fix32_to_float(fCm);
    float fCircF = fix32_to_float(fWheelCircumferenceCm);
    float fPprF = fix32_to_float(fPulsePerRev);
    if (fCircF < 0.001f || fPprF < 1.0f) {
        return 0;
    }
    return (int32_t)(fCmF / fCircF * fPprF);
}

/**
 * @brief 脉冲数 转 cm
 */
static fix32_t fPulseToCm(int32_t lPulse, fix32_t fWheelCircumferenceCm, fix32_t fPulsePerRev)
{
    float fCircF = fix32_to_float(fWheelCircumferenceCm);
    float fPprF = fix32_to_float(fPulsePerRev);
    if (fCircF < 0.001f || fPprF < 1.0f) {
        return FIX32_ZERO;
    }
    return fix32_from_float((float)lPulse / fPprF * fCircF);
}

/**
 * @brief 根据转弯类型获取目标角度偏移
 */
static float fGetTurnAngleOffset(emTurnTypeTdf emTurnType)
{
    switch (emTurnType) {
        case emTurnLeft90:   return  90.0f;
        case emTurnRight90:  return -90.0f;
        case emTurnLeft180:  return  180.0f;
        case emTurnRight180: return -180.0f;
        default:             return 0.0f;
    }
}

/* ==================== 初始化 ==================== */

void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit)
{
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;

    if (pstInit != NULL) {
        g_stMotorSystemController.stStaticParam = pstInit;
    }

    memset(pstRunning, 0, sizeof(stMotorSystemRunningParamTdf));
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

    g_fBaseLeftSpeed = FIX32_ZERO;
    g_fBaseRightSpeed = FIX32_ZERO;
    g_fSavedBaseLeftSpeed = FIX32_ZERO;
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

    /* 存储基准速度用于传感器修正（使用除法避免移位对负值的向下偏置） */
    {
        fix32_t fSumL = fLeftFrontSpeed + fLeftBackSpeed;
        fix32_t fSumR = fRightFrontSpeed + fRightBackSpeed;
        fix32_t fTwo = fix32_from_float(2.0f);
        g_fBaseLeftSpeed = fix32_div(fSumL, fTwo);
        g_fBaseRightSpeed = fix32_div(fSumR, fTwo);
    }

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
                (uint16_t)fix32_to_float(fix32_abs(fLeftBackSpeed)), 0, 1);
            vMotorVelControl(pstStatic->emRightBackMotorDevNum, emRbDir,
                (uint16_t)fix32_to_float(fix32_abs(fRightBackSpeed)), 0, 1);
            /* fall through */
        case emChassisDiff2:
            if (pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorVelControl(pstStatic->emLeftFrontMotorDevNum, emLfDir,
                    (uint16_t)fix32_to_float(fix32_abs(fLeftFrontSpeed)), 0, 1);
            if (pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorVelControl(pstStatic->emRightFrontMotorDevNum, emRfDir,
                    (uint16_t)fix32_to_float(fix32_abs(fRightFrontSpeed)), 0, 1);
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
    g_fBaseLeftSpeed  = fix32_from_float((float)(usLeftFrontSpeed  + usLeftBackSpeed)  / 2.0f);
    g_fBaseRightSpeed = fix32_from_float((float)(usRightFrontSpeed + usRightBackSpeed) / 2.0f);

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
            /* 麦轮：前后速度 → 四个轮子速度 */
            int speed[4];
            vMecanumKinematics(0, (int)usLeftFrontSpeed, 0, speed);
            vMotorVelControl(pstStatic->emLeftFrontMotorDevNum, emGetActualDir(emLfDir, speed[0] < 0), (uint16_t)((speed[0] > 0) ? speed[0] : -speed[0]), ucAcc, 1);
            vMotorVelControl(pstStatic->emRightFrontMotorDevNum, emGetActualDir(emRfDir, speed[1] < 0), (uint16_t)((speed[1] > 0) ? speed[1] : -speed[1]), ucAcc, 1);
            vMotorVelControl(pstStatic->emLeftBackMotorDevNum, emGetActualDir(emLbDir, speed[2] < 0), (uint16_t)((speed[2] > 0) ? speed[2] : -speed[2]), ucAcc, 1);
            vMotorVelControl(pstStatic->emRightBackMotorDevNum, emGetActualDir(emRbDir, speed[3] < 0), (uint16_t)((speed[3] > 0) ? speed[3] : -speed[3]), ucAcc, 1);
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
    float fVelRPM = (pstStatic->fDefaultPosSpeedRPM > FIX32_ZERO) ? fix32_to_float(pstStatic->fDefaultPosSpeedRPM) : 60.0f;
    uint16_t usVel = (uint16_t)fVelRPM;

    pstRunning->fTargetPositionCm = fTargetCm;
    /* 位置模式下传感器不介入 */
    pstRunning->ucSensorSuppressed = 1;

    vMotorSystemSetState(emMotorStateRunning);

    emMotorDirTdf emLfDir = emGetActualDir(emDir, pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(emDir, pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(emDir, pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(emDir, pstStatic->ucRightBackMotorReversed);

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
            vMotorPosControl(pstStatic->emLeftBackMotorDevNum, emLbDir, usVel, 0, ulAbsPulse, 0, 1);
            vMotorPosControl(pstStatic->emRightBackMotorDevNum, emRbDir, usVel, 0, ulAbsPulse, 0, 1);
            /* fall through */
        case emChassisDiff2:
            vMotorPosControl(pstStatic->emLeftFrontMotorDevNum, emLfDir, usVel, 0, ulAbsPulse, 0, 1);
            vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulAbsPulse, 0, 1);
            break;
        default:
            break;
    }
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
    emMotorDirTdf emDirOpposite = (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;
    float fVelRPM = (pstStatic->fDefaultPosSpeedRPM > FIX32_ZERO) ? fix32_to_float(pstStatic->fDefaultPosSpeedRPM) : 60.0f;
    uint16_t usVel = (uint16_t)fVelRPM;

    pstRunning->fTargetPositionXCm = fTargetXCm;
    pstRunning->ucSensorSuppressed = 1;
    vMotorSystemSetState(emMotorStateRunning);

    emMotorDirTdf emLfDir = emGetActualDir(emDir, pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(emDirOpposite, pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(emDirOpposite, pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(emDir, pstStatic->ucRightBackMotorReversed);

    /* 麦轮 X 轴：LF 和 RB 同向，RF 和 LB 反向 */
    vMotorPosControl(pstStatic->emLeftFrontMotorDevNum, emLfDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emLeftBackMotorDevNum, emLbDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emRightBackMotorDevNum, emRbDir, usVel, 0, ulAbsPulse, 0, 1);
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
    float fVelRPM = (pstStatic->fDefaultPosSpeedRPM > FIX32_ZERO) ? fix32_to_float(pstStatic->fDefaultPosSpeedRPM) : 60.0f;
    uint16_t usVel = (uint16_t)fVelRPM;

    pstRunning->fTargetPositionYCm = fTargetYCm;
    pstRunning->ucSensorSuppressed = 1;
    vMotorSystemSetState(emMotorStateRunning);

    emMotorDirTdf emLfDir = emGetActualDir(emDir, pstStatic->ucLeftFrontMotorReversed);
    emMotorDirTdf emRfDir = emGetActualDir(emDir, pstStatic->ucRightFrontMotorReversed);
    emMotorDirTdf emLbDir = emGetActualDir(emDir, pstStatic->ucLeftBackMotorReversed);
    emMotorDirTdf emRbDir = emGetActualDir(emDir, pstStatic->ucRightBackMotorReversed);

    vMotorPosControl(pstStatic->emLeftFrontMotorDevNum, emLfDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emLeftBackMotorDevNum, emLbDir, usVel, 0, ulAbsPulse, 0, 1);
    vMotorPosControl(pstStatic->emRightBackMotorDevNum, emRbDir, usVel, 0, ulAbsPulse, 0, 1);
}

/* ==================== 姿态控制 ==================== */

void vMotorSystemSetPose(fix32_t fTargetYawDeg, fix32_t fOmegaRadS)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    pstRunning->fTargetYaw = fTargetYawDeg;
    /* fOmegaRadS 预留，用于限制最大旋转速度 */

#if SENSOR_IS_ENABLE
    if (pstStatic->emSensorDevNum != emNoSensor && pstStatic->emSensorPidDevNum != emNoPid) {
        vSensorSetTarget(pstStatic->emSensorDevNum, fTargetYawDeg);
        vPidSetTarget(pstStatic->emSensorPidDevNum, fTargetYawDeg);
    }
#endif

    /* 差速旋转：左轮前进，右轮后退（或反之，取决于角度符号）
     * fTurnSpeed = fOmegaRadS * fWheelBaseCm / (2 * fWheelCircumferenceCm) * 60
     * 预计算 factor = fWheelBaseCm * 60 / (2 * fWheelCircumferenceCm)，避免运行时除法 */
    fix32_t fDenom = fix32_from_float(fix32_to_float(pstStatic->fWheelCircumferenceCm) * 2.0f);
    fix32_t fFactor = fix32_div(fix32_mul(pstStatic->fWheelBaseCm, fix32_from_float(60.0f)), fDenom);
    fix32_t fTurnSpeed = fix32_mul(fOmegaRadS, fFactor);

    if (fTargetYawDeg > FIX32_ZERO) {
        /* 左转：左轮后退，右轮前进 */
        vMotorSystemSetSpeed(-fTurnSpeed, fTurnSpeed, -fTurnSpeed, fTurnSpeed);
    } else {
        /* 右转：左轮前进，右轮后退 */
        vMotorSystemSetSpeed(fTurnSpeed, -fTurnSpeed, fTurnSpeed, -fTurnSpeed);
    }

    g_fSavedBaseLeftSpeed = g_fBaseLeftSpeed;
    g_fSavedBaseRightSpeed = g_fBaseRightSpeed;

    vMotorSystemSetState(emMotorStateRunning);
}

/* ==================== 转弯 ==================== */

void vMotorSystemTurn(emTurnTypeTdf emTurnType, uint8_t bClosedLoop)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    if (pstStatic == NULL) return;

#if SENSOR_IS_ENABLE
    if (bClosedLoop && pstStatic->emSensorDevNum == emNoSensor) {
        /* 无传感器时拒绝闭环转弯，转为开环 */
        vMotorSystemTurnOpenLoop(emTurnType, pstStatic->fOpenLoopTurnK);
        return;
    }
#endif

    if (!bClosedLoop) {
        vMotorSystemTurnOpenLoop(emTurnType, pstStatic->fOpenLoopTurnK);
        return;
    }

#if SENSOR_IS_ENABLE
    /* 闭环转弯：使用传感器 PID 控制 */
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;

    fix32_t fAngleOffset = fix32_from_float(fGetTurnAngleOffset(emTurnType));
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;
    pstRunning->ucTurningActive = 1;
    pstRunning->ucSensorSuppressed = 0;  /* 闭环转弯时传感器介入 */

    vSensorSetTarget(pstStatic->emSensorDevNum, pstRunning->fTargetYaw);

    if (pstStatic->emSensorPidDevNum != emNoPid) {
        vPidReset(pstStatic->emSensorPidDevNum);
        vPidSetTarget(pstStatic->emSensorPidDevNum, pstRunning->fTargetYaw);
    }

    /* 启动初始转速，由传感器 PID 在周期执行中修正 */
    vMotorSystemSetPose(pstRunning->fTargetYaw, FIX32_ONE);

    vMotorSystemSetState(emMotorStateRunning);
#endif
}

void vMotorSystemTurnOpenLoop(emTurnTypeTdf emTurnType, fix32_t fK)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    fix32_t fAngleOffset = fix32_from_float(fGetTurnAngleOffset(emTurnType));
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;
    pstRunning->ucTurningActive = 1;
    pstRunning->ucSensorSuppressed = 1;  /* 开环转弯时传感器不干涉 */

    /* 开环：用 K 值估算差速 */
    fix32_t fTurnSpeed = fix32_mul(fK, fix32_from_float(10.0f));  /* K 值映射到 RPM */

    if (fAngleOffset > FIX32_ZERO) {
        vMotorSystemSetSpeed(-fTurnSpeed, fTurnSpeed, -fTurnSpeed, fTurnSpeed);
    } else {
        vMotorSystemSetSpeed(fTurnSpeed, -fTurnSpeed, fTurnSpeed, -fTurnSpeed);
    }

    g_fSavedBaseLeftSpeed = g_fBaseLeftSpeed;
    g_fSavedBaseRightSpeed = g_fBaseRightSpeed;

    vMotorSystemSetState(emMotorStateRunning);
}

void vMotorSystemTurnWithScheme(emTurnTypeTdf emTurnType, emTurnSchemeTdf emScheme,
                                 uint8_t bClosedLoop, fix32_t fK)
{
    stMotorSystemStaticParamTdf *pstStatic = g_stMotorSystemController.stStaticParam;
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    if (pstStatic == NULL) return;

    fix32_t fAngleOffset = fix32_from_float(fGetTurnAngleOffset(emTurnType));
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;

    if (!bClosedLoop) {
        pstRunning->ucSensorSuppressed = 1;
    }

    pstRunning->ucTurningActive = 1;

    /* Drift 方案使用位置控制，传感器速度修正不兼容 */
    if (emScheme == emTurnScheme_Drift) {
        pstRunning->ucSensorSuppressed = 1;
    }

    fix32_t fTurnSpeed = fix32_mul(fK, fix32_from_float(10.0f));

    switch (emScheme) {
        case emTurnScheme_DiffSpin: {
            /* 原地差速旋转：左右反向 */
            if (fAngleOffset > FIX32_ZERO) {
                vMotorSystemSetSpeed(-fTurnSpeed, fTurnSpeed, -fTurnSpeed, fTurnSpeed);
            } else {
                vMotorSystemSetSpeed(fTurnSpeed, -fTurnSpeed, fTurnSpeed, -fTurnSpeed);
            }
            g_fSavedBaseLeftSpeed = g_fBaseLeftSpeed;
            g_fSavedBaseRightSpeed = g_fBaseRightSpeed;
            break;
        }
        case emTurnScheme_OneSideStop: {
            /* 单侧停止旋转：一侧速度为0 */
            if (fAngleOffset > FIX32_ZERO) {
                vMotorSystemSetSpeed(FIX32_ZERO, fTurnSpeed, FIX32_ZERO, fTurnSpeed);
            } else {
                vMotorSystemSetSpeed(fTurnSpeed, FIX32_ZERO, fTurnSpeed, FIX32_ZERO);
            }
            g_fSavedBaseLeftSpeed = g_fBaseLeftSpeed;
            g_fSavedBaseRightSpeed = g_fBaseRightSpeed;
            break;
        }
        case emTurnScheme_Drift: {
            /* 漂移转弯：使用位置模式，左右路程差
             * fArcLength = (fWheelBaseCm/2) * (fAngleOffset * PI / 180)
             * = fWheelBaseCm * fAngleOffset * PI / 360
             * 预计算 factor = fWheelBaseCm * PI / 360 */
            float fFactorF = fix32_to_float(pstStatic->fWheelBaseCm) * 3.141592653f / 360.0f;
            fix32_t fArcLength = fix32_from_float(fFactorF * fix32_to_float(fAngleOffset));
            int32_t lPulse = lCmToPulse(fArcLength, pstStatic->fWheelCircumferenceCm, pstStatic->fEncoderPulsePerRev);
            uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
            float fVelRPM = (pstStatic->fDefaultPosSpeedRPM > FIX32_ZERO) ? fix32_to_float(pstStatic->fDefaultPosSpeedRPM) : 60.0f;
            uint16_t usVel = (uint16_t)fVelRPM;

            emMotorDirTdf emLfDir = emGetActualDir(emMotorDir_Forward, pstStatic->ucLeftFrontMotorReversed);
            emMotorDirTdf emRfDir = emGetActualDir(emMotorDir_Forward, pstStatic->ucRightFrontMotorReversed);
            emMotorDirTdf emLbDir = emGetActualDir(emMotorDir_Forward, pstStatic->ucLeftBackMotorReversed);
            emMotorDirTdf emRbDir = emGetActualDir(emMotorDir_Forward, pstStatic->ucRightBackMotorReversed);

            if (fAngleOffset > FIX32_ZERO) {
                /* 左转：右侧走更多（左轮停，右轮走 arc*2） */
                vMotorPosControl(pstStatic->emLeftFrontMotorDevNum, emLfDir, usVel, 0, 0, 0, 1);
                vMotorPosControl(pstStatic->emRightFrontMotorDevNum, emRfDir, usVel, 0, ulAbsPulse * 2, 0, 1);
            } else {
                vMotorPosControl(pstStatic->emLeftFrontMotorDevNum, emLfDir, usVel, 0, ulAbsPulse * 2, 0, 1);
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
    if (pstStatic->emSensorDevNum == emNoSensor) {
        /* 无传感器时拒绝自动矫正 */
        return;
    }

    pstRunning->ucRectifyingActive = 1;
    pstRunning->ulRectifyTimeoutMs = ulTimeoutMs;
    pstRunning->fRectifyElapsedMs = FIX32_ZERO;
    pstRunning->ucSensorSuppressed = 0;

    vSensorSetTarget(pstStatic->emSensorDevNum, fTargetTheta);

    if (pstStatic->emSensorPidDevNum != emNoPid) {
        vPidReset(pstStatic->emSensorPidDevNum);
        vPidSetTarget(pstStatic->emSensorPidDevNum, fTargetTheta);
    }

    /* 启动基础差速旋转，由传感器 PID 在周期执行中修正 */
    {
        fix32_t fTurnSpeed = fix32_from_float(2.0f);
        if (fTargetTheta > FIX32_ZERO) {
            vMotorSystemSetSpeed(-fTurnSpeed, fTurnSpeed, -fTurnSpeed, fTurnSpeed);
        } else {
            vMotorSystemSetSpeed(fTurnSpeed, -fTurnSpeed, fTurnSpeed, -fTurnSpeed);
        }
    }

    g_fSavedBaseLeftSpeed = g_fBaseLeftSpeed;
    g_fSavedBaseRightSpeed = g_fBaseRightSpeed;

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

    /* 手动矫正：使用差速原地旋转方案 */
    fix32_t fTurnSpeed = fix32_mul(fOffset, pstStatic->fOpenLoopTurnK);

    if (fOffset > FIX32_ZERO) {
        vMotorSystemSetSpeed(-fTurnSpeed, fTurnSpeed, -fTurnSpeed, fTurnSpeed);
    } else {
        vMotorSystemSetSpeed(fTurnSpeed, -fTurnSpeed, fTurnSpeed, -fTurnSpeed);
    }

    g_fSavedBaseLeftSpeed = g_fBaseLeftSpeed;
    g_fSavedBaseRightSpeed = g_fBaseRightSpeed;

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
    g_stMotorSystemController.stStaticParam->fSensorWeight = fWeight;
    if (g_stMotorSystemController.stStaticParam->emSensorDevNum != emNoSensor) {
        vSensorSetWeight(g_stMotorSystemController.stStaticParam->emSensorDevNum, fWeight);
    }
    if (g_stMotorSystemController.stStaticParam->emSensorDevNum2 != emNoSensor) {
        vSensorSetWeight(g_stMotorSystemController.stStaticParam->emSensorDevNum2, FIX32_ONE - fWeight);
    }
}

void vMotorSystemSetSensorInfluenceMode(emSensorInfluenceModeTdf emMode)
{
    /* ucSensorInfluenceActive 在 PeriodExecute 中使用 */
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
        uint8_t bAllStopped = 1;
        emMotorStateTdf eLf = emGetMotorState(pstStatic->emLeftFrontMotorDevNum);
        emMotorStateTdf eRf = emGetMotorState(pstStatic->emRightFrontMotorDevNum);
        if (eLf != emMotorStateStop && eLf != emMotorStateIdle && eLf != emMotorStateNULL) bAllStopped = 0;
        if (eRf != emMotorStateStop && eRf != emMotorStateIdle && eRf != emMotorStateNULL) bAllStopped = 0;
        if (pstStatic->emChassisType == emChassisDiff4 || pstStatic->emChassisType == emChassisMecanum4) {
            emMotorStateTdf eLb = emGetMotorState(pstStatic->emLeftBackMotorDevNum);
            emMotorStateTdf eRb = emGetMotorState(pstStatic->emRightBackMotorDevNum);
            if (eLb != emMotorStateStop && eLb != emMotorStateIdle && eLb != emMotorStateNULL) bAllStopped = 0;
            if (eRb != emMotorStateStop && eRb != emMotorStateIdle && eRb != emMotorStateNULL) bAllStopped = 0;
        }
        if (bAllStopped) {
            pstRunning->ucSensorSuppressed = 0;
        }
    }

    /* 手动矫正超时检查（独立于传感器） */
    if (pstRunning->ucRectifyingActive && pstRunning->ucSensorSuppressed) {
        pstRunning->fRectifyElapsedMs += fix32_from_float((float)MOTOR_SYSTEM_CONTROLLER_PERIOD_MS);
        if (pstRunning->fRectifyElapsedMs >= fix32_from_float((float)pstRunning->ulRectifyTimeoutMs)) {
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

        fix32_t fSensorValue;

        /* 互补滤波融合 */
        if (pstStatic->emSensorDevNum2 != emNoSensor) {
            fSensorValue = fSensorFuseValue(pstStatic->emSensorDevNum, pstStatic->emSensorDevNum2);
        } else {
            fSensorValue = fSensorGetValue(pstStatic->emSensorDevNum);
        }

        fix32_t fTarget = fSensorGetTarget(pstStatic->emSensorDevNum);

        vPidCalc(pstStatic->emSensorPidDevNum, fTarget, fSensorValue);

        fix32_t fCorrection;
        if (ePidGetOutput(pstStatic->emSensorPidDevNum, &fCorrection) == QE_OK) {

            /* 转弯模式：传感器 PID 输出控制旋转 */
            if (pstRunning->ucTurningActive) {
                /* 差速修正旋转：PID 输出加到左右轮差速（以保存的原始基准为底） */
                fix32_t fLeftAdj = g_fSavedBaseLeftSpeed - fCorrection;
                fix32_t fRightAdj = g_fSavedBaseRightSpeed + fCorrection;
                vMotorSystemSetSpeed(fLeftAdj, fRightAdj, fLeftAdj, fRightAdj);

                /* 检查是否到达目标角度 */
                fix32_t fError = fTarget - fSensorValue;
                if (fError < FIX32_ZERO) fError = -fError;
                if (fError < fix32_from_float(2.0f)) {  /* 2° 容差 */
                    pstRunning->ucTurningActive = 0;
                    vPidReset(pstStatic->emSensorPidDevNum);
                    vMotorSystemStop();
                }
            }
            /* 直走差速修正模式 */
            else if (pstRunning->ucSensorInfluenceActive == (uint8_t)emSensorInfluence_DiffCorrect) {
                fix32_t fLeftAdj = g_fSavedBaseLeftSpeed - fCorrection;
                fix32_t fRightAdj = g_fSavedBaseRightSpeed + fCorrection;
                vMotorSystemSetSpeed(fLeftAdj, fRightAdj, fLeftAdj, fRightAdj);
            }
            /* 自动矫正模式 */
            else if (pstRunning->ucRectifyingActive) {
                fix32_t fLeftAdj = g_fSavedBaseLeftSpeed - fCorrection;
                fix32_t fRightAdj = g_fSavedBaseRightSpeed + fCorrection;
                vMotorSystemSetSpeed(fLeftAdj, fRightAdj, fLeftAdj, fRightAdj);
            }
        }

        /* 自动矫正模式 */
        if (pstRunning->ucRectifyingActive) {
            pstRunning->fRectifyElapsedMs += fix32_from_float((float)MOTOR_SYSTEM_CONTROLLER_PERIOD_MS);

            fix32_t fError = fTarget - fSensorValue;
            if (fError < FIX32_ZERO) fError = -fError;

            if (fError < FIX32_ONE || pstRunning->fRectifyElapsedMs >= fix32_from_float((float)pstRunning->ulRectifyTimeoutMs)) {
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
