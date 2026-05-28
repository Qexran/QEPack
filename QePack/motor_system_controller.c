/**
  * @file       motor_system_controller.c
  * @author     QePack
  * @version    V3.0.0
  * @date       2026/5/24
  * @brief      电机系统控制器实现（重构版）
  */

#include "motor_system_controller.h"
#include <math.h>

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

#if EMM_MOTOR_IS_ENABLE
#include "emm_step_motor_device.h"
#endif

stMotorSystemParamTdf g_stMotorSystemController;

/* ==================== 内部辅助 ==================== */

/**
 * @brief  获取静态参数指针
 */
static stMotorSystemStaticParamTdf *pstGetStatic(void) {
    return &g_stMotorSystemController.stStaticParam;
}

/**
 * @brief  获取运行参数指针
 */
static stMotorSystemRunningParamTdf *pstGetRunning(void) {
    return &g_stMotorSystemController.stRunningParam;
}

/**
 * @brief  获取底盘轮子数
 * @param  emType 底盘类型
 * @return 2（差速2轮）或 4（其他）
 */
static uint8_t ucMscWheelCount(emChassisTypeTdf emType)
{
    return (emType == emChassisDiff2) ? 2 : 4;
}

/**
 * @brief  遍历所有已配置轮子执行使能/失能
 * @param  pstStatic 静态参数指针
 * @param  bEnable   1=使能, 0=失能
 */
static void vMscForeachEnable(stMotorSystemStaticParamTdf *pstStatic, uint8_t bEnable)
{
    uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
    for (uint8_t i = 0; i < ucCount; i++) {
        if (pstStatic->astWheels[i].emDevNum != emNoMotorDevNum) {
            vMotorEnable(pstStatic->astWheels[i].emDevNum, bEnable, 1);
        }
    }
}

/**
 * @brief  遍历所有已配置轮子执行停止
 * @param  pstStatic 静态参数指针
 */
static void vMscForeachStop(stMotorSystemStaticParamTdf *pstStatic)
{
    uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
    for (uint8_t i = 0; i < ucCount; i++) {
        if (pstStatic->astWheels[i].emDevNum != emNoMotorDevNum) {
            vMotorStop(pstStatic->astWheels[i].emDevNum, 1);
        }
    }
}

/**
 * @brief  遍历已配置轮子发出同步信号
 * @note   EMM 用广播触发，齿轮电机空操作跳过
 * @param  pstStatic 静态参数指针
 */
static void vMscForeachSync(stMotorSystemStaticParamTdf *pstStatic)
{
#if EMM_MOTOR_IS_ENABLE
    uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
    for (uint8_t i = 0; i < ucCount; i++) {
        emMotorDevNumTdf emDev = pstStatic->astWheels[i].emDevNum;
        if (emDev == emNoMotorDevNum) continue;
        const stEmmMotorStaticParamTdf *pstEmm = c_pstGetEmmMotorStaticParam(emDev);
        if (pstEmm != NULL) {
            vEmmMotorSyncBroadcast(pstEmm->emUartDevNum);
            return;
        }
    }
#endif
    (void)pstStatic;
}

/**
 * @brief  检查所有已配置电机是否已停止
 * @param  pstStatic 静态参数指针
 * @return 1=全部停止, 0=仍有电机在运行
 */
static uint8_t ucMscAllMotorsStopped(stMotorSystemStaticParamTdf *pstStatic)
{
    uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
    for (uint8_t i = 0; i < ucCount; i++) {
        if (pstStatic->astWheels[i].emDevNum != emNoMotorDevNum) {
            emMotorStateTdf e = emGetMotorState(pstStatic->astWheels[i].emDevNum);
            if (e != emMotorStateStop && e != emMotorStateIdle) return 0;
        }
    }
    return 1;
}

/**
 * @brief  转换到空闲状态并重置运动标志
 * @param  pstRunning 运行参数指针
 */
static void vMscTransitionToIdle(stMotorSystemRunningParamTdf *pstRunning)
{
    pstRunning->emState = emMscState_Idle;
    pstRunning->fSavedBaseLeftSpeed  = FIX32_ZERO;
    pstRunning->fSavedBaseRightSpeed = FIX32_ZERO;
    pstRunning->ucMotionDone = 1;
    pstRunning->ulEstimateStopTick = 0;
}

/**
 * @brief  转弯类型转换为角度偏移量
 * @param  emTurnType 转弯类型枚举
 * @return 角度偏移（正=左转CCW, 负=右转CW）
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
 * @brief  获取位置模式默认速度
 * @param  pstStatic 静态参数指针
 * @return 默认速度(RPM)，最小返回 60
 */
static fix32_t fGetDefaultPosSpeed(stMotorSystemStaticParamTdf *pstStatic)
{
    return (pstStatic->fDefaultPosSpeedRPM > FIX32_ZERO)
           ? pstStatic->fDefaultPosSpeedRPM : FIX32_60;
}

/**
 * @brief  从主轮（LF）的 EMM 参数将厘米转换为脉冲数
 * @param  pstStatic 静态参数指针
 * @param  fCm      目标距离(cm)
 * @return 脉冲数，0=参数未配置
 */
static int32_t lMscGetMasterPulse(stMotorSystemStaticParamTdf *pstStatic, fix32_t fCm)
{
#if EMM_MOTOR_IS_ENABLE
    emMotorDevNumTdf emMasterDev = pstStatic->astWheels[MOTOR_WHEEL_LF].emDevNum;
    if (emMasterDev == emNoMotorDevNum) return 0;

    const stEmmMotorStaticParamTdf *pstEmm = c_pstGetEmmMotorStaticParam(emMasterDev);
    if (pstEmm == NULL) return 0;

    return lCmToPulse(fCm, pstEmm->fWheelDiameterCm, pstEmm->fEncoderPulsePerRev);
#else
    (void)pstStatic; (void)fCm;
    return 0;
#endif
}

/**
 * @brief  从主轮（LF）的 EMM 参数获取轮子周长
 * @param  pstStatic 静态参数指针
 * @return 轮子周长(cm)，0=未配置
 */
static fix32_t fMscGetMasterCircumference(stMotorSystemStaticParamTdf *pstStatic)
{
#if EMM_MOTOR_IS_ENABLE
    emMotorDevNumTdf emMasterDev = pstStatic->astWheels[MOTOR_WHEEL_LF].emDevNum;
    if (emMasterDev == emNoMotorDevNum) return FIX32_ZERO;

    const stEmmMotorStaticParamTdf *pstEmm = c_pstGetEmmMotorStaticParam(emMasterDev);
    if (pstEmm == NULL || pstEmm->fWheelDiameterCm < FIX32_ONE) return FIX32_ZERO;

    return fix32_mul(FIX32_PI, pstEmm->fWheelDiameterCm);
#else
    (void)pstStatic;
    return FIX32_ZERO;
#endif
}

/** 运动轴 */
typedef enum {
    emMscAxis_Y = 0,
    emMscAxis_X = 1,
} emMscAxisTdf;

/* ==================== 内部运动原语 ==================== */

/**
 * @brief  原始设速（不改变状态机状态）
 * @note   自动处理 EMM 电机 ucReversed 补偿，供内部分速度控制和姿态控制复用
 * @param  fLeftFrontSpeed  LF 速度(RPM)
 * @param  fRightFrontSpeed RF 速度(RPM)
 * @param  fLeftBackSpeed   LB 速度(RPM)
 * @param  fRightBackSpeed  RB 速度(RPM)
 */
static void vMscSetSpeedRaw(fix32_t fLeftFrontSpeed, fix32_t fRightFrontSpeed,
                             fix32_t fLeftBackSpeed,  fix32_t fRightBackSpeed)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);

    fix32_t afSpeed[4] = { fLeftFrontSpeed, fRightFrontSpeed, fLeftBackSpeed, fRightBackSpeed };

    for (uint8_t i = 0; i < ucCount; i++) {
        stMotorWheelConfigTdf *pW = &pstStatic->astWheels[i];
        if (pW->emDevNum == emNoMotorDevNum) continue;

        fix32_t fSpd = afSpeed[i];

        emMotorDirTdf emDir = (fSpd >= FIX32_ZERO) ? emMotorDir_Forward : emMotorDir_Backward;

        vMotorVelControl(pW->emDevNum, emDir,
            (uint16_t)FIX32_TO_INT(fix32_abs(fSpd)), pstStatic->ucGlobalAcc, 1);
    }
    vMscForeachSync(pstStatic);
}

/**
 * @brief  差速设速并保存基准值（供传感器闭环修正使用）
 * @param  fLeft  左侧速度(RPM)
 * @param  fRight 右侧速度(RPM)
 */
static void vSetDiffSpeedAndSave(fix32_t fLeft, fix32_t fRight)
{
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();
    pstRunning->fSavedBaseLeftSpeed  = fLeft;
    pstRunning->fSavedBaseRightSpeed = fRight;
    vMscSetSpeedRaw(fLeft, fRight, fLeft, fRight);
}

/**
 * @brief  统一位置控制（脉冲）
 * @note   Y轴所有底盘通用，X轴仅在 Mecanum4 底盘有效
 * @param  pstStatic  静态参数指针
 * @param  pstRunning 运行参数指针
 * @param  emAxis     运动轴：emMscAxis_Y=前后，emMscAxis_X=横向
 * @param  lPulse     目标脉冲数（正=前进/右移，负=后退/左移）
 */
static void vMscSetPositionByPulse(stMotorSystemStaticParamTdf *pstStatic,
                                    stMotorSystemRunningParamTdf *pstRunning,
                                    emMscAxisTdf emAxis, int32_t lPulse)
{
    uint16_t usVel      = (uint16_t)FIX32_TO_INT(fGetDefaultPosSpeed(pstStatic));
    uint8_t  ucAcc      = pstStatic->ucGlobalAcc;
    uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
    emMotorDirTdf emDir = (lPulse >= 0) ? emMotorDir_Forward : emMotorDir_Backward;

    pstRunning->emState = emMscState_Position;
    pstRunning->ucMotionDone = 0;

    /* TimeCalc 模式：梯形速度曲线估算运动时间 */
    if (pstStatic->emStopDetectMode == emMscStopDetect_TimeCalc) {
        pstRunning->ulPositionPulse = ulAbsPulse;
        pstRunning->ulEstimateStopTick = 0;

        fix32_t fPpr = FIX32_ZERO;
#if EMM_MOTOR_IS_ENABLE
        {
            emMotorDevNumTdf emMasterDev = pstStatic->astWheels[MOTOR_WHEEL_LF].emDevNum;
            if (emMasterDev != emNoMotorDevNum) {
                const stEmmMotorStaticParamTdf *pstEmm = c_pstGetEmmMotorStaticParam(emMasterDev);
                if (pstEmm != NULL) fPpr = pstEmm->fEncoderPulsePerRev;
            }
        }
#endif
        if (fPpr > FIX32_ZERO && usVel > 0 && ulAbsPulse > 0) {
            uint32_t ulPpr = (uint32_t)FIX32_TO_INT(fPpr);
            uint32_t ulTimeMs;

            if (ucAcc == 0) {
                /* 加速度为 0（直接启动），用简单线性估算 */
                ulTimeMs = (uint32_t)(((uint64_t)ulAbsPulse * 60000)
                            / ((uint64_t)ulPpr * usVel));
            } else {
                /*
                 * 梯形/三角形速度曲线估算（基于 EMM 电机实测公式）：
                 *   ulAccDecPulse = PPR * V² / (6 * acc)   — 加减速段总脉冲
                 *   ulAccTimeMs   = 2000 * V / acc          — 加速段时间(ms)
                 *
                 * 若总脉冲 < 加减速段脉冲 → 三角形（未达目标速度即减速）
                 *    time = accTime * sqrt(pulse / accDecPulse)
                 * 否则 → 梯形（加速 + 匀速 + 减速）
                 *    time = (pulse - accDecPulse) * 60000 / (PPR * V) + accTime
                 */
                uint32_t ulAccDecPulse = (uint32_t)(((uint64_t)ulPpr * usVel * usVel)
                                          / ((uint64_t)ucAcc * 6));
                uint32_t ulAccTimeMs   = (2000UL * usVel) / ucAcc;

                if (ulAccDecPulse >= ulAbsPulse) {
                    /* 三角形：time = accTime * sqrt(pulse / accDecPulse) */
                    float fRatio = (float)ulAbsPulse / (float)ulAccDecPulse;
                    ulTimeMs = (uint32_t)((float)ulAccTimeMs * sqrtf(fRatio));
                } else {
                    /* 梯形：匀速段时间 + 加减速段时间 */
                    uint32_t ulCruiseMs = (uint32_t)(((uint64_t)(ulAbsPulse - ulAccDecPulse) * 60000)
                                          / ((uint64_t)ulPpr * usVel));
                    ulTimeMs = ulCruiseMs + ulAccTimeMs;
                }
            }
            pstRunning->ulEstimateStopTick = QE_GET_TICK() + ulTimeMs + 500;
        }
    }

    if (emAxis == emMscAxis_Y || pstStatic->emChassisType != emChassisMecanum4) {
        /* Y轴 / 差速底盘：所有轮同向同脉冲 */
        uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
        for (uint8_t i = 0; i < ucCount; i++) {
            stMotorWheelConfigTdf *pW = &pstStatic->astWheels[i];
            if (pW->emDevNum != emNoMotorDevNum) {
                vMotorPosControl(pW->emDevNum, emDir, usVel, ucAcc, ulAbsPulse, 0, 1);
            }
        }
    } else {
        /* X轴（麦轮横向）：LF+RB 同向，RF+LB 反向 */
        emMotorDirTdf emDirOpp = (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;

        stMotorWheelConfigTdf *pLF = &pstStatic->astWheels[MOTOR_WHEEL_LF];
        stMotorWheelConfigTdf *pRF = &pstStatic->astWheels[MOTOR_WHEEL_RF];
        stMotorWheelConfigTdf *pLB = &pstStatic->astWheels[MOTOR_WHEEL_LB];
        stMotorWheelConfigTdf *pRB = &pstStatic->astWheels[MOTOR_WHEEL_RB];

        vMotorPosControl(pLF->emDevNum, emDir,    usVel, ucAcc, ulAbsPulse, 0, 1);
        vMotorPosControl(pRF->emDevNum, emDirOpp, usVel, ucAcc, ulAbsPulse, 0, 1);
        vMotorPosControl(pLB->emDevNum, emDirOpp, usVel, ucAcc, ulAbsPulse, 0, 1);
        vMotorPosControl(pRB->emDevNum, emDir,    usVel, ucAcc, ulAbsPulse, 0, 1);
    }
    vMscForeachSync(pstStatic);
}

/* ==================== 传感器闭环 ==================== */

#if SENSOR_IS_ENABLE

/**
 * @brief 传感器闭环修正 — 从 sensor_device 读取 PID 输出，叠加到基准速度
 *          仅在 emMscState_TurningClosed 状态下生效
 */
static void vMscSensorClosedLoop(stMotorSystemStaticParamTdf *pstStatic,
                                  stMotorSystemRunningParamTdf *pstRunning)
{
    if (pstRunning->emState != emMscState_TurningClosed) return;
    if (pstStatic->emSensorDevNum == emNoSensor) return;
    if (pstSensorGetDevice(pstStatic->emSensorDevNum) == NULL) return;

    fix32_t fCorrection = fSensorGetPidOutput(pstStatic->emSensorDevNum);

    fix32_t fLeft  = pstRunning->fSavedBaseLeftSpeed  - fCorrection;
    fix32_t fRight = pstRunning->fSavedBaseRightSpeed + fCorrection;
    vMscSetSpeedRaw(fLeft, fRight, fLeft, fRight);  /* 不改变状态，保持 TurningClosed */

    /* 检查是否到达目标角度 */
    fix32_t fTarget = fSensorGetTarget(pstStatic->emSensorDevNum);
    fix32_t fValue  = fSensorGetAccumulatedValue(pstStatic->emSensorDevNum);
    fix32_t fError  = fix32_abs(fTarget - fValue);

    if (fError < FIX32_2) {
        vMotorSystemStop();
    }
}

#endif /* SENSOR_IS_ENABLE */

/* ==================== 初始化 ==================== */

/**
 * @brief  电机系统初始化
 * @param  pstInit 静态参数配置指针，内部 memcpy 不持有指针
 */
void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit)
{
    memset(&g_stMotorSystemController, 0, sizeof(g_stMotorSystemController));
    if (pstInit != NULL) {
        memcpy(&g_stMotorSystemController.stStaticParam, pstInit, sizeof(stMotorSystemStaticParamTdf));
    }
}

/**
 * @brief  获取电机系统参数指针
 * @return 电机系统参数指针
 */
const stMotorSystemParamTdf *c_pstGetMotorSystemControllerParam(void)
{
    return &g_stMotorSystemController;
}

/**
 * @brief  获取电机系统当前状态
 * @return 空闲或运行状态枚举
 */
emMotorStateTdf emGetMotorSystemState(void)
{
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();
    return (pstRunning->emState == emMscState_Idle) ? emMotorStateIdle : emMotorStateRunning;
}

/**
 * @brief  查询上一次运动是否已完成
 * @return 1=已完成, 0=进行中
 */
uint8_t ucMotorSystemIsMotionDone(void)
{
    return pstGetRunning()->ucMotionDone;
}

/* ==================== 使能 / 停止 ==================== */

/**
 * @brief  使能/失能所有已配置电机
 * @param  bEnable 1=使能, 0=失能
 */
void vMotorSystemEnable(uint8_t bEnable)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    vMscForeachEnable(pstStatic, bEnable);
    vMscForeachSync(pstStatic);
}

/**
 * @brief  急停所有电机并回到空闲状态
 */
void vMotorSystemStop(void)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    vMscTransitionToIdle(pstRunning);
    vMscForeachStop(pstStatic);
    vMscForeachSync(pstStatic);
}

/* ==================== 速度控制 ==================== */

/**
 * @brief  原始四轮速度控制
 * @note   所有底盘通用，直接设置每轮速度
 * @param  fLeftFrontSpeed  LF 速度(RPM)，正=前进
 * @param  fRightFrontSpeed RF 速度(RPM)，正=前进
 * @param  fLeftBackSpeed   LB 速度(RPM)，正=前进
 * @param  fRightBackSpeed  RB 速度(RPM)，正=前进
 */
void vMotorSystemSetSpeed(fix32_t fLeftFrontSpeed, fix32_t fRightFrontSpeed,
                          fix32_t fLeftBackSpeed,  fix32_t fRightBackSpeed)
{
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();
    vMscSetSpeedRaw(fLeftFrontSpeed, fRightFrontSpeed, fLeftBackSpeed, fRightBackSpeed);
    pstRunning->emState = emMscState_Velocity;
    pstRunning->ucMotionDone = 0;
    pstRunning->ulEstimateStopTick = 0;
}

/* ==================== 位置控制 ==================== */

/**
 * @brief  设置 Y 轴位置（cm）
 * @note   所有底盘通用
 * @param  fTargetCm 目标距离(cm)，正=前进，负=后退
 */
void vMotorSystemSetPosition(fix32_t fTargetCm)
{
    
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    int32_t lPulse = lMscGetMasterPulse(pstStatic, fTargetCm);
    if (lPulse == 0 && fTargetCm != FIX32_ZERO) return;
    vMscSetPositionByPulse(pstStatic, pstRunning, emMscAxis_Y, lPulse);
}

/**
 * @brief  设置 Y 轴位置（cm）
 * @note   同 vMotorSystemSetPosition，别名
 * @param  fTargetYCm 目标距离(cm)，正=前进，负=后退
 */
void vMotorSystemSetPositionY(fix32_t fTargetYCm)
{
    vMotorSystemSetPosition(fTargetYCm);
}

/**
 * @brief  设置 Y 轴位置（脉冲）
 * @note   所有底盘通用，跳过 cm 转换
 * @param  lPulse 目标脉冲数，正=前进，负=后退
 */
void vMotorSystemSetPositionYPulse(int32_t lPulse)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    vMscSetPositionByPulse(pstStatic, pstRunning, emMscAxis_Y, lPulse);
}

/**
 * @brief  设置 X 轴位置（cm）
 * @note   仅在 Mecanum4 底盘有效
 * @param  fTargetXCm 目标距离(cm)，正=右移，负=左移
 */
void vMotorSystemSetPositionX(fix32_t fTargetXCm)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    if (pstStatic->emChassisType != emChassisMecanum4) return;

    int32_t lPulse = lMscGetMasterPulse(pstStatic, fTargetXCm);
    if (lPulse == 0 && fTargetXCm != FIX32_ZERO) return;
    vMscSetPositionByPulse(pstStatic, pstRunning, emMscAxis_X, lPulse);
}

/**
 * @brief  设置 X 轴位置（脉冲）
 * @note   仅在 Mecanum4 底盘有效，跳过 cm 转换
 * @param  lPulse 目标脉冲数，正=右移，负=左移
 */
void vMotorSystemSetPositionXPulse(int32_t lPulse)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    if (pstStatic->emChassisType != emChassisMecanum4) return;
    vMscSetPositionByPulse(pstStatic, pstRunning, emMscAxis_X, lPulse);
}

/* ==================== 麦轮运动学 ==================== */

/**
 * @brief  麦轮逆运动学 — 将底盘中心(Vx, Vy)分解为四轮分量
 * @param  pfWheel 输出：四轮分量数组（LF/RF/LB/RB），正值=Forward
 * @param  vx      X方向分量（正=右移）
 * @param  vy      Y方向分量（正=前进）
 */
static void vMscMecanumIK(fix32_t *pfWheel, fix32_t vx, fix32_t vy)
{
    pfWheel[MOTOR_WHEEL_LF] = vy + vx;
    pfWheel[MOTOR_WHEEL_RF] = vy - vx;
    pfWheel[MOTOR_WHEEL_LB] = vy - vx;
    pfWheel[MOTOR_WHEEL_RB] = vy + vx;
}

/**
 * @brief  麦轮位置控制 — 同时设置 X/Y 方向脉冲
 * @note   仅在 Mecanum4 底盘有效，运动学叠加后每轮独立控制
 * @param  lPulseX X方向脉冲数（正=右移）
 * @param  lPulseY Y方向脉冲数（正=前进）
 */
void vMotorSystemSetPositionMecanum(int32_t lPulseX, int32_t lPulseY)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    if (pstStatic->emChassisType != emChassisMecanum4) return;
    if (lPulseX == 0 && lPulseY == 0) return;

    uint16_t usVel = (uint16_t)FIX32_TO_INT(fGetDefaultPosSpeed(pstStatic));
    uint8_t  ucAcc = pstStatic->ucGlobalAcc;

    fix32_t afWheel[4];
    vMscMecanumIK(afWheel, FIX32_FROM_INT(lPulseX), FIX32_FROM_INT(lPulseY));

    pstRunning->emState = emMscState_Position;
    pstRunning->ucMotionDone = 0;

    uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
    for (uint8_t i = 0; i < ucCount; i++) {
        stMotorWheelConfigTdf *pW = &pstStatic->astWheels[i];
        if (pW->emDevNum == emNoMotorDevNum) continue;

        int32_t lPulse = FIX32_TO_INT(afWheel[i]);
        if (lPulse == 0) continue;

        emMotorDirTdf emDir = (lPulse >= 0) ? emMotorDir_Forward : emMotorDir_Backward;
        uint32_t ulAbs = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);

        vMotorPosControl(pW->emDevNum, emDir, usVel, ucAcc, ulAbs, 0, 1);
    }
    vMscForeachSync(pstStatic);
}

/**
 * @brief  麦轮速度控制 — 以底盘中心为参考
 * @note   仅在 Mecanum4 底盘有效，内部完成运动学分解
 * @param  fSpeedX X方向速度(RPM)，正=右移
 * @param  fSpeedY Y方向速度(RPM)，正=前进
 */
void vMotorSystemSetSpeedMecanum(fix32_t fSpeedX, fix32_t fSpeedY)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();

    if (pstStatic->emChassisType != emChassisMecanum4) return;

    fix32_t afWheel[4];
    vMscMecanumIK(afWheel, fSpeedX, fSpeedY);

    vMscSetSpeedRaw(afWheel[MOTOR_WHEEL_LF], afWheel[MOTOR_WHEEL_RF],
                    afWheel[MOTOR_WHEEL_LB], afWheel[MOTOR_WHEEL_RB]);

    pstGetRunning()->emState = emMscState_Velocity;
    pstGetRunning()->ucMotionDone = 0;
    pstGetRunning()->ulEstimateStopTick = 0;
}

/**
 * @brief  原地旋转固定脉冲数
 * @note   所有底盘通用。麦轮四轮同向，差速左轮反向右轮正向
 * @param  lPulse 脉冲数（正=CCW/左转，负=CW/右转）
 */
void vMotorSystemRotatePulse(int32_t lPulse)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    if (lPulse == 0) return;

    uint16_t usVel      = (uint16_t)FIX32_TO_INT(fGetDefaultPosSpeed(pstStatic));
    uint8_t  ucAcc      = pstStatic->ucGlobalAcc;
    uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);

    pstRunning->emState = emMscState_Position;
    pstRunning->ucMotionDone = 0;
    pstRunning->ulEstimateStopTick = 0;  /* 旋转不走 TimeCalc */

    if (pstStatic->emChassisType == emChassisMecanum4) {
        /* 麦轮：四轮同向同脉冲（正值=CCW/左转） */
        emMotorDirTdf emDir = (lPulse >= 0) ? emMotorDir_Forward : emMotorDir_Backward;
        uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
        for (uint8_t i = 0; i < ucCount; i++) {
            if (pstStatic->astWheels[i].emDevNum != emNoMotorDevNum) {
                vMotorPosControl(pstStatic->astWheels[i].emDevNum, emDir,
                                 usVel, ucAcc, ulAbsPulse, 0, 1);
            }
        }
    } else {
        /* 差速：左轮反向、右轮正向 */
        emMotorDirTdf emDirPos = (lPulse >= 0) ? emMotorDir_Forward  : emMotorDir_Backward;
        emMotorDirTdf emDirNeg = (lPulse >= 0) ? emMotorDir_Backward : emMotorDir_Forward;
        uint8_t ucCount = ucMscWheelCount(pstStatic->emChassisType);
        for (uint8_t i = 0; i < ucCount; i++) {
            if (pstStatic->astWheels[i].emDevNum == emNoMotorDevNum) continue;
            emMotorDirTdf ed = (i == MOTOR_WHEEL_LF || i == MOTOR_WHEEL_LB) ? emDirNeg : emDirPos;
            vMotorPosControl(pstStatic->astWheels[i].emDevNum, ed,
                             usVel, ucAcc, ulAbsPulse, 0, 1);
        }
    }
    vMscForeachSync(pstStatic);
}

/* ==================== 姿态控制 ==================== */

/**
 * @brief  自旋姿态控制（通过底盘差速实现）
 * @param  fTargetYawDeg 目标偏航角(deg)
 * @param  fOmegaRadS    目标角速度(rad/s)
 */
static void vMotorSystemSetPose(fix32_t fTargetYawDeg, fix32_t fOmegaRadS)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    pstRunning->fTargetYaw = fTargetYawDeg;

    /* fTurnSpeed = fOmegaRadS * fWheelBaseCm * 60 / (2 * fWheelCircumferenceCm) */
    fix32_t fCirc = fMscGetMasterCircumference(pstStatic);
    if (fCirc < FIX32_ONE) return;

    fix32_t fDenom = fix32_mul(fCirc, FIX32_2);
    fix32_t fFactor = fix32_div(fix32_mul(pstStatic->fWheelBaseCm, FIX32_60), fDenom);
    fix32_t fTurnSpeed = fix32_mul(fOmegaRadS, fFactor);

    if (fTargetYawDeg > FIX32_ZERO) {
        vSetDiffSpeedAndSave(-fTurnSpeed, fTurnSpeed);
    } else {
        vSetDiffSpeedAndSave(fTurnSpeed, -fTurnSpeed);
    }
}

/* ==================== 转弯 ==================== */

/**
 * @brief  开环转弯
 * @param  emTurnType 转弯类型（左/右 90°/180°）
 * @param  emScheme   转弯方案（差速自旋/单侧停止/漂移）
 * @param  fK         转弯速度系数，0 时使用静态参数中的默认值
 */
void vMotorSystemTurnOpen(emTurnTypeTdf emTurnType, emTurnSchemeTdf emScheme, fix32_t fK)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    if (fK == FIX32_ZERO) fK = pstStatic->stOpenLoopCfg.fOpenLoopTurnK;

    fix32_t fAngleOffset = fGetTurnAngleOffset(emTurnType);
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;

    fix32_t fTurnSpeed = fix32_mul(fK, FIX32_10);

    switch (emScheme) {
        case emTurnScheme_DiffSpin:
        default:
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
#if EMM_MOTOR_IS_ENABLE
            {
                emMotorDevNumTdf emMasterDev = pstStatic->astWheels[MOTOR_WHEEL_LF].emDevNum;
                if (emMasterDev == emNoMotorDevNum) return;
                const stEmmMotorStaticParamTdf *pstEmm = c_pstGetEmmMotorStaticParam(emMasterDev);
                if (pstEmm == NULL) return;
                if (pstEmm->fWheelDiameterCm < FIX32_ONE || pstEmm->fEncoderPulsePerRev < FIX32_ONE) return;

                /* 弧长 → 脉冲，外侧多走、内侧少走 */
                fix32_t fFactor = fix32_div(fix32_mul(pstStatic->fWheelBaseCm, FIX32_PI_V), FIX32_360);
                fix32_t fArcLength = fix32_mul(fFactor, fAngleOffset);
                int32_t lPulse = lCmToPulse(fArcLength, pstEmm->fWheelDiameterCm, pstEmm->fEncoderPulsePerRev);

                uint32_t ulAbsPulse = (uint32_t)((lPulse >= 0) ? lPulse : -lPulse);
                uint16_t usVel = (uint16_t)FIX32_TO_INT(fGetDefaultPosSpeed(pstStatic));

                vMotorPosControl(pstStatic->astWheels[MOTOR_WHEEL_LF].emDevNum,
                    emMotorDir_Forward, usVel, pstStatic->ucGlobalAcc,
                    (fAngleOffset > FIX32_ZERO) ? 0 : ulAbsPulse * 2, 0, 1);
                vMotorPosControl(pstStatic->astWheels[MOTOR_WHEEL_RF].emDevNum,
                    emMotorDir_Forward, usVel, pstStatic->ucGlobalAcc,
                    (fAngleOffset > FIX32_ZERO) ? ulAbsPulse * 2 : 0, 0, 1);
                vMscForeachSync(pstStatic);

                pstRunning->emState = emMscState_Position;
                pstRunning->ucMotionDone = 0;
                pstRunning->ulEstimateStopTick = 0;  /* Drift 不走 TimeCalc，回退到状态轮询 */
                return;
            }
#else
            return;
#endif
        }
    }

    pstRunning->emState = emMscState_Velocity;
    pstRunning->ucMotionDone = 0;
    pstRunning->ulEstimateStopTick = 0;  /* 速度模式无预估时间 */
}

/**
 * @brief  闭环转弯（需传感器支持）
 * @note   需 SENSOR_IS_ENABLE 且传感器已绑定，否则无操作
 * @param  emTurnType 转弯类型（左/右 90°/180°）
 */
void vMotorSystemTurnClosed(emTurnTypeTdf emTurnType)
{
#if SENSOR_IS_ENABLE
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    if (pstStatic->emSensorDevNum == emNoSensor) return;

    fix32_t fAngleOffset = fGetTurnAngleOffset(emTurnType);
    pstRunning->fAccumulatedYaw += fAngleOffset;
    pstRunning->fTargetYaw = pstRunning->fAccumulatedYaw;

    pstRunning->emState = emMscState_TurningClosed;
    pstRunning->ucMotionDone = 0;
    vSensorSetTarget(pstStatic->emSensorDevNum, pstRunning->fTargetYaw);
    vMotorSystemSetPose(pstRunning->fTargetYaw, FIX32_ONE);
#else
    (void)emTurnType;
#endif
}

/* ==================== 传感器绑定 ==================== */

#if SENSOR_IS_ENABLE

/**
 * @brief  绑定传感器设备号
 * @param  emSensorDevNum 传感器设备号，emNoSensor 为解绑
 */
void vMotorSystemSetSensor(emSensorDevNumTdf emSensorDevNum)
{
    pstGetStatic()->emSensorDevNum = emSensorDevNum;
}

#endif /* SENSOR_IS_ENABLE */

/* ==================== 周期执行 ==================== */

/**
 * @brief  电机系统周期执行
 * @note   由主循环定时调用，处理运动完成检测和传感器闭环修正
 */
void vMotorSystemPeriodExecute(void)
{
    stMotorSystemStaticParamTdf *pstStatic = pstGetStatic();
    stMotorSystemRunningParamTdf *pstRunning = pstGetRunning();

    /* Phase 1: 各电机周期执行由 it_controller ISR 统一处理，此处不重复调用 */

    /* Phase 2: 状态机完成检测 */
    switch (pstRunning->emState) {
        case emMscState_Idle:
            break;

        case emMscState_Velocity:
        case emMscState_Position:
            switch (pstStatic->emStopDetectMode) {
                case emMscStopDetect_StatePoll:
                default:
                    if (ucMscAllMotorsStopped(pstStatic)) {
                        vMscTransitionToIdle(pstRunning);
                    }
                    break;

                case emMscStopDetect_TimeCalc:
                    if (ucMscAllMotorsStopped(pstStatic)) {
                        vMscTransitionToIdle(pstRunning);
                        pstRunning->ulEstimateStopTick = 0;
                    } else if (pstRunning->ulEstimateStopTick > 0
                               && QE_GET_TICK() >= pstRunning->ulEstimateStopTick) {
                        vMscForeachStop(pstStatic);
                        vMscForeachSync(pstStatic);
                        vMscTransitionToIdle(pstRunning);
                        pstRunning->ulEstimateStopTick = 0;
                    }
                    break;

                case emMscStopDetect_WheelSpeed:
                    /* TODO: 需电机速度反馈接口，暂回退到状态轮询 */
                    if (ucMscAllMotorsStopped(pstStatic)) {
                        vMscTransitionToIdle(pstRunning);
                    }
                    break;
            }
            break;

        case emMscState_TurningClosed:
            /* 由 vMscSensorClosedLoop 处理完成检测 */
            break;
    }

    /* Phase 3: 传感器闭环修正（传感器周期执行由 it_controller ISR 统一处理） */
#if SENSOR_IS_ENABLE
    if (pstStatic->emSensorDevNum != emNoSensor
        && pstSensorGetDevice(pstStatic->emSensorDevNum) != NULL) {
        vMscSensorClosedLoop(pstStatic, pstRunning);
    }
#endif
}

#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */
