/**
  * @file       motor_system_controller.h
  * @author     QePack
  * @version    V2.0.0
  * @date       2026/5/10
  * @brief      电机系统控制器
  *             支持差速/麦轮底盘，传感器闭环控制，转弯/矫正功能
  */

#ifndef __MOTOR_SYSTEM_CONTROLLER_H
#define __MOTOR_SYSTEM_CONTROLLER_H

#include "project_config.h"

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

#include "string.h"
#include "arithmetic.h"
#include "motor_device.h"
#include "pid_controller.h"

#if SENSOR_IS_ENABLE
#include "sensor_device.h"
#endif

/** @brief 转弯类型枚举 */
typedef enum {
    emTurnLeft90 = 0,       /* 左转 90° */
    emTurnRight90,          /* 右转 90° */
    emTurnLeft180,          /* 左转 180° */
    emTurnRight180,         /* 右转 180° */
} emTurnTypeTdf;

/** @brief 转弯方案枚举 */
typedef enum {
    emTurnScheme_DiffSpin = 0,    /* 差速原地旋转（左右轮反向） */
    emTurnScheme_OneSideStop,     /* 单侧停止旋转（一侧速度为0） */
    emTurnScheme_Drift,           /* 漂移旋转（左右路程差） */
} emTurnSchemeTdf;

/** @brief 传感器影响模式枚举 */
typedef enum {
    emSensorInfluence_None = 0,        /* 传感器不影响 */
    emSensorInfluence_DiffCorrect,     /* 直走偏移 → 左右轮差速修正 */
    emSensorInfluence_AngleControl,    /* 转弯 → 传感器 PID 控制角度 */
} emSensorInfluenceModeTdf;

/** @brief 电机系统设备号 */
typedef enum {
    emMotorSystemDevNum0 = 0,
} emMotorSystemDevNumTdf;

/** @brief 底盘类型 */
typedef enum {
    emChassisDiff2,         /* 2轮差速 */
    emChassisDiff4,         /* 4轮差速 */
    emChassisMecanum4,      /* 4轮麦克纳姆 */
} emChassisTypeTdf;

/** @brief 电机系统运行参数 */
typedef struct {
    emMotorStateTdf emMotorSystemState;        /* 电机系统运动状态 */

    /* 转弯累积 */
    fix32_t fAccumulatedYaw;                   /* 累积 yaw 角（度） */
    fix32_t fTargetYaw;                        /* 转弯目标角度 */

    /* 位置控制 */
    fix32_t fTargetPositionCm;                 /* 目标位置(cm) */
    fix32_t fTargetPositionXCm;                /* 麦轮 X 目标(cm) */
    fix32_t fTargetPositionYCm;                /* 麦轮 Y 目标(cm) */

    /* 传感器相关 */
    uint8_t ucSensorInfluenceActive;           /* 传感器影响是否激活 */
    uint8_t ucTurningActive;                   /* 转弯进行中标志 */
    uint8_t ucRectifyingActive;                /* 矫正进行中标志 */
    uint8_t ucSensorSuppressed;                /* 传感器暂时抑制（开环转弯/矫正时） */

    /* 矫正超时 */
    uint32_t ulRectifyTimeoutMs;               /* 矫正超时时间 */
    fix32_t fRectifyElapsedMs;                 /* 矫正已用时间 */
} stMotorSystemRunningParamTdf;

/** @brief 电机系统静态参数 */
typedef struct {
    emChassisTypeTdf emChassisType;            /* 底盘类型 */

    /* 电机绑定 */
    emMotorDevNumTdf emLeftFrontMotorDevNum;   /* 左前电机 */
    emMotorDevNumTdf emRightFrontMotorDevNum;  /* 右前电机 */
    emMotorDevNumTdf emLeftBackMotorDevNum;    /* 左后电机 */
    emMotorDevNumTdf emRightBackMotorDevNum;   /* 右后电机 */

    /* 电机方向反转标志（1=反转，0=不转） */
    uint8_t ucLeftFrontMotorReversed;         /* 左前电机方向反转 */
    uint8_t ucRightFrontMotorReversed;        /* 右前电机方向反转 */
    uint8_t ucLeftBackMotorReversed;          /* 左后电机方向反转 */
    uint8_t ucRightBackMotorReversed;         /* 右后电机方向反转 */

    /* 传感器绑定 */
    #if SENSOR_IS_ENABLE
    emSensorDevNumTdf emSensorDevNum;          /* 传感器实例（emNoSensor = 无传感器） */
    emSensorDevNumTdf emSensorDevNum2;         /* 第二传感器（互补滤波用，emNoSensor = 无） */
    emPidDevNumTdf emSensorPidDevNum;          /* 传感器 PID 实例 */
    #endif

    /* 底盘几何参数 */
    fix32_t fWheelCircumferenceCm;             /* 轮子周长(cm) */
    fix32_t fWheelBaseCm;                      /* 轴距(cm)，差速转弯用 */
    fix32_t fEncoderPulsePerRev;               /* 编码器每圈脉冲数（含倍频） */

    /* 传感器融合权重 */
    fix32_t fSensorWeight;                     /* 主传感器权重(0~1)，第二传感器 = 1-weight */

    /* 开环转弯 K 值 */
    fix32_t fOpenLoopTurnK;                    /* 开环转弯系数 */

    /* 位置控制默认速度 */
    fix32_t fDefaultPosSpeedRPM;               /* 位置模式默认速度(RPM)，0=使用内置默认60RPM */

} stMotorSystemStaticParamTdf;


typedef struct
{
    stMotorSystemStaticParamTdf  *stStaticParam;       /* 静态参数 */
    stMotorSystemRunningParamTdf stRunningParam;       /* 运行参数 */
} stMotorSystemParamTdf;

/* 初始化 */
void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit);

/* 获取参数 */
const stMotorSystemParamTdf* c_pstGetMotorSystemControllerParam(void);

/* 速度控制 */
void vMotorSystemVelControl(
    emMotorDirTdf emDir, uint8_t ucAcc,
    uint16_t usLeftFrontSpeed, uint16_t usRightFrontSpeed,
    uint16_t usLeftBackSpeed,  uint16_t usRightBackSpeed);

/* 直接设速 */
void vMotorSystemSetSpeed(fix32_t fLeftFrontSpeed, fix32_t fRightFrontSpeed,
                          fix32_t fLeftBackSpeed,  fix32_t fRightBackSpeed);

/* 位置控制（直走，cm） */
void vMotorSystemSetPosition(fix32_t fTargetCm);

/* 麦轮 X 轴位置控制 */
void vMotorSystemSetPositionX(fix32_t fTargetXCm);

/* 麦轮 Y 轴位置控制 */
void vMotorSystemSetPositionY(fix32_t fTargetYCm);

/* 姿态控制（Z轴旋转） */
void vMotorSystemSetPose(fix32_t fTargetYawDeg, fix32_t fOmegaRadS);

/* 转弯（支持开环/闭环） */
void vMotorSystemTurn(emTurnTypeTdf emTurnType, uint8_t bClosedLoop);
void vMotorSystemTurnOpenLoop(emTurnTypeTdf emTurnType, fix32_t fK);
void vMotorSystemTurnWithScheme(emTurnTypeTdf emTurnType, emTurnSchemeTdf emScheme, uint8_t bClosedLoop, fix32_t fK);

/* 矫正 */
void vMotorSystemRectifyAuto(fix32_t fTargetTheta, uint32_t ulTimeoutMs);
void vMotorSystemRectifyManual(fix32_t fOffset, uint32_t ulTimeoutMs);

/* 传感器切换 */
#if SENSOR_IS_ENABLE
void vMotorSystemSetSensor(emSensorDevNumTdf emSensorDevNum);
void vMotorSystemSetSensorWeight(fix32_t fWeight);
void vMotorSystemSetSensorInfluenceMode(emSensorInfluenceModeTdf emMode);
#endif

/* 辅助 */
void vMotorSystemStop(void);
void vMotorSystemPeriodExecute(void);
void vMotorSystemEnable(uint8_t bEnable);
emMotorStateTdf emGetMotorSystemState(void);

#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */

#endif /* __MOTOR_SYSTEM_CONTROLLER_H */
