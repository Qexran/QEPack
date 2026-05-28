/**
  * @file       motor_system_controller.h
  * @author     QePack
  * @version    V3.0.0
  * @date       2026/5/24
  * @brief      电机系统控制器（重构版）
  *             支持差速/麦轮底盘，传感器闭环控制，转弯功能
  *
  *             传感器 PID 由 sensor_device 内部管理，motor_system
  *             通过 fSensorGetPidOutput() 获取修正值。
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
    emTurnLeft90 = 0,
    emTurnRight90,
    emTurnLeft180,
    emTurnRight180,
} emTurnTypeTdf;

/** @brief 转弯方案枚举 */
typedef enum {
    emTurnScheme_DiffSpin = 0,
    emTurnScheme_OneSideStop,
    emTurnScheme_Drift,
} emTurnSchemeTdf;

/** @brief 电机系统设备号 */
typedef enum {
    emMotorSystemDevNum0 = 0,
} emMotorSystemDevNumTdf;

/** @brief 底盘类型 */
typedef enum {
    emChassisDiff2,
    emChassisDiff4,
    emChassisMecanum4,
} emChassisTypeTdf;

/** @brief 电机系统状态机 */
typedef enum {
    emMscState_Idle = 0,
    emMscState_Velocity,
    emMscState_Position,
    emMscState_TurningClosed,
} emMscStateTdf;

/** @brief 停止检测策略 */
typedef enum {
    emMscStopDetect_StatePoll = 0,              /* 轮询电机状态寄存器（默认） */
    emMscStopDetect_TimeCalc,                   /* 根据脉冲数+速度估算时间 */
    emMscStopDetect_WheelSpeed,                 /* 读取轮速判断停止 */
} emMscStopDetectTdf;

/* 四轮命名索引 */
#define MOTOR_WHEEL_LF  0
#define MOTOR_WHEEL_RF  1
#define MOTOR_WHEEL_LB  2
#define MOTOR_WHEEL_RB  3

/** @brief 单轮配置 */
typedef struct {
    emMotorDevNumTdf emDevNum;
} stMotorWheelConfigTdf;

/** @brief 开环转弯配置 */
typedef struct {
    fix32_t fOpenLoopTurnK;                    /* 开环转弯 K 系数 */
} stMscOpenLoopConfigTdf;

/** @brief 电机系统运行参数 */
typedef struct {
    emMscStateTdf emState;

    fix32_t fAccumulatedYaw;
    fix32_t fTargetYaw;

    /* 传感器修正基准速度（差速修正时以此为基准叠加 PID 输出） */
    fix32_t fSavedBaseLeftSpeed;
    fix32_t fSavedBaseRightSpeed;

    uint8_t ucMotionDone;                       /* 运动完成标志，进入 Idle 时置 1，发起新运动时清 0 */
                
    uint32_t ulEstimateStopTick;                /* TimeCalc 模式预计停止 tick */
    uint32_t ulPositionPulse;                   /* 本次位置运动总脉冲数（TimeCalc 用） */
} stMotorSystemRunningParamTdf;

/** @brief 电机系统静态参数 */
typedef struct {
    emChassisTypeTdf emChassisType;
    stMotorWheelConfigTdf astWheels[4];

#if SENSOR_IS_ENABLE
    emSensorDevNumTdf emSensorDevNum;           /* emNoSensor = 无传感器 */
#endif

    fix32_t fWheelBaseCm;

    stMscOpenLoopConfigTdf stOpenLoopCfg;       /* 开环转弯参数 */
    fix32_t fDefaultPosSpeedRPM;
    uint8_t ucGlobalAcc;                        /* 全局加速度（速度/位置模式共用） */
    emMscStopDetectTdf emStopDetectMode;        /* 停止检测策略 */
} stMotorSystemStaticParamTdf;

typedef struct {
    stMotorSystemStaticParamTdf  stStaticParam;
    stMotorSystemRunningParamTdf stRunningParam;
} stMotorSystemParamTdf;

/* 初始化 */
void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit);

/* 获取参数 */
const stMotorSystemParamTdf *c_pstGetMotorSystemControllerParam(void);

/* 获取状态 */
emMotorStateTdf emGetMotorSystemState(void);

/* 查询运动是否完成（位置/速度模式电机停止后置位，发起新运动时清零） */
uint8_t ucMotorSystemIsMotionDone(void);

/* 直接设速 */
void vMotorSystemSetSpeed(fix32_t fLeftFrontSpeed, fix32_t fRightFrontSpeed,
                          fix32_t fLeftBackSpeed,  fix32_t fRightBackSpeed);

/* 位置控制 — Y轴前后（cm），所有底盘通用 */
void vMotorSystemSetPosition(fix32_t fTargetCm);

/* 位置控制 — Y轴前后（脉冲），所有底盘通用，跳过 cm 转换 */
void vMotorSystemSetPositionYPulse(int32_t lPulse);

/* 位置控制 — X轴横向（cm），仅麦轮 */
void vMotorSystemSetPositionX(fix32_t fTargetXCm);

/* 位置控制 — X轴横向（脉冲），仅麦轮 */
void vMotorSystemSetPositionXPulse(int32_t lPulse);

/* 麦轮位置控制 — 同时设置X和Y脉冲，仅麦轮底盘 */
void vMotorSystemSetPositionMecanum(int32_t lPulseX, int32_t lPulseY);

/* 麦轮速度控制 — 以底盘中心为参考(Vx=横向向右, Vy=纵向前进)，仅麦轮底盘 */
void vMotorSystemSetSpeedMecanum(fix32_t fSpeedX, fix32_t fSpeedY);

/* 原地旋转固定脉冲数，所有底盘通用（正=CCW/左转，负=CW/右转） */
void vMotorSystemRotatePulse(int32_t lPulse);

/* 位置控制 — Y轴前后（cm），所有底盘通用，同 vMotorSystemSetPosition */
void vMotorSystemSetPositionY(fix32_t fTargetYCm);

/* 开环转弯，fK=0 时使用静态参数中的 stOpenLoopCfg.fOpenLoopTurnK */
void vMotorSystemTurnOpen(emTurnTypeTdf emTurnType, emTurnSchemeTdf emScheme, fix32_t fK);

/* 闭环转弯，需 SENSOR_IS_ENABLE 且传感器已绑定，否则无操作 */
void vMotorSystemTurnClosed(emTurnTypeTdf emTurnType);

/* 传感器绑定 */
#if SENSOR_IS_ENABLE
void vMotorSystemSetSensor(emSensorDevNumTdf emSensorDevNum);
#endif

/* 控制 */
void vMotorSystemStop(void);
void vMotorSystemPeriodExecute(void);
void vMotorSystemEnable(uint8_t bEnable);

#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */

#endif /* __MOTOR_SYSTEM_CONTROLLER_H */
