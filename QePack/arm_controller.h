/**
  * @file       arm_controller.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/06/02
  * @brief      多轴协调运动控制器
  *
  *             支持舵机轴（SmoothStep S 曲线）和电机轴（PosControl）的协调运动。
  *             所有轴同时启动，根据最慢轴计算总时间，全部完成后置 ucMotionDone。
  */

#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_

#include "project_config.h"

#if ARM_CONTROLLER_IS_ENABLE

#include "servo_device.h"
#include "motor_device.h"

/* ==================== 枚举定义 ==================== */

/* 轴类型枚举 */
typedef enum {
    emArmAxisType_Servo = 0,    /* 舵机轴 */
    emArmAxisType_Motor,        /* 电机轴 */
} emArmAxisTypeTdf;

/* ==================== 数据结构定义 ==================== */

/* 轴配置结构体（静态，初始化后不变） */
typedef struct {
    emArmAxisTypeTdf    emType;             /* 轴类型 */
    uint8_t             ucDevIndex;         /* 设备号索引（舵机 emServoDevNumTdf 或电机 emMotorDevNumTdf） */
    uint16_t            usMotorVel;         /* 电机速度（仅电机轴有效） */
    uint8_t             ucMotorAcc;         /* 电机加速度（仅电机轴有效） */
    float               fMotorStepsPerUnit; /* 电机每单位脉冲数（用于时间估算，0=不估算） */
} stArmAxisConfigTdf;

/* 轴命令结构体 */
typedef struct {
    uint8_t             ucAxisIdx;          /* 轴索引（对应配置数组下标） */
    float               fTarget;            /* 目标值（舵机=角度，电机=脉冲数） */
    uint32_t            ulDurationMs;       /* 运动时长(ms)，舵机轴使用，电机轴设为 0 表示自动估算 */
} stArmAxisCommandTdf;

/* 控制器静态参数 */
typedef struct {
    uint8_t                     ucAxisCount;        /* 轴数量 */
    const stArmAxisConfigTdf    *pstAxisConfigs;    /* 轴配置数组指针 */
} stArmControllerStaticParamTdf;

/* 控制器运行参数 */
typedef struct {
    uint8_t             ucMotionDone;       /* 运动完成标志（1=完成或空闲） */
    uint32_t            ulTotalTimeMs;      /* 预估总运动时长(ms) */
    uint32_t            ulElapsedMs;        /* 已用时间(ms) */
} stArmControllerRunningParamTdf;

/* 控制器设备参数 */
typedef struct {
    stArmControllerStaticParamTdf   stStaticParam;
    stArmControllerRunningParamTdf  stRunningParam;
} stArmControllerDeviceParamTdf;

/* ==================== 公共 API ==================== */

/* 初始化协调运动控制器 */
void vArmControllerInit(const stArmControllerStaticParamTdf *pstInit);

/* 发起协调运动（命令数组 + 数量） */
void vArmControllerMove(const stArmAxisCommandTdf *pstCmds, uint8_t ucCmdCount);

/* 查询运动是否完成 */
uint8_t ucArmControllerIsMotionDone(void);

/* 周期执行（主循环调用，检测完成状态） */
void vArmControllerPeriodExecute(void);

/* 紧急停止所有轴 */
void vArmControllerStop(void);

#endif /* ARM_CONTROLLER_IS_ENABLE */
#endif /* _ARM_CONTROLLER_H_ */
