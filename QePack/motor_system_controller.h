/** 
  * @file       motor_system_controller.h 
  * @author     QePack
  * @version    V1.0.0
  * @date       2026/4/30
  * @brief      电机系统控制器
  * 
  */

#ifndef __MOTOR_SYSTEM_CONTROLLER_H
#define __MOTOR_SYSTEM_CONTROLLER_H

#include "project_config.h"

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

#include "string.h"
#include "motor_device.h"
#include "pid_controller.h"

typedef enum {
    emRectNone = 0,    /* 无 */
    emRectGray,        /* 灰度 */
    emRect
} emRectificationTypeTdf;

typedef enum {
    emChassisDiff2,         /* 2轮差 */
    emChassisDiff4,         /* 4轮差 */
    emChassisMecanum4,      /* 4轮麦 */
} emChassisTypeTdf;

typedef struct stMotorSystemRunningParamTdf {
    emMotorStateTdf emMotorSystemState;               /* 电机系统运动状态 */
} stMotorSystemRunningParamTdf;

typedef struct stMotorSystemStaticParamTdf {
    emChassisTypeTdf emChassisType;               /* 底盘类型 */
    emPidDevNumTdf emPidDevNum;                   /* PID实例 */

    emMotorDevNumTdf emLeftFrontMotorDevNum;      /* 左前电机实例 */
    emMotorDevNumTdf emRightFrontMotorDevNum;     /* 右前电机实例 */
    emMotorDevNumTdf emLeftBackMotorDevNum;       /* 左后电机实例 */
    emMotorDevNumTdf emRightBackMotorDevNum;      /* 右后电机实例 */

} stMotorSystemStaticParamTdf;


typedef struct
{
    stMotorSystemStaticParamTdf    *stStaticParam;       /* 静态参数 */
    stMotorSystemRunningParamTdf   stRunningParam;      /* 运行参数 */
} stMotorSystemParamTdf;

void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit);
const stMotorSystemParamTdf* c_pstGetMotorSystemControllerParam(void);
void vMotorSystemSetSpeed(float fLeftFrontSpeed, float fRightFrontSpeed, float fLeftBackSpeed,  float fRightBackSpeed);
void vMotorSystemSetPosition(float fTargetCm);
void vMotorSystemSetPose(float fTargetYawDeg, float fOmegaRadS);
void vMotorSystemStop(void);
void vMotorSystemPeriodExecute(void);
void vMotorSystemSetPositionX(float fTargetXCm);
void vMotorSystemSetPositionY(float fTargetYCm);

#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */


#endif /* __MOTOR_SYSTEM_CONTROLLER_H */
