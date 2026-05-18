/**
  * @file       gear_motor_device.h
  * @author     Qe_xr
  * @version    V2.0.0
  * @date       2026/5/11
  * @brief      直流减速电机控制驱动，基于 QePack 编码风格
  *             支持 PID 绑定、梯形加速度规划、速度模式控制
  *             内部使用 Q16.16 定点数运算
  */

#ifndef _GEAR_MOTOR_DEVICE_H_
#define _GEAR_MOTOR_DEVICE_H_

#include "project_config.h"

#if GEAR_MOTOR_IS_ENABLE

#include "encoder_device.h"
#include "motor_device.h"
#include "pid_controller.h"


#if (QEPACK_PLATFORM == TI)
    #include "ti_platform.h"
#endif

/** @brief 加速度规划阶段枚举 */
typedef enum {
    emGearMotorAccPhase_Idle = 0,     /* 空闲 */
    emGearMotorAccPhase_Accel,        /* 加速阶段 */
    emGearMotorAccPhase_Cruise,       /* 匀速阶段 */
    emGearMotorAccPhase_Decel,        /* 减速阶段 */
} emGearMotorAccPhaseTdf;

/** @brief 位置模式梯形规划状态 */
typedef enum {
    emGearPosProfile_Accel = 0,  /* 加速阶段 */
    emGearPosProfile_Cruise,     /* 匀速阶段 */
    emGearPosProfile_Decel,      /* 减速阶段 */
} emGearPosProfileStateTdf;

/** @brief 减速电机控制模式枚举 */
typedef enum {
    emGearMotorCtrlMode_Vel = 0,      /* 速度模式 */
    emGearMotorCtrlMode_Pos,          /* 位置模式（串级PID） */
} emGearMotorCtrlModeTdf;


/**
 * @brief          减速电机静态参数结构体
 */
typedef struct
{
    #if (QEPACK_PLATFORM == TI)
        stTimerTdf          *stTimer;         // TIM句柄
        DL_TIMER_CC_INDEX   emChannel;          // PWM通道
        GPIO_Regs           *pstDir1GpioBase;        // 检测编码器方向的 GPIOx
        uint32_t            u32DirPin1;              // 检测编码器方向的 GPIO_PIN_1
        GPIO_Regs           *pstDir2GpioBase;        // 检测编码器方向的 GPIOx
        uint32_t            u32DirPin2;              // 检测编码器方向的 GPIO_PIN_2
        GPIO_Regs           *pstStbyGpioBase;        // 电机待机引脚对应的GPIOX
        uint32_t            u32StbyPin;               // 电机待机引脚的GPIO_PIN号
    #else
        TIM_HandleTypeDef   *pstPWM_htim;         // 电机PWM使用的定时器
        uint32_t            u32PWM_Channel;                // PWM输出通道
        GPIO_TypeDef        *pstDir1GpioBase;          // 电机控制引脚1对应的GPIOX
        uint32_t            u32DirPin1;                    // 电机方向控制引脚1
        GPIO_TypeDef        *pstDir2GpioBase;          // 电机控制引脚2对应的GPIOX
        uint32_t            u32DirPin2;                    // 电机方向控制引脚2
        GPIO_TypeDef        *pstStbyGpioBase;        // 电机待机引脚对应的GPIOX
        uint32_t            u32StbyPin;               // 电机待机引脚的GPIO_PIN号
    #endif


    fix32_t Tire_R;                             // 轮胎半径
    emEncoderDevNumTdf emEncoderDevNum;         // 编码器设备号

    /* PID 绑定参数 */
    emPidDevNumTdf emPidDevNum;                 // 速度环 PID 设备号（emNoPid = 不启用）
    emPidDevNumTdf emPosPidDevNum;              // 位置环 PID 设备号（emNoPid = 不启用）
    uint16_t usPidPeriodMs;                     // PID 更新周期(ms)

    /* 占空比死区 */
    uint16_t u16MinDuty;                        // 最小占空比（克服摩擦的死区）

    /* 速度限幅 */
    fix32_t fMaxVelRPM;                         // 最大速度限幅(RPM)，0 表示不限幅
}
stGearMotorStaticParamTdf;

/**
 * @brief          减速电机运行参数结构体
 */
typedef struct
{
    /* 控制模式 */
    emGearMotorCtrlModeTdf emCtrlMode;          // 当前控制模式

    /* 目标值 */
    int16_t  sTargetSpeedRPM;                   // 目标速度(RPM)
    emMotorDirTdf emTargetDir;                  // 目标方向
    emMotorDirTdf emCurrentDir;                 // 当前实际输出方向（斜坡减速到0后才翻转）

    /* PID 相关 */
    uint32_t ulPidLastTickMs;                   // 上次 PID 执行的时间戳(ms)

    /* 加速度规划 */
    emGearMotorAccPhaseTdf emAccPhase;          // 当前阶段
    uint8_t  ucAccProfileActive;                // 加速度规划是否激活
    fix32_t  fAccelRPMpS;                       // 加速度(RPM/s)
    fix32_t  fCurrentSetpointRPM;               // 当前设定点速度(RPM)
    fix32_t  fRampStartSpeedRPM;                // 起始速度(RPM)
    fix32_t  fRampPeakSpeedRPM;                 // 峰值速度(RPM) — 三角/梯形中到达的最大速度

    /* 预估时间 */
    fix32_t  fEstimatedTimeMs;                  // 预估到达目标的总时间(ms)
    fix32_t  fElapsedTimeMs;                    // 已用时间(ms)

    /* 位置控制 */
    int32_t  lTargetPos;                        // 目标位置（编码器脉冲数）
    uint8_t  ucInPosition;                      // 到位标志
    uint8_t  ucMotionEnable;                    // 运动使能

    /* 位置模式专用速度上限（不污染 fMaxVelRPM，避免速度模式被错误限幅） */
    fix32_t  fPosMaxVelRPM;                     // 位置模式最大速度(RPM)，0=使用静态参数

    /* 梯形速度规划 */
    fix32_t  fProfileSpd;                       // 当前规划速度 (RPM, Q16.16)
    fix32_t  fAccStep;                          // 加速度步长 (RPM/ms, Q16.16)
    fix32_t  fDecStep;                          // 减速度步长 (RPM/ms, Q16.16)
    uint8_t  ucAccelEn;                         // 梯形规划使能

}
stGearMotorRunningParamTdf;


/**
 * @brief          减速电机总结构体
 * @note           继承自电机基类，stBase 必须作为第一个成员
 */
typedef struct
{
    stMotorDeviceTdf            stBase;               // 基类成员（必须作为第一个成员）
    stGearMotorStaticParamTdf   stStaticParam;        // 静态参数（硬件配置）
    stGearMotorRunningParamTdf  stRunningParam;       // 运行参数（动态状态）
}
stGearMotorDeviceParamTdf;

/* 减速电机虚方法实现 */
void vGearMotorInit(void *pstInit);
void vGearMotorPeriodExecute(void *pstMotor);
void vGearMotorSetSpeed(void *pstMotor, int16_t speed);
void vGearMotorStop(void *pstMotor, uint8_t bSyncFlag);
void vGearMotorEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag);
emMotorStateTdf emGetGearMotorState(void *pstMotor);

/* 减速电机注册函数 */
void vGearMotorRegister(emMotorDevNumTdf emDevNum, stGearMotorStaticParamTdf *pstInit);

/* VTable 位置控制（串级PID） */
void vGearMotorPosControl(
    void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc,
    uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag);

/* VTable 速度控制（带梯形加速度规划） */
void vGearMotorVelControl(
    void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc,
    uint8_t bSyncFlag);

/* 新增便捷接口 */
void vGearMotorSetTargetSpeed(void *pstMotor, fix32_t fTargetRPM);
fix32_t fGearMotorGetEstimatedTime(void *pstMotor);
fix32_t fGearMotorGetCurrentSpeed(void *pstMotor);

/* 调试接口 */
fix32_t fGearMotorGetSetpointRPM(emMotorDevNumTdf emDevNum);
uint8_t ucGearMotorGetAccPhase(emMotorDevNumTdf emDevNum);

#endif

#endif
