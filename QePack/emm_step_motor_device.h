/**
  * @file       emm_step_motor_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm步进闭环电机驱动，基于QePack编码风格
  *
  */
#ifndef __EMM_MOTOR_DEVICE_H
#define __EMM_MOTOR_DEVICE_H

#include "project_config.h"
#if EMM_MOTOR_IS_ENABLE

#include "uart_device.h"
#include "motor_device.h"

/// @brief  EmmMotor系统参数读取枚举
typedef enum
{
    emEmmMotorSysParam_Ver     = 0,    /* 读取固件版本和对应的硬件版本 */
    emEmmMotorSysParam_RL      = 1,    /* 读取相电阻和相电感 */
    emEmmMotorSysParam_PID     = 2,    /* 读取PID参数 */
    emEmmMotorSysParam_VBus    = 3,    /* 读取总线电压 */
    emEmmMotorSysParam_Cpha    = 5,    /* 读取相电流 */
    emEmmMotorSysParam_Encl    = 7,    /* 读取经过线性化校准后的编码器值 */
    emEmmMotorSysParam_TPos    = 8,    /* 读取电机目标位置角度 */
    emEmmMotorSysParam_Vel     = 9,    /* 读取电机实时转速 */
    emEmmMotorSysParam_CPos    = 10,   /* 读取电机实时位置角度 */
    emEmmMotorSysParam_PErr    = 11,   /* 读取电机位置误差角度 */
    emEmmMotorSysParam_Flag    = 13,   /* 读取使能/到位/堵转状态标志位 */
    emEmmMotorSysParam_Conf    = 14,   /* 读取驱动参数 */
    emEmmMotorSysParam_State   = 15,   /* 读取系统状态参数 */
    emEmmMotorSysParam_Org     = 16,   /* 读取正在回零/回零失败状态标志位 */
} emEmmMotorSysParamTdf;

/// @brief  EmmMotor控制模式枚举
typedef enum
{
    emEmmMotorCtrlMode_PulseOff     = 0,    /* 关闭脉冲输入引脚 */
    emEmmMotorCtrlMode_OpenLoop     = 1,    /* 开环模式 */
    emEmmMotorCtrlMode_CloseLoop    = 2,    /* 闭环模式 */
    emEmmMotorCtrlMode_MultiLimit   = 3,    /* En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能 */
} emEmmMotorCtrlModeTdf;

/// @brief  EmmMotor回零模式枚举
typedef enum
{
    emEmmMotorOrgMode_SingleNear     = 0,    /* 单圈就近回零 */
    emEmmMotorOrgMode_SingleDir      = 1,     /* 单圈方向回零 */
    emEmmMotorOrgMode_MultiNoLimit   = 2,  /* 多圈无限位碰撞回零 */
    emEmmMotorOrgMode_MultiLimit     = 3,    /* 多圈有限位开关回零 */
} emEmmMotorOrgModeTdf;


/// @brief  EmmMotor接收数据回调函数类型
typedef void (*vEmmMotorRxCallback)(uint8_t ucAddr, uint8_t *pucData, uint16_t usLen);

/// @brief  EmmMotor运行参数定义
typedef struct
{
    uint8_t  aucRxFrameBuf[64];   /* 接收帧缓冲区 */
    uint16_t usRxFrameLen;         /* 接收帧长度 */
    uint8_t  ucRxFrameComplete;    /* 接收帧完成标志 */
} stEmmMotorRunningParamTdf;

/// @brief  EmmMotor静态参数定义
typedef struct
{
    uint8_t     emUartDevNum;               /* 关联的UART设备号 */
    uint8_t     ucAddr;                     /* 电机地址 */
} stEmmMotorStaticParamTdf;

/// @brief  EmmMotor设备参数定义
/// @note   继承自电机基类
typedef struct
{
    stMotorDeviceTdf            stBase;               // 基类成员（必须作为第一个成员）
    stEmmMotorStaticParamTdf    stStaticParam;       /* 静态参数 */
    stEmmMotorRunningParamTdf   stRunningParam;      /* 运行参数 */
    uint8_t                     emDevNum;             /* 设备号 */
} stEmmMotorDeviceParamTdf;

/* 虚方法实现 */
void vEmmMotorInit(void *pstInit);
void vEmmMotorPeriodExecute(void *pstMotor);
void vEmmMotorSetSpeed(void *pstMotor, int16_t speed);
void vEmmMotorStop(void *pstMotor, uint8_t bSyncFlag);
void vEmmMotorEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag);
emMotorStateTdf emGetEmmMotorState(void *pstMotor);


/* 注册函数 */
void vEmmMotorRegister(emMotorDevNumTdf emDevNum, stEmmMotorStaticParamTdf *pstInit);

/* 原有方法保持不变 */
void vEmmMotorResetCurPosToZero(void *pstMotor);
void vEmmMotorResetClogPro(void *pstMotor);
void vEmmMotorReadSysParams(void *pstMotor, emEmmMotorSysParamTdf emSysParam);
void vEmmMotorModifyCtrlMode(void *pstMotor, uint8_t bSave, emEmmMotorCtrlModeTdf emCtrlMode);
void vEmmMotorVelControl(void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag);
void vEmmMotorPosControl(void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag);
void vEmmMotorSynchronousMotion(void *pstMotor);
void vEmmMotorOriginSetO(void *pstMotor, uint8_t bSave);
void vEmmMotorOriginModifyParams(void *pstMotor, uint8_t bSave, emEmmMotorOrgModeTdf emOrgMode, emMotorDirTdf emDir, 
                                uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                                uint16_t usSlMs, uint8_t bPotFlag);
void vEmmMotorOriginTriggerReturn(void *pstMotor, emEmmMotorOrgModeTdf emOrgMode, uint8_t bSyncFlag);
void vEmmMotorOriginInterrupt(void *pstMotor);
void vEmmMotorStopNow(void *pstMotor, uint8_t bSyncFlag);
void vEmmMotorEnControl(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag);

#endif
#endif
