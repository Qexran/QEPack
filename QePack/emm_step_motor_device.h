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

/// @brief  EmmMotor校验模式枚举
typedef enum
{
    emEmmMotorChecksum_Fixed6B = 0,    /* 固定0x6B结尾（默认，无校验能力） */
    emEmmMotorChecksum_XOR     = 1,    /* XOR校验 */
    emEmmMotorChecksum_CRC8    = 2,    /* CRC-8校验 */
} emEmmMotorChecksumTdf;


/// @brief  EmmMotor接收数据回调函数类型
typedef void (*vEmmMotorRxCallback)(uint8_t ucAddr, uint8_t *pucData, uint16_t usLen);

/// @brief  EmmMotor上一次运动指令类型（用于超时重试）
typedef enum
{
    emEmmMotorLastCmd_None = 0,   /* 无上一次指令 */
    emEmmMotorLastCmd_Pos,        /* 位置模式 */
    emEmmMotorLastCmd_Vel,        /* 速度模式 */
    emEmmMotorLastCmd_Enable,     /* 使能控制 */
} emEmmMotorLastCmdTdf;

/// @brief  EmmMotor 错误类型（供上层排查通信/电机故障）
typedef enum
{
    emEmmMotorErr_None       = 0,  /* 无错误 */
    emEmmMotorErr_NoResponse = 1,  /* 重试耗尽，电机无应答 */
    emEmmMotorErr_Stall      = 2,  /* 电机堵转（Flag bit2） */
} emEmmMotorErrTdf;

/// @brief  EmmMotor运行参数定义
typedef struct
{
    uint8_t  aucRxFrameBuf[64];   /* 接收帧缓冲区 */
    uint16_t usRxFrameLen;         /* 接收帧长度 */
    uint8_t  ucRxFrameComplete;    /* 接收帧完成标志 */
    uint8_t  ucPollCnt;            /* 状态轮询分频计数 */
    uint8_t  ucPollWait;           /* 等待响应倒计时（非阻塞，每周期 -1） */
    uint16_t usPollTimeout;        /* 轮询超时计数，超时后触发重试或强制切 Stop */
    uint8_t  ucLastFlag;           /* 最新 Flag 响应字节（由 UART 回调更新） */
    uint8_t  ucFlagUpdated;        /* Flag 更新标志（1=有新数据待消费） */
    uint8_t  ucCmdAckReceived;     /* 上次指令已收到应答（由 UART 回调置1） */
    uint8_t  ucRetryCnt;           /* 当前重试次数 */
    uint8_t  ucRetryMax;           /* 最大重试次数（Init 时设为默认值） */
    emEmmMotorLastCmdTdf emLastCmd; /* 上一次运动指令类型 */
    /* 上一次指令的原始参数（供重试时复用） */
    uint8_t  aucLastCmdData[12];
    uint8_t  ucLastCmdDataLen;
    emEmmMotorSysParamTdf emLastQueryParam; /* 最近一次查询的参数类型（用于响应校验） */
    uint8_t  ucExpectedRespLen;    /* 下一次响应的期望长度（含校验字节），0=未知（固定0x6B模式用） */
    emEmmMotorErrTdf emLastError;  /* 最近一次错误类型（由 PeriodExecute 写入） */
    uint32_t ulErrorTick;          /* 错误发生时刻（QE_GET_TICK），0 表示无错误 */
} stEmmMotorRunningParamTdf;

/// @brief  EmmMotor静态参数定义
typedef struct
{
    emUartDevNumTdf     emUartDevNum;               /* 关联的UART设备号 */
    uint8_t             ucAddr;                     /* 电机地址 */
    uint8_t             ucReversed;                 /* 方向反转: 0=正常, 1=反转 */
    emEmmMotorChecksumTdf emChecksumMode;           /* 校验模式（需与电机端配置一致） */
    fix32_t             fWheelDiameterCm;           /* 轮子直径(cm), Q16.16, 0=未配置 */
    fix32_t             fEncoderPulsePerRev;        /* 编码器单圈脉冲数, Q16.16, 0=未配置 */
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

/* 访问器 */
const stEmmMotorStaticParamTdf *c_pstGetEmmMotorStaticParam(emMotorDevNumTdf emDevNum);

/* 原有方法保持不变 */
void vEmmMotorResetCurPosToZero(void *pstMotor);
void vEmmMotorResetClogPro(void *pstMotor);
void vEmmMotorReadSysParams(void *pstMotor, emEmmMotorSysParamTdf emSysParam);
void vEmmMotorModifyCtrlMode(void *pstMotor, uint8_t bSave, emEmmMotorCtrlModeTdf emCtrlMode);
void vEmmMotorVelControl(void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag);
void vEmmMotorPosControl(void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag);
void vEmmMotorSynchronousMotion(void *pstMotor);
void vEmmMotorSyncBroadcast(emUartDevNumTdf emUartDevNum);
void vEmmMotorOriginSetO(void *pstMotor, uint8_t bSave);
void vEmmMotorOriginModifyParams(void *pstMotor, uint8_t bSave, emEmmMotorOrgModeTdf emOrgMode, emMotorDirTdf emDir, 
                                uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                                uint16_t usSlMs, uint8_t bPotFlag);
void vEmmMotorOriginTriggerReturn(void *pstMotor, emEmmMotorOrgModeTdf emOrgMode, uint8_t bSyncFlag);
void vEmmMotorOriginInterrupt(void *pstMotor);
void vEmmMotorStopNow(void *pstMotor, uint8_t bSyncFlag);
void vEmmMotorEnControl(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag);
emEmmMotorErrTdf emGetEmmMotorLastError(void *pstMotor);
void vEmmMotorClearError(void *pstMotor);
uint8_t ucGetEmmMotorLastFlag(emMotorDevNumTdf emDevNum);
uint16_t usGetEmmMotorFlagRxCnt(emMotorDevNumTdf emDevNum);

#endif
#endif
