/**
  * @file       emm_v5_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm_V5步进闭环电机驱动，基于QePack编码风格
  *
  */
#ifndef __EMM_V5_DEVICE_H
#define __EMM_V5_DEVICE_H

#include "project_config.h"
#if EMM_V5_IS_ENABLE

#include "uart_device.h"

/// @brief  Emm_V5系统参数读取枚举
typedef enum
{
    emEmmV5SysParam_Ver     = 0,    /* 读取固件版本和对应的硬件版本 */
    emEmmV5SysParam_RL      = 1,    /* 读取相电阻和相电感 */
    emEmmV5SysParam_PID     = 2,    /* 读取PID参数 */
    emEmmV5SysParam_VBus    = 3,    /* 读取总线电压 */
    emEmmV5SysParam_Cpha    = 5,    /* 读取相电流 */
    emEmmV5SysParam_Encl    = 7,    /* 读取经过线性化校准后的编码器值 */
    emEmmV5SysParam_TPos    = 8,    /* 读取电机目标位置角度 */
    emEmmV5SysParam_Vel     = 9,    /* 读取电机实时转速 */
    emEmmV5SysParam_CPos    = 10,   /* 读取电机实时位置角度 */
    emEmmV5SysParam_PErr    = 11,   /* 读取电机位置误差角度 */
    emEmmV5SysParam_Flag    = 13,   /* 读取使能/到位/堵转状态标志位 */
    emEmmV5SysParam_Conf    = 14,   /* 读取驱动参数 */
    emEmmV5SysParam_State   = 15,   /* 读取系统状态参数 */
    emEmmV5SysParam_Org     = 16,   /* 读取正在回零/回零失败状态标志位 */
} emEmmV5SysParamTdf;

/// @brief  Emm_V5控制模式枚举
typedef enum
{
    emEmmV5CtrlMode_PulseOff = 0,    /* 关闭脉冲输入引脚 */
    emEmmV5CtrlMode_OpenLoop = 1,    /* 开环模式 */
    emEmmV5CtrlMode_CloseLoop = 2,   /* 闭环模式 */
    emEmmV5CtrlMode_MultiLimit = 3,   /* En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能 */
} emEmmV5CtrlModeTdf;

/// @brief  Emm_V5回零模式枚举
typedef enum
{
    emEmmV5OrgMode_SingleNear = 0,    /* 单圈就近回零 */
    emEmmV5OrgMode_SingleDir = 1,     /* 单圈方向回零 */
    emEmmV5OrgMode_MultiNoLimit = 2,  /* 多圈无限位碰撞回零 */
    emEmmV5OrgMode_MultiLimit = 3,    /* 多圈有限位开关回零 */
} emEmmV5OrgModeTdf;

/// @brief  Emm_V5方向枚举
typedef enum
{
    emEmmV5Dir_CW  = 0,    /* 顺时针 */
    emEmmV5Dir_CCW = 1,    /* 逆时针 */
} emEmmV5DirTdf;

/// @brief  Emm_V5接收数据回调函数类型
typedef void (*vEmmV5RxCallback)(uint8_t ucAddr, uint8_t *pucData, uint16_t usLen);

/// @brief  Emm_V5运行参数定义
typedef struct
{
    uint8_t  ucAddr;              /* 电机地址 */
    uint8_t  aucRxFrameBuf[64];   /* 接收帧缓冲区 */
    uint16_t usRxFrameLen;         /* 接收帧长度 */
    uint8_t  ucRxFrameComplete;    /* 接收帧完成标志 */
} stEmmV5RunningParamTdf;

/// @brief  Emm_V5静态参数定义
typedef struct
{
    emUartDevNumTdf  emUartDevNum;    /* 关联的UART设备号 */
    vEmmV5RxCallback vCallbackFcn;     /* 接收数据回调函数 */
} stEmmV5StaticParamTdf;

/// @brief  Emm_V5设备参数定义
typedef struct
{
    stEmmV5StaticParamTdf  stStaticParam;   /* 静态参数 */
    stEmmV5RunningParamTdf stRunningParam;  /* 运行参数 */
} stEmmV5DeviceParamTdf;

/// @brief  Emm_V5初始化
/// @param  pstInit ：静态参数指针
void vEmmV5DeviceInit(stEmmV5StaticParamTdf *pstInit);

/// @brief  Emm_V5周期执行（建议放在主循环）
void vEmmV5DevicePeriodExecute(void);

/// @brief  将当前位置清零
/// @param  ucAddr ：电机地址
void vEmmV5ResetCurPosToZero(uint8_t ucAddr);

/// @brief  解除堵转保护
/// @param  ucAddr ：电机地址
void vEmmV5ResetClogPro(uint8_t ucAddr);

/// @brief  读取系统参数
/// @param  ucAddr ：电机地址
/// @param  emSysParam ：系统参数类型
void vEmmV5ReadSysParams(uint8_t ucAddr, emEmmV5SysParamTdf emSysParam);

/// @brief  修改开环/闭环控制模式
/// @param  ucAddr ：电机地址
/// @param  bSave ：是否存储标志，false为不存储，true为存储
/// @param  emCtrlMode ：控制模式
void vEmmV5ModifyCtrlMode(uint8_t ucAddr, uint8_t bSave, emEmmV5CtrlModeTdf emCtrlMode);

/// @brief  电机使能控制
/// @param  ucAddr ：电机地址
/// @param  bEnable ：使能状态，true为使能电机，false为关闭电机
/// @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
void vEmmV5EnControl(uint8_t ucAddr, uint8_t bEnable, uint8_t bSyncFlag);

/// @brief  速度模式控制
/// @param  ucAddr ：电机地址
/// @param  emDir ：方向
/// @param  usVel ：速度，范围0 - 5000RPM
/// @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
/// @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
void vEmmV5VelControl(uint8_t ucAddr, emEmmV5DirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag);

/// @brief  位置模式控制
/// @param  ucAddr ：电机地址
/// @param  emDir ：方向
/// @param  usVel ：速度(RPM)，范围0 - 5000RPM
/// @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
/// @param  ulClk ：脉冲数，范围0- (2^32 - 1)个
/// @param  bAbsFlag ：相位/绝对标志，false为相对运动，true为绝对值运动
/// @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
void vEmmV5PosControl(uint8_t ucAddr, emEmmV5DirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag);

/// @brief  让电机立即停止运动
/// @param  ucAddr ：电机地址
/// @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
void vEmmV5StopNow(uint8_t ucAddr, uint8_t bSyncFlag);

/// @brief  触发多机同步开始运动
/// @param  ucAddr ：电机地址
void vEmmV5SynchronousMotion(uint8_t ucAddr);

/// @brief  设置单圈回零的零点位置
/// @param  ucAddr ：电机地址
/// @param  bSave ：是否存储标志，false为不存储，true为存储
void vEmmV5OriginSetO(uint8_t ucAddr, uint8_t bSave);

/// @brief  修改回零参数
/// @param  ucAddr ：电机地址
/// @param  bSave ：是否存储标志，false为不存储，true为存储
/// @param  emOrgMode ：回零模式
/// @param  emDir ：回零方向
/// @param  usOrgVel ：回零速度，单位：RPM（转/分钟）
/// @param  ulOrgTm ：回零超时时间，单位：毫秒
/// @param  usSlVel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
/// @param  usSlMa ：无限位碰撞回零检测电流，单位：Ma（毫安）
/// @param  usSlMs ：无限位碰撞回零检测时间，单位：Ms（毫秒）
/// @param  bPotFlag ：上电自动触发回零，false为不使能，true为使能
void vEmmV5OriginModifyParams(uint8_t ucAddr, uint8_t bSave, emEmmV5OrgModeTdf emOrgMode, emEmmV5DirTdf emDir, 
                               uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                               uint16_t usSlMs, uint8_t bPotFlag);

/// @brief  触发回零
/// @param  ucAddr ：电机地址
/// @param  emOrgMode ：回零模式
/// @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
void vEmmV5OriginTriggerReturn(uint8_t ucAddr, emEmmV5OrgModeTdf emOrgMode, uint8_t bSyncFlag);

/// @brief  强制中断并退出回零
/// @param  ucAddr ：电机地址
void vEmmV5OriginInterrupt(uint8_t ucAddr);

#endif
#endif
