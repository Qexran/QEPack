/**
  * @file       hc05_device.h
  * @brief      HC05蓝牙模块驱动
  */
#ifndef __HC05_DEVICE_H
#define __HC05_DEVICE_H

#include "project_config.h"
#if HC05_IS_ENABLE

#include "uart_device.h"

/// @brief  HC05设备号枚举
typedef enum
{
    emHC05DevNum0       = 0,
    emHC05DevNum1,
    emHC05DevNum2,
    emHC05DevNum3,
} emHC05DevNumTdf;

/// @brief  HC05角色枚举
typedef enum
{
    emHC05RoleSlave          = 0,       /* < 从角色   */
    emHC05RoleMaster         = 1,       /* < 主角色   */
    emHC05RoleLoop           = 2,       /* < 循环角色 */
} emHC05RoleTdf;

/// @brief  HC05静态参数定义
typedef struct
{
    emUartDevNumTdf      emUartDevNum;
} stHC05StaticParamTdf;

/* 初始化 */
void vHC05DeviceInit(stHC05StaticParamTdf *pstInit, emHC05DevNumTdf emDevNum);

/* AT命令 */
void vHC05SendATCmd(emHC05DevNumTdf emDevNum, char *pcCmd);

/* 配置 */
void vHC05SetName(emHC05DevNumTdf emDevNum, char *pcName);
void vHC05SetPin(emHC05DevNumTdf emDevNum, char *pcPin);
void vHC05SetBaudRate(emHC05DevNumTdf emDevNum, uint32_t ulBaudRate);
void vHC05SetRole(emHC05DevNumTdf emDevNum, emHC05RoleTdf emRole);
void vHC05Reset(emHC05DevNumTdf emDevNum);
void vHC05RestoreDefault(emHC05DevNumTdf emDevNum);

/* 数据发送 */
void vHC05Transmit(emHC05DevNumTdf emDevNum, uint8_t *pucData, uint16_t usLen);
void vHC05Printf(emHC05DevNumTdf emDevNum, const char *pcFormat, ...);

#endif
#endif
