/**
  * @file       hmi_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/06/02
  * @brief      HMI 串口屏模块（陶晶驰 TJC 协议）
  *
  * 基于 QEPack uart_device 模块，提供 TJC 串口屏的发送/接收/解析功能。
  * MCU→屏幕：ASCII 命令 + 0xFF×3 终止符
  * 屏幕→MCU：type_byte + data + 0xFF×3
  */

#ifndef _HMI_DEVICE_H_
#define _HMI_DEVICE_H_

#include "project_config.h"

#if HMI_IS_ENABLE

#include "uart_device.h"

/* ==================== 枚举定义 ==================== */

/* HMI 设备号枚举 */
typedef enum {
    emHmiDevNum0 = 0,
    emHmiDevNum1,
    emHmiDevNum2,
    emHmiDevNum3,
} emHmiDevNumTdf;

/* HMI 响应类型枚举 */
typedef enum {
    emHmiRespTouch      = 0x65,     /* 触摸事件 */
    emHmiRespPageId     = 0x66,     /* 当前页面ID */
    emHmiRespString     = 0x70,     /* 字符串数据返回 */
    emHmiRespNumber     = 0x71,     /* 数值数据返回 */
    emHmiRespStartup    = 0x88,     /* 设备启动成功 */
    emHmiRespOverflow   = 0x24,     /* 串口缓冲区溢出 */
} emHmiRespTypeTdf;

/* HMI 触摸事件类型 */
typedef enum {
    emHmiTouchRelease   = 0x00,     /* 释放 */
    emHmiTouchPress     = 0x01,     /* 按下 */
} emHmiTouchEventTypeTdf;

/* ==================== 数据结构定义 ==================== */

/* HMI 触摸事件数据 */
typedef struct {
    uint8_t             ucPageId;
    uint8_t             ucComponentId;
    uint8_t             ucEvent;        /* 0x01=按下, 0x00=释放 */
} stHmiTouchDataTdf;

/* HMI 数值返回数据 */
typedef struct {
    uint8_t             ucComponentId;
    int32_t             lValue;
} stHmiNumberDataTdf;

/* HMI 字符串返回数据 */
typedef struct {
    uint8_t             ucComponentId;
    uint8_t             ucLength;
    char                acStr[HMI_RX_BUF_MAX_LEN];
} stHmiStringDataTdf;

/* HMI 响应数据联合体 */
typedef union {
    stHmiTouchDataTdf   stTouch;
    stHmiNumberDataTdf  stNumber;
    stHmiStringDataTdf  stString;
    uint8_t             ucPageId;
} unHmiRespDataTdf;

/* HMI 响应解析结果 */
typedef struct {
    emHmiRespTypeTdf    emType;
    unHmiRespDataTdf    unData;
} stHmiResponseTdf;

/* ==================== 回调类型 ==================== */

/* HMI 事件回调函数类型 */
typedef void (*vHmiEventCallback)(emHmiDevNumTdf emDevNum, const stHmiResponseTdf *pstResp);

/* ==================== 静态参数结构体 ==================== */

typedef struct {
    emUartDevNumTdf     emUartDevNum;       /* 绑定的 UART 设备号 */
    vHmiEventCallback   vCallback;          /* 事件回调函数（可选） */
} stHmiStaticParamTdf;

/* ==================== 运行参数结构体 ==================== */

typedef struct {
    uint8_t             ucReady;            /* 屏幕就绪标志（收到 0x88） */
    uint8_t             ucCurrentPageId;    /* 当前页面 ID */
    uint32_t            ulRespCount;        /* 响应计数 */
} stHmiRunningParamTdf;

/* ==================== 设备参数结构体 ==================== */

typedef struct {
    stHmiStaticParamTdf     stStaticParam;
    stHmiRunningParamTdf    stRunningParam;
} stHmiDeviceParamTdf;

/* ==================== 公共 API ==================== */

/* 初始化 HMI 设备 */
void vHmiDeviceInit(const stHmiStaticParamTdf *pstInit, emHmiDevNumTdf emDevNum);

/* 周期执行函数（处理接收数据）— 在主循环中调用 */
void vHmiDevicePeriodExecute(emHmiDevNumTdf emDevNum);

/* ---- 命令发送 API ---- */

/* 发送原始命令（自动追加 0xFF×3 终止符） */
void vHmiSendCmd(emHmiDevNumTdf emDevNum, const char *pcCmd);

/* 发送格式化命令（printf 风格，自动追加 0xFF×3） */
void vHmiSendCmdf(emHmiDevNumTdf emDevNum, const char *pcFormat, ...);

/* 发送 0xFF 终止符（3字节） */
void vHmiSendEndBytes(emHmiDevNumTdf emDevNum);

/* ---- 高层 UI 操作 API ---- */

/* 切换页面 */
void vHmiSetPage(emHmiDevNumTdf emDevNum, const char *pcPageName);

/* 设置文本值 */
void vHmiSetText(emHmiDevNumTdf emDevNum, const char *pcComponent, const char *pcText);

/* 设置数值 */
void vHmiSetNumber(emHmiDevNumTdf emDevNum, const char *pcComponent, int32_t lValue);

/* 设置可见性 */
void vHmiSetVisible(emHmiDevNumTdf emDevNum, const char *pcComponent, uint8_t ucVisible);

/* 设置图片 */
void vHmiSetPicture(emHmiDevNumTdf emDevNum, const char *pcComponent, uint16_t usImageId);

/* 请求当前页面 ID */
void vHmiGetPageId(emHmiDevNumTdf emDevNum);

/* 设置触摸事件回调模式 (bkcmd) */
void vHmiSetBkCmd(emHmiDevNumTdf emDevNum, uint8_t ucMode);

/* 获取屏幕就绪状态 */
uint8_t ucHmiIsReady(emHmiDevNumTdf emDevNum);

/* 获取当前页面 ID */
uint8_t ucHmiGetCurrentPageId(emHmiDevNumTdf emDevNum);

#endif /* HMI_IS_ENABLE */
#endif /* _HMI_DEVICE_H_ */
