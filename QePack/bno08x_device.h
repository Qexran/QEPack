/**
 * @file    bno08x_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/05/15
 * @brief   BNO08x 九轴 IMU 驱动模块 (UART RVC 模式)
 *
 * BNO08x 通过 UART 以 115200bps 发送旋转向量数据包。
 * 本模块封装 UART DMA 接收与数据解析，提供 QEPack 标准 API。
 */

#include "project_config.h"

#if BNO08X_IS_ENABLE

#ifndef _BNO08X_DEVICE_H_
#define _BNO08X_DEVICE_H_

#include "uart_device.h"

/** @brief BNO08x 设备号枚举 */
typedef enum {
    emBno08xDevNum0 = 0,
    emBno08xDevNum1,
    emBno08xDevNum2,
    emBno08xDevNum3,
} emBno08xDevNumTdf;

/** @brief BNO08x 姿态角数据结构 */
typedef struct {
    float fPitch;          /* 俯仰角 (°) */
    float fRoll;           /* 横滚角 (°) */
    float fYaw;            /* 航向角 (°) */
} stBno08xAttitudeTdf;

/** @brief BNO08x 加速度数据结构（原始值） */
typedef struct {
    int16_t sX;
    int16_t sY;
    int16_t sZ;
} stBno08xAccelTdf;

/** @brief BNO08x 传感器完整数据 */
typedef struct {
    uint8_t           ucIndex;       /* 数据包序号 */
    stBno08xAttitudeTdf stAttitude;  /* 姿态角 */
    stBno08xAccelTdf  stAccel;       /* 加速度原始值 */
    uint8_t           ucUpdated;     /* 数据更新标志 */
} stBno08xDataTdf;

/** @brief BNO08x 静态参数 */
typedef struct {
    emUartDevNumTdf emUartDevNum;    /* 绑定的 UART 设备号 */
} stBno08xStaticParamTdf;

/** @brief BNO08x 运行参数 */
typedef struct {
    stBno08xDataTdf stData;          /* 最新传感器数据 */
    uint8_t         aucDmaBuf[19];   /* DMA 接收缓冲区 */
} stBno08xRunningParamTdf;

/** @brief BNO08x 设备参数总结构体 */
typedef struct {
    stBno08xStaticParamTdf  stStaticParam;
    stBno08xRunningParamTdf stRunningParam;
} stBno08xDeviceParamTdf;

/* 初始化 */
void vBno08xDeviceInit(emBno08xDevNumTdf emDevNum, emUartDevNumTdf emUartDevNum);

/* 读取最新数据（非阻塞） */
const stBno08xDataTdf *pstBno08xReadData(emBno08xDevNumTdf emDevNum);

/* 获取姿态角 */
void vBno08xGetAttitude(emBno08xDevNumTdf emDevNum, stBno08xAttitudeTdf *pstAttitude);

/* 获取欧拉角分量 */
float fBno08xGetPitch(emBno08xDevNumTdf emDevNum);
float fBno08xGetRoll(emBno08xDevNumTdf emDevNum);
float fBno08xGetYaw(emBno08xDevNumTdf emDevNum);

/* 获取加速度原始值 */
void vBno08xGetAccel(emBno08xDevNumTdf emDevNum, stBno08xAccelTdf *pstAccel);

/* ======================== 使用方法 ========================
 * 1. 在 project_config.h 中设置 BNO08X_IS_ENABLE 为 1
 *    设置 BNO08X_DEV_NUM 为所需设备数量
 *    设置 BNO08X0 为 emBno08xDevNum0
 *
 * 2. 在 SysConfig 中配置 UART：
 *    - 添加 UART 模块，命名为 "UART_BNO08X"
 *    - 波特率 115200，RX only
 *    - 启用 FIFO，RX Timeout 中断计数设为 1
 *    - 启用 RX timeout 中断
 *    - 配置 DMA RX Trigger 为 UART RX interrupt
 *    - DMA Channel RX 命名为 "DMA_BNO08X"
 *    - Address Mode: Fixed addr. to Block addr.
 *    - Source/Destination Length: Byte
 *    - DMA 传输长度: 18 字节
 *
 * 3. 在 ti_interrupt.c 中注册 UART 中断：
 *    CREATE_UART_IRQ_HANDLER(UART_BNO08X, UART_DEVICE_N)
 *
 * 4. 在 ti_platform.h 中添加 BNO08x 的 UART 结构体宏：
 *    #define TI_GET_BNO08X_UART_STRUCTURE  TI_GET_UART_STRUCTURE(UART_BNO08X)
 *
 * 5. 调用示例：
 *    vUartDeviceInit(..., UART_DEVICE_0);
 *    vBno08xDeviceInit(BNO08X0, UART_DEVICE_0);
 *
 *    // 在主循环中读取
 *    float fPitch = fBno08xGetPitch(BNO08X0);
 *    float fRoll  = fBno08xGetRoll(BNO08X0);
 *    float fYaw   = fBno08xGetYaw(BNO08X0);
 * ========================================================= */

#endif
#endif
