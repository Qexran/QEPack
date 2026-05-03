/** 
 * @file    gray_sensor_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/04/18
 * @brief   灰度传感器巡线驱动模块
 * 
 * 本模块实现8路灰度传感器的巡线检测功能，包括：
 * - 8路传感器状态采集与位运算组合
 * - 状态编码与偏移量计算
 * - 均值滤波处理
 * - 停止线/拐角检测
 * - 异常状态计数与处理
 */
#include "project_config.h"
#if GRAY_SENSOR_IS_ENABLE

#ifndef _GRAY_SENSOR_DEVICE_H_
#define _GRAY_SENSOR_DEVICE_H_

#include "string.h"

#if (QEPACK_PLATFORM == ST)
    #include "gpio.h"
#else
    #include "ti_platform.h"
#endif

/** @brief 灰度传感器设备号枚举 */
typedef enum {
    emGraySensorDevNum0 = 0,
    emGraySensorDevNum1,
    emGraySensorDevNum2,
    emGraySensorDevNum3,
} emGraySensorDevNumTdf;

/** @brief 巡线停止状态枚举 */
typedef enum {
    emGrayStopNormal = 0,      // 正常巡线
    emGrayStopLeftCorner,      // 左拐角
    emGrayStopRightCorner,     // 右拐角
    emGrayStopEdge,            // 横线/边缘
    emGrayStopAllBlank,        // 全空白区域
} emGrayStopStateTdf;

/** @brief 传感器触发追踪结构体（时间窗口检测） */
typedef struct {
    uint8_t ucTriggered;       // 是否在时间窗口内触发过（0=未触发，1=已触发）
    uint8_t ucValidCnt;        // 触发状态的有效计数器（超过阈值则失效）
} stGraySensorTrackTdf;

/** @brief 灰度传感器静态参数定义 */
typedef struct {
    uint8_t ucBackupLength;               // 均值滤波历史长度
    uint8_t ucTrackValidThreshold;        // 时间窗口阈值（单位：调用周期）
    uint8_t ucGrayDirection;              // 灰度矫正方向（0/1）
    uint8_t ucEnableStopLineDetect;       // 是否开启停止线检测
    uint8_t ucNeedBlankFix;               // 是否需要空白区域纠正
    uint8_t ucDisableWhileTurning;        // 转弯时是否禁用灰度
    
    #if (QEPACK_PLATFORM == TI)
        GPIO_Regs *pstGpioPort[8];        // 8路传感器GPIO端口
        uint32_t ulGpioPin[8];            // 8路传感器GPIO引脚号
    #else
        GPIO_TypeDef *pstGpioPort[8];     // 8路传感器GPIO端口
        uint16_t usGpioPin[8];            // 8路传感器GPIO引脚号
    #endif
} stGraySensorStaticParamTdf;

/** @brief 灰度传感器运行参数定义 */
typedef struct {
    uint16_t usGrayState;                 // 当前灰度传感器状态(0x00-0xFF)
    int16_t sGrayStatus;                  // 滤波后偏移量
    int16_t sLastGrayStatus;              // 上一次偏移量
    int16_t *psStatusBackup;              // 灰度状态历史记录缓冲区
    uint8_t ucBackupIdx;                  // 环形缓冲区索引
    uint32_t ulStatusWorse;               // 异常状态计数器
    uint8_t ucEnableSensor;               // 传感器使能标志
    uint8_t ucEnteredFixBlankMode;        // 空白修正模式标志
    
    emGrayStopStateTdf emStopState;       // 巡线停止状态
    stGraySensorTrackTdf stTrack[3];      // 传感器6/7/8号触发追踪
} stGraySensorRunningParamTdf;

/** @brief 灰度传感器设备参数总结构体 */
typedef struct {
    stGraySensorStaticParamTdf stStaticParam;   // 静态参数
    stGraySensorRunningParamTdf stRunningParam; // 运行参数
} stGraySensorDeviceParamTdf;

/* 获取灰度传感器设备参数 */
const stGraySensorDeviceParamTdf *c_pstGetGraySensorDeviceParam(emGraySensorDevNumTdf emDevNum);

/* 初始化函数 */
void vGraySensorDeviceInit(stGraySensorStaticParamTdf *pstInit, emGraySensorDevNumTdf emDevNum);

/* 获取8路传感器状态 */
uint16_t usGraySensorGetState(emGraySensorDevNumTdf emDevNum);

/* 灰度传感器状态分析与处理 */
void vGraySensorCheck(emGraySensorDevNumTdf emDevNum);

/* 停止线/拐角检测 */
void vGraySensorCheckStop(emGraySensorDevNumTdf emDevNum);

/* 获取滤波后的偏移量 */
int16_t sGraySensorGetStatus(emGraySensorDevNumTdf emDevNum);

/* 获取异常状态计数器 */
uint32_t ulGraySensorGetWorse(emGraySensorDevNumTdf emDevNum);

/* 设置异常状态计数器 */
void vGraySensorSetWorse(emGraySensorDevNumTdf emDevNum, uint32_t ulGrayWorse);

/* 获取停止状态 */
emGrayStopStateTdf emGraySensorGetStopState(emGraySensorDevNumTdf emDevNum);

/* 设置停止状态 */
void vGraySensorSetStopState(emGraySensorDevNumTdf emDevNum, emGrayStopStateTdf emStopState);

/* 设置停止检测冻结阈值 */
void vGraySensorSetFreezeStopThreshold(emGraySensorDevNumTdf emDevNum, uint16_t usTime);

/* 设置编码器目标值（停止检测用） */
void vGraySensorSetEncoderTarget(emGraySensorDevNumTdf emDevNum, int32_t lEncoderTarget);

#endif
#endif
