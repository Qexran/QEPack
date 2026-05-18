/**
  * @file       sensor_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/10
  * @brief      传感器抽象基类，基于 QePack VTable 多态模式
  */

#ifndef __SENSOR_DEVICE_H
#define __SENSOR_DEVICE_H

#include "project_config.h"

#if SENSOR_IS_ENABLE

#include "arithmetic.h"
#include "pid_controller.h"

/* 编译期常量：0.0001 的 Q16.16 表示 ≈ 7 */
#define FIX32_EPSILON  ((fix32_t)7)

/** @brief 传感器类型枚举 */
typedef enum {
    emSensorTypeGyro = 0,    /* 陀螺仪 */
    emSensorTypeGray,        /* 灰度传感器 */
    emSensorTypeCCD,         /* 线性CCD */
} emSensorTypeTdf;

/** @brief 传感器设备号枚举 */
typedef enum {
    // 灰度传感器设备号
    emSensorGrayDevNum0     = 0,
    emSensorGrayDevNum1     = 1,
    emSensorGrayDevNum2     = 2,

    // 线性CCD传感器设备号
    emSensorCCDDevNum0      = 3,
    emSensorCCDDevNum1      = 4,
    emSensorCCDDevNum2      = 5,
    emSensorCCDDevNum3      = 6,

    // 陀螺仪传感器设备号
    emSensorMPU6050DevNum0     = 7,     /* MPU6050 陀螺仪 */
    emSensorMPU6050DevNum1     = 8,
    emSensorMPU6050DevNum2     = 9,

    emSensorAtkMs901MDevNum0   = 10,    /* ATK901M 陀螺仪 */
    emSensorAtkMs901MDevNum1   = 11,
    emSensorAtkMs901MDevNum2   = 12,

    emSensor660RBDevNum0       = 13,    /* 660RB 陀螺仪 */
    emSensor660RBDevNum1       = 14,
    emSensor660RBDevNum2       = 15,
    
    emSensorNBO08XDevNum0        = 16,    /* NBO08X 陀螺仪 */
    emSensorNBO08XDevNum1        = 17,
    emSensorNBO08XDevNum2        = 18,

    emSensorMaxDevNum      = 19,

    emNoSensor = 0xFF,
} emSensorDevNumTdf;

/** @brief 传感器虚方法表 */
typedef struct stSensorVTableTdf {
    void    (*vInit)(void *pstSensor);                          /* 初始化 */
    void    (*vPeriodExecute)(void *pstSensor);                 /* 周期执行 */
    fix32_t (*fGetValue)(void *pstSensor);                      /* 获取当前值 */
    void    (*vReset)(void *pstSensor);                         /* 重置传感器状态 */
    void    (*vSetTarget)(void *pstSensor, fix32_t fTarget);    /* 设置目标值 */
    fix32_t (*fGetTarget)(void *pstSensor);                     /* 获取目标值 */
} stSensorVTableTdf;

/** @brief 传感器基类结构体 */
typedef struct stSensorDeviceTdf {
    emSensorTypeTdf    emType;          /* 传感器类型 */
    stSensorVTableTdf  *pstVTable;      /* 虚方法表 */
    uint8_t            ucEnable;        /* 使能标志 */
    fix32_t            fWeight;         /* 互补滤波权重（0~1） */
} stSensorDeviceTdf;

/* 全局传感器设备数组 */
extern stSensorDeviceTdf *g_astSensorDevices[];

/* 传感器基类方法 */
stSensorDeviceTdf *pstSensorGetDevice(emSensorDevNumTdf emDevNum);
void vSensorRegisterDevice(emSensorDevNumTdf emDevNum, stSensorDeviceTdf *pstSensor);
void vSensorInit(emSensorDevNumTdf emDevNum);
void vSensorPeriodExecute(emSensorDevNumTdf emDevNum);
fix32_t fSensorGetValue(emSensorDevNumTdf emDevNum);
void vSensorReset(emSensorDevNumTdf emDevNum);
void vSensorSetTarget(emSensorDevNumTdf emDevNum, fix32_t fTarget);
fix32_t fSensorGetTarget(emSensorDevNumTdf emDevNum);
void vSensorSetEnable(emSensorDevNumTdf emDevNum, uint8_t bEnable);
void vSensorSetWeight(emSensorDevNumTdf emDevNum, fix32_t fWeight);

/* 传感器融合 互补滤波 */
fix32_t fSensorFuseValue(emSensorDevNumTdf emDevNumA, emSensorDevNumTdf emDevNumB);

/* 陀螺仪传感器注册（ucLocalIdx: 0-2 对应 ATK-MS901M 设备 0-2） */
#if ATK_MS901M_IS_ENABLE
    #include "atk_ms901m_device.h"
    void vGyroSensorRegister(uint8_t ucLocalIdx, emAtkMs901mDevNumTdf emAtkDevNum);
#endif

/* 灰度传感器注册（ucLocalIdx: 0-2 对应灰度设备 0-2） */
#if GRAY_SENSOR_IS_ENABLE
    #include "gray_sensor_device.h"
    void vGraySensorWrapperRegister(uint8_t ucLocalIdx, emGraySensorDevNumTdf emGrayDevNum);
#endif

/* CCD 传感器注册（ucLocalIdx: 0-3 对应 CCD 设备 0-3） */
#if LINEAR_CCD_IS_ENABLE
    #include "linear_ccd_device.h"
    void vCCDSensorRegister(uint8_t ucLocalIdx, emLinerCcdDevNumTdf emCCDDevNum);
#endif

#endif /* SENSOR_IS_ENABLE */

#endif /* __SENSOR_DEVICE_H */
