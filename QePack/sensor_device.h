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
    emSensorTypeMPU6050Gyro = 0,    /* MPU6050 陀螺仪 */
    emSensorTypeAtkMs901MGyro ,     /* ATK901M 陀螺仪 */
    emSensorType660RBGyro,          /* 660RB 陀螺仪 */
    emSensorTypeNBO08XGyro,         /* NBO08X 陀螺仪 */
    emSensorTypeHWT101Gyro,         /* HWT101 陀螺仪 */
    emSensorTypeGray,               /* 灰度传感器 */
    emSensorTypeCCD,                /* 线性CCD */
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

    emSensorHWT101DevNum0      = 19,
    emSensorHWT101DevNum1      = 20,
    emSensorHWT101DevNum2      = 21,

    emSensorMaxDevNum      = 22,

    emNoSensor = 0xFF,
} emSensorDevNumTdf;

/** @brief 传感器虚方法表 */
typedef struct stSensorVTableTdf {
    void    (*vInit)(void *pstSensor);                          /* 初始化 */
    void    (*vPeriodExecute)(void *pstSensor);                 /* 周期执行 */
    fix32_t (*fGetValue)(void *pstSensor);                      /* 获取当前值（原始角度） */
    fix32_t (*fGetAccumulatedValue)(void *pstSensor);           /* 获取累加值（跨圈连续） */
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

    emPidDevNumTdf     emPidDevNum;     /* PID 设备号（emNoPid=0xFF 表示不使用） */
    uint16_t           usPidPeriodMs;   /* PID 计算周期 (ms) */
    
    uint32_t           ulPidLastTickMs; /* 上次 PID 计算的时间戳 */
    fix32_t            fPidOutput;      /* PID 输出值（基类维护） */
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
fix32_t fSensorGetAccumulatedValue(emSensorDevNumTdf emDevNum);
fix32_t fSensorGetPidOutput(emSensorDevNumTdf emDevNum);
void vSensorSetWeight(emSensorDevNumTdf emDevNum, fix32_t fWeight);
void vSensorSetPidConfig(emSensorDevNumTdf emDevNum, emPidDevNumTdf emPidDevNum, uint16_t usPeriodMs);

/* 传感器融合 互补滤波 */
fix32_t fSensorFuseValue(emSensorDevNumTdf emDevNumA, emSensorDevNumTdf emDevNumB);

#endif /* SENSOR_IS_ENABLE */

#endif /* __SENSOR_DEVICE_H */

/*
    注意：基类字段必须显式填写
    
    使用方式

    // 注册 HWT101 到 sensor 基类
    vHwt101SensorRegister(emSensorHWT101DevNum0);

    // 通过统一接口访问
    vSensorInit(emSensorHWT101DevNum0);
    vSensorPeriodExecute(emSensorHWT101DevNum0);
    fix32_t fAngle = fSensorGetValue(emSensorHWT101DevNum0);

    // 配置 PID 角度环纠偏（10ms 周期）
    vSensorSetPidConfig(emSensorHWT101DevNum0, emPidDevNum0, 10);
    vSensorSetTarget(emSensorHWT101DevNum0, fix32_from_float(90.0f));
*/
