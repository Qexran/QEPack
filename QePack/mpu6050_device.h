/**
 * @file    mpu6050_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/05/15
 * @brief   MPU6050 六轴 IMU 驱动模块 (I2C + DMP)
 *
 * MPU6050 通过 I2C 总线通信，内置 DMP (Digital Motion Processor)
 * 实现传感器融合，直接输出四元数姿态。
 * 依赖 InvenSense inv_mpu 库和 DMP 固件。
 */

#include "project_config.h"

#if MPU6050_IS_ENABLE

#ifndef _MPU6050_DEVICE_H_
#define _MPU6050_DEVICE_H_

#if (QEPACK_PLATFORM == ST)
    #include "gpio.h"
#else
    #include "ti_platform.h"
#endif

/** @brief MPU6050 设备号枚举 */
typedef enum {
    emMpu6050DevNum0 = 0,
    emMpu6050DevNum1,
    emMpu6050DevNum2,
    emMpu6050DevNum3,
} emMpu6050DevNumTdf;

/** @brief MPU6050 姿态角数据 */
typedef struct {
    float fPitch;              /* 俯仰角 (°) */
    float fRoll;               /* 横滚角 (°) */
    float fYaw;                /* 航向角 (°) */
} stMpu6050AttitudeTdf;

/** @brief MPU6050 陀螺仪数据 */
typedef struct {
    int16_t sX;                /* X 轴原始值 */
    int16_t sY;                /* Y 轴原始值 */
    int16_t sZ;                /* Z 轴原始值 */
} stMpu6050GyroRawTdf;

/** @brief MPU6050 加速度数据 */
typedef struct {
    int16_t sX;                /* X 轴原始值 */
    int16_t sY;                /* Y 轴原始值 */
    int16_t sZ;                /* Z 轴原始值 */
} stMpu6050AccelRawTdf;

/** @brief MPU6050 四元数数据 */
typedef struct {
    float fQ0;
    float fQ1;
    float fQ2;
    float fQ3;
} stMpu6050QuaternionTdf;

/** @brief MPU6050 静态参数 */
typedef struct {
    /* I2C 硬件配置 — 使用 TI_GET_I2C_STRUCTURE(I2C_MPU6050) 生成 */
    stI2CTdf stI2c;
    /* 数据更新速率 (Hz)，典型值 50 */
    uint16_t usSampleRateHz;
} stMpu6050StaticParamTdf;

/** @brief MPU6050 运行参数 */
typedef struct {
    stMpu6050AttitudeTdf   stAttitude;   /* 姿态角（主循环按需计算） */
    stMpu6050QuaternionTdf stQuat;       /* 四元数（Q30 浮点值） */
    stMpu6050GyroRawTdf    stGyro;       /* 陀螺仪原始值 */
    stMpu6050AccelRawTdf   stAccel;      /* 加速度原始值 */
    long                   alQuatRaw[4]; /* 原始四元数（Q30 定点，ISR 仅缓存） */
    uint8_t                ucDmpReady;   /* DMP 初始化完成标志 */
    uint8_t                ucDataUpdated;/* 原始数据更新标志（ISR 设置） */
    uint8_t                ucAttitudeDirty;/* 姿态需重算标志 */
} stMpu6050RunningParamTdf;

/** @brief MPU6050 设备参数总结构体 */
typedef struct {
    stMpu6050StaticParamTdf  stStaticParam;
    stMpu6050RunningParamTdf stRunningParam;
} stMpu6050DeviceParamTdf;

/* 初始化（阻塞，加载 DMP 固件） */
QE_StatusTypeDef emMpu6050DeviceInit(emMpu6050DevNumTdf emDevNum, const stMpu6050StaticParamTdf *pstInit);

/* 读取 DMP FIFO 并更新姿态（在 GPIO 中断回调中调用） */
QE_StatusTypeDef emMpu6050ReadDmp(emMpu6050DevNumTdf emDevNum);

/* 获取姿态角 */
void vMpu6050GetAttitude(emMpu6050DevNumTdf emDevNum, stMpu6050AttitudeTdf *pstAttitude);
float fMpu6050GetPitch(emMpu6050DevNumTdf emDevNum);
float fMpu6050GetRoll(emMpu6050DevNumTdf emDevNum);
float fMpu6050GetYaw(emMpu6050DevNumTdf emDevNum);

/* 获取陀螺仪原始值 */
void vMpu6050GetGyroRaw(emMpu6050DevNumTdf emDevNum, stMpu6050GyroRawTdf *pstGyro);

/* 获取加速度原始值 */
void vMpu6050GetAccelRaw(emMpu6050DevNumTdf emDevNum, stMpu6050AccelRawTdf *pstAccel);

/* ======================== 使用方法 ========================
 * 1. 在 project_config.h 中设置 MPU6050_IS_ENABLE 为 1
 *    设置 MPU6050_DEV_NUM 为 1
 *    设置 MPU60500 为 emMpu6050DevNum0
 *
 * 2. 在 SysConfig 中配置 I2C 和 GPIO：
 *    I2C: 添加 I2C 模块，命名为 "I2C_MPU6050"
 *         启用 Controller Mode，速度选 Fast Mode (400kHz)
 *    GPIO: 添加 GPIO 模块，命名为 "GPIO_MPU6050"
 *          引脚命名为 "PIN_MPU6050_INT"
 *          方向 Input，内部上拉，使能中断
 *          优先级 Level 3 - Lowest，触发 Falling Edge
 *
 * 3. 在 ti_interrupt.c 中注册 GPIO 中断处理：
 *    CREATE_MPU6050_GPIO_HANDLER(GPIO_MPU6050, MPU60500)
 *
 * 4. 调用示例：
 *    stMpu6050StaticParamTdf stInit = {
 *        .stI2c = TI_GET_I2C_STRUCTURE(I2C_MPU6050),
 *        .usSampleRateHz = 50,
 *    };
 *    emMpu6050DeviceInit(MPU60500, &stInit);
 *
 *    // 在 GROUP1_IRQHandler 中（由 INT 引脚触发）
 *    emMpu6050ReadDmp(MPU60500);
 *
 *    // 在主循环中读取
 *    float fPitch = fMpu6050GetPitch(MPU60500);
 * ========================================================= */

#endif
#endif
