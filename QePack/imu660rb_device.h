/**
 * @file    imu660rb_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/05/15
 * @brief   IMU660RB 六轴 IMU 驱动模块 (SPI + LSM6DSR + Fusion AHRS)
 *
 * IMU660RB 搭载 LSM6DSR 六轴传感器，通过 SPI 3-Wire 模式通信。
 * 使用 ST lsm6dsr_reg 寄存器库驱动传感器，Fusion AHRS 库实现姿态融合。
 */

#include "project_config.h"

#if IMU660RB_IS_ENABLE

#ifndef _IMU660RB_DEVICE_H_
#define _IMU660RB_DEVICE_H_

#if (QEPACK_PLATFORM == ST)
    #include "gpio.h"
#else
    #include "ti_platform.h"
#endif

/** @brief IMU660RB 设备号枚举 */
typedef enum {
    emImu660rbDevNum0 = 0,
    emImu660rbDevNum1,
    emImu660rbDevNum2,
    emImu660rbDevNum3,
} emImu660rbDevNumTdf;

/** @brief IMU660RB 姿态角数据 */
typedef struct {
    float fPitch;              /* 俯仰角 (°) */
    float fRoll;               /* 横滚角 (°) */
    float fYaw;                /* 航向角 (°) */
} stImu660rbAttitudeTdf;

/** @brief IMU660RB 加速度数据 (mg) */
typedef struct {
    float fX;
    float fY;
    float fZ;
} stImu660rbAccelTdf;

/** @brief IMU660RB 陀螺仪数据 (mdps) */
typedef struct {
    float fX;
    float fY;
    float fZ;
} stImu660rbGyroTdf;

/** @brief IMU660RB SPI 硬件配置 */
typedef struct {
    SPI_Regs      *spi_inst;        /* SPI 模块寄存器基地址 */
    GPIO_Regs     *pstCsPort;       /* CS 引脚 GPIO 端口 */
    uint32_t      ulCsPin;          /* CS 引脚号 */
    GPIO_Regs     *pstIntPort;      /* INT1 引脚 GPIO 端口 */
    uint32_t      ulIntPin;         /* INT1 引脚号 */
} stImu660rbSpiTdf;

/** @brief IMU660RB 静态参数 */
typedef struct {
    stImu660rbSpiTdf stSpi;          /* SPI 硬件配置 */
} stImu660rbStaticParamTdf;

/** @brief IMU660RB 运行参数 */
typedef struct {
    stImu660rbAttitudeTdf  stAttitude;   /* 姿态角 */
    stImu660rbAccelTdf     stAccel;      /* 加速度 (mg) */
    stImu660rbGyroTdf      stGyro;       /* 陀螺仪 (mdps) */
    uint8_t                ucDataReady;  /* 数据就绪标志 */
    uint8_t                ucAhrsInit;   /* AHRS 正在初始化 */
} stImu660rbRunningParamTdf;

/** @brief IMU660RB 设备参数总结构体 */
typedef struct {
    stImu660rbStaticParamTdf  stStaticParam;
    stImu660rbRunningParamTdf stRunningParam;
} stImu660rbDeviceParamTdf;

/* 初始化（阻塞，含陀螺仪零偏校准） */
QE_StatusTypeDef emImu660rbDeviceInit(emImu660rbDevNumTdf emDevNum, const stImu660rbStaticParamTdf *pstInit);

/* 读取传感器数据并更新姿态（在 GPIO INT1 中断回调中调用） */
void vImu660rbReadData(emImu660rbDevNumTdf emDevNum);

/* 获取姿态角 */
void vImu660rbGetAttitude(emImu660rbDevNumTdf emDevNum, stImu660rbAttitudeTdf *pstAttitude);
float fImu660rbGetPitch(emImu660rbDevNumTdf emDevNum);
float fImu660rbGetRoll(emImu660rbDevNumTdf emDevNum);
float fImu660rbGetYaw(emImu660rbDevNumTdf emDevNum);

/* 获取加速度 (mg) */
void vImu660rbGetAccel(emImu660rbDevNumTdf emDevNum, stImu660rbAccelTdf *pstAccel);

/* 获取陀螺仪 (mdps) */
void vImu660rbGetGyro(emImu660rbDevNumTdf emDevNum, stImu660rbGyroTdf *pstGyro);

/* ==================== 传感器基类适配 ==================== */
#if SENSOR_IS_ENABLE
    #include "sensor_device.h"
    void vImu660rbSensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit);
#endif

/* ======================== 使用方法 ========================
 * 1. 将以下库文件加入 CCS 工程：
 *    - Drivers/IMU660RB/lsm6dsr_reg.c, lsm6dsr_reg.h
 *    - Drivers/IMU660RB/Fusion/FusionAhrs.c, FusionAhrs.h
 *    - Drivers/IMU660RB/Fusion/FusionOffset.c, FusionOffset.h
 *    - Drivers/IMU660RB/Fusion/Fusion.h (umbrella header)
 *    - Drivers/IMU660RB/Fusion/FusionMath.h, FusionAxes.h,
 *      FusionCalibration.h, FusionConvention.h
 *
 * 2. 在 project_config.h 中设置：
 *    #define IMU660RB_IS_ENABLE    1
 *    #define IMU660RB_DEV_NUM      1
 *    #define IMU660RB0             emImu660rbDevNum0
 *
 * 3. 在 SysConfig 中配置 SPI 和 GPIO：
 *    SPI: 添加 SPI 模块，命名为 "SPI_IMU660RB"
 *         Frame Format 选 "Motorola 3-wire"
 *         速率 ≤ 10MHz
 *    GPIO: 添加 GPIO 模块，命名为 "GPIO_IMU660RB"
 *          引脚 PIN_IMU660RB_CS:   方向 Output，初始 High
 *          引脚 PIN_IMU660RB_INT1: 方向 Input，使能中断
 *          优先级 Level 3 - Lowest，触发 Rising Edge
 *
 * 4. 在 ti_interrupt.c 中注册 GPIO 中断处理：
 *    CREATE_IMU660RB_GPIO_HANDLER(GPIO_IMU660RB, IMU660RB0)
 *
 * 5. 调用示例：
 *    stImu660rbStaticParamTdf stInit = {
 *        .stSpi = TI_GET_IMU660RB_SPI_STRUCTURE(IMU660RB),
 *    };
 *    emImu660rbDeviceInit(IMU660RB0, &stInit);
 *
 *    // 在 GROUP1_IRQHandler 中（由 INT1 引脚触发）
 *    vImu660rbReadData(IMU660RB0);
 *
 *    // 在主循环中读取
 *    float fPitch = fImu660rbGetPitch(IMU660RB0);
 *    float fRoll  = fImu660rbGetRoll(IMU660RB0);
 *    float fYaw   = fImu660rbGetYaw(IMU660RB0);
 * ========================================================= */

#endif
#endif
