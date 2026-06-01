/**
  * @file       hwt101_device.h
  * @author     Qe_xr
  * @version    V3.0.0
  * @date       2026/5/21
  * @brief      HWT101 Z轴陀螺仪/角度传感器驱动
  *
  * HWT101 是维特智能 (WitMotion) 的 Z轴陀螺仪传感器，基于 JY901S 系列裁剪。
  * 支持 UART (TTL) 和 I2C 两种通信模式。
  *
  * UART 模式：传感器主动推送 11 字节数据包（0x55 帧头），通过 uart_device 非帧
  *           回调机制接收数据。
  * I2C 模式：MCU 作为主设备，按分时轮询 GZ (0x39) 和 Yaw (0x3F) 寄存器。
  *
  * 数据以 Q16.16 定点数存储，避免 M0+ 平台的浮点运算开销。
  *
  * 继承自 sensor_device 基类，统一使用 emSensorDevNumTdf 管理设备。
  */

#ifndef _HWT101_DEVICE_H_
#define _HWT101_DEVICE_H_

#include "project_config.h"

#if HWT101_IS_ENABLE

#if (QEPACK_PLATFORM == TI)
    #include "ti_platform.h"
#endif

#include "string.h"
#include "uart_device.h"
#include "arithmetic.h"

#if SENSOR_IS_ENABLE
    #include "sensor_device.h"
#endif


/// @brief HWT101 通信模式枚举
typedef enum
{
    emHwt101ComModeUart = 0,            // UART (TTL) 模式
    emHwt101ComModeI2C  = 1,            // I2C 模式
} emHwt101ComModeTdf;


/// @brief HWT101 工作模式枚举（对应 WORKMODE 寄存器 0x48）
typedef enum
{
    emHwt101WorkModeNormal      = 0x00, // 正常数据模式
    emHwt101WorkModePeak        = 0x01, // 求峰峰值模式
    emHwt101WorkModeZeroBias    = 0x02, // 求零偏模式
    emHwt101WorkModeScaleFactor = 0x03, // 求标度因素模式
} emHwt101WorkModeTdf;


/// @brief HWT101 输出速率枚举（对应 RRATE 寄存器 0x03）
typedef enum
{
    emHwt101Rate0_2Hz  = 0x01,          // 0.2Hz
    emHwt101Rate0_5Hz  = 0x02,          // 0.5Hz
    emHwt101Rate1Hz    = 0x03,          // 1Hz
    emHwt101Rate2Hz    = 0x04,          // 2Hz
    emHwt101Rate5Hz    = 0x05,          // 5Hz
    emHwt101Rate10Hz   = 0x06,          // 10Hz（默认）
    emHwt101Rate20Hz   = 0x07,          // 20Hz
    emHwt101Rate50Hz   = 0x08,          // 50Hz
    emHwt101Rate100Hz  = 0x09,          // 100Hz
    emHwt101Rate200Hz  = 0x0B,          // 200Hz
    emHwt101Rate500Hz  = 0x0C,          // 500Hz
    emHwt101Rate1000Hz = 0x0D,          // 1000Hz
} emHwt101RateTdf;


/// @brief HWT101 传感器数据更新标志位
#define HWT101_DATA_GYRO_UPDATE     0x01    // 角速度数据更新
#define HWT101_DATA_ANGLE_UPDATE    0x02    // 角度数据更新


/// @brief HWT101 静态参数定义
/// @note  硬件绑定参数，初始化时设置后不变
typedef struct
{
    emHwt101ComModeTdf   emComMode;       // 通信模式：UART 或 I2C
    emUartDevNumTdf      emUartDev;       // UART 设备号（UART 模式有效）
    #if (QEPACK_PLATFORM == ST)
        I2C_HandleTypeDef  *pstI2cHandle; // I2C 句柄（I2C 模式有效）
    #else
        stI2CTdf          *pstI2cHandle;  // I2C 句柄（I2C 模式有效）
    #endif
    uint8_t              u8I2cAddr;       // I2C 从机地址（默认 0x50，可设 0x01~0x7F）

    emHwt101RateTdf      emOutputRate;    // 输出速率，默认 emHwt101Rate100Hz

    /* sensor 基类可配置参数（留 0 则使用默认值） */
    fix32_t              fWeight;         // 互补滤波权重，默认 FIX32_ONE
    emPidDevNumTdf       emPidDevNum;     // PID 设备号，默认 emNoPid
    uint16_t             usPidPeriodMs;   // PID 周期(ms)，默认 0
} stHwt101StaticParamTdf;


/// @brief HWT101 运行参数定义
/// @note  运行时动态变化的状态数据
typedef struct
{
    /* UART 数据包解析状态机 */
    uint8_t     au8PktBuf[11];            // 11 字节数据包缓冲区
    uint8_t     u8PktIdx;                 // 缓冲区写入位置
    uint8_t     u8PktState;               // 0=找帧头 0x55, 1=累积数据

    /* 原始传感器数值（有符号 short，低字节在前） */
    int16_t     s16GzRaw;                 // Z轴角速度原始值 (寄存器 0x39)
    int16_t     s16YawRaw;                // Z轴偏航角原始值   (寄存器 0x3F)
    uint16_t    u16Version;               // 版本号

    /* 物理值 — Q16.16 定点数 */
    fix32_t     s32AngularVelZ;           // Z轴角速度 (°/s)
    fix32_t     s32AngleZ;                // Z轴偏航角 (°)

    /* 角度累加 */
    fix32_t     fLastAngleZ;              // 上次角度值（用于跨圈检测）
    int32_t     lTurnCount;               // 跨圈计数（+180→-180 时 +1，-180→+180 时 -1）

    /* 零偏（软件归零时记录当前原始角度，后续读数减去此值） */
    fix32_t     fZeroOffset;              // 零偏值（Q16.16）

    /* 状态标志 */
    uint8_t     u8DataFlags;              // HWT101_DATA_GYRO_UPDATE | HWT101_DATA_ANGLE_UPDATE
    uint8_t     u8IsOnline;               // 传感器在线标志

    /* I2C 轮询控制 */
    uint32_t    u32LastPollTick;          // 上次 I2C 轮询时间戳 (ms)
    uint8_t     u8PollPhase;              // 轮询阶段（0=读GZ, 1=读Yaw, 2/3=跳过）

    /* UART 离线检测 */
    uint32_t    u32LastRxTick;            // 上次收到 UART 数据的时间戳 (ms)

    /* 目标值（用于 PID 纠偏） */
    fix32_t     fTargetValue;             // 目标角度值
} stHwt101RunningParamTdf;


/**
 * @brief  HWT101 设备参数定义（继承自 sensor 基类）
 * @note   stBase 必须作为第一个成员，实现 C 语言多态
 */
typedef struct
{
#if SENSOR_IS_ENABLE
    stSensorDeviceTdf        stBase;          // 传感器基类成员（必须作为第一个成员）
#endif
    stHwt101StaticParamTdf   stStaticParam;   // 静态参数（硬件配置）
    stHwt101RunningParamTdf  stRunningParam;  // 运行参数（动态状态）
} stHwt101DeviceParamTdf;


/* 全局设备数组（以 sensor 设备号为索引） */
extern stHwt101DeviceParamTdf gastHwt101DeviceParam[HWT101_DEV_NUM];


/* ==================== 注册函数 ==================== */

/// @brief  注册 HWT101 设备到 sensor 基类
/// @param  emSensorDevNum ：sensor 统一设备号（emSensorHWT101DevNum0/1/2）
/// @param  pstInit        ：初始化参数指针
void vHwt101Register(emSensorDevNumTdf emSensorDevNum, void *pstInit);


/* ==================== 虚方法实现（供 VTable 调用） ==================== */

/// @brief  HWT101 初始化（VTable 实现）
/// @param  pstSensor ：传感器基类指针
void vHwt101Init(void *pstSensor);

/// @brief  HWT101 周期执行（VTable 实现）
/// @param  pstSensor ：传感器基类指针
void vHwt101PeriodExecute(void *pstSensor);

/// @brief  获取 HWT101 当前角度值（VTable 实现）
/// @param  pstSensor ：传感器基类指针
/// @return Q16.16 定点数，单位 °
fix32_t fHwt101GetValue(void *pstSensor);

/// @brief  重置 HWT101（VTable 实现）
/// @param  pstSensor ：传感器基类指针
void vHwt101Reset(void *pstSensor);

/// @brief  设置目标角度（VTable 实现）
/// @param  pstSensor ：传感器基类指针
/// @param  fTarget   ：目标角度值
void vHwt101SetTarget(void *pstSensor, fix32_t fTarget);

/// @brief  获取目标角度（VTable 实现）
/// @param  pstSensor ：传感器基类指针
/// @return 目标角度值
fix32_t fHwt101GetTarget(void *pstSensor);

/// @brief  获取累加角度值（VTable 实现）
/// @param  pstSensor ：传感器基类指针
/// @return Q16.16 定点数，跨圈连续累加，单位 °
fix32_t fHwt101GetAccumulatedValue(void *pstSensor);


/* ==================== 直接访问 API ==================== */

/// @brief  输入接收到的原始字节（供 uart_device 回调调用）
/// @param  emSensorDevNum ：sensor 统一设备号
/// @param  pucData        ：数据指针
/// @param  ulLen          ：数据长度
void vHwt101RxDataInput(emSensorDevNumTdf emSensorDevNum, uint8_t *pucData, uint32_t ulLen);

/// @brief  获取 Z 轴偏航角
/// @param  emSensorDevNum ：sensor 统一设备号
/// @return Q16.16 定点数，单位 °
fix32_t s32Hwt101GetAngleZ(emSensorDevNumTdf emSensorDevNum);

/// @brief  获取 Z 轴角速度
/// @param  emSensorDevNum ：sensor 统一设备号
/// @return Q16.16 定点数，单位 °/s
fix32_t s32Hwt101GetAngularVelZ(emSensorDevNumTdf emSensorDevNum);

/// @brief  获取传感器在线状态
/// @param  emSensorDevNum ：sensor 统一设备号
/// @return 1=在线, 0=离线
uint8_t u8Hwt101IsOnline(emSensorDevNumTdf emSensorDevNum);

/// @brief  获取版本号
/// @param  emSensorDevNum ：sensor 统一设备号
/// @return 版本号
uint16_t u16Hwt101GetVersion(emSensorDevNumTdf emSensorDevNum);

/// @brief  获取数据更新标志并清除
/// @param  emSensorDevNum ：sensor 统一设备号
/// @return HWT101_DATA_GYRO_UPDATE | HWT101_DATA_ANGLE_UPDATE
uint8_t u8Hwt101GetDataFlags(emSensorDevNumTdf emSensorDevNum);

/// @brief  读取传感器寄存器
/// @param  emSensorDevNum ：sensor 统一设备号
/// @param  u8Addr         ：寄存器地址
/// @param  ps16Val        ：输出值指针
/// @return QE_OK 成功，其他值失败
QE_StatusTypeDef emHwt101ReadReg(emSensorDevNumTdf emSensorDevNum, uint8_t u8Addr, int16_t *ps16Val);

/// @brief  写入传感器寄存器
/// @param  emSensorDevNum ：sensor 统一设备号
/// @param  u8Addr         ：寄存器地址
/// @param  s16Data        ：16 位数据
/// @return QE_OK 成功，其他值失败
QE_StatusTypeDef emHwt101WriteReg(emSensorDevNumTdf emSensorDevNum, uint8_t u8Addr, int16_t s16Data);

/// @brief  解锁传感器（向 KEY 寄存器 0x69 写入 0xB588）
/// @param  emSensorDevNum ：sensor 统一设备号
void vHwt101Unlock(emSensorDevNumTdf emSensorDevNum);

/// @brief  保存配置到传感器 Flash
/// @param  emSensorDevNum ：sensor 统一设备号
void vHwt101Save(emSensorDevNumTdf emSensorDevNum);

/// @brief  Z 轴角度归零
/// @param  emSensorDevNum ：sensor 统一设备号
void vHwt101SetZero(emSensorDevNumTdf emSensorDevNum);

/// @brief  启动自动零偏校准
/// @param  emSensorDevNum ：sensor 统一设备号
void vHwt101StartAutoCali(emSensorDevNumTdf emSensorDevNum);

/// @brief  设置输出速率
/// @param  emSensorDevNum ：sensor 统一设备号
/// @param  emRate         ：输出速率
void vHwt101SetOutputRate(emSensorDevNumTdf emSensorDevNum, emHwt101RateTdf emRate);

/// @brief  关闭/打开陀螺仪自动校准
/// @param  emSensorDevNum ：sensor 统一设备号
/// @param  ucEnable       ：0=打开自动校准, 1=关闭自动校准
void vHwt101SetAutoCali(emSensorDevNumTdf emSensorDevNum, uint8_t ucEnable);

/// @brief  手动获取零偏（MANUALCALI）
/// @param  emSensorDevNum ：sensor 统一设备号
/// @param  ucStart        ：1=进入获取零偏, 0=退出获取零偏
void vHwt101ManualCali(emSensorDevNumTdf emSensorDevNum, uint8_t ucStart);


/* ============================================================
 * ======================  使 用 教 程  ========================
 * ============================================================
 *
 * HWT101 继承自 sensor_device 基类，统一使用 emSensorDevNumTdf 管理。
 *
 * ==================== 1. 注册方式 ====================
 *
 *   // 初始化（注册 + 硬件初始化一步完成）
 *   stHwt101StaticParamTdf stHwt = {
 *       .emComMode = emHwt101ComModeI2C,
 *       .pstI2cHandle = &stI2c,
 *       .u8I2cAddr = 0x50,
 *   };
 *   vSensorInit(emSensorHWT101DevNum0, &stHwt);
 *
 * ==================== 2. 多态调用 ====================
 *
 *   // 通过 sensor 统一接口访问（多态）
 *   vSensorPeriodExecute(emSensorHWT101DevNum0);
 *   fix32_t fAngle = fSensorGetValue(emSensorHWT101DevNum0);
 *
 * ==================== 3. 直接访问 ====================
 *
 *   // 也可以直接调用 HWT101 专用函数
 *   fix32_t fAngle = s32Hwt101GetAngleZ(emSensorHWT101DevNum0);
 *   vHwt101SetZero(emSensorHWT101DevNum0);
 */

#endif
#endif
