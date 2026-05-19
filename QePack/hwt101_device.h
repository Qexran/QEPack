/**
  * @file       hwt101_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/19
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
  */

#ifndef _HWT101_DEVICE_H_
#define _HWT101_DEVICE_H_

#include "project_config.h"

#if HWT101_IS_ENABLE

#if (QEPACK_PLATFORM == ST)
    #include "stm32f1xx_hal.h"
#else
    #include "ti_platform.h"
#endif

#include "string.h"
#include "uart_device.h"
#include "arithmetic.h"

/// @brief HWT101 设备号枚举
typedef enum
{
    emHwt101DevNum0 = 0,
    emHwt101DevNum1,
} emHwt101DevNumTdf;


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

    /* 状态标志 */
    uint8_t     u8DataFlags;              // HWT101_DATA_GYRO_UPDATE | HWT101_DATA_ANGLE_UPDATE
    uint8_t     u8IsOnline;               // 传感器在线标志

    /* I2C 轮询控制 */
    uint32_t    u32LastPollTick;          // 上次 I2C 轮询时间戳 (ms)
    uint8_t     u8PollPhase;              // 轮询阶段（0=读GZ, 1=读Yaw, 2/3=跳过）

    /* UART 离线检测 */
    uint32_t    u32LastRxTick;            // 上次收到 UART 数据的时间戳 (ms)
} stHwt101RunningParamTdf;


/// @brief HWT101 设备参数定义
typedef struct
{
    stHwt101StaticParamTdf   stStaticParam;
    stHwt101RunningParamTdf  stRunningParam;
} stHwt101DeviceParamTdf;


/* 全局设备数组 */
extern stHwt101DeviceParamTdf gastHwt101DeviceParam[HWT101_DEV_NUM];


/* ==================== 初始化 ==================== */

/// @brief  HWT101 设备初始化（通用入口）
/// @param  pstInit  ：初始化参数指针
/// @param  emDevNum ：设备号
void vHwt101DeviceInit(stHwt101StaticParamTdf *pstInit, emHwt101DevNumTdf emDevNum);


/* ==================== 周期执行 ==================== */
/// @brief  HWT101 周期执行（1ms 主循环中调用）
/// @note   UART 模式：检查离线超时
///         I2C 模式：按分时轮询 GZ/Yaw 寄存器
/// @warning I2C 模式不可在 ISR 中调用（I2C 操作可能阻塞）
/// @param  emDevNum ：设备号
void vHwt101DevicePeriodExecute(emHwt101DevNumTdf emDevNum);


/* ==================== UART 数据输入 ==================== */
/// @brief  输入接收到的原始字节（供 uart_device 回调调用）
/// @param  emDevNum ：设备号
/// @param  pucData  ：数据指针
/// @param  ulLen    ：数据长度
void vHwt101RxDataInput(emHwt101DevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen);


/* ==================== 数据获取 ==================== */

/// @brief  获取 Z 轴偏航角
/// @param  emDevNum ：设备号
/// @return Q16.16 定点数，单位 °
fix32_t s32Hwt101GetAngleZ(emHwt101DevNumTdf emDevNum);

/// @brief  获取 Z 轴角速度
/// @param  emDevNum ：设备号
/// @return Q16.16 定点数，单位 °/s
fix32_t s32Hwt101GetAngularVelZ(emHwt101DevNumTdf emDevNum);

/// @brief  获取传感器在线状态
/// @param  emDevNum ：设备号
/// @return 1=在线, 0=离线
uint8_t u8Hwt101IsOnline(emHwt101DevNumTdf emDevNum);

/// @brief  获取版本号
/// @param  emDevNum ：设备号
/// @return 版本号
uint16_t u16Hwt101GetVersion(emHwt101DevNumTdf emDevNum);

/// @brief  获取数据更新标志并清除
/// @param  emDevNum ：设备号
/// @return HWT101_DATA_GYRO_UPDATE | HWT101_DATA_ANGLE_UPDATE
uint8_t u8Hwt101GetDataFlags(emHwt101DevNumTdf emDevNum);


/* ==================== 寄存器操作 ==================== */

/// @brief  读取传感器寄存器
/// @param  emDevNum ：设备号
/// @param  u8Addr   ：寄存器地址
/// @param  ps16Val  ：输出值指针
/// @return QE_OK 成功，其他值失败
QE_StatusTypeDef emHwt101ReadReg(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t *ps16Val);

/// @brief  写入传感器寄存器
/// @param  emDevNum ：设备号
/// @param  u8Addr   ：寄存器地址
/// @param  s16Data  ：16 位数据
/// @return QE_OK 成功，其他值失败
QE_StatusTypeDef emHwt101WriteReg(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t s16Data);


/* ==================== 控制命令 ==================== */

/// @brief  解锁传感器（向 KEY 寄存器 0x69 写入 0xB588）
/// @note   所有写操作前必须先解锁
/// @param  emDevNum ：设备号
void vHwt101Unlock(emHwt101DevNumTdf emDevNum);

/// @brief  保存配置到传感器 Flash
/// @param  emDevNum ：设备号
void vHwt101Save(emHwt101DevNumTdf emDevNum);

/// @brief  Z 轴角度归零
/// @note   执行后延时约 700ms（内部已含解锁+归零+保存+延时）
/// @param  emDevNum ：设备号
void vHwt101SetZero(emHwt101DevNumTdf emDevNum);

/// @brief  启动自动零偏校准（通过 CALSW 寄存器）
/// @note   传感器需保持静止约 20s，校准完成后手动调用 vHwt101Save 保存到 Flash
/// @param  emDevNum ：设备号
void vHwt101StartAutoCali(emHwt101DevNumTdf emDevNum);

/// @brief  设置输出速率
/// @param  emDevNum ：设备号
/// @param  emRate   ：输出速率
void vHwt101SetOutputRate(emHwt101DevNumTdf emDevNum, emHwt101RateTdf emRate);

/// @brief  关闭/打开陀螺仪自动校准
/// @param  emDevNum ：设备号
/// @param  ucEnable ：0=打开自动校准, 1=关闭自动校准
void vHwt101SetAutoCali(emHwt101DevNumTdf emDevNum, uint8_t ucEnable);

/// @brief  手动获取零偏（MANUALCALI）
/// @param  emDevNum ：设备号
/// @param  ucStart  ：1=进入获取零偏, 0=退出获取零偏
/// @note   进入时务必保持传感器静止，退出时保存配置
void vHwt101ManualCali(emHwt101DevNumTdf emDevNum, uint8_t ucStart);


/* ============================================================
 * ======================  使 用 教 程  ========================
 * ============================================================
 *
 * HWT101 是维特智能 (WitMotion) 的 Z 轴陀螺仪 / 角度传感器模块。
 * 支持 UART (TTL) 和 I2C 两种通信模式，二选一。
 *
 * 数据以 Q16.16 定点数存储，通过 arithmetic.h 提供的 fix32_to_float()
 * 转换为浮点数，或直接用定点数运算。
 *
 * ==================== 1. 模块配置 ====================
 *
 * 在 project_config.h 中启用模块并配置参数：
 *
 *   #define HWT101_IS_ENABLE                    1
 *   #define HWT101_DEV_NUM                      1
 *   #define HWT101_I2C_POLL_INTERVAL_MS         10
 *   #define HWT101_OFFLINE_TIMEOUT_MS           500
 *   #define HWT101_0                            emHwt101DevNum0
 *
 * HWT101_I2C_POLL_INTERVAL_MS — I2C 轮询间隔（仅 I2C 模式），默认 10ms
 * HWT101_OFFLINE_TIMEOUT_MS   — UART 离线超时时间（仅 UART 模式），默认 500ms
 *
 * ==================== 2. UART 模式 ====================
 *
 * 2.1 硬件接线
 *
 *   传感器引脚         MCU 引脚
 *   ──────────────────────────
 *   VCC      →    3.3V / 5V
 *   GND      →    GND
 *   TX       →    MCU RX
 *   RX       →    MCU TX
 *
 * 2.2 初始化
 *
 *   #include "QEPack.h"
 *
 *   void main(void)
 *   {
 *       // ... 系统初始化 ...
 *
 *       // Step 1: 初始化 UART 设备（传感器默认波特率 115200）
 *       stUartStaticParamTdf stUart = {
 *           .emUartDev = emUartDevNum0,
 *           .ulBaudRate = 115200,
 *           // ... 其他引脚配置 ...
 *       };
 *       vUartDeviceInit(&stUart);
 *
 *       // Step 2: 初始化 HWT101
 *       stHwt101StaticParamTdf stHwt = {
 *           .emComMode    = emHwt101ComModeUart,
 *           .emUartDev    = emUartDevNum0,
 *           .pstI2cHandle = NULL,
 *           .u8I2cAddr    = 0,
 *       };
 *       vHwt101DeviceInit(&stHwt, emHwt101DevNum0);
 *
 *       while (1)
 *       {
 *           // ... 主循环 ...
 *       }
 *   }
 *
 * 2.3 UART 模式注意事项
 *
 *   - 传感器上电后自动推送数据包，无需主动查询。
 *   - vHwt101DevicePeriodExecute() 检测数据超时，超过 OFFLINE_TIMEOUT_MS
 *     未收到数据则标记离线。
 *   - 写寄存器前必须先调用 vHwt101Unlock() 解锁。
 *
 * ==================== 3. I2C 模式 ====================
 *
 * 3.1 硬件接线
 *
 *   传感器引脚         MCU 引脚
 *   ──────────────────────────
 *   VCC      →    3.3V / 5V
 *   GND      →    GND
 *   SCL      →    MCU SCL（需 4.7kΩ 上拉）
 *   SDA      →    MCU SDA（需 4.7kΩ 上拉）
 *
 * 3.2 TI 平台初始化

 *   #include "QEPack.h"
 *
 *   stI2CTdf stI2c;
 *
 *   void main(void)
 *   {
 *       // ... 系统初始化 ...
 *
 *       // Step 1: 初始化 I2C（默认地址 0x50）
 *       TI_GET_I2C_STRUCTURE(&stI2c, I2C_0_INST, 0);
 *       vTI_I2C_Init(&stI2c, 400000);  // 400KHz
 *
 *       // Step 2: 初始化 HWT101
 *       stHwt101StaticParamTdf stHwt = {
 *           .emComMode    = emHwt101ComModeI2C,
 *           .emUartDev    = emUartDevNum0,
 *           .pstI2cHandle = &stI2c,
 *           .u8I2cAddr    = 0x50,
 *       };
 *       vHwt101DeviceInit(&stHwt, emHwt101DevNum0);
 *
 *       while (1)
 *       {
 *           vHwt101DevicePeriodExecute(emHwt101DevNum0);
 *           // ... 其他业务逻辑 ...
 *       }
 *   }
 *
 * 3.3 STM32 平台初始化
 *
 *   #include "QEPack.h"
 *
 *   I2C_HandleTypeDef hi2c1;
 *
 *   void main(void)
 *   {
 *       // ... 系统初始化（HAL_I2C_Init 配置 hi2c1）...
 *
 *       stHwt101StaticParamTdf stHwt = {
 *           .emComMode    = emHwt101ComModeI2C,
 *           .emUartDev    = emUartDevNum0,
 *           .pstI2cHandle = &hi2c1,
 *           .u8I2cAddr    = 0x50,
 *       };
 *       vHwt101DeviceInit(&stHwt, emHwt101DevNum0);
 *
 *       while (1)
 *       {
 *           vHwt101DevicePeriodExecute(emHwt101DevNum0);
 *           // ... 其他业务逻辑 ...
 *       }
 *   }
 *
 * 3.4 I2C 模式注意事项
 *
 *   - I2C 模式采用分时轮询：先读 GZ (角速度)，再读 Yaw (角度)，
 *     每次间隔 HWT101_I2C_POLL_INTERVAL_MS ms。
 *   - vHwt101DevicePeriodExecute() 不能放在中断中调用（I2C 操作可能阻塞）。
 *
 * ==================== 4. 周期执行 ====================
 *
 * UART 模式：vHwt101DevicePeriodExecute() 检测离线超时，可放 ISR 中。
 * I2C  模式：vHwt101DevicePeriodExecute() 执行 I2C 轮询读数，放在 main 循环。
 *
 *   // 方式 A: 在 main 循环中调用（UART / I2C 均可）
 *   while (1)
 *   {
 *       vHwt101DevicePeriodExecute(emHwt101DevNum0);
 *   }
 *
 *   // 方式 B: 在 1ms 定时器 ISR 中调用（仅 UART 模式）
 *   void TIMER_IRQHandler(void)
 *   {
 *       vHwt101DevicePeriodExecute(emHwt101DevNum0);
 *   }
 *
 * ==================== 5. 读取数据 ====================
 *
 * 5.1 获取角度和角速度
 *
 *   fix32_t s32Angle = s32Hwt101GetAngleZ(emHwt101DevNum0);
 *   fix32_t s32Vel   = s32Hwt101GetAngularVelZ(emHwt101DevNum0);
 *
 *   // 转为浮点数
 *   float fAngle = fix32_to_float(s32Angle);  // 单位: °
 *   float fVel   = fix32_to_float(s32Vel);    // 单位: °/s
 *
 *   // 或直接用定点数比较 / 运算（不依赖浮点）
 *   if (fix32_abs(s32Angle) > fix32_from_float(90.0f))
 *   {
 *       // 角度超过 90°
 *   }
 *
 * 5.2 检查数据更新
 *
 *   uint8_t u8Flags = u8Hwt101GetDataFlags(emHwt101DevNum0);
 *   if (u8Flags & HWT101_DATA_ANGLE_UPDATE)
 *   {
 *       // 有新的角度数据到达
 *       fix32_t s32Angle = s32Hwt101GetAngleZ(emHwt101DevNum0);
 *   }
 *   if (u8Flags & HWT101_DATA_GYRO_UPDATE)
 *   {
 *       // 有新的角速度数据到达
 *   }
 *
 *   注意：u8Hwt101GetDataFlags() 读取后自动清除标志位。
 *
 * 5.3 检查传感器在线状态
 *
 *   if (u8Hwt101IsOnline(emHwt101DevNum0))
 *   {
 *       // 传感器在线
 *   }
 *   else
 *   {
 *       // 传感器离线，检查接线
 *   }
 *
 * ==================== 6. 常用控制命令 ====================
 *
 * 6.1 Z 轴角度归零（"一键归零"，约 700ms）
 *
 *   vHwt101SetZero(emHwt101DevNum0);
 *   // 内部已包含 解锁 → 写 CALIYAW → 保存，延时约 700ms
 *
 * 6.2 设置输出速率
 *
 *   vHwt101SetOutputRate(emHwt101DevNum0, emHwt101Rate50Hz);  // 50Hz
 *
 * 6.3 自动零偏校准（传感器需静止约 20s，完成后需手动保存）
 *
 *   vHwt101StartAutoCali(emHwt101DevNum0);
 *   HAL_Delay(20000);                      // 等待 20s（TI 平台用 TI_Delay）
 *   vHwt101Save(emHwt101DevNum0);          // 保存校准结果到 Flash
 *
 * 6.4 关闭自动校准（减少运动中漂移）
 *
 *   vHwt101SetAutoCali(emHwt101DevNum0, 1);  // 1 = 关闭自动校准
 *
 * 6.5 手动零偏校准
 *
 *   vHwt101ManualCali(emHwt101DevNum0, 1);  // 进入校准（传感器需保持静止）
 *   HAL_Delay(10000);                       // 等待 10s（TI 平台用 TI_Delay）
 *   vHwt101ManualCali(emHwt101DevNum0, 0);  // 退出校准
 *   vHwt101Save(emHwt101DevNum0);           // 保存
 *
 * ==================== 7. 寄存器读写（高级） ====================
 *
 *   int16_t s16Val;
 *   emHwt101ReadReg(emHwt101DevNum0, 0x2E, &s16Val);  // 读版本号寄存器
 *
 *   vHwt101Unlock(emHwt101DevNum0);
 *   emHwt101WriteReg(emHwt101DevNum0, 0x03, 0x0005);  // 写 RRATE 寄存器
 *   vHwt101Save(emHwt101DevNum0);
 *
 * ==================== 8. 完整示例 (UART 模式) ====================
 *
 *   #include "QEPack.h"
 *
 *   void main(void)
 *   {
 *       // 系统初始化 ...
 *
 *       stUartStaticParamTdf stUart = { ... };
 *       vUartDeviceInit(&stUart);
 *
 *       stHwt101StaticParamTdf stHwt = {
 *           .emComMode = emHwt101ComModeUart,
 *           .emUartDev = emUartDevNum0,
 *       };
 *       vHwt101DeviceInit(&stHwt, emHwt101DevNum0);
 *
 *       // 设置输出速率为 50Hz
 *       vHwt101SetOutputRate(emHwt101DevNum0, emHwt101Rate50Hz);
 *
 *       while (1)
 *       {
 *           vHwt101DevicePeriodExecute(emHwt101DevNum0);
 *
 *           if (u8Hwt101IsOnline(emHwt101DevNum0))
 *           {
 *               uint8_t u8Flags = u8Hwt101GetDataFlags(emHwt101DevNum0);
 *               if (u8Flags & HWT101_DATA_ANGLE_UPDATE)
 *               {
 *                   fix32_t s32Angle = s32Hwt101GetAngleZ(emHwt101DevNum0);
 *                   float fAngle = fix32_to_float(s32Angle);
 *                   // fAngle 即为当前 Z 轴偏航角 (°)
 *               }
 *           }
 *       }
 *   }
 */

#endif
#endif
