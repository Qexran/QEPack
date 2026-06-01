/**
  * @file       ultrasonic_device.h
  * @author     Qe_xr
  * @version    V1.0.1
  * @date       2026/1/20
  * @brief      超声波测量驱动，基于 STM32 HAL 库
  * 
  */
  
#include "project_config.h"
#if ULTRASONIC_IS_ENABLE

#ifndef _ULTRASONIC_DEVICE_H_
#define _ULTRASONIC_DEVICE_H_

#include "string.h"
#include "math.h"
#include "arithmetic.h"

#if (QEPACK_PLATFORM == ST)
    #include "gpio.h"
    #include "tim.h"
#else
    #include "ti_platform.h"
#endif

/**
 * @brief          超声波设备号枚举
 * @note 
 */
typedef enum
{
    emUltrasonicDevNum0        = 0,
    emUltrasonicDevNum1,
    emUltrasonicDevNum2,
    emUltrasonicDevNum3,
} 
emUltrasonicDevNumTdf;

/**
 * @brief          超声波测量状态枚举
 * @note 
 */
typedef enum
{
    emUltrasonicStatus_Idle     = 0,        // 空闲状态
    emUltrasonicStatus_Measuring,           // 测量中
    emUltrasonicStatus_Completed,          // 测量完成
    emUltrasonicStatus_Timeout,            // 测量超时
}
emUltrasonicStatusTdf;


/**
 * @brief          超声波静态参数定义（硬件相关）
 * @note
 */
typedef struct
{
#if (QEPACK_PLATFORM == ST)
    GPIO_TypeDef        *pstTrigGpioBase;  // 触发引脚GPIO端口
    uint16_t            usTrigGpioPin;     // 触发引脚GPIO_PIN_x
    TIM_HandleTypeDef   *pstTimHandle;     // 输入捕获定时器句柄
#else
    GPIO_Regs           *pstTrigGpioBase;  // 触发引脚GPIO端口
    uint32_t            usTrigGpioPin;     // 触发引脚GPIO_PIN_x
    stTimerTdf          *pstTimHandle;     // 输入捕获定时器句柄
#endif
    uint32_t            ulICChannel1;      // 输入捕获通道1（上升沿）
    uint32_t            ulICChannel2;      // 输入捕获通道2（下降沿）
    float               fTimerPeriod;      // 定时器计数周期（秒， 1e-6 代表1us）
}
stUltrasonicStaticParamTdf;

/**
 * @brief          超声波运行参数定义（测量过程相关）
 * @note           
 */
typedef struct
{
    emUltrasonicStatusTdf  emCurrentStatus; // 当前测量状态
    uint8_t                ucIsSuccess;     // 测量是否成功（0-失败 1-成功）
    uint32_t               ulCCR1;          // 通道1捕获值
    uint32_t               ulCCR2;          // 通道2捕获值
    uint32_t               ulTimeoutMs;     // 测量超时时间(ms)
    float                  fDistance;       // 测量距离(m)
    uint32_t               ulExpireTime;    // 超时时间戳（HAL_GetTick()）
	float				   fTemperature;	// 环境温度
}
stUltrasonicRunningParamTdf;

/**
 * @brief          超声波设备参数总结构体
 * @note           
 */
typedef struct
{
    stUltrasonicStaticParamTdf     stStaticParam;  // 静态参数（硬件配置）
    stUltrasonicRunningParamTdf    stRunningParam; // 运行参数（动态状态）
}
stUltrasonicDeviceParamTdf;

/* 获取当前超声波设备参数（只读） */
const stUltrasonicDeviceParamTdf *c_pstGetUltrasonicDeviceParam(emUltrasonicDevNumTdf emDevNum);

/* 基本控制函数 */
void vUltrasonicStartMeasure(emUltrasonicDevNumTdf emDevNum); // 启动单次测量
float fUltrasonicGetDistance(emUltrasonicDevNumTdf emDevNum); // 获取测量距离
uint8_t ucUltrasonicIsMeasureSuccess(emUltrasonicDevNumTdf emDevNum); // 获取测量结果状态

/* 周期执行函数 */
void vUltrasonicDevicePeriodExecute(emUltrasonicDevNumTdf emDevNum);

/* 初始化函数 */
void vUltrasonicDeviceRunningParamInit(stUltrasonicRunningParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum);
void vUltrasonicDeviceInit(stUltrasonicStaticParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum);

/* ==================== 传感器基类适配 ==================== */
#if SENSOR_IS_ENABLE
    #include "sensor_device.h"
    void vUltrasonicSensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit);
#endif

#endif
#endif
/*
  ============================================================================
  CubeMX 配置步骤 (STM32)
  ============================================================================

    STM32F407VET6 定时器总线归属（简明版）
        APB1 总线（PCLK1，最高 42 MHz）
            通用定时器：TIM2、TIM3、TIM4、TIM5
            基本定时器：TIM6、TIM7
            通用定时器：TIM12、TIM13、TIM14
        APB2 总线（PCLK2，最高 84 MHz）
            高级定时器：TIM1、TIM8
            通用定时器：TIM9、TIM10、TIM11
    补充：定时器实际时钟（168 MHz 典型配置）
        APB1 预分频 = 4 → PCLK1 = 42 MHz → TIMx 时钟 = 84 MHz（×2）
        APB2 预分频 = 2 → PCLK2 = 84 MHz → TIMx 时钟 = 168 MHz（×2）

  1. 定时器（输入捕获 — ECHO 引脚）
     - 选择一个定时器，例如 TIM2（32位）或 TIM3（16位）
     - Clock Source: Internal Clock
     - Combined Channels: "Input Capture direct mode" → Channel 1
     - 再点开 "Input Capture indirect mode" → Channel 2
       （CH1 捕获上升沿，CH2 捕获下降沿，共用一个输入引脚）
     - Parameter Settings:
         Prescaler: <APBx_TimerClock / 1000000> - 1
           STM32F4: APB1 Timer = 84MHz  → PSC = 84-1
           STM32F1: APB1 Timer = 72MHz  → PSC = 72-1
           STM32F4: APB2 Timer = 168MHz → PSC = 168-1
         Counter Mode: Up
         Counter Period: 65535  （16位定时器最大值；32位定时器可设 0xFFFFFFFF-1）
         Auto-reload preload: Enable
     - NVIC Settings: TIMx global interrupt → 勾选（库用轮询方式，不会进中断，但 HAL 要求）

  2. TRIG 引脚（GPIO 输出）
     - 选一个空闲 GPIO，例如 PB9，设为 GPIO_Output
     - Parameter Settings:
         GPIO output level: Low
         GPIO mode: Output Push Pull
         GPIO Pull-up/Pull-down: No pull-up and no pull-down
         Maximum output speed: Low

  3. 确认 fTimerPeriod 计算
     fTimerPeriod = 1 / (定时器时钟 / (Prescaler + 1))
     例：84MHz / 84 = 1MHz → fTimerPeriod = 1e-6  (1µs)

  4. project_config.h 开关
     #define ULTRASONIC_IS_ENABLE   1
     #define ULTRASONIC_DEV_NUM     1

  ============================================================================
  SysConfig 配置步骤 (TI MSPM0)
  ============================================================================

  超声波驱动已支持 TI MSPM0 平台，通过 ti_platform.h 中的抽象函数实现跨平台。

  1. 定时器（SysConfig 配置）
     - 添加 GPTIMER 模块，命名为 "TIMER_ULTRASONIC"
     - Timer Mode: Capture
     - Capture CH0: 上升沿（ECHO 引脚）
     - Capture CH1: 下降沿（同一 ECHO 引脚）
     - 计数器频率: 1MHz（预分频使计数周期 = 1us）
     - 使能 Timer interrupt（最低优先级）

  2. TRIG 引脚（SysConfig 配置）
     - 添加 GPIO 模块，命名为 "GPIO_ULTRASONIC"
     - 引脚 PIN_ULTRASONIC_TRIG: 方向 Output，初始 Low

  3. 初始化参数
     - ulICChannel1 = TIM_CHANNEL_0  (CC0 上升沿)
     - ulICChannel2 = TIM_CHANNEL_1  (CC1 下降沿)
       注意: TI 平台使用 0-based 通道号，STM32 使用 1-based

  ============================================================================
  使用示例 (STM32)
  ============================================================================

  void vUltrasonicInit(void)
  {
      stUltrasonicStaticParamTdf stInit = {
          .pstTrigGpioBase = GPIOB,        // TRIG 引脚端口
          .usTrigGpioPin   = GPIO_PIN_9,   // TRIG 引脚编号
          .pstTimHandle    = &htim2,       // 定时器句柄
          .ulICChannel1    = TIM_CHANNEL_1,// 捕获通道1（上升沿）
          .ulICChannel2    = TIM_CHANNEL_2,// 捕获通道2（下降沿）
          .fTimerPeriod    = 1e-6          // 计数器周期 (1us)
      };
      vUltrasonicDeviceInit(&stInit, ULTR0);
  }

  int main(void)
  {
      HAL_Init();
      SystemClock_Config();
      MX_GPIO_Init();
      MX_TIM2_Init();          // 定时器初始化必须在超声波初始化之前
      // ... 其他 MX_xxx_Init() ...

      vUltrasonicInit();

      while (1)
      {
          // 1. 空闲时启动测量
          if (c_pstGetUltrasonicDeviceParam(ULTR0)->stRunningParam.emCurrentStatus
                  != emUltrasonicStatus_Measuring) {
              vUltrasonicStartMeasure(ULTR0);
          }

          // 2. 每轮主循环只调一次（非阻塞，状态机会自动推进）
          vUltrasonicDevicePeriodExecute(ULTR0);

          // 3. 处理结果
          if (ucUltrasonicIsMeasureSuccess(ULTR0)) {
              float fDist = fUltrasonicGetDistance(ULTR0);
              // fDist 单位为米
          } else if (c_pstGetUltrasonicDeviceParam(ULTR0)->stRunningParam.emCurrentStatus
                     == emUltrasonicStatus_Timeout) {
              // 超时处理（无回波）
          }
          // 若状态仍为 Measuring，说明测量还在进行中，下轮循环继续等待

          // 4. 主循环其余工作 ...
          HAL_Delay(50);
      }
  }

  @note 关键注意事项
    - vUltrasonicDevicePeriodExecute 每轮主循环调一次即可（非阻塞），不要用 while 阻塞轮询
    - 默认超时 200ms，请确保主循环周期 < 200ms，否则需调大 ulTimeoutMs
    - SYSTEM_CORE_CLOCK 必须与实际 CPU 频率一致，否则 Delay_us(10) 产生的 TRIG 脉宽不足
    - fTimerPeriod 要与定时器的 Prescaler 设置匹配

  ============================================================================
  使用示例 (TI MSPM0，通过 Sensor 基类多态接口)
  ============================================================================

  #include "QEPack.h"

  void vUltrasonicInit(void)
  {
      stUltrasonicStaticParamTdf stInit = {
          .pstTrigGpioBase = GPIO_ULTRASONIC_INST,         // SysConfig 生成
          .usTrigGpioPin   = DL_GPIO_PIN_0,                // TRIG 引脚
          .pstTimHandle    = &g_stTimerUltrasonic,         // stTimerTdf 结构体
          .ulICChannel1    = TIM_CHANNEL_0,                // CC0（上升沿）
          .ulICChannel2    = TIM_CHANNEL_1,                // CC1（下降沿）
          .fTimerPeriod    = 1e-6                          // 计数器周期 (1us)
      };
      vSensorInit(emSensorUltrasonicDevNum0, &stInit);
  }

  // 主循环中使用 sensor 基类接口
  vSensorPeriodExecute(emSensorUltrasonicDevNum0);
  fix32_t fDist = fSensorGetValue(emSensorUltrasonicDevNum0);  // 单位: 米 (Q16.16)
*/
