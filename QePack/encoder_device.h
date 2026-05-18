/** 
 * @file    encoder_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/2/21
 * @brief   编码器驱动模块，基于 STM32 HAL 库
 * 
 * 本模块基于博客 2 (weixin_44567668) 的设计思路，实现了编码器位置和速度的测量，
 * 包括溢出处理、软件滤波等功能。
 */
#ifndef _ENCODER_DEVICE_H_
#define _ENCODER_DEVICE_H_

#include "project_config.h"
#if ENCODER_IS_ENABLE

#include "string.h"
#include "arithmetic.h"

#if (QEPACK_PLATFORM == ST)
    #include "tim.h"
#else
    #if (ENCODER_IS_USE_PARASITISM)
        #include "timer_controller.h"
    #endif
    #include "ti_platform.h"
#endif

/** @brief 编码器设备号枚举 */
typedef enum {
    emEncoderDevNum0 = 0,  // 编码器0
    emEncoderDevNum1,      // 编码器1
    emEncoderDevNum2,      // 编码器2
    emEncoderDevNum3,      // 编码器3
} emEncoderDevNumTdf;

/** @brief 编码器方向枚举 */
typedef enum {
    emEncoderLow,
    emEncoderHigh,
} emEncoderDirTdf;

/** @brief 编码器静态参数定义 */
typedef struct {
    #if ENCODER_HANDLE_PLAN // TIM

        #if (QEPACK_PLATFORM == TI)                     // 编码器定时器句柄
            stTimerTdf *pstTimerBase;
        #else
            TIM_HandleTypeDef   *pstTimerBase;
        #endif

    #else // GPIO

        #if (QEPACK_PLATFORM == TI )
            stTimerTdf          *pstCompareTimerBase;      // 比较定时器句柄
            GPIO_Regs           *pstDirGpioBase;           // 检测编码器方向的 GPIOx
            uint32_t            usDirGpioPin;              // 检测编码器方向的 GPIO_PIN_x
            uint8_t             ucNumberofEdgesToDetect;   // 比较定时器每次触发中断需要触发边缘的次数
        #else
            GPIO_TypeDef        *EXTI_GpioPort; 
            uint16_t            EXTI_Pin;       
            GPIO_TypeDef        *Input_GpioPort;
            uint16_t            Input_Pin;      
        #endif

    #endif
    
    uint16_t Roto_Ratio;                                // 倍频系数

    uint16_t A_Round_Count;                             // 一圈的编码器计数

    fix32_t fGearRatio;                                  // 电机减速比（输出轴转速 = 编码器转速 / 减速比）

    emEncoderDirTdf Encoder_Dir;                        // 编码器方向
    
    fix32_t fWheelDiameterMm;                            // 轮子直径（单位：毫米）

    #if !ENCODER_IS_USE_PARASITISM                      // 用于处理数据的定时器句柄
        #if (QEPACK_PLATFORM == TI)
            stTimerTdf *pstHandleTimerBase;
        #else
            TIM_HandleTypeDef *pstHandleTimerBase; 
        #endif
    #endif
    
} stEncoderStaticParamTdf;

/** @brief 编码器运行参数定义 */
typedef struct {
    int32_t TotalPosition;             // 当前位置
    int32_t LastPosition;              // 上一次位置
    fix32_t fSpeed;                    // 当前速度（单位：转/分钟）
    fix32_t fDistanceMm;               // 累计路程（单位：毫米）
    //float LastSpeed;                   // 上一次速度
    int32_t times_reach;            // 到达ARR的次数
    uint16_t _1ms_time_count;            // 1ms计数
    int8_t direction_map[2];            // 方向映射表
    int8_t intEncoderCompareCurrentDir;   // 编码器比较当前的方向

    GPIO_PinState emCurrentPinState;    // 当前读取到的电平状态
    uint8_t isDoneAInterrupt;           // 是否完成一次中断触发
} stEncoderRunningParamTdf;

/** @brief 编码器设备参数总结构体 */
typedef struct {
    stEncoderStaticParamTdf stStaticParam; // 静态参数
    stEncoderRunningParamTdf stRunningParam; // 运行参数
} stEncoderDeviceParamTdf;

/* 获取编码器设备参数 */
const stEncoderDeviceParamTdf *c_pstGetEncoderDeviceParam(emEncoderDevNumTdf emDevNum);

/* 初始化函数 */
void vEncoderDeviceInit(stEncoderStaticParamTdf *pstInit, emEncoderDevNumTdf emDevNum);

/* 启动编码器 */
void vEncoderStart(emEncoderDevNumTdf emDevNum);

/* 启动定时中断（用于计算速度） */
void vEncoderStartTimer(emEncoderDevNumTdf emDevNum, uint32_t u32Period);

/* 获取编码器速度 */
#if (QEPACK_PLATFORM == TI)
    void vEncoderComputeSpeed(emEncoderDevNumTdf emDevNum);
#else
    void vEncoderComputeSpeed(TIM_HandleTypeDef *htim);
#endif

//fix32_t fEncoderGetDistance(emEncoderDevNumTdf emDevNum);

/* 获取编码器数据状态 */
//emEncoderDataStateTdf emEncoderGetDataState(emEncoderDevNumTdf emDevNum);

/* 标记编码器数据状态 (该函数被废弃)*/
// void vEncoderSetDataState(TIM_HandleTypeDef *htim);



#if (ENCODER_HANDLE_PLAN == TIM) // TIM
    void vEncoder_Handler(TIM_HandleTypeDef *htim);
#else
    #if (QEPACK_PLATFORM == TI)
        void vEncoder_Handler(emEncoderDevNumTdf emDevNum);
        GPIO_PinState ReadPin(emEncoderDevNumTdf emDevNum);
    #else
        void vEncoder_Handler(uint16_t GPIO_Pin);
    #endif
#endif

fix32_t fEncoderGetSpeed(emEncoderDevNumTdf emDevNum);

int32_t ulEncoderGetCount(emEncoderDevNumTdf emDevNum);

fix32_t fEncoderGetDistance(emEncoderDevNumTdf emDevNum);

void vEncoderResetDistance(emEncoderDevNumTdf emDevNum);

#endif

#endif // _ENCODER_DEVICE_

/*
    // 初始化编码器参数
    stEncoderStaticParamTdf stEncoderStaticInit;
    stEncoderStaticInit.pstTimerBase = &htim2;  // TIM2编码器
    stEncoderStaticInit.ulEncoderPulseCount = 1024;  // 每圈1024脉冲
    stEncoderStaticInit.ulGearRatio = 1;  // 无减速比

    // 初始化编码器设备
    vEncoderDeviceInit(&stEncoderStaticInit, emEncoderDevNum0);

    // 在主循环中获取数据
    while (1) {
        if (emEncoderGetDataState(emEncoderDevNum0) == UPDATED) {
            fix32_t fSpeed = fEncoderGetSpeed(emEncoderDevNum0);
            // 例如：vOledPrintf(OLED0, 1, 16, OLED_8X16, "Speed = %.2f RPM", fSpeed);
        }
    }
*/
