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
#include "project_config.h"
#if ENCODER_IS_ENABLE

#ifndef _ENCODER_DEVICE_H_
#define _ENCODER_DEVICE_H_

#include "string.h"
#include "tim.h"

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
        TIM_HandleTypeDef *pstTimerBase;                // 定时器外设基地址（如TIM2）
    #else // GPIO
        GPIO_TypeDef        *EXTI_GpioPort;             // Encoder1 GPIO端口
        uint16_t            EXTI_Pin;                   // Encoder1 引脚
        GPIO_TypeDef        *Input_GpioPort;            // Encoder2 GPIO端口
        uint16_t            Input_Pin;                  // Encoder2 引脚
    #endif
    
    uint16_t Roto_Ratio;                                // 倍频系数
    
    uint16_t A_Round_Count;                             // 一圈的编码器计数
    
    emEncoderDirTdf Encoder_Dir;                        // 编码器方向
    
    float Wheel_Diameter_mm;                             // 轮子直径（单位：毫米）
    
} stEncoderStaticParamTdf;

/** @brief 编码器运行参数定义 */
typedef struct {
    int32_t TotalPosition;             // 当前位置
    int32_t LastPosition;              // 上一次位置
    float Speed;                       // 当前速度（单位：转/分钟）
    float Distance_mm;                 // 累计路程（单位：毫米）
    //float LastSpeed;                   // 上一次速度
    int32_t times_reach_arr;            // 到达ARR的次数
    uint16_t _1ms_time_count;            // 1ms计数
    int8_t direction_map[2];            // 方向映射表
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
void vEncoderComputeSpeed(emEncoderDevNumTdf emDevNum);

//float fEncoderGetDistance(emEncoderDevNumTdf emDevNum);

/* 获取编码器数据状态 */
//emEncoderDataStateTdf emEncoderGetDataState(emEncoderDevNumTdf emDevNum);

/* 标记编码器数据状态 */
void vEncoderSetDataState(TIM_HandleTypeDef *htim);

#if ENCODER_HANDLE_PLAN // TIM
    void vEncoder_Handler(TIM_HandleTypeDef *htim);
#else
    void vEncoder_Handler(uint16_t GPIO_Pin);
#endif

float fEncoderGetSpeed(emEncoderDevNumTdf emDevNum);

int32_t ulEncoderGetCount(emEncoderDevNumTdf emDevNum);

float fEncoderGetDistance(emEncoderDevNumTdf emDevNum);

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
            float fSpeed = fEncoderGetSpeed(emEncoderDevNum0);
            // 例如：vOledPrintf(OLED0, 1, 16, OLED_8X16, "Speed = %.2f RPM", fSpeed);
        }
    }
*/
