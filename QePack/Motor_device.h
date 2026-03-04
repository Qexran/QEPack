/**
  * @file       motor_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/26
  * @brief      电机控制驱动，基于 STM32 HAL 库
  * 
  */
  
#include "project_config.h"
#include "encoder_device.h"
#if MOTOR_IS_ENABLE

#ifndef _MOTOR_DEVICE_H_
#define _MOTOR_DEVICE_H_

/// @brief          电机设备号枚举
/// @note 
typedef enum
{
    emMotorDevNum0        = 0,
    emMotorDevNum1,
    emMotorDevNum2,
    emMotorDevNum3,
} 
emMotorDevNumTdf;


/// @brief          电机静态参数结构体
/// @note           
typedef struct
{
    TIM_HandleTypeDef *pstPWM_htim;         // 电机PWM使用的定时器
    uint32_t u32PWM_Channel;                // PWM输出通道
    GPIO_TypeDef *pstDir1GpioBase;          // 电机控制引脚1对应的GPIOX
    uint32_t u32DirPin1;                    // 电机方向控制引脚1
    GPIO_TypeDef *pstDir2GpioBase;          // 电机控制引脚2对应的GPIOX
    uint32_t u32DirPin2;                    // 电机方向控制引脚2
    float Tire_R;						    // 轮胎半径
    emEncoderDevNumTdf emEncoderDevNum;     // 编码器设备号
}
stMotorStaticParamTdf;

/// @brief          电机运行参数结构体
/// @note           
typedef struct
{
    uint8_t i;
}
stMotorRunningParamTdf;


/// @brief          电机总结构体
/// @note           
typedef struct
{
    stMotorStaticParamTdf     stStaticParam;  // 静态参数（硬件配置）
    stMotorRunningParamTdf    stRunningParam; // 运行参数（动态状态）
}
stMotorDeviceParamTdf;

void vMotorDeviceInit(stMotorStaticParamTdf *pstInit, emMotorDevNumTdf emDevNum);
void vMotorSetSpeed_by_PWM(emMotorDevNumTdf emDevNum, int16_t speed);

#endif

#endif
