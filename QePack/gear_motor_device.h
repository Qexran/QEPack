/**
  * @file       gear_motor_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/26
  * @brief      直流减速电机控制驱动，基于 QePack 编码风格
  * 
  */
  
#include "project_config.h"

#if GEAR_MOTOR_IS_ENABLE

#include "encoder_device.h"
#include "motor_device.h"

#ifndef _GEAR_MOTOR_DEVICE_H_
#define _GEAR_MOTOR_DEVICE_H_

/**
 * @brief          减速电机静态参数结构体
 * @note           
 */
typedef struct
{
    #if (QEPACK_PLATFORM == TI)
		stTimerTdf 		    *stTimer;         // TIM句柄
		DL_TIMER_CC_INDEX 	emChannel;          // PWM通道
        GPIO_Regs           *pstDir1GpioBase;        // 检测编码器方向的 GPIOx
        uint32_t            u32DirPin1;              // 检测编码器方向的 GPIO_PIN_1
        GPIO_Regs           *pstDir2GpioBase;        // 检测编码器方向的 GPIOx
        uint32_t            u32DirPin2;              // 检测编码器方向的 GPIO_PIN_2
	#else
   		TIM_HandleTypeDef *pstPWM_htim;         // 电机PWM使用的定时器
        uint32_t u32PWM_Channel;                // PWM输出通道
        GPIO_TypeDef *pstDir1GpioBase;          // 电机控制引脚1对应的GPIOX
        uint32_t u32DirPin1;                    // 电机方向控制引脚1
        GPIO_TypeDef *pstDir2GpioBase;          // 电机控制引脚2对应的GPIOX
        uint32_t u32DirPin2;                    // 电机方向控制引脚2
    #endif
    
    
    float Tire_R;						    // 轮胎半径
    emEncoderDevNumTdf emEncoderDevNum;     // 编码器设备号
}
stGearMotorStaticParamTdf;

/**
 * @brief          减速电机运行参数结构体
 * @note           
 */
typedef struct
{
    uint8_t i;
}
stGearMotorRunningParamTdf;


/**
 * @brief          减速电机总结构体
 * @note           继承自电机基类
 */
typedef struct
{
    stMotorDeviceTdf            stBase;               // 基类成员（必须作为第一个成员）
    stGearMotorStaticParamTdf   stStaticParam;        // 静态参数（硬件配置）
    stGearMotorRunningParamTdf  stRunningParam;       // 运行参数（动态状态）
}
stGearMotorDeviceParamTdf;

/* 减速电机虚方法实现 */
void vGearMotorInit(void *pstInit);
void vGearMotorPeriodExecute(void *pstMotor);
void vGearMotorSetSpeed(void *pstMotor, int16_t speed);
void vGearMotorStop(void *pstMotor);
void vGearMotorEnable(void *pstMotor, uint8_t bEnable);
emMotorStateTdf emGetGearMotorState(void *pstMotor);

/* 减速电机注册函数 */
void vGearMotorRegister(emMotorDevNumTdf emDevNum, stGearMotorStaticParamTdf *pstInit);

#endif

#endif
