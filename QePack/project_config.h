/**
  * @file       project_config.h
  * @author     Qe_xr
  * @version    V1.2.0
  * @date       2026/03/04
  * @brief      qepack_ST_HAL库工程配置文件
  *
  */ 
  
#ifndef _PROJECT_CONFIG_H_
#define _PROJECT_CONFIG_H_

/* ########################### System Configuration ######################### */

#include					"stm32f1xx_hal.h"			// 设备头文件
#include					"stm32f1xx_hal_def.h"	    // 设备定义文件
#define PI					3.141592653					// PI值
#define	SYSTEM_CORE_CLOCK	72000000U					// 系统时钟频率

#define GPIO                0                           // 占位符
#define TIM                 1                           // 占位符

/* ########################### Device Section ############################### */

/* LED 相关 */
#define LED_IS_ENABLE									0							// LED 模块开关
#define LED_DEV_NUM                                     1                           // LED 设备数量
#define LED0                                           emLedDevNum0
#define LED1                                           emLedDevNum1
#define LED2                                           emLedDevNum2
#define LED3                                           emLedDevNum3
#define LED4                                           emLedDevNum4
#define LED5                                           emLedDevNum5
#define LED6                                           emLedDevNum6
#define LED7                                           emLedDevNum7


/* KEY 相关 */
#define KEY_IS_ENABLE									0							// 按键 模块开关
#define KEY_DEV_NUM   								    3							// 按键 设备数量
#define KEY0                                           emKeyDevNum0
#define KEY1                                           emKeyDevNum1
#define KEY2                                           emKeyDevNum2


/* UART 相关 */
#define UART_IS_ENABLE									0							// UART 模块开关
#define UART_DEV_NUM        							2               			// UART 设备数量
#define UART_BUF_MAX_LEN    							256             			// UART 收发缓存最大长度
#define UART_FRAME_MAX_LEN  							128             			// UART 帧数据最大长度
#define UART_TX_QUEUE_MAX_LEN 							256  						// UART 发送队列最大长度，可根据需求调整
#define UART_TX_BUF_MAX_LEN 							512   						// UART vUartPrintf格式化缓冲区长度
#define UART_IS_USE_DMA		  							1             				// UART 是否使用DMA传输（正常模式）
#define UART0                                           emUartDevNum0
#define UART1                                           emUartDevNum1


/* OLED 相关 */
#define OLED_IS_ENABLE									0							// OLED 模块开关
#define OLED_DEV_NUM    								1							// OLED 设备数量
#define OLED_IS_USE_HARDWARE    						1							// OLED 是否使用硬件I2C
#define OLED0                                           emOledDevNum0

/* 超声波测距 相关 */
#define ULTRASONIC_IS_ENABLE							0							    // 超声波 模块开关
#define ULTRASONIC_DEV_NUM        						1               			    // 超声波 设备数量
#define ULTRASONIC_DEFAULT_ENV_TEMP 					25.0f						    // 超声波 环境默认温度	(若有温度传感器时应给运行参数赋值)
#define ULTRASONIC_SOUND_SPEED_BASE 					331.3f						    // 超声波 声速基准值 	(0°C 时的声速, m/s)
#define ULTRASONIC_SOUND_SPEED_TEMP_COEF 				0.606f						    // 超声波 温度系数 	(声速随温度变化率, m/s/°C)
#define ULTR0                                   		emUltrasonicDevNum0		


/* 舵机 相关 */
#define SERVO_IS_ENABLE								    0								// 舵机模块总开关
#define SERVO_DEV_NUM        						    1               				// 舵机设备数量
#define SERVO_DEFAULT_PWM_FREQ						    50.0f							// 舵机默认PWM频率(Hz)，常规50Hz
										// 180°角度型舵机默认参数 [使用 vServoDeviceDefaultInit_Angle() ]
#define SERVO_DEFAULT_ANGLE_MIN						    0.0f							// 最小可控角度(°)
#define SERVO_DEFAULT_ANGLE_MAX						    180.0f							// 最大可控角度(°)
#define SERVO_DEFAULT_PULSE_MIN						    500.0f							// 最小脉冲宽度(us)
#define SERVO_DEFAULT_PULSE_MAX						    2500.0f							// 最大脉冲宽度(us)
										// 360°连续旋转型舵机默认参数 [使用 vServoDeviceDefaultInit_360() ]
#define SERVO_360_PULSE_MID							    1500.0f      					// 360°舵机停转脉冲(us)
#define SERVO_360_SPEED_MIN							    -100.0f      					// 360°舵机最小速度(-100~0，反转)
#define SERVO_360_SPEED_MAX							    100.0f       					// 360°舵机最大速度(0~100，正转)
										// 通用默认参数
#define SERVO_DEFAULT_SPEED							    1.0f							// 平滑调速速度(°/ms 或 速度值/ms)
#define SERVO0                                          emServoDevNum0
#define SERVO1                                          emServoDevNum1

/* ADC 相关 */
#define ADC_MOD_IS_ENABLE							    0								// ADC 模块总开关
#define ADC_IS_USE_DMA                                  1                               // ADC 是否使用DMA功能
#define ADC_RESOLUTION                                  4095                            // ADC 精度(12位: 2^12 - 1)
#define ADC_VREF                                        3.3                             // ADC 电压
#define ADC_CONVERSION_TIMEOUT_MS                       50                              // ADC 转换超时时间
#define ADC_DEV_NUM        						        2               				// ADC 模块数量
#define ADC_0                                           emAdcDevNum0

/* 编码器 相关 */
#define ENCODER_IS_ENABLE                               0                               // 编码器 模块总开关
#define ENCODER_HANDLE_FREQ                             50                              // 编码器处理数据的时间 (ms)
                            /* tips:对于GPIO模式，应一引脚设置外部中断，一引脚设置输入模式 */
#define ENCODER_HANDLE_PLAN                             TIM                             // 编码器处理方案 (TIM/GPIO)
#define ENCODER_COMPUTE_IT_TIM                          htim1                           // 处理编码器数据的定时器 (1ms中断)
#define ENCODER_DEV_NUM    								              1							                  // 编码器 数量
#define ENCODER_0                                       emEncoderDevNum0

/* PID 相关 */
#define PID_IS_ENABLE                                   0                               // PID 模块总开关
#define PID_DEV_NUM                                     2                               // PID 设备数量
#define PID0                                            emPidDevNum0
#define PID1                                            emPidDevNum1

/* 电机 相关 */
#define MOTOR_IS_ENABLE                                 0                               // 电机 模块总开关
#define MOTOR_DEV_NUM                                   1                               // 电机 设备数量
#define MOTOR0                                          emMotorDevNum0

/* MPU6050 相关 (开发中) */
#define MPU6050_IS_ENABLE                               0                               // MPU6050 模块总开关

/* ATK_MS901M 相关 */
#define ATK_MS901M_IS_ENABLE                            0                               // ATK_MS901M 模块总开关

/* EMM_V5 相关 (待测试) */
#define EMM_V5_IS_ENABLE                                0                               // EMM_V5 模块总开关

/* W25Q64 相关 */
#define W25Q64_IS_ENABLE                                0                               // W25Q64 模块总开关
#define W25Q64_DEV_NUM                                  1                               // W25Q64 设备数量
#define W25Q64_SECTOR_SIZE                              4096                            // W25Q64 扇区大小（4KB）
#define W25Q64_PAGE_SIZE                                256                             // W25Q64 页大小（256B）
#define W25Q64_TOTAL_SIZE                               (8 * 1024 * 1024)               // W25Q64 总容量（8MB）
#define W25Q640                                         emW25q64DevNum0

#endif



