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

#define GPIO                0                           // 占位符
#define TIM                 1                           // 占位符
#define TI                  0                           // 占位符
#define ST                  1                           // 占位符

/**
 * @brief 全局状态枚举
 */
typedef enum
{
  QE_OK       = 0x00U,
  QE_ERROR    = 0x01U,
  QE_BUSY     = 0x02U,
  QE_TIMEOUT  = 0x03U,
  QE_IDLE     = 0X04U
} QE_StatusTypeDef;

/* ########################### System Configuration ######################### */
#include                        "qepack_settings.h"

#if (QEPACK_PLATFORM == ST) // 设备头文件
    #include					          "stm32f4xx_hal.h"			    
    #include					          "stm32f4xx_hal_def.h"
#else
    #include                    "ti_msp_dl_config.h"
    #include                    <ti/driverlib/dl_flashctl.h>
    // #include                    "ti_platform.h"
#endif


#if (QEPACK_PLATFORM == ST)
  #define	SYSTEM_CORE_CLOCK	168000000U					// 系统时钟频率
#else
  #define SYSTEM_CORE_CLOCK CPUCLK_FREQ
#endif
/* ########################### Device Section ############################### */

/* LED 相关 */
#define LED_IS_ENABLE									0							// LED 模块开关
#define LED_DEV_NUM                                     1                           // LED 设备数量
#define LED0                                            emLedDevNum0
#define LED1                                            emLedDevNum1
#define LED2                                            emLedDevNum2
#define LED3                                            emLedDevNum3
#define LED4                                            emLedDevNum4
#define LED5                                            emLedDevNum5
#define LED6                                            emLedDevNum6
#define LED7                                            emLedDevNum7


/* KEY 相关 */
#define KEY_IS_ENABLE									1							// 按键 模块开关
#define KEY_DEV_NUM   								    1							// 按键 设备数量
#define KEY0                                            emKeyDevNum0


/* UART 相关 */
#define UART_IS_ENABLE									  1							// UART 模块开关
#define UART_BUF_MAX_LEN    							512             			// UART 收发缓存最大长度
#define UART_FRAME_MAX_LEN  							256             			// UART 帧数据最大长度
#define UART_TX_QUEUE_MAX_LEN 							512  						// UART 发送队列最大长度，可根据需求调整
#define UART_TX_BUF_MAX_LEN 							2048   						// UART vUartPrintf格式化缓冲区长度
#define UART_IS_USE_DMA		  							0             				// UART 是否使用DMA传输（正常模式）
#define UART_DEV_NUM        							4               			// UART 设备数量
#define UART_DEVICE_0                                           emUartDevNum0
#define UART_DEVICE_1                                           emUartDevNum1
#define UART_DEVICE_2                                           emUartDevNum2
#define UART_DEVICE_3                                           emUartDevNum3


/* OLED 相关 */
#define OLED_IS_ENABLE									1							// OLED 模块开关
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
#define SERVO_IS_ENABLE								    1								// 舵机模块总开关
#define SERVO_DEV_NUM        						    1               				// 舵机设备数量
#define SERVO_DEFAULT_PWM_FREQ						    50.0f							// 舵机默认PWM频率(Hz)，常规50Hz
										// 180°角度型舵机默认参数 [使用 vServoDeviceDefaultInit_Angle() ]
#define SERVO_DEFAULT_ANGLE_MIN						    0.0f							// 最小可控角度(°)
#define SERVO_DEFAULT_ANGLE_MAX						    180.0f							// 最大可控角度(°)
									// 270°角度型舵机默认参数 [使用 vServoDeviceDefaultInit_Angle270() ]
#define SERVO_DEFAULT_ANGLE_270_MAX					    270.0f                          // 270°舵机最大可控角度(°)
										// 360°角度型舵机默认参数 [使用 vServoDeviceDefaultInit_Angle360() ]
#define SERVO_DEFAULT_ANGLE_360_MAX					    360.0f                          // 360°舵机最大可控角度(°)
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
#define ADC_DEVICE_IS_ENABLE							0								// ADC 模块总开关
#define ADC_IS_USE_DMA                                  0                               // ADC 是否使用DMA功能
#define ADC_RESOLUTION                                  4095                            // ADC 精度(12位: 2^12 - 1)
#define ADC_VREF                                        3.3                             // ADC 电压
#define ADC_CONVERSION_TIMEOUT_MS                       50                              // ADC 转换超时时间
#define ADC_DEV_NUM        						                  2               				// ADC 模块数量
#define ADC_0                                           emAdcDevNum0

/* 编码器 相关 */
#define ENCODER_IS_ENABLE                               0                               // 编码器 模块总开关
#define ENCODER_HANDLE_FREQ                             50                              // 编码器处理数据的时间 (ms)
#define ENCODER_IS_USE_PARASITISM                       0                               // 使用寄生的定时器处理数据
                            /* tips:对于GPIO模式，应一引脚设置外部中断，一引脚设置输入模式 */
#define ENCODER_HANDLE_PLAN                             GPIO                            // 编码器处理方案 (TIM/GPIO)
#define ENCODER_DEV_NUM    							                1							                  // 编码器 数量
#define ENCODER_0                                       emEncoderDevNum0

/* 线性CCD 相关 */
#define LINEAR_CCD_IS_ENABLE                            0                               // 线性CCD 模块总开关
#define LINER_CCD_DEV_NUM                               1                               // 线性CCD 设备数量
#define LINER_CCD_PIXEL_COUNT                           128                             // 线性CCD 像素数量
#define LINER_CCD_EXPOSURE_TIME                         2                               // 线性CCD 曝光时间
#define LINER_CCD_NEGLECT_THREHOLD                      5                               // 线性CCD 忽略像素阈值
#define LINER_CCD_CENTERLINE_ERROR_THREHOLD             100                             // 线性CCD 中线过偏差忽略阈值
#define LINER_CCD_IS_DEBUG_MODE                         1                               // 线性CCD 调试模式
#define LINER_CCD0                                      emLinerCcdDevNum0

/* 灰度传感器 相关 */
#define GRAY_SENSOR_IS_ENABLE                           0                               // 灰度传感器 模块总开关
#define GRAY_SENSOR_DEV_NUM                             1                               // 灰度传感器 设备数量
#define GRAY_SENSOR0                                    emGraySensorDevNum0
#define GRAY_SENSOR_BACKUP_LENGTH                       2                               // 均值滤波历史长度
#define GRAY_SENSOR_TRACK_THRESHOLD                     5                               // 时间窗口阈值
#define GRAY_SENSOR_DIRECTION                           0                               // 灰度矫正方向

/* PID 相关 */
#define PID_IS_ENABLE                                   1                               // PID 模块总开关
#define PID_DEV_NUM                                     4                               // PID 设备数量
#define PID0                                            emPidDevNum0
#define PID1                                            emPidDevNum1
#define PID2                                            emPidDevNum2
#define PID3                                            emPidDevNum3

/* 电机 相关 */
#define MOTOR_IS_ENABLE                                 1                               // 电机基类总开关
#define MOTOR_DEV_NUM                                   8                               // 电机设备总数

/* 直流减速电机 相关 */
#define GEAR_MOTOR_IS_ENABLE                              0                               // 减速电机 模块总开关
#define GEAR_MOTOR_DEV_NUM                                4                               // 减速电机 设备数量
#define MOTOR_GEAR0                                     emGearMotorDevNum0
#define MOTOR_GEAR1                                     emGearMotorDevNum1
#define MOTOR_GEAR2                                     emGearMotorDevNum2
#define MOTOR_GEAR3                                     emGearMotorDevNum3

/* Emm步进电机 相关 */
#define EMM_MOTOR_IS_ENABLE                             1                               // EMM步进电机 模块总开关
#define EMM_MOTOR_DEV_NUM                               4                               // Emm步进电机 设备数量
#define MOTOR_EMM0                                      emEmmMotorDevNum0
#define MOTOR_EMM1                                      emEmmMotorDevNum1
#define MOTOR_EMM2                                      emEmmMotorDevNum2
#define MOTOR_EMM3                                      emEmmMotorDevNum3

/* 电机系统控制器 相关 */
#define MOTOR_SYSTEM_CONTROLLER_IS_ENABLE                1                               // 电机系统控制器 模块总开关
#define MOTOR_SYSTEM_CONTROLLER_DEV_NUM                  1                               // 电机系统控制器 设备数量
#define MSC0                                             emMotorSystemDevNum0
#define MOTOR_SYSTEM_CONTROLLER_PERIOD_MS                  10                              // 电机系统控制器 周期 (ms)
#define MOTOR_SYSTEM_CONTROLLER_MAX_WHEEL_COUNT          4                               // 最大支持轮子数
#define MOTOR_SYSTEM_CONTROLLER_MAX_ERROR_MM             5.0f                            // 位置控制精度要求 (mm)

#define SENSOR_IS_ENABLE                                1

/* MPU6050 相关 (开发中) */
#define MPU6050_IS_ENABLE                               0                               // MPU6050 模块总开关

/* ATK_MS901M 相关 */
#define ATK_MS901M_IS_ENABLE                            1                               // ATK_MS901M 模块总开关
#define ATK_MS901M_DEV_NUM                              1                               // W25Q64 设备数量
#define ATK_MS901M0                                     emAtkMs901mDevNum0                               // W25Q64 设备数量

/* W25Q64 Flash 相关 */
#define W25Q64_IS_ENABLE                                0                               // W25Q64 模块总开关
#define W25Q64_DEV_NUM                                  1                               // W25Q64 设备数量
#define W25Q64_SECTOR_SIZE                              4096                            // W25Q64 扇区大小（4KB）
#define W25Q64_PAGE_SIZE                                256                             // W25Q64 页大小（256B）
#define W25Q64_TOTAL_SIZE                               (8 * 1024 * 1024)             // W25Q64 总容量（8MB）
#define W25Q640                                         emW25q64DevNum0

/** HC05 蓝牙模块 相关 */
#define HC05_IS_ENABLE                                  1                               // HC05 蓝牙模块总开关
#define HC05_DEV_NUM                                    1                               // HC05 设备数量
#define HC05_RX_BUF_MAX_LEN                             256                             // HC05 接收缓冲区最大长度
#define HC050                                           emHC05DevNum0

/** QMC5883 相关 */
#define QMC5883_IS_ENABLE                               0                               // QMC5883 模块总开关
#define QMC_DEV_NUM                                     1                               // QMC5883 设备数量
#define QMC0                                            emQmcDevNum0


/* 摇杆 相关 */
#define JOYSTICK_IS_ENABLE                              0                               // 摇杆 模块总开关（依赖 ADC + SERVO）
#define JOYSTICK_DEV_NUM                                1                               // 摇杆 设备数量
#define JOYSTICK0                                       emJoystickDevNum0

/* 摇杆 ADC 硬件配置（对应 empty.syscfg 中 ADC12_0 的配置） */
#define JOYSTICK_ADC_DEV                                ADC_0                           // ADC 设备号（ADC12_0, sequence 模式, FIFO + DMA）
#define JOYSTICK_ADC_X_CHANNEL                          0                               // X轴 DMA 缓存索引（MEM0, PA27, ADC12_INPUT_CHAN_0）
#define JOYSTICK_ADC_Y_CHANNEL                          1                               // Y轴 DMA 缓存索引（MEM1, PA26, DL_ADC12_INPUT_CHAN_1）
#define JOYSTICK_ADC_CLK_DIV                            DL_ADC12_CLOCK_DIVIDE_8         // 采样时钟分频（ULPCLK / 8）
#define JOYSTICK_ADC_DMA_TRIGGER                        DL_ADC12_DMA_MEM1_RESULT_LOADED // DMA 触发源：最后一个通道转换完成

/* Timer Controller 相关 */
#define TIMER_CONTROLLER_IS_ENABLE                      1                               // Timer Controller 模块总开关
#define TIMER_CONTROLLER_NUM                            2                               // 定时器对象数量
#define ST_TIMER_CONTROLLER_TICK_TIM                    htim14                           // 1ms 定时器句柄(STM)
/* 若使用TI平台，请在ti_interrupt.c 中传入定时器实例名 */
#define TIMER0                                          emTimerDevNum0
#define TIMER1                                          emTimerDevNum1


/* HWT101 Z轴陀螺仪 相关 */
#define HWT101_IS_ENABLE                               1                               // HWT101 模块总开关
#define HWT101_DEV_NUM                                 1                               // HWT101 设备数量
#define HWT101_I2C_POLL_INTERVAL_MS                    10                              // I2C 轮询间隔(ms)
#define HWT101_OFFLINE_TIMEOUT_MS                      500                             // 离线超时(ms)
#define HWT101_0                                       emSensorHWT101DevNum0

/* 步骤机 相关 */
#define STEP_M_ENABLE                                   1                               // 步骤机 模块总开关
#define STEP_M_MAX_STEP_NUM                             20                              // 最大步骤数量
#define STEP_M_MAX_RULE_NUM                             4                               // 最大规则数量
#define STEP_M_NUM                                      1                               // 定时器对象数量
#define STEP0                                          emStepDevNum0

#endif


