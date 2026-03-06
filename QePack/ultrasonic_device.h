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
    GPIO_TypeDef        *pstTrigGpioBase;  // 触发引脚GPIO端口
    uint16_t            usTrigGpioPin;     // 触发引脚GPIO_PIN_x
    TIM_HandleTypeDef   *pstTimHandle;     // 输入捕获定时器句柄
    uint32_t            ulICChannel1;      // 输入捕获通道1（上升沿）
    uint32_t            ulICChannel2;      // 输入捕获通道2（下降沿）
	float 				fTimerPeriod;      // 定时器计数周期（秒， 1e-6 代表1us）
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

/* 周期执行函数（需放在while循环中） */
void vUltrasonicDevicePeriodExecute(emUltrasonicDevNumTdf emDevNum);

/* 初始化函数 */
void vUltrasonicDeviceRunningParamInit(stUltrasonicRunningParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum);
void vUltrasonicDeviceInit(stUltrasonicStaticParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum);

#endif
#endif
/*
	main.c使用示例

	void vOLEDInit(){
		stOledStaticParamTdf stOledStaticInitTdf;
		stOledStaticInitTdf.pstSclGpioPort = GPIOB;
		stOledStaticInitTdf.pstSdaGpioPort = GPIOB;
		stOledStaticInitTdf.usSclPin	   = GPIO_PIN_9;
		stOledStaticInitTdf.usSdaPin	   = GPIO_PIN_7;
		vOledDeviceInit(&stOledStaticInitTdf, OLED0);
	}

	// 超声波初始化函数
	void vUltrasonicInit()
	{
		stUltrasonicStaticParamTdf stUltrasonicInit = {
			.pstTrigGpioBase = GPIOE,        // 触发引脚端口
			.usTrigGpioPin = GPIO_PIN_8,     // 触发引脚
			.pstTimHandle = &htim1,          // 定时器句柄
			.ulICChannel1 = TIM_CHANNEL_1,   // 捕获通道1
			.ulICChannel2 = TIM_CHANNEL_2,   // 捕获通道2
			.fTimerPeriod = 1e-6			 // 计数器的计数周期,即自增+1所需时间
		};
		// 初始化超声波设备0
		vUltrasonicDeviceInit(&stUltrasonicInit, emUltrasonicDevNum0);
	}


	int main(void)
	{
	  HAL_Init();
	  SystemClock_Config();
	  MX_GPIO_Init();
	  MX_TIM1_Init();
	  MX_I2C2_Init();
	 
	   vOLEDInit();
	   vUltrasonicInit();  // 初始化超声波驱动
	   
	   vOledPrintf(OLED0, 1, 1,  OLED_8X16, "READY", 0);
	   vOledUpdate(OLED0);
	  

	  
	  while (1)
	  {
		// 1. 启动超声波测量
		vUltrasonicStartMeasure(emUltrasonicDevNum0);
		
		// 2. 周期执行测量逻辑（核心）
		vUltrasonicDevicePeriodExecute(emUltrasonicDevNum0);
		
		// 3. 处理测量结果
		if (ucUltrasonicIsMeasureSuccess(emUltrasonicDevNum0))
		{
			float fDistance = fUltrasonicGetDistance(emUltrasonicDevNum0);
			vOledPrintf(OLED0, 1, 1,  OLED_8X16, "result:%f m", fDistance);
		}
		else
		{
			vOledPrintf(OLED0, 1, 1,  OLED_8X16, "Measure Failed", 0);
		}
		vOledUpdate(OLED0);
		
		// 4. 测量间隔（避免频率过高）
		HAL_Delay(100);
	  }
	}

*/
