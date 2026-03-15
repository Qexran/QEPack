

#include "led_device.h"
#include "string.h"
#include "test.h"
#include "key_device.h"
#include "uart_device.h"
#include "usart.h"
#include "OLED.h"
#include "i2c.h"
#include "ultrasonic_device.h"
#include "tim.h"
#include "servo_device.h"
#include "adc_device.h"
#include "qmc5883_device.h"


//void vLedInit(){
//	stLedStaticParamTdf		stStaticInit;
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOB;
//	stStaticInit.usGpioPin 		= GPIO_PIN_8;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum0);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOB;
//	stStaticInit.usGpioPin 		= GPIO_PIN_7;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum1);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOB;
//	stStaticInit.usGpioPin 		= GPIO_PIN_6;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum2);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOB;
//	stStaticInit.usGpioPin 		= GPIO_PIN_5;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum3);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOB;
//	stStaticInit.usGpioPin 		= GPIO_PIN_4;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum4);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOB;
//	stStaticInit.usGpioPin 		= GPIO_PIN_3;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum5);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOA;
//	stStaticInit.usGpioPin 		= GPIO_PIN_15;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum6);
//	
//	stStaticInit.emOnLevel 		= emLedOnLevel_High;
//	stStaticInit.pstGpioBase 	= GPIOA;
//	stStaticInit.usGpioPin 		= GPIO_PIN_12;
//	
//	vLedDeviceInit(&stStaticInit, emLedDevNum7);
//	
//	/* running state init */
//	
//	stLedRunningParamTdf	stRunningInit;
//	stRunningInit.emMode = emLedMode_Blink;
//	stRunningInit.ulOnCountThreshold = 20;
//	stRunningInit.ulOffCountThreshold = 80;
//	uint8_t i ;
//	  for(i = 0; i < 8;i++){
//		  vLedDeviceRunningParamInit(&stRunningInit, (emLedDevNumTdf)i);
//		  HAL_Delay(10);
//	  }

//}


void vKeyInit(){
	stKeyStaticParamTdf stKeyStaticInit;
    
	stKeyStaticInit.emValidLevel = emKeyValidLevel_Low;
	stKeyStaticInit.pstGpioBase  = GPIOB;
	stKeyStaticInit.usGpioPin 	 = GPIO_PIN_13;
	stKeyStaticInit.ulDebounceThreshold = 20;
    stKeyStaticInit.ulDoubleClickThreshold = 300;
	stKeyStaticInit.ulLongPressThreshold = 1000;

    
	vKeyDeviceInit(&stKeyStaticInit, KEY0);

	
//	stKeyStaticInit.emValidLevel = emKeyValidLevel_Low;
//	stKeyStaticInit.pstGpioBase  = GPIOB;
//	stKeyStaticInit.usGpioPin 	 = GPIO_PIN_1;
//	stKeyStaticInit.ulDebounceThreshold = 20;
//	stKeyStaticInit.ulLongPressThreshold = 1000;
//	stKeyStaticInit.ulDoubleClickThreshold = 300;
//	
//	vKeyDeviceInit(&stKeyStaticInit, KEY1);

	
//	stKeyStaticInit.emValidLevel = emKeyValidLevel_Low;
//	stKeyStaticInit.pstGpioBase  = GPIOB;
//	stKeyStaticInit.usGpioPin 	 = GPIO_PIN_11;
//	stKeyStaticInit.ulDebounceThreshold = 20;
//	stKeyStaticInit.ulLongPressThreshold = 0;
//	stKeyStaticInit.ulDoubleClickThreshold = 0;
//	
//	vKeyDeviceInit(&stKeyStaticInit, KEY2);
}

//void callBackFcn(emUartDevNumTdf emDevNum, stUartRunningParamTdf* stRunningTdf){
//	// 处理接收到的帧数据
//    vUartPrintf(UART0, "QE_A: RECEIVED!\r\n", stRunningTdf->ulFrameDataCount);
//    vUartSendArray(UART0, stRunningTdf->aucFrameDataBuf, stRunningTdf->ulFrameDataCount);
//}

void vCallBackFcn2(emUartDevNumTdf emDevNum, stUartRunningParamTdf* stRunningTdf){
	// 处理接收到的帧数据
    vUartPrintf(UART1, "QE_B: RECEIVED!\r\n");
	
}

//	vUartPrintf(UART1, "%d \r\n", stRunningTdf->ulFrameDataCount);

//    vUartSendArray(UART1, stRunningTdf->aucFrameDataBuf, stRunningTdf->ulFrameDataCount);
//	
//	
//    // 发送一个固定字符串，而不是帧数据
//    vUartSendArray(UART1, (uint8_t*)"Fixed String\r\n", 12);
//	
////	vOledPrintf(OLED0, 1, 1, OLED_6X8, "%s", stRunningTdf->aucFrameDataBuf);
////	vOledUpdate(OLED0);
//}

void vUartInit(){
//static uint8_t frameHead[] = {0X66, 0X2B};
//static uint8_t frameTail[] = {0X78};

stUartStaticParamTdf stUartStaticInit;
//stUartStaticInit.pstUartHandle	= &huart1;
//stUartStaticInit.pucFrameHead  	= frameHead;
//stUartStaticInit.pucFrameTail  	= frameTail;
//stUartStaticInit.emFrameEn	   	= emUartFrameOn;
//stUartStaticInit.ucFrameHeadLen	= sizeof(frameHead);
//stUartStaticInit.ucFrameTailLen	= sizeof(frameTail);
//stUartStaticInit.ulBaudRate		= 115200;
//stUartStaticInit.vCallbackFcn	= callBackFcn;

//// 初始化静态参数
//vUartDeviceInit(&stUartStaticInit, UART0);

	
	static uint8_t frameHead2[] = {0XAA, 0XFF};
	static uint8_t frameTail2[] = {0X2B};

	stUartStaticInit.pstUartHandle	= &huart2;
	stUartStaticInit.pucFrameHead  	= frameHead2;
	stUartStaticInit.pucFrameTail  	= frameTail2;
	stUartStaticInit.emFrameEn	   	= emUartFrameOn;
	stUartStaticInit.ucFrameHeadLen	= sizeof(frameHead2);
	stUartStaticInit.ucFrameTailLen	= sizeof(frameTail2);
	stUartStaticInit.ulBaudRate		= 115200;
	stUartStaticInit.vCallbackFcn	= vCallBackFcn2;
	
	// 初始化静态参数
    vUartDeviceInit(&stUartStaticInit, UART1);
//	
	
}


void vOLEDInit(){
	stOledStaticParamTdf stOledStaticInitTdf;
//	stOledStaticInitTdf.pstSclGpioPort = GPIOB;
//	stOledStaticInitTdf.pstSdaGpioPort = GPIOB;
//	stOledStaticInitTdf.usSclPin	   = GPIO_PIN_6;
//	stOledStaticInitTdf.usSdaPin	   = GPIO_PIN_7;
    stOledStaticInitTdf.hi2c           = &hi2c1;
	vOledDeviceInit(&stOledStaticInitTdf, OLED0);
	
//	stOledStaticInitTdf.pstSclGpioPort = GPIOB;
//	stOledStaticInitTdf.pstSdaGpioPort = GPIOB;
//	stOledStaticInitTdf.usSclPin	   = GPIO_PIN_14;
//	stOledStaticInitTdf.usSdaPin	   = GPIO_PIN_15;
//	stOledStaticInitTdf.hi2c		   = &hi2c2;
//	vOledDeviceInit(&stOledStaticInitTdf, OLED1);
}


//void vUltrasonicInit()
//{
//    stUltrasonicStaticParamTdf stUltrasonicInit = {
//        .pstTrigGpioBase = GPIOA,        // 触发引脚端口
//        .usTrigGpioPin = GPIO_PIN_8,     // 触发引脚
//        .pstTimHandle = &htim2,          // 定时器句柄
//        .ulICChannel1 = TIM_CHANNEL_1,   // 捕获通道1
//        .ulICChannel2 = TIM_CHANNEL_2    // 捕获通道2
//    };
//    // 初始化超声波设备0
//    vUltrasonicDeviceInit(&stUltrasonicInit, ULTR0);
//}

//void ServoDeviceInit(void)
//{
//    vServoDeviceDefaultInit_Angle(SERVO0, &htim3, TIM_CHANNEL_3);
//}
//uint16_t adc_dma_buffer[2] = {0};
//extern DMA_HandleTypeDef hdma_adc1;
//void vAdcInit(){
//    stAdcStaticParamTdf stAdcStaticInit;
//    stAdcStaticInit.pstAdcBase = &hadc1;
//    
//    //stAdcStaticInit.pstDmaHandle = &hdma_adc1;
//    //stAdcStaticInit.pulDmaBuffer = adc_dma_buffer, // 提前定义的DMA缓存数组
//    //stAdcStaticInit.usDmaBufLen = 2,                // 提前定义的DMA缓存数组
//    vAdcDeviceInit(&stAdcStaticInit, ADC_0);
//    

    
    
//    // 1. 定义通道配置
//    stAdcChannelConfigTdf channelConfig[] = {
//        {ADC_CHANNEL_0, emAdcSamplingTime_239Cycles5},
//        {ADC_CHANNEL_1, emAdcSamplingTime_239Cycles5},
//        {ADC_CHANNEL_2, emAdcSamplingTime_239Cycles5},
//    };

//    // 2. 初始化静态参数
//    stAdcStaticParamTdf staticParam = {
//        .pstAdcBase = &hadc1,
//        .ucChannelCount = 3,
//        .pstChannelConfig = channelConfig,
//        .emTriggerSource = emAdcTrigger_Software,
//        .pstDmaHandle = &hdma_adc1,
//    };

//    vAdcDeviceInit(&staticParam, emAdcDevNum0);

    // 3. 启动DMA转换
    //vAdcStartDma(emAdcDevNum0);
//}


void vHMC5883Init(){

}
