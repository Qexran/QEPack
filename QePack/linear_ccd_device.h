/** 
 * @file    linear_ccd_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/4/18
 * @brief   线性CCD设备驱动模块，实现线性CCD的数据采集、阈值处理和中线检测功能。
 */
 
#ifndef _LINER_CCD_DEVICE_H_
#define _LINER_CCD_DEVICE_H_

#include "project_config.h"
#if LINEAR_CCD_IS_ENABLE

#include "string.h"
#include "delay.h"
#include "arithmetic.h"
#include "stdlib.h"

#include "adc_device.h"

#if (QEPACK_PLATFORM == ST)
    #include "adc.h"
    #include "gpio.h"
#else
    #include "ti_platform.h"
#endif

#if LINER_CCD_IS_DEBUG_MODE
    #include "uart_device.h"
#endif

/** @brief 线性CCD设备号枚举 */
typedef enum {
    emLinerCcdDevNum0 = 0,
    emLinerCcdDevNum1,
    emLinerCcdDevNum2,
    emLinerCcdDevNum3,
} emLinerCcdDevNumTdf;

/** @brief 线性CCD静态参数定义 */
typedef struct {
    #if (QEPACK_PLATFORM == ST)
        GPIO_TypeDef        *pstSiGpioPort;     // SI引脚端口
        uint16_t            usSiGpioPin;        // SI引脚号
        GPIO_TypeDef        *pstClkGpioPort;    // CLK引脚端口
        uint16_t            usClkGpioPin;       // CLK引脚号
        emAdcDevNumTdf      emAdcDevNum;        // ADC设备号
        uint32_t            emAdcChannel;       // CCD对应的ADC通道
        #if LINER_CCD_IS_DEBUG_MODE
            UART_HandleTypeDef   *huart;        // 调试串口句柄
        #endif
    #else
        GPIO_Regs           *pstSiGpioPort;     // SI引脚端口
        uint32_t            usSiGpioPin;        // SI引脚号
        GPIO_Regs           *pstClkGpioPort;    // CLK引脚端口
        uint32_t            usClkGpioPin;       // CLK引脚号
        emAdcDevNumTdf      emAdcDevNum;        // ADC设备号
        DL_ADC12_MEM_IDX    emAdcChannel;       // CCD对应的ADC通道

        #if LINER_CCD_IS_DEBUG_MODE
            stUartTdf       *huart;              // 调试串口句柄
        #endif
    #endif

    
    
} stLinerCcdStaticParamTdf;

/** @brief 线性CCD运行参数定义 */
typedef struct {
    uint16_t ausPixelData[LINER_CCD_PIXEL_COUNT]; // 像素数据缓存
    volatile uint16_t adcValue;         // 当前存储的adc值

    uint16_t usMaxValue;                // 当前最大值
    uint16_t usMinValue;                // 当前最小值
    
    uint16_t usThreshold;               // 黑白阈值
    
    int16_t sLeftEdge;                  // 左边界位置
    int16_t sRightEdge;                 // 右边界位置
    int16_t sCenterLine;                // 中线位置
    int16_t sLastCenterLine;            // 上一次中线位置
    
} stLinerCcdRunningParamTdf;

/** @brief 线性CCD设备参数总结构体 */
typedef struct {
    stLinerCcdStaticParamTdf stStaticParam;     // 静态参数
    stLinerCcdRunningParamTdf stRunningParam;   // 运行参数
} stLinerCcdDeviceParamTdf;

/* 获取线性CCD设备参数 */
const stLinerCcdDeviceParamTdf *c_pstGetLinerCcdDeviceParam(emLinerCcdDevNumTdf emDevNum);

/* 初始化函数 */
void vLinerCcdDeviceInit(stLinerCcdStaticParamTdf *pstInit, emLinerCcdDevNumTdf emDevNum);


/* 数据采集函数 */
QE_StatusTypeDef vLinerCcdReadData(emLinerCcdDevNumTdf emDevNum);
/* 阈值计算函数 */
QE_StatusTypeDef vLinerCcdCalculateThreshold(emLinerCcdDevNumTdf emDevNum);
/* 中线检测函数 */
QE_StatusTypeDef vLinerCcdFindCenterLine(emLinerCcdDevNumTdf emDevNum);

/* 获取像素数据 */
const uint16_t *pusLinerCcdGetPixelData(emLinerCcdDevNumTdf emDevNum);

/* 获取中线位置 */
int16_t sLinerCcdGetCenterLine(emLinerCcdDevNumTdf emDevNum);

/* 获取阈值 */
uint16_t usLinerCcdGetThreshold(emLinerCcdDevNumTdf emDevNum);

/* 数据发送到上位机 */
void vLinerCcdSendToPc(emLinerCcdDevNumTdf emDevNum);

/* ==================== 传感器基类适配 ==================== */
#if SENSOR_IS_ENABLE
    #include "sensor_device.h"
    void vCCDSensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit);
#endif


#endif
#endif

/*
    // 初始化ADC设备
    uint16_t dmabuffer[2];
    stAdcStaticParamTdf stAdcInit = {
        .pstAdcBase = &TI_GET_ADC_STRUCTURE(ADC12_0),
        .ulConversionNumber = 1,
        // .pulDmaBuffer = dmabuffer,
        // .usDmaBufLen = 2
    };

    vAdcDeviceInit(&stAdcInit, ADC_0);

    // 初始化CCD设备
    stLinerCcdStaticParamTdf stCCDInit = {
        .huart = &TI_GET_UART_STRUCTURE(UART_0),
        .emAdcChannel = ADC_CHANNEL_0,
        .emAdcDevNum = ADC_0,
        .pstClkGpioPort = LINER_CCD_PORT,
        .usClkGpioPin = LINER_CCD_CLK_PIN,
        .pstSiGpioPort = LINER_CCD_PORT,
        .usSiGpioPin = LINER_CCD_SI_PIN
    };

    vLinerCcdDeviceInit(&stCCDInit, LINER_CCD0);

    // CCD
    // 1. 从设备采集数据
    if(vLinerCcdReadData(LINER_CCD0) != QE_OK) while(1);
    // 2. 计算黑白阈值
    if(vLinerCcdCalculateThreshold(LINER_CCD0) == QE_OK)
        vOledPrintf(
            OLED0, 1, 1, OLED_8X16, "b/w = %d  ", 
            usLinerCcdGetThreshold(LINER_CCD0)
        );  
    // 3. 计算中线位置
    if(vLinerCcdFindCenterLine(LINER_CCD0) == QE_OK)
        vOledPrintf(
            OLED0, 1, 16, OLED_8X16, "center = %d  ", 
            sLinerCcdGetCenterLine(LINER_CCD0)
        );  
    // 4. 发送到上位机
    vLinerCcdSendToPc(LINER_CCD0);
*/
