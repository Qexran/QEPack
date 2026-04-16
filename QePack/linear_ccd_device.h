/** 
 * @file    linear_ccd_device.h
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/03/12
 * @brief   线性CCD设备驱动模块，实现线性CCD的数据采集、阈值处理和中线检测功能。
 */
 
#include "project_config.h"
#if LINEAR_CCD_IS_ENABLE

#ifndef _LINER_CCD_DEVICE_H_
#define _LINER_CCD_DEVICE_H_

#include "string.h"
#include "delay.h"

#include "adc_device.h"

#if (QEPACK_PLATFORM == ST)
    #include "adc.h"
    #include "gpio.h"
#else
    #include "ti_platform.h"
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
    uint8_t ucPixelCount;               // 像素点数（默认128）
    uint8_t ucStartPixel;               // 起始像素（忽略边缘）
    uint8_t ucEndPixel;                 // 结束像素（忽略边缘）
    uint16_t usExposureTimeUs;          // 曝光时间（微秒）

    #if (QEPACK_PLATFORM == ST)
        GPIO_TypeDef        *pstSiGpioPort;     // SI引脚端口
        uint16_t            usSiGpioPin;        // SI引脚号
        GPIO_TypeDef        *pstClkGpioPort;    // CLK引脚端口
        uint16_t            usClkGpioPin;       // CLK引脚号
        emAdcDevNumTdf      emAdcDevNum;        // ADC设备号
    #else
        GPIO_Regs           *pstSiGpioPort;     // SI引脚端口
        uint32_t            usSiGpioPin;        // SI引脚号
        GPIO_Regs           *pstClkGpioPort;    // CLK引脚端口
        uint32_t            usClkGpioPin;       // CLK引脚号
        emAdcDevNumTdf      emAdcDevNum;        // ADC设备号
        DL_ADC12_MEM_IDX    emAdcChannel;       // CCD对应的ADC通道
    #endif
    
} stLinerCcdStaticParamTdf;

/** @brief 线性CCD运行参数定义 */
typedef struct {
    uint16_t ausPixelData[128];         // 像素数据缓存
    uint16_t usMaxValue;                // 当前最大值
    uint16_t usMinValue;                // 当前最小值
    uint16_t usThreshold;               // 动态阈值
    
    int16_t sLeftEdge;                  // 左边界位置
    int16_t sRightEdge;                 // 右边界位置
    int16_t sCenterLine;                // 中线位置
    int16_t sLastCenterLine;            // 上一次中线位置
    
    uint8_t ucDataValid;                // 数据有效标志
    uint8_t ucFrameCount;               // 帧计数

    uint8_t ucCheckADC;                  // ADC转换标志位
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
void vLinerCcdReadData(emLinerCcdDevNumTdf emDevNum);

/* 阈值计算函数 */
void vLinerCcdCalculateThreshold(emLinerCcdDevNumTdf emDevNum);

/* 中线检测函数 */
void vLinerCcdFindCenterLine(emLinerCcdDevNumTdf emDevNum);

/* 获取像素数据 */
const uint16_t *pusLinerCcdGetPixelData(emLinerCcdDevNumTdf emDevNum);

/* 获取中线位置 */
int16_t sLinerCcdGetCenterLine(emLinerCcdDevNumTdf emDevNum);

/* 获取阈值 */
uint16_t usLinerCcdGetThreshold(emLinerCcdDevNumTdf emDevNum);

/* 数据发送到上位机（调试用） */
void vLinerCcdSendToPc(emLinerCcdDevNumTdf emDevNum);

/* 设置ADC转换标志位 */
void vSetAdcConvertFlag(emLinerCcdDevNumTdf emDevNum, uint8_t state);


#endif
#endif
