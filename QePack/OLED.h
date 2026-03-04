/**
  * @file       oled_device.c
  * @author     Qe_xr,基于江协科技修改
  * @version    V1.0.0
  * @date       2026/1/29
  * @brief      OLED 驱动，基于 STM32 HAL 库
  *
  */
#include "project_config.h"
#if OLED_IS_ENABLE

#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "OLED_Data.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "stdlib.h"
#include "i2c.h"


/// @brief          设备号枚举
///
/// @note 
typedef enum
{
    emOledDevNum0       = 0,
    emOledDevNum1,
    emOledDevNum2,
    emOledDevNum3,
} 
emOledDevNumTdf;

/// @brief          IsFilled参数数值
///
/// @note 			
typedef enum
{
    OLED_UNFILLED   		= 0,
    OLED_FILLED  			= 1,
}
emOledFillStateTdf;

/// @brief          FontSize参数取值
///
/// @note 			此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度
typedef enum
{
    OLED_6X8   			= 6,
    OLED_8X16  			= 8,
}
emOledFontSizeTdf;


/// @brief          静态参数定义
///
/// @note           
typedef struct
{
    #if !OLED_IS_USE_HARDWARE
        GPIO_TypeDef        *pstSclGpioPort;     // SCL GPIO端口
        uint16_t            usSclPin;            // SCL引脚
        GPIO_TypeDef        *pstSdaGpioPort;     // SDA GPIO端口
        uint16_t            usSdaPin;            // SDA引脚
    #else
        I2C_HandleTypeDef 	*hi2c;			     // I2C句柄(若使用硬件I2C)
    #endif
}
stOledStaticParamTdf;

/// @brief          运行参数定义
///
/// @note           
typedef struct
{
    uint8_t             aucDisplayBuf[8][128];  // 显示缓冲区
    
}
stOledRunningParamTdf;

/// @brief          设备参数定义
///
/// @note           
typedef struct
{
    stOledStaticParamTdf    stStaticParam;      // 静态参数
    stOledRunningParamTdf   stRunningParam;     // 运行参数
}
stOledDeviceParamTdf;


/*初始化函数*/
void vOledDeviceInit(stOledStaticParamTdf *pstInit, emOledDevNumTdf emDevNum);

const stOledDeviceParamTdf *c_pstGetOledDeviceParam(emOledDevNumTdf emDevNum);

/*更新函数*/
void vOledUpdateAll(void);
void vOledUpdate(emOledDevNumTdf emDevNum);
void vOledUpdateArea(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/*显存控制函数*/
void vOledClear(emOledDevNumTdf emDevNum);
void vOledClearArea(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);
void vOledReverse(emOledDevNumTdf emDevNum);
void vOledReverseArea(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/*显示函数*/
void vOledShowChar(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, char Char);
void vOledShowString(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, char *String);
void vOledShowNum(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, uint32_t Number, uint8_t Length);
void vOledShowSignedNum(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, int32_t Number, uint8_t Length);
void vOledShowHexNum(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, uint32_t Number, uint8_t Length);
void vOledShowBinNum(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, uint32_t Number, uint8_t Length);
void vOledShowFloatNum(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, double Number, uint8_t IntLength, uint8_t FraLength);
void vOledShowChinese(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, char *Chinese);
void vOledShowImage(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);
void vOledPrintf(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, emOledFontSizeTdf FontSize, char *format, ...);

/*绘图函数*/
uint8_t uOledGetPoint(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y);
void vOledDrawPoint(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y);
void vOledDrawLine(emOledDevNumTdf emDevNum, uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1);
void vOledDrawRectangle(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, emOledFillStateTdf IsFilled);
void vOledDrawTriangle(emOledDevNumTdf emDevNum, uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, emOledFillStateTdf IsFilled);
void vOledDrawCircle(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Radius, emOledFillStateTdf IsFilled);
void vOledDrawEllipse(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t A, uint8_t B, emOledFillStateTdf IsFilled);
void vOledDrawArc(emOledDevNumTdf emDevNum, uint8_t X, uint8_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, emOledFillStateTdf IsFilled);

#endif
#endif
