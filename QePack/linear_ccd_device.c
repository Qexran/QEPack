/** 
 * @file    linear_ccd_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/03/12
 * @brief   线性CCD设备驱动模块实现
 */
#include "linear_ccd_device.h"
#if LINEAR_CCD_IS_ENABLE

volatile uint16_t adcValue;
stLinerCcdDeviceParamTdf astLinerCcdDeviceParam[LINER_CCD_DEV_NUM];

/**
 * @brief 获取线性CCD设备参数
 * @param emDevNum 设备号
 * @return const stLinerCcdDeviceParamTdf* 设备参数指针
 */
const stLinerCcdDeviceParamTdf *c_pstGetLinerCcdDeviceParam(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) {
        return NULL;
    }
    return &astLinerCcdDeviceParam[emDevNum];
}

/**
 * @brief 设置SI引脚电平
 * @param emDevNum 设备号
 * @param state 电平状态（0/1）
 */
static void vLinerCcdSetSi(emLinerCcdDevNumTdf emDevNum, uint8_t state)
{
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    #if (QEPACK_PLATFORM == ST)
        HAL_GPIO_WritePin(pstStatic->pstSiGpioPort, pstStatic->usSiGpioPin, 
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    #else
        TI_GPIO_WritePin(pstStatic->pstSiGpioPort, pstStatic->usSiGpioPin, 
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    #endif
}

/**
 * @brief 设置CLK引脚电平
 * @param emDevNum 设备号
 * @param state 电平状态（0/1）
 */
static void vLinerCcdSetClk(emLinerCcdDevNumTdf emDevNum, uint8_t state)
{
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    #if (QEPACK_PLATFORM == ST)
        HAL_GPIO_WritePin(pstStatic->pstClkGpioPort, pstStatic->usClkGpioPin, 
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    #else
        TI_GPIO_WritePin(pstStatic->pstClkGpioPort, pstStatic->usClkGpioPin, 
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    #endif
}

/**
 * @brief 设置ADC转换标志位
 * @param emDevNum 设备号
 * @param state 标志位状态（0/1）
 */
void vSetAdcConvertFlag(emLinerCcdDevNumTdf emDevNum, uint8_t state)
{
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    pstRunning->ucCheckADC = state;
}

/**
 * @brief 读取ADC值
 * @param emDevNum 设备号
 * @return uint16_t ADC采样值
 */
static uint16_t usLinerCcdReadAdc(emLinerCcdDevNumTdf emDevNum)
{
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    
    #if (QEPACK_PLATFORM == ST)
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = pstStatic->ulAdcChannel;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
        if (HAL_ADC_ConfigChannel(pstStatic->hadc, &sConfig) == HAL_OK) {
            HAL_ADC_Start(pstStatic->hadc);
            HAL_ADC_PollForConversion(pstStatic->hadc, 10);
            adcValue = HAL_ADC_GetValue(pstStatic->hadc);
            HAL_ADC_Stop(pstStatic->hadc);
        }
    #else
        /** TODO.. */
        // vAdcStart(ADC_0);
        #if ADC_IS_USE_DMA
        #else
            vAdcStart(pstStatic->emAdcDevNum, pstStatic->emAdcChannel);
            if(emAdcGetDataState(pstStatic->emAdcDevNum) == UPDATED){
                adcValue = usADCGetValue(pstStatic->emAdcDevNum);
            }
        #endif
    #endif
    
    return adcValue;
}

/**
 * @brief 初始化线性CCD设备
 * @param pstInit 初始化参数指针
 * @param emDevNum 设备号
 */
void vLinerCcdDeviceInit(stLinerCcdStaticParamTdf *pstInit, emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM || pstInit == NULL) {
        return;
    }
    
    memcpy(&astLinerCcdDeviceParam[emDevNum].stStaticParam, 
           pstInit, 
           sizeof(stLinerCcdStaticParamTdf));
    
    memset(&astLinerCcdDeviceParam[emDevNum].stRunningParam, 
           0, 
           sizeof(stLinerCcdRunningParamTdf));
    
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    pstRunning->sCenterLine = 64;
    pstRunning->sLastCenterLine = 64;
    pstRunning->ucDataValid = 0;

}

/**
 * @brief 读取CCD像素数据
 * @param emDevNum 设备号
 */
void vLinerCcdReadData(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return;
    
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    uint8_t ucPixelCount = pstStatic->ucPixelCount;
    if (ucPixelCount > 128) ucPixelCount = 128;
    
    vLinerCcdSetClk(emDevNum, 1);
    vLinerCcdSetSi(emDevNum, 0);
    Delay_us(1);
    
    vLinerCcdSetSi(emDevNum, 1);
    vLinerCcdSetClk(emDevNum, 0);
    Delay_us(1);
    
    vLinerCcdSetClk(emDevNum, 1);
    vLinerCcdSetSi(emDevNum, 0);
    Delay_us(1);
    
    for (uint8_t i = 0; i < ucPixelCount; i++) {
        vLinerCcdSetClk(emDevNum, 0);
        Delay_us(pstStatic->usExposureTimeUs);
        
        pstRunning->ausPixelData[i] = usLinerCcdReadAdc(emDevNum) >> 4;
        
        vLinerCcdSetClk(emDevNum, 1);
        Delay_us(1);
    }
    
    pstRunning->ucDataValid = 1;
    pstRunning->ucFrameCount++;
}

/**
 * @brief 计算动态阈值
 * @param emDevNum 设备号
 */
void vLinerCcdCalculateThreshold(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return;
    
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    uint8_t ucStart = pstStatic->ucStartPixel;
    uint8_t ucEnd = pstStatic->ucEndPixel;
    
    if (ucEnd > 127) ucEnd = 127;
    if (ucStart >= ucEnd) return;
    
    uint16_t usMax = pstRunning->ausPixelData[ucStart];
    uint16_t usMin = pstRunning->ausPixelData[ucStart];
    
    for (uint8_t i = ucStart; i <= ucEnd; i++) {
        if (pstRunning->ausPixelData[i] > usMax) {
            usMax = pstRunning->ausPixelData[i];
        }
        if (pstRunning->ausPixelData[i] < usMin) {
            usMin = pstRunning->ausPixelData[i];
        }
    }
    
    pstRunning->usMaxValue = usMax;
    pstRunning->usMinValue = usMin;
    pstRunning->usThreshold = (usMax + usMin) / 2;
}

/**
 * @brief 检测中线位置
 * @param emDevNum 设备号
 */
void vLinerCcdFindCenterLine(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return;
    
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    uint16_t usThreshold = pstRunning->usThreshold;
    uint8_t ucStart = pstStatic->ucStartPixel;
    uint8_t ucEnd = pstStatic->ucEndPixel;
    
    int16_t sLeft = -1;
    int16_t sRight = -1;
    
    for (uint8_t i = ucStart; i < ucEnd - 5; i++) {
        if (pstRunning->ausPixelData[i] > usThreshold &&
            pstRunning->ausPixelData[i + 1] > usThreshold &&
            pstRunning->ausPixelData[i + 2] > usThreshold &&
            pstRunning->ausPixelData[i + 3] < usThreshold &&
            pstRunning->ausPixelData[i + 4] < usThreshold &&
            pstRunning->ausPixelData[i + 5] < usThreshold) {
            sLeft = i;
            break;
        }
    }
    
    for (int8_t j = ucEnd; j > ucStart + 5; j--) {
        if (pstRunning->ausPixelData[j] < usThreshold &&
            pstRunning->ausPixelData[j + 1] < usThreshold &&
            pstRunning->ausPixelData[j + 2] < usThreshold &&
            pstRunning->ausPixelData[j + 3] > usThreshold &&
            pstRunning->ausPixelData[j + 4] > usThreshold &&
            pstRunning->ausPixelData[j + 5] > usThreshold) {
            sRight = j;
            break;
        }
    }
    
    pstRunning->sLeftEdge = sLeft;
    pstRunning->sRightEdge = sRight;
    
    pstRunning->sLastCenterLine = pstRunning->sCenterLine;
    
    if (sLeft >= 0 && sRight >= 0) {
        pstRunning->sCenterLine = (sLeft + sRight) / 2;
    }
}

/**
 * @brief 获取像素数据
 * @param emDevNum 设备号
 * @return const uint16_t* 像素数据数组指针
 */
const uint16_t *pusLinerCcdGetPixelData(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return NULL;
    return astLinerCcdDeviceParam[emDevNum].stRunningParam.ausPixelData;
}

/**
 * @brief 获取中线位置
 * @param emDevNum 设备号
 * @return int16_t 中线位置
 */
int16_t sLinerCcdGetCenterLine(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return -1;
    return astLinerCcdDeviceParam[emDevNum].stRunningParam.sCenterLine;
}

/**
 * @brief 获取阈值
 * @param emDevNum 设备号
 * @return uint16_t 阈值
 */
uint16_t usLinerCcdGetThreshold(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return 0;
    return astLinerCcdDeviceParam[emDevNum].stRunningParam.usThreshold;
}

/**
 * @brief 发送数据到上位机（调试用）
 * @param emDevNum 设备号
 */
void vLinerCcdSendToPc(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return;
    
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    #if (QEPACK_PLATFORM == ST)
        extern UART_HandleTypeDef huart1;
        uint8_t header = 0xFF;
        HAL_UART_Transmit(&huart1, &header, 1, 100);
        for (uint8_t i = 0; i < 128; i++) {
            uint8_t data = (uint8_t)pstRunning->ausPixelData[i];
            if (data == 0xFF) data--;
            HAL_UART_Transmit(&huart1, &data, 1, 10);
        }
    #endif
}

#endif
