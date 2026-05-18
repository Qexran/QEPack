/** 
 * @file    linear_ccd_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/4/18
 * @brief   线性CCD设备驱动模块实现
 */
#include "linear_ccd_device.h"
#if LINEAR_CCD_IS_ENABLE

#if (QEPACK_PLATFORM == TI)
#include "ti_msp_dl_config.h"
#endif


stLinerCcdDeviceParamTdf astLinerCcdDeviceParam[LINER_CCD_DEV_NUM];

/**
 * @brief 微秒延时（平台适配）
 */
static void Dly_us(uint32_t us)
{
#if (QEPACK_PLATFORM == TI)
    DL_Common_delayCycles(us * (CPUCLK_FREQ / 1000000UL));
#else
    Delay_us(us);
#endif
}

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
 * @brief 读取ADC值
 * @param emDevNum 设备号
 */
static QE_StatusTypeDef usLinerCcdReadAdc(emLinerCcdDevNumTdf emDevNum)
{
    stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;

    #if (QEPACK_PLATFORM == ST)
        // 无论在普通还是DMA模式下，都应只设置一个通道
        
        const stAdcDeviceParamTdf *stAdc = c_pstGetAdcDeviceParam(pstStatic->emAdcDevNum);
        
        if(stAdc->stRunningParam.ulConversionNumber > 1) return QE_ERROR;

        #if ADC_IS_USE_DMA
            vAdcStart(pstStatic->emAdcDevNum);
        #else
            vAdcStart(pstStatic->emAdcDevNum, pstStatic->emAdcChannel);
        #endif
        
        if(HAL_ADC_PollForConversion(stGetAdcHandle(pstStatic->emAdcDevNum), HAL_MAX_DELAY) == QE_OK){
            #if ADC_IS_USE_DMA
                
                uint16_t *result = pstADCGetValue(pstStatic->emAdcDevNum);
                pstRunning->adcValue = result[0];
            #else
                pstRunning->adcValue = usADCGetValue(pstStatic->emAdcDevNum);
            #endif
        }
    #else
        // 无论在普通还是DMA模式下，都应只设置一个通道
        
        const stAdcDeviceParamTdf *stAdc = c_pstGetAdcDeviceParam(pstStatic->emAdcDevNum);
        
        if(stAdc->stStaticParam.ulConversionNumber > 1) return QE_ERROR;

        #if ADC_IS_USE_DMA
            vAdcStart(pstStatic->emAdcDevNum);
        #else
            vAdcStart(pstStatic->emAdcDevNum, pstStatic->emAdcChannel);
        #endif
        
        if(TI_ADC_PollForConversion(pstStatic->emAdcDevNum, TI_MAX_DELAY) == QE_OK){
            #if ADC_IS_USE_DMA
                
                uint16_t *result = pstADCGetValue(pstStatic->emAdcDevNum);
                pstRunning->adcValue = result[0];
            #else
                pstRunning->adcValue = usADCGetValue(pstStatic->emAdcDevNum);
            #endif
        }
    #endif

    return QE_OK;
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

    pstRunning->sCenterLine = 0;
    pstRunning->sLastCenterLine = 0;

}

/**
 * @brief 读取CCD像素数据
 * @param emDevNum 设备号
 */

QE_StatusTypeDef vLinerCcdReadData(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return QE_ERROR;
    
    #if (QEPACK_PLATFORM == TI)
        stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    #endif
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    vLinerCcdSetClk(emDevNum, 1);
    vLinerCcdSetSi(emDevNum, 0);
    Dly_us(1);
    
    vLinerCcdSetSi(emDevNum, 1);
    vLinerCcdSetClk(emDevNum, 0);
    Dly_us(1);
    
    vLinerCcdSetClk(emDevNum, 1);
    vLinerCcdSetSi(emDevNum, 0);
    Dly_us(1);
    
    for (uint8_t i = 0; i < LINER_CCD_PIXEL_COUNT; i++) {
        vLinerCcdSetClk(emDevNum, 0);
        
        // 调节曝光时间
        Dly_us(LINER_CCD_EXPOSURE_TIME);
        
        usLinerCcdReadAdc(emDevNum);

        pstRunning->ausPixelData[i] = pstRunning->adcValue >> 4;
        
        vLinerCcdSetClk(emDevNum, 1);
        Dly_us(1);
    }
    
    return QE_OK;
}

/**
 * @brief 计算黑白阈值
 * @param emDevNum 设备号
 */
QE_StatusTypeDef vLinerCcdCalculateThreshold(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return QE_ERROR;
    
    #if (QEPACK_PLATFORM == TI)
        stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    #endif
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    uint8_t ucStart = 0 + LINER_CCD_NEGLECT_THREHOLD;
    uint8_t ucEnd = LINER_CCD_PIXEL_COUNT - LINER_CCD_NEGLECT_THREHOLD;;

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
    
    if(usMax == usMin) return QE_ERROR;
    
    pstRunning->usMaxValue = usMax;
    pstRunning->usMinValue = usMin;
    pstRunning->usThreshold = (usMax + usMin) / 2;

    return QE_OK;
}

/**
 * @brief 检测中线位置(单线)
 * @param emDevNum 设备号
 */

QE_StatusTypeDef vLinerCcdFindCenterLine(emLinerCcdDevNumTdf emDevNum)
{
    if (emDevNum >= LINER_CCD_DEV_NUM) return QE_ERROR;
    
    #if (QEPACK_PLATFORM == TI)
        stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
    #endif
    stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
    
    uint16_t usThreshold = pstRunning->usThreshold;
    
    uint8_t ucStart = 0 + LINER_CCD_NEGLECT_THREHOLD;
    uint8_t ucEnd = LINER_CCD_PIXEL_COUNT - LINER_CCD_NEGLECT_THREHOLD;

    int16_t sLeft = -1;
    int16_t sRight = -1;
    
    //寻找左边跳变沿，连续三个白像素后连续三个黑像素判断左边跳变沿
    for (uint8_t i = ucStart; i < ucEnd - 5; i++) {
        if (pstRunning->ausPixelData[i]     > usThreshold &&
            pstRunning->ausPixelData[i + 1] > usThreshold &&
            pstRunning->ausPixelData[i + 2] > usThreshold &&
            pstRunning->ausPixelData[i + 3] < usThreshold &&
            pstRunning->ausPixelData[i + 4] < usThreshold &&
            pstRunning->ausPixelData[i + 5] < usThreshold) {
            sLeft = i;
            break;
        }
    }
    
    //寻找右边跳变沿，连续三个黑像素后连续三个白像素判断左边跳变沿
    for (int16_t j = ucEnd; j > (int16_t)ucStart + 5; j--) {
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
    
    /* 计算中线的偏差,如果太大则取上次的值 */
    #if (QEPACK_PLATFORM == TI)
        if(abs(pstRunning->sLastCenterLine - pstRunning->sCenterLine) >= LINER_CCD_CENTERLINE_ERROR_THREHOLD){
    #else
        if(fabs(pstRunning->sLastCenterLine - pstRunning->sCenterLine) >= LINER_CCD_CENTERLINE_ERROR_THREHOLD){
    #endif
        pstRunning->sCenterLine = pstRunning->sLastCenterLine;  
        return QE_ERROR; 
    }
    return QE_OK;
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
    if (emDevNum >= LINER_CCD_DEV_NUM) return 0;
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

#if LINER_CCD_IS_DEBUG_MODE
    // 仅在测试阶段需要用到SciBuf
    uint8_t SciBuf[200] = {0};

    /**
    * @brief 发送数据到上位机
    * @param emDevNum 设备号
    */
    void vLinerCcdSendToPc(emLinerCcdDevNumTdf emDevNum)
    {
        if (emDevNum >= LINER_CCD_DEV_NUM) return;
        
        #if (QEPACK_PLATFORM == TI)
        stLinerCcdStaticParamTdf *pstStatic = &astLinerCcdDeviceParam[emDevNum].stStaticParam;
        #endif
        stLinerCcdRunningParamTdf *pstRunning = &astLinerCcdDeviceParam[emDevNum].stRunningParam;
        
        SciBuf[0] = 0; 
        SciBuf[1] = 132;
        SciBuf[2] = 0; 
        SciBuf[3] = 0;
        SciBuf[4] = 0;
        SciBuf[5] = 0; 
        for (uint8_t i = 0; i < LINER_CCD_PIXEL_COUNT; i++)
            SciBuf[6 + i] = pstRunning->ausPixelData[i] & 0xFF;

        vUartPrintf(UART_DEVICE_0, "*LD");

        for (uint8_t i = 2; i < 6 + LINER_CCD_PIXEL_COUNT; i++) {
            vUartSendByte(UART_DEVICE_0, ucBinToHexHigh(SciBuf[i]));
            vUartSendByte(UART_DEVICE_0, ucBinToHexLow(SciBuf[i]));
            // __BKPT();
        }

        vUartPrintf(UART_DEVICE_0, "00#");
    }
#endif

#endif
