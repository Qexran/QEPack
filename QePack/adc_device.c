/**
  * @file       adc_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      ADC 转换驱动，基于 STM32 HAL 库
  * 
  */
#include "adc_device.h"
#if ADC_MOD_IS_ENABLE

/* 全局ADC设备参数数组 */
stAdcDeviceParamTdf astAdcDeviceParam[ADC_DEV_NUM];

#if !ADC_IS_USE_DMA
    ADC_ChannelConfTypeDef sConfig = {0};
#endif

/// @brief      获取ADC设备参数
/// @param      emDevNum   ：设备号
/// @note       返回值为只读指针，避免参数被非法修改
const stAdcDeviceParamTdf *c_pstGetAdcDeviceParam(emAdcDevNumTdf emDevNum)
{
    if(emDevNum >= ADC_DEV_NUM)
    {
        return NULL; // 设备号越界返回空
    }
    return &astAdcDeviceParam[emDevNum];
}


/// @brief      初始化ADC静态参数
/// @param      pstInit    ：静态参数初始化结构体
/// @param      emDevNum   ：设备号
void vAdcDeviceInit(stAdcStaticParamTdf *pstInit, emAdcDevNumTdf emDevNum)
{
    if(emDevNum >= ADC_DEV_NUM || pstInit == NULL)
    {
        return;
    }
    
    memcpy(&astAdcDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stAdcStaticParamTdf));
	memset(&astAdcDeviceParam[emDevNum].stRunningParam, 0, sizeof(stAdcRunningParamTdf));
    
    // 判断ADC模式
    ADC_HandleTypeDef *hadc = astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase;
    #if ADC_IS_USE_DMA
        DMA_HandleTypeDef *hdma = astAdcDeviceParam[emDevNum].stStaticParam.pstDmaHandle;
    #endif
    
    stAdcRunningParamTdf *pstRunning = &astAdcDeviceParam[emDevNum].stRunningParam;
    
    /* 对齐模式 */
    pstRunning->ulDataAlign = hadc->Init.DataAlign;
    
    /* 是否连续转换 */
    pstRunning->emContinuousState = hadc->Init.ContinuousConvMode;
    pstRunning->emDisContinuousState = hadc->Init.DiscontinuousConvMode;
    
    /* 扫描模式 */
    pstRunning->ulScanConvMode = hadc->Init.ScanConvMode;
    
    /* 扫描通道数 */
    pstRunning->ulConversionNumber = hadc->Init.NbrOfConversion;
    
    #if ADC_IS_USE_DMA
        /* DMA模式 (NORMAL/CIRCULAR) */
        pstRunning->ulDmaInitMode = hdma->Init.Mode;
    #endif
    
//    // 启动DMA转换
//    vAdcStart(emDevNum);
}

/// @brief      启动DMA连续转换
/// @param      emDevNum   ：设备号
/// @note       需提前初始化DMA缓存和缓存长度

#if ADC_IS_USE_DMA
/**
 * @brief 启动DMA连续转换
 * @param emDevNum 设备号
 */
void vAdcStart(emAdcDevNumTdf emDevNum)
{

//    stAdcRunningParamTdf *pstRunning = &astAdcDeviceParam[emDevNum].stRunningParam;
    stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;
    

    if(pstStatic->pulDmaBuffer == NULL || pstStatic->usDmaBufLen == 0 || pstStatic->pstDmaHandle == NULL)
        return;

    
    // 校准
    HAL_ADCEx_Calibration_Start(astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase);

    if (HAL_ADC_Start_DMA(
        astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase,
        (uint32_t *)pstStatic->pulDmaBuffer,
        pstStatic->usDmaBufLen
    ) != HAL_OK)
    {
        Error_Handler();
    }

    
}
#else
/**
 * @brief 启动单次转换
 * @param emDevNum 设备号
 * @param Channel 通道号
 */
void vAdcStart(emAdcDevNumTdf emDevNum, uint32_t Channel){
    stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;
    
    sConfig.Channel = Channel;                                         /* 通道 */
	sConfig.Rank = ADC_REGULAR_RANK_1;                              
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;                  /* 采样时间 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)             
	{
		Error_Handler();
	}
    HAL_ADC_Start(pstStatic->pstAdcBase);
}
#endif

/**
 * @brief 将ADC转换值转换为电压值
 * @param usValue ADC转换值
 * @return float 电压值
 */
float fADCConvertToResult(
    #if ADC_IS_USE_DMA
        uint16_t usValue
    #else
        uint32_t usValue
    #endif
){
    return (float)usValue / ADC_RESOLUTION * ADC_VREF;
}



#if ADC_IS_USE_DMA
    /**
    * @brief 获取DMA转换值
    * @param emDevNum 设备号
    * @return uint16_t* DMA转换值指针
    */
    uint16_t* vADCGetValue(emAdcDevNumTdf emDevNum){
        
            stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;
            return pstStatic->pulDmaBuffer;
       
    }
#else
    /**
    * @brief 获取单次转换值
    * @param emDevNum 设备号
    * @return uint16_t 单次转换值
    */
    uint16_t usADCGetValue(emAdcDevNumTdf emDevNum){
        ADC_HandleTypeDef* pstAdcBase = astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase;
        
        // 停止当前转换
//        HAL_ADC_Stop(pstAdcBase);
        
        return (uint16_t)HAL_ADC_GetValue(pstAdcBase);
    }
#endif

/**
 * @brief 获取ADC转换状态
 * @param emDevNum 设备号
 * @return emAdcDataStateTdf 转换状态
 */
emAdcDataStateTdf emAdcGetDataState(emAdcDevNumTdf emDevNum){
#if ADC_IS_USE_DMA
    emAdcDataStateTdf result;
    result = astAdcDeviceParam[emDevNum].stRunningParam.ucDmaDataState;

    if(result == UPDATED){
        astAdcDeviceParam[emDevNum].stRunningParam.ucDmaDataState = NOT_UPDATE;
    }
    return result;
#else
//  //废弃的方案
//  result = (emAdcDataStateTdf)
//      HAL_IS_BIT_SET(
//          HAL_ADC_GetState(
//              astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase
//          ),
//          HAL_ADC_STATE_REG_EOC
//      );
    return UPDATED;
#endif
}


/**
 * @brief ADC DMA传输完成回调函数
 * @param hadc ADC句柄
 * @note 需要在stm32f1xx_it.c中重定向，或修改为全局回调
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    emAdcDevNumTdf i;
    // 匹配句柄后更新标志
    for(i = emAdcDevNum0; i < ADC_DEV_NUM; i++)
    {
        if(astAdcDeviceParam[i].stStaticParam.pstAdcBase == hadc)
        {
            astAdcDeviceParam[i].stRunningParam.ucDmaDataState = UPDATED;
            break;
        }
    }
}

#endif
