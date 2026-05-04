/**
  * @file       adc_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      ADC 转换驱动，基于 STM32 HAL 库
  * 
  */

#include "adc_device.h"
#if ADC_DEVICE_IS_ENABLE

/* 全局ADC设备参数数组 */
stAdcDeviceParamTdf astAdcDeviceParam[ADC_DEV_NUM];

#if (QEPACK_PLATFORM == ST)
    ADC_HandleTypeDef *stGetAdcHandle(emAdcDevNumTdf emDevNum)
    {
        if(emDevNum >= ADC_DEV_NUM)
            return NULL;
        return astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase;
    }
#endif

#if !ADC_IS_USE_DMA
    #if (QEPACK_PLATFORM == ST)
        ADC_ChannelConfTypeDef sConfig = {0};
    #endif
#endif

/// @brief      获取ADC设备参数
/// @param      emDevNum   ：设备号
/// @note       只读指针
const stAdcDeviceParamTdf *c_pstGetAdcDeviceParam(emAdcDevNumTdf emDevNum)
{
    if(emDevNum >= ADC_DEV_NUM)
        return NULL;
    return &astAdcDeviceParam[emDevNum];
}


/// @brief      初始化ADC静态参数
/// @param      pstInit    ：静态参数初始化结构体
/// @param      emDevNum   ：设备号
void vAdcDeviceInit(stAdcStaticParamTdf *pstInit, emAdcDevNumTdf emDevNum)
{
    if(emDevNum >= ADC_DEV_NUM || pstInit == NULL)
        return;
    
    memcpy(&astAdcDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stAdcStaticParamTdf));
	memset(&astAdcDeviceParam[emDevNum].stRunningParam, 0, sizeof(stAdcRunningParamTdf));
    
    #if (QEPACK_PLATFORM == TI)
        stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;
    #endif
    stAdcRunningParamTdf *pstRunning = &astAdcDeviceParam[emDevNum].stRunningParam;

    #if (QEPACK_PLATFORM == ST)
        
        // 判断ADC模式
        ADC_HandleTypeDef *hadc = astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase;

        #if ADC_IS_USE_DMA
            DMA_HandleTypeDef *hdma = astAdcDeviceParam[emDevNum].stStaticParam.pstDmaHandle;
        #endif
        
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

    #else
        #if ADC_IS_USE_DMA

            /* Configure DMA source, destination and size */
            DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
                (uint32_t) DL_ADC12_getFIFOAddress(pstStatic->pstAdcBase->adc_inst));

            DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &pstStatic->pulDmaBuffer[0]);

            DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
        #endif

        /*
        * Check the ADC started converting in single channel repeat mode.
        * Once the ADC is triggered, the ADC will keep sampling until 1024
        * samples are captured, regardless of conversion been stopped.
        * 即防止sysconfig中设置成了重复采样模式，因为我们是手动采样的
        */
        if (DL_ADC12_STATUS_CONVERSION_ACTIVE ==
            DL_ADC12_getStatus(ADC12_0_INST)) {
            DL_ADC12_stopConversion(ADC12_0_INST);
        }
        
            
        /* 使能 ADC12_0 的中断请求 */
        NVIC_EnableIRQ(pstStatic->pstAdcBase->adc_irqn);
        
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
        

        if(pstStatic->pulDmaBuffer == NULL || pstStatic->usDmaBufLen == 0)
            return;

        #if (QEPACK_PLATFORM == ST)
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

        #else
            TI_ADC_Start(pstStatic->pstAdcBase);
        #endif
    }

#else

/**
 * @brief 启动单次转换
 * @param emDevNum 设备号
 * @param Channel 通道号
 */
#if (QEPACK_PLATFORM == ST)                   // 对特定通道转换
    void vAdcStart(emAdcDevNumTdf emDevNum, uint32_t Channel)
#else
    void vAdcStart(emAdcDevNumTdf emDevNum, DL_ADC12_MEM_IDX Channel)
#endif
{
    stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;

    #if (QEPACK_PLATFORM == ST)
        sConfig.Channel = Channel;                                         /* 通道 */
        sConfig.Rank = ADC_REGULAR_RANK_1;                              
        sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;                  /* 采样时间 */
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
            Error_Handler();
        }
        HAL_ADC_Start(pstStatic->pstAdcBase);
    #else
        TI_ADC_Start(pstStatic->pstAdcBase);
    #endif
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
    uint16_t* pstADCGetValue(emAdcDevNumTdf emDevNum){
        #if (QEPACK_PLATFORM == TI)
            DL_ADC12_enableConversions(ADC12_0_INST);
            DL_ADC12_disableConversions(ADC12_0_INST);
        #endif
        
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
        #if (QEPACK_PLATFORM == ST)
            ADC_HandleTypeDef* pstAdcBase = astAdcDeviceParam[emDevNum].stStaticParam.pstAdcBase;
            // 停止当前转换
            // HAL_ADC_Stop(pstAdcBase);
            return (uint16_t)HAL_ADC_GetValue(pstAdcBase);
        #else
            stAdcRunningParamTdf *pstRunning = &astAdcDeviceParam[emDevNum].stRunningParam;
            stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;
            
            /* 转换完成，读取转换结果 */
            return (uint16_t)DL_ADC12_getMemResult(
                pstStatic->pstAdcBase->adc_inst,
                pstStatic->pstAdcBase->adc_mem_idx
            );
        #endif
    }
#endif

/**
    函数原型:
    HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
*/
#if (QEPACK_PLATFORM == TI)
    QE_StatusTypeDef TI_ADC_PollForConversion(
        emAdcDevNumTdf emDevNum, uint32_t ulTimeOut
    ){
        unsigned long start, cur;
        
        mspm0_get_clock_ms(&start);
        
        while(emAdcGetDataState(emDevNum) != UPDATED){
            mspm0_get_clock_ms(&cur);
            
            if(cur >= (start + ulTimeOut)) return QE_TIMEOUT;
        }

        return QE_OK;
    }
#endif 

/**
 * @brief 获取ADC转换状态
 * @param emDevNum 设备号
 * @return emAdcDataStateTdf 转换状态
 */
emAdcDataStateTdf emAdcGetDataState(emAdcDevNumTdf emDevNum){
#if ADC_IS_USE_DMA
    emAdcDataStateTdf result = astAdcDeviceParam[emDevNum].stRunningParam.emDataState;

    if(result == UPDATED){
        astAdcDeviceParam[emDevNum].stRunningParam.emDataState = NOT_UPDATE;
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
    #if (QEPACK_PLATFORM == ST)
        return UPDATED; // 为什么要直接返回UPDATED呢...
    #else
        emAdcDataStateTdf result = astAdcDeviceParam[emDevNum].stRunningParam.emDataState;

        if(result == UPDATED){
            astAdcDeviceParam[emDevNum].stRunningParam.emDataState = NOT_UPDATE;
        }
        
        return result;
    #endif
#endif
}


/**
 * @brief ADC DMA传输完成回调函数
 * @param hadc ADC句柄
 */
#if (QEPACK_PLATFORM == ST)
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
#else
    
#endif

#if (QEPACK_PLATFORM == TI)
    emAdcDevNumTdf emCheckCallbackBelong(ADC12_Regs* stReg){
        for(emAdcDevNumTdf i = emAdcDevNum0; i < ADC_DEV_NUM; i++){
            if(astAdcDeviceParam[i].stStaticParam.pstAdcBase->adc_inst == stReg){
                return (emAdcDevNumTdf)i;
            }
        }
        return 0;
    }
    
    /** 以MSPM0G3507为例，只能开两个ADC，故只适配两个对应的回调函数 */
    void ADC0_IRQHandler(void){
        emAdcDevNumTdf emDevNum = emCheckCallbackBelong(ADC0);
        stAdcRunningParamTdf *pstRunning = &astAdcDeviceParam[emDevNum].stRunningParam;
        stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;
        
        switch (DL_ADC12_getPendingInterrupt(ADC0)) {
            case DL_ADC12_IIDX_DMA_DONE:
            case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM4_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM5_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM6_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM7_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM8_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM9_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM10_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM11_RESULT_LOADED:
                DL_ADC12_stopConversion(pstStatic->pstAdcBase->adc_inst);
                DL_ADC12_disableConversions(pstStatic->pstAdcBase->adc_inst);
                pstRunning->emDataState = UPDATED;
                break;
            default:
                break;
        }
    }

    void ADC1_IRQHandler(void){
        emAdcDevNumTdf emDevNum = emCheckCallbackBelong(ADC1);
        stAdcRunningParamTdf *pstRunning = &astAdcDeviceParam[emDevNum].stRunningParam;
        stAdcStaticParamTdf *pstStatic = &astAdcDeviceParam[emDevNum].stStaticParam;

        switch (DL_ADC12_getPendingInterrupt(ADC1)) {
            case DL_ADC12_IIDX_DMA_DONE:
            case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM2_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM4_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM5_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM6_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM7_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM8_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM9_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM10_RESULT_LOADED:
            case DL_ADC12_IIDX_MEM11_RESULT_LOADED:
                DL_ADC12_stopConversion(pstStatic->pstAdcBase->adc_inst);
                DL_ADC12_disableConversions(pstStatic->pstAdcBase->adc_inst);
                pstRunning->emDataState = UPDATED;
                break;
            default:
                break;
        }
    }

#endif

#endif
