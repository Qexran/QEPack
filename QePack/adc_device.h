/**
  * @file       adc_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/2/17
  * @brief      ADC 转换驱动，基于 STM32 HAL 库
  * 
  */
  

#include "project_config.h"
#if ADC_DEVICE_IS_ENABLE

#ifndef _ADC_DEVICE_H_
#define _ADC_DEVICE_H_

#include "string.h"

#if (QEPACK_PLATFORM == ST)
    #include "adc.h"
#endif

#if ADC_IS_USE_DMA
#endif

/// @brief          ADC设备号枚举
typedef enum
{
    emAdcDevNum0        = 0,
    emAdcDevNum1,           
    emAdcDevNum2,           
    emAdcDevNum3,           
} 
emAdcDevNumTdf;

/// @brief          ADC转换状态枚举
typedef enum
{
   NOT_UPDATE,
   UPDATED,
} 
emAdcDataStateTdf;


/// @brief          ADC静态参数定义
/// @note           包含ADC外设、通道、采样时间、DMA句柄等固定参数
typedef struct
{
    #if (QEPACK_PLATFORM == ST)             // ADC句柄
        ADC_HandleTypeDef       *pstAdcBase; 
    #else
        stAdcTdf                *pstAdcBase;
        uint32_t                ulConversionNumber;             // 扫描通道数
    #endif
    
    #if ADC_IS_USE_DMA
        uint16_t                *pulDmaBuffer;              // DMA缓存地址
        uint16_t                usDmaBufLen;                // DMA缓存长度（计算方式: 通道数 X 采样数）
        #if (QEPACK_PLATFORM == ST)
            DMA_HandleTypeDef       *pstDmaHandle;              // DMA句柄
        #endif
    #endif
}
stAdcStaticParamTdf;

/// @brief          ADC运行参数定义
/// @note           包含转换模式、当前值、DMA缓存等动态参数
typedef struct
{
    uint32_t            ulCurrentValue;                 // 单次转换当前值


    #if (QEPACK_PLATFORM == ST)         /* 这些参数都可以通过cubemx自动推断，故放在running里 */
        FunctionalState     emContinuousState;              // 连续转换状态
        FunctionalState     emDisContinuousState;           // 非连续转换状态
        uint32_t            ulScanConvMode;                 // 扫描模式
        uint32_t            ulConversionNumber;             // 扫描通道数
        uint32_t            ulDataAlign;                    // 对齐模式
        uint32_t            ulDmaInitMode;                  // DMA模式
        emAdcDataStateTdf   ucDmaDataState;                 // DMA数据更新标志
        uint8_t             isContinousRunning;             // 连续模式正在运行
    #else
        volatile emAdcDataStateTdf   emDataState;                    // DMA数据更新标志
        volatile DL_ADC12_MEM_IDX    emCurrentConvertChannel;        // 当前正在转换的通道
    #endif    

}
stAdcRunningParamTdf;

/// @brief/// @note           整合静态参数和运行参数
typedef struct
{
    stAdcStaticParamTdf     stStaticParam;  // 静态参数
    stAdcRunningParamTdf    stRunningParam; // 运行参数
}
stAdcDeviceParamTdf;

/* 获取ADC设备参数 */
const stAdcDeviceParamTdf *c_pstGetAdcDeviceParam(emAdcDevNumTdf emDevNum);
#if ADC_IS_USE_DMA                                                              // 获取原始结果
    uint16_t* pstADCGetValue(emAdcDevNumTdf emDevNum);
#else
    uint16_t  usADCGetValue(emAdcDevNumTdf emDevNum);
#endif

float fADCConvertToResult(                                                   // 对原始结果转换
    #if ADC_IS_USE_DMA
        uint16_t usValue
    #else
        uint32_t usValue
    #endif
);

/* 基本控制函数 */
#if ADC_IS_USE_DMA
    void vAdcStart(emAdcDevNumTdf emDevNum);                                     // 启动DMA连续转换
#else
    #if (QEPACK_PLATFORM == ST)                   // 对特定通道转换
        void vAdcStart(emAdcDevNumTdf emDevNum, uint32_t Channel);
    #else
        void vAdcStart(emAdcDevNumTdf emDevNum, DL_ADC12_MEM_IDX Channel);
    #endif
    
#endif

emAdcDataStateTdf emAdcGetDataState(emAdcDevNumTdf emDevNum);

/* 初始化函数 */
void vAdcDeviceInit(stAdcStaticParamTdf *pstInit, emAdcDevNumTdf emDevNum);                  // 初始化静态参数
void vAdcDeviceRunningParamInit(stAdcRunningParamTdf *pstInit, emAdcDevNumTdf emDevNum);    // 初始化运行参数

#if (QEPACK_PLATFORM == TI)
    TI_StatusTypeDef TI_ADC_PollForConversion(emAdcDevNumTdf emDevNum, uint32_t ulTimeOut);
#else
    ADC_HandleTypeDef *stGetAdcHandle(emAdcDevNumTdf emDevNum); 
#endif

#endif

#endif

/** adc test */
// 待提供功能： TI poll conversion
/**
    Power Down Mode 模式应配置为 Manual，不需要节省能源
    多通道转换时，只需要使能最后一个通道的中断，这样就能知道一次序列转换结束
    采样时间推荐 Desired Sample Time 0 = 62.5 ns
    
    DMA Samples Count ：测一次触发dma后传的个数，几个通道填几个
    使用dma模式时，应该用ADC的中断来触发dma传输，而不是触发到程序中
    所以不需要配置Enable Interrupts，而应配置 Enable DMA Triggers 
    Enable Interrupts 需要配置 DMA done
    Address Mode 选 Fixed addr. to Block addr.
    Source Length / Destination Length 应配置成 Half Word

    Destination Address Direction 终点的地址递增方向选Increment

    Transfer Mode dma传输完成一次（Transfer Size）的时候，block的地址要不要回到开头

    勾选Enable FIFO Mode 的情况：
    DMA Samples Count选6
    Enable DMA Triggers选MEM10 result loaded interrupt
    Source Length / Destination Length 应配置成 Word
*/

    // 1. 初始化参数
//    stAdcStaticParamTdf stAdcStaticInit;
//    stAdcStaticInit.pstAdcBase = &hadc1;
//    
    //stAdcStaticInit.pstDmaHandle = &hdma_adc1;
    //stAdcStaticInit.pulDmaBuffer = adc_dma_buffer, // 提前定义的DMA缓存数组
    //stAdcStaticInit.usDmaBufLen = 2,                // 提前定义的DMA缓存数组

//    vAdcDeviceInit(&stAdcStaticInit, ADC_0);

    // 3. 启动DMA转换
//    vAdcStart(emAdcDevNum0);

    // 4. 主循环中周期执行
//    while(1)
//    {     
//          /* DMA模式开启 */
//            vAdcStart(ADC_0);
          //uint16_t* results = vADCGetValue(ADC_0);
//          vOledPrintf(OLED0, 1, 16, OLED_8X16, "A = %.2f", fADCConvertToResult(results[0]));
//          vOledPrintf(OLED0, 1, 32, OLED_8X16, "B = %.2f", fADCConvertToResult(results[1]));
          
//          /* DMA模式关闭 */
//          for(uint8_t i = 0;i < 2;i++){
//             vAdcStart(ADC_0, i == 0 ? ADC_CHANNEL_0 : ADC_CHANNEL_1);
//             if(emAdcGetDataState(ADC_0) == UPDATED){
//                uint32_t result = usADCGetValue(ADC_0);//
//                vOledPrintf(OLED0, 1, (i) * 16, OLED_8X16, "AD%d = %.2f", i, fADCConvertToResult(result));
//             }
//          }
//    }

