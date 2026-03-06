/** 
 * @file    encoder_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/2/21
 * @brief   编码器驱动模块，基于 STM32 HAL 库
 */
#include "encoder_device.h"
#if ENCODER_IS_ENABLE

stEncoderDeviceParamTdf astEncoderDeviceParam[ENCODER_DEV_NUM];

/**
 * @brief 获取编码器设备参数
 * @param emDevNum 编码器设备号
 * @return const stEncoderDeviceParamTdf* 编码器设备参数指针
 */
const stEncoderDeviceParamTdf *c_pstGetEncoderDeviceParam(emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM) {
        return NULL; // 设备号越界返回空
    }
    return &astEncoderDeviceParam[emDevNum];
}

/**
 * @brief 中位值平均滤波
 * @param fValue 输入值
 * @param afBuffer 缓冲区指针
 * @param u8BufferSize 缓冲区大小
 * @return float 中位值平均滤波后的值
 */
float fMedianFilter(float fValue, float* afBuffer, uint8_t u8BufferSize)
{
    // 将新值加入缓冲区
    for (int i = u8BufferSize - 1; i > 0; i--) {
        afBuffer[i] = afBuffer[i - 1];
    }
    afBuffer[0] = fValue;
    
    // 排序缓冲区
    for (int i = 0; i < u8BufferSize; i++) {
        for (int j = i + 1; j < u8BufferSize; j++) {
            if (afBuffer[i] > afBuffer[j]) {
                float temp = afBuffer[i];
                afBuffer[i] = afBuffer[j];
                afBuffer[j] = temp;
            }
        }
    }
    
    // 返回中间值
    return afBuffer[u8BufferSize / 2];
}

/**
 * @brief       获取编码器的值
 * @param       emDevNum 设备代号
 * @retval      编码器值
 * @note        使用补码的原理解决反转溢出问题。
 */
static int32_t lEncoderGetEncoder(emEncoderDevNumTdf emDevNum)
{
    stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
    
    #if ENCODER_HANDLE_PLAN // TIM
    
        stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;
    
        return ( int32_t )
            __HAL_TIM_GET_COUNTER(
                pstStatic->pstTimerBase
            ) + pstRunning->times_reach_arr * (pstStatic->pstTimerBase->Init.Period + 1);
    #else
        return pstRunning->TotalPosition;
    #endif
}

/**
 * @brief 初始化编码器静态参数
 * @param pstInit 编码器静态参数指针
 * @param emDevNum 编码器设备号
 */
void vEncoderDeviceInit(stEncoderStaticParamTdf *pstInit, emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM || pstInit == NULL) {
        return;
    }
    
    memcpy(&astEncoderDeviceParam[emDevNum].stStaticParam, 
           pstInit, 
           sizeof(stEncoderStaticParamTdf));
    
    memset(&astEncoderDeviceParam[emDevNum].stRunningParam, 
           0, 
           sizeof(stEncoderRunningParamTdf));
    
    // 初始化方向映射表
    if (pstInit->Encoder_Dir == emEncoderHigh) {
        astEncoderDeviceParam[emDevNum].stRunningParam.direction_map[0] = 1;   // dir_val = 0 -> +1
        astEncoderDeviceParam[emDevNum].stRunningParam.direction_map[1] = -1;  // dir_val = 1 -> -1
    } else {
        astEncoderDeviceParam[emDevNum].stRunningParam.direction_map[0] = -1;  // dir_val = 0 -> -1
        astEncoderDeviceParam[emDevNum].stRunningParam.direction_map[1] = 1;   // dir_val = 1 -> +1
    }   
    
    // 初始化滤波缓冲区
//    for (int i = 0; i < 5; i++) {
//        astEncoderDeviceParam[emDevNum].stRunningParam.afFilterBuffer[i] = 0.0f;
//    }
    
    #if ENCODER_HANDLE_PLAN // TIM
        // 启动编码器定时器
        if (HAL_TIM_Encoder_Start_IT(astEncoderDeviceParam[emDevNum].stStaticParam.pstTimerBase, TIM_CHANNEL_ALL) != HAL_OK) {
            while(1);
        }
    #endif
    
    // 启动编码器绑定的定时器
    HAL_TIM_Base_Start_IT(&ENCODER_COMPUTE_IT_TIM);
    
    // 初始化LastPosition为当前编码器位置
    astEncoderDeviceParam[emDevNum].stRunningParam.LastPosition = lEncoderGetEncoder(emDevNum);
    //astEncoderDeviceParam[emDevNum].stRunningParam.LastPositionForDistance = lEncoderGetEncoder(emDevNum);
    astEncoderDeviceParam[emDevNum].stRunningParam.Distance_mm = 0.0f;
}

/**
 * @brief 获取编码器计数
 * @param emDevNum 编码器设备号
 * @return uint32_t 编码器计数
 */
int32_t ulEncoderGetCount(emEncoderDevNumTdf emDevNum){
    /* 单纯的获取计数值没有什么意义 */
    return astEncoderDeviceParam[emDevNum].stRunningParam.TotalPosition;
}

/**
 * @brief 获取编码器速度
 * @param emDevNum 编码器设备号
 * @return float 编码器速度
 */
float fEncoderGetSpeed(emEncoderDevNumTdf emDevNum){
    return astEncoderDeviceParam[emDevNum].stRunningParam.Speed;
}


/**
 * @brief 计算编码器速度
 * @param emDevNum 编码器设备号
 */
void vEncoderComputeSpeed(emEncoderDevNumTdf emDevNum)
{
    stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
    stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;
    
    if(pstRunning->_1ms_time_count++ < ENCODER_HANDLE_FREQ){
        return;
    }
    
    pstRunning->_1ms_time_count = 0; 

//    float temp = 0.0;

    /* 计算电机转速 
       第一步 ：计算ms毫秒内计数变化量
       第二步 ；计算1min内计数变化量：g_encode.speed * ((1000 / ms) * 60 ，
       第三步 ：除以编码器旋转一圈的计数次数（倍频倍数 * 编码器分辨率）
       第四步 ：除以减速比即可得出电机转速
    */
    pstRunning->TotalPosition = lEncoderGetEncoder(emDevNum);                               /* 取出编码器当前总位置 */
    
    
    int32_t delta = pstRunning->TotalPosition - pstRunning->LastPosition;
    
    /* 计算路程 */
    if(pstStatic->Wheel_Diameter_mm > 0.0f)
    {
        float wheel_circumference = pstStatic->Wheel_Diameter_mm * 3.1415926535f;
        float distance_per_count = wheel_circumference / (pstStatic->Roto_Ratio * pstStatic->A_Round_Count);
        pstRunning->Distance_mm += (float)delta * distance_per_count;
        //pstRunning->LastPosition = pstRunning->TotalPosition;
    }
    
    pstRunning->Speed = (float)                                                             /* 计算编码器计数值对应速度 */
         delta * 
        ((1000 / ENCODER_HANDLE_FREQ) * 60.0) / pstStatic->Roto_Ratio / pstStatic->A_Round_Count;
    
    pstRunning->LastPosition = pstRunning->TotalPosition;
    
//    g_encode.encode_old = g_encode.encode_now;          /* 保存当前编码器的值 */

    /* 一阶低通滤波
     * 公式为：Y(n)= qX(n) + (1-q)Y(n-1)
     * 其中X(n)为本次采样值；Y(n-1)为上次滤波输出值；Y(n)为本次滤波输出值，q为滤波系数
     * q值越小则上一次输出对本次输出影响越大，整体曲线越平稳，但是对于速度变化的响应也会越慢
     */
    //g_motor_data.speed = (float)( ((float)0.48 * temp) + (g_motor_data.speed * (float)0.52) );

}

/**
 * @brief 标记编码器数据状态
 * @param htim 定时器句柄指针
 * @return 无
 */
//void vEncoderSetDataState(TIM_HandleTypeDef *htim)
//{   
//    if (&ENCODER_COMPUTE_IT_TIM != htim) return;
//    
//    for (int i = 0; i < ENCODER_DEV_NUM; i++) {
//        stEncoderDeviceParamTdf *pstParam = &astEncoderDeviceParam[i];
//        pstParam->stRunningParam.ucDataState = UPDATED;
//    }
//}

/**
 * @brief 获取编码器数据状态
 * @param emDevNum 编码器设备号
 * @return emEncoderDataStateTdf 编码器数据状态
 */
//emEncoderDataStateTdf emEncoderGetDataState(emEncoderDevNumTdf emDevNum)
//{
//    stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
//    
//    if (pstRunning == NULL) {
//        return NOT_UPDATE;
//    }
//    
//    if (pstRunning->ucDataState == UPDATED) {
//        pstRunning->ucDataState = NOT_UPDATE;
//        return UPDATED;
//    }
//    
//    return NOT_UPDATE;
//}

#if ENCODER_HANDLE_PLAN // TIM
/**
 * @brief       编码器溢出处理函数
 * @param       htim:定时器句柄指针
 * @retval      无
 */
void vEncoder_Handler(TIM_HandleTypeDef *htim)
{
    // 匹配句柄
    for (int i = 0; i < ENCODER_DEV_NUM; i++) {
        stEncoderDeviceParamTdf *pstParam = &astEncoderDeviceParam[i];
        if(pstParam->stStaticParam.pstTimerBase == htim){
            // 检查是否是编码器定时器的溢出中断
            if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
                if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
                    pstParam->stRunningParam.times_reach_arr--;
                } else {
                    pstParam->stRunningParam.times_reach_arr++;
                }
                __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
            }
            break;
        }
        
    }
}

#else
/**
 * @brief       编码器中断处理函数
 * @param       GPIO_Pin: GPIO端口
 * @retval      无
 */
void vEncoder_Handler(uint16_t GPIO_Pin)
{
    // 匹配句柄
    for (int i = 0; i < ENCODER_DEV_NUM; i++) {
        stEncoderDeviceParamTdf *pstParam = &astEncoderDeviceParam[i];
        if(pstParam->stStaticParam.EXTI_Pin == GPIO_Pin){
            GPIO_PinState result = HAL_GPIO_ReadPin(
                                        pstParam->stStaticParam.Input_GpioPort, 
                                        pstParam->stStaticParam.Input_Pin
                                    );
 
            pstParam->stRunningParam.TotalPosition += pstParam->stRunningParam.direction_map[result];

            
            break;
        }
    }
}

/**
 * @brief 获取编码器距离
 * @param emDevNum 编码器设备号
 * @return float 编码器距离
 */
float fEncoderGetDistance(emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM) {
        return 0.0f;
    }
    return astEncoderDeviceParam[emDevNum].stRunningParam.Distance_mm;
}

/**
 * @brief 重置编码器距离
 * @param emDevNum 编码器设备号
 */
void vEncoderResetDistance(emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM) {
        return;
    }
    stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
    pstRunning->Distance_mm = 0.0f;
    pstRunning->LastPosition = lEncoderGetEncoder(emDevNum);
}
#endif

#endif
