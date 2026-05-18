/**
 * @file    encoder_device.c
 * @author  Qe_xr
 * @version V2.0.0
 * @date    2026/5/12
 * @brief   编码器驱动模块 — 定点数版本 (Q16.16)
 */
#include "encoder_device.h"
#if ENCODER_IS_ENABLE

/* 编译期定点数常量 */
#define FIX32_3_14159    ((fix32_t)205887)   /* 3.14159265 in Q16.16 */

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
 * @brief 中位值平均滤波（定点数版本）
 * @param fValue 输入值 (Q16.16)
 * @param afBuffer 缓冲区指针
 * @param u8BufferSize 缓冲区大小
 * @return fix32_t 中位值平均滤波后的值
 */
fix32_t fixMedianFilter(fix32_t fValue, fix32_t* afBuffer, uint8_t u8BufferSize)
{
    for (int i = u8BufferSize - 1; i > 0; i--) {
        afBuffer[i] = afBuffer[i - 1];
    }
    afBuffer[0] = fValue;

    for (int i = 0; i < u8BufferSize; i++) {
        for (int j = i + 1; j < u8BufferSize; j++) {
            if (afBuffer[i] > afBuffer[j]) {
                fix32_t temp = afBuffer[i];
                afBuffer[i] = afBuffer[j];
                afBuffer[j] = temp;
            }
        }
    }

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
    #if (QEPACK_PLATFORM == TI)
        stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;
    #endif
    
    #if (ENCODER_HANDLE_PLAN == TIM) // TIM
        return ( int32_t )
            __HAL_TIM_GET_COUNTER(
                pstStatic->pstTimerBase
            ) + pstRunning->times_reach * (pstStatic->pstTimerBase->Init.Period + 1);
    #else

        #if (QEPACK_PLATFORM == TI)
            if (pstStatic->ucNumberofEdgesToDetect > 1 && pstRunning->isDoneAInterrupt) {
                pstRunning->isDoneAInterrupt = 0;
                
                pstRunning->TotalPosition +=
                pstRunning->direction_map[pstRunning->emCurrentPinState] 
                * pstStatic->pstCompareTimerBase->timer_inst->COUNTERREGS.CTR;

                pstStatic->pstCompareTimerBase->timer_inst->COUNTERREGS.CTR = 0;
            }
            return pstRunning->TotalPosition;
        #else
            /*
                为什么直接返回TotalPosition？
                因为TotalPosition的更新由每次的比较定时器更新中断/GPIO外部中断处理
            */
            return pstRunning->TotalPosition;
        #endif

        
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

    stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;
    

    /** 与编码器直接对应的定时器 */
    #if (ENCODER_HANDLE_PLAN == TIM) // TIM
        // 启动编码器定时器
        if (HAL_TIM_Encoder_Start_IT(pstStatic->pstTimerBase, TIM_CHANNEL_ALL) != HAL_OK) {
            while(1);
        }

    #else
        #if (QEPACK_PLATFORM == TI)
            //使能比较中断
            NVIC_EnableIRQ(pstStatic->pstCompareTimerBase->timer_irqn);
            //编码定时器
            //墙裂建议配置在PWM输出前面
            DL_Timer_startCounter(pstStatic->pstCompareTimerBase->timer_inst);
        #endif
    #endif
    

    /** 用来处理数据的定时器 */

    // 是否使用寄生定时器
    #if ENCODER_IS_USE_PARASITISM
        // TODO
    #else
        // 启动编码器绑定的定时器
        #if (QEPACK_PLATFORM == TI)
            // 若不使用寄生定时器，则需要手动创建回调函数
            DL_Timer_enableInterrupt(
                pstStatic->pstHandleTimerBase->timer_inst, 
                DL_TIMER_IIDX_ZERO
            );
            NVIC_EnableIRQ(pstStatic->pstHandleTimerBase->timer_irqn);
        #else
            HAL_TIM_Base_Start_IT(pstStatic->pstHandleTimerBase);
        #endif
    #endif
    
    astEncoderDeviceParam[emDevNum].stRunningParam.LastPosition = lEncoderGetEncoder(emDevNum);
    astEncoderDeviceParam[emDevNum].stRunningParam.fDistanceMm = FIX32_ZERO;
}

/**
 * @brief 获取编码器计数
 * @param emDevNum 编码器设备号
 * @return uint32_t 编码器计数
 */
int32_t ulEncoderGetCount(emEncoderDevNumTdf emDevNum){
    if (emDevNum >= ENCODER_DEV_NUM) return 0;
    return astEncoderDeviceParam[emDevNum].stRunningParam.TotalPosition;
}

/**
 * @brief 获取编码器速度
 * @param emDevNum 编码器设备号
 * @return fix32_t 编码器速度 (Q16.16)
 */
fix32_t fEncoderGetSpeed(emEncoderDevNumTdf emDevNum){
    if (emDevNum >= ENCODER_DEV_NUM) return FIX32_ZERO;
    return astEncoderDeviceParam[emDevNum].stRunningParam.fSpeed;
}

void vEncoderCalculateSpeed(emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM) return;
    stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
    stEncoderStaticParamTdf  *pstStatic  = &astEncoderDeviceParam[emDevNum].stStaticParam;

    pstRunning->TotalPosition = lEncoderGetEncoder(emDevNum);
    int32_t delta = pstRunning->TotalPosition - pstRunning->LastPosition;

    /* 计算路程（定点数）
     * distance = delta * D * pi / (Roto * Counts * Gear)
     */
    if (pstStatic->fWheelDiameterMm > FIX32_ZERO) {
        fix32_t fGear = (pstStatic->fGearRatio > FIX32_ZERO) ? pstStatic->fGearRatio : FIX32_ONE;
        fix32_t fDenom = (fix32_t)((int64_t)((int32_t)pstStatic->Roto_Ratio * (int32_t)pstStatic->A_Round_Count) * 65536);
        fDenom = fix32_mul(fDenom, fGear);
        fix32_t fDelta = (fix32_t)((int64_t)delta * 65536);
        fix32_t fDist  = fix32_mul(fix32_mul(fDelta, pstStatic->fWheelDiameterMm), FIX32_3_14159);
        fDist = fix32_div(fDist, fDenom);
        pstRunning->fDistanceMm += fDist;
    }

    /* 计算速度（定点数）
     * speed_rpm = delta * (60000 / freq) / Roto / Counts / Gear
     * 注意：先做除法避免 fix32_from_int 溢出
     */
    int32_t lRotoCounts = (int32_t)pstStatic->Roto_Ratio * (int32_t)pstStatic->A_Round_Count;
    if (lRotoCounts == 0) lRotoCounts = 1;
    int32_t lSpeedRaw = delta * (60000 / (int32_t)ENCODER_HANDLE_FREQ) / lRotoCounts;
    pstRunning->fSpeed = (fix32_t)((int64_t)lSpeedRaw * 65536);

    if (pstStatic->fGearRatio > FIX32_ZERO) {
        pstRunning->fSpeed = fix32_div(pstRunning->fSpeed, pstStatic->fGearRatio);
    }

    pstRunning->LastPosition = pstRunning->TotalPosition;
}

/**
 * @brief 计算编码器速度
 * @param htim 定时器句柄指针
 */
#if (QEPACK_PLATFORM == TI)
    void vEncoderComputeSpeed(emEncoderDevNumTdf emDevNum){
        stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
        stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;
        
        if(pstRunning->_1ms_time_count++ < ENCODER_HANDLE_FREQ){
            return;
        }
        pstRunning->_1ms_time_count = 0; 

        vEncoderCalculateSpeed(emDevNum);
    }
#else
    void vEncoderComputeSpeed(TIM_HandleTypeDef *htim){
        // 匹配句柄
        for (int i = 0; i < ENCODER_DEV_NUM; i++) {
            stEncoderDeviceParamTdf *pstParam = &astEncoderDeviceParam[i];
            if(pstParam->stStaticParam.pstHandleTimerBase == htim){
                stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[i].stRunningParam;
                
                if(pstRunning->_1ms_time_count++ < ENCODER_HANDLE_FREQ){
                    return;
                }
                pstRunning->_1ms_time_count = 0; 

                vEncoderCalculateSpeed((emEncoderDevNumTdf)i);
            }
        }
    }
#endif

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

#if (ENCODER_HANDLE_PLAN == TIM) // TIM
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
                    pstParam->stRunningParam.times_reach--;
                } else {
                    pstParam->stRunningParam.times_reach++;
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
#if (QEPACK_PLATFORM == TI)
// 引脚测试接口
GPIO_PinState ReadPin(emEncoderDevNumTdf emDevNum){
    stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;

    return TI_GPIO_ReadPin(
        pstStatic->pstDirGpioBase, 
        pstStatic->usDirGpioPin
    );
}

void vEncoder_Handler(emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM) return;
    stEncoderRunningParamTdf *pstRunning = &astEncoderDeviceParam[emDevNum].stRunningParam;
    stEncoderStaticParamTdf  *pstStatic = &astEncoderDeviceParam[emDevNum].stStaticParam;

    pstRunning->emCurrentPinState = TI_GPIO_ReadPin(
        pstStatic->pstDirGpioBase,
        pstStatic->usDirGpioPin
    );

    if (pstStatic->ucNumberofEdgesToDetect == 1) {
        /* 单边沿模式：中断中直接更新位置并清零 */
        pstRunning->TotalPosition +=
            pstRunning->direction_map[pstRunning->emCurrentPinState];
        pstStatic->pstCompareTimerBase->timer_inst->COUNTERREGS.CTR = 0;
    } else {
        /* 多边沿模式：只标记，不清零，由 lEncoderGetEncoder 处理累积值 */
        if(!pstRunning->isDoneAInterrupt) pstRunning->isDoneAInterrupt = 1;
    }
}
#else

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

            pstParam->stRunningParam.TotalPosition += 
            pstParam->stRunningParam.direction_map[result];
            
            break;
        }
    }
}         
#endif
/**
 * @brief 获取编码器距离
 * @param emDevNum 编码器设备号
 * @return fix32_t 编码器距离 (Q16.16, 单位: mm)
 */
fix32_t fEncoderGetDistance(emEncoderDevNumTdf emDevNum)
{
    if (emDevNum >= ENCODER_DEV_NUM) {
        return FIX32_ZERO;
    }
    return astEncoderDeviceParam[emDevNum].stRunningParam.fDistanceMm;
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
    pstRunning->fDistanceMm = FIX32_ZERO;
    pstRunning->LastPosition = lEncoderGetEncoder(emDevNum);
}
#endif

#endif
