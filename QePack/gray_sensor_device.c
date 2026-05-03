/** 
 * @file    gray_sensor_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/04/18
 * @brief   灰度传感器巡线驱动模块实现
 */
#include "gray_sensor_device.h"
#if GRAY_SENSOR_IS_ENABLE

stGraySensorDeviceParamTdf astGraySensorDeviceParam[GRAY_SENSOR_DEV_NUM];

/* 编码器目标值（停止检测用）- 兼容原有接口 */
static int32_t slEncoderTarget[GRAY_SENSOR_DEV_NUM];

/**
 * @brief 获取灰度传感器设备参数
 * @param emDevNum 设备号
 * @return const stGraySensorDeviceParamTdf* 设备参数指针
 */
const stGraySensorDeviceParamTdf *c_pstGetGraySensorDeviceParam(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) {
        return NULL;
    }
    return &astGraySensorDeviceParam[emDevNum];
}

/**
 * @brief 初始化灰度传感器设备
 * @param pstInit 初始化参数指针
 * @param emDevNum 设备号
 */
void vGraySensorDeviceInit(stGraySensorStaticParamTdf *pstInit, emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM || pstInit == NULL) {
        return;
    }
    
    memcpy(&astGraySensorDeviceParam[emDevNum].stStaticParam, 
           pstInit, 
           sizeof(stGraySensorStaticParamTdf));
    
    memset(&astGraySensorDeviceParam[emDevNum].stRunningParam, 
           0, 
           sizeof(stGraySensorRunningParamTdf));
    
    stGraySensorRunningParamTdf *pstRunning = &astGraySensorDeviceParam[emDevNum].stRunningParam;
    pstRunning->emStopState = emGrayStopNormal;
    pstRunning->ucEnableSensor = 1;
    slEncoderTarget[emDevNum] = 0;
}

/**
 * @brief 异常状态计数器递增（内部使用）
 * @param emDevNum 设备号
 */
static void vGraySensorAddWorse(emGraySensorDevNumTdf emDevNum)
{
    stGraySensorRunningParamTdf *pstRunning = &astGraySensorDeviceParam[emDevNum].stRunningParam;
    if (pstRunning->ulStatusWorse <= 20) {
        pstRunning->ulStatusWorse++;
    }
}

/**
 * @brief 计算历史数据的均值（均值滤波核心）
 * @param emDevNum 设备号
 * @return int16_t 历史数据平均值
 */
static int16_t sGraySensorCalcAverage(emGraySensorDevNumTdf emDevNum)
{
    stGraySensorStaticParamTdf *pstStatic = &astGraySensorDeviceParam[emDevNum].stStaticParam;
    stGraySensorRunningParamTdf *pstRunning = &astGraySensorDeviceParam[emDevNum].stRunningParam;
    
    int32_t lSum = 0;
    for (uint8_t i = 0; i < pstStatic->ucBackupLength; i++) {
        lSum += pstRunning->psStatusBackup[i];
    }
    return (int16_t)(lSum / pstStatic->ucBackupLength);
}

/**
 * @brief 获取8路灰度传感器状态
 * @param emDevNum 设备号
 * @return uint16_t 传感器状态(0x00-0xFF)
 */
uint16_t usGraySensorGetState(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) {
        return 0;
    }
    
    stGraySensorStaticParamTdf *pstStatic = &astGraySensorDeviceParam[emDevNum].stStaticParam;
    uint16_t usState = 0;
    
    #if (QEPACK_PLATFORM == TI)
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[0], pstStatic->ulGpioPin[0]) ? 1 : 0) << 0;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[1], pstStatic->ulGpioPin[1]) ? 1 : 0) << 1;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[2], pstStatic->ulGpioPin[2]) ? 1 : 0) << 2;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[3], pstStatic->ulGpioPin[3]) ? 1 : 0) << 3;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[4], pstStatic->ulGpioPin[4]) ? 1 : 0) << 4;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[5], pstStatic->ulGpioPin[5]) ? 1 : 0) << 5;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[6], pstStatic->ulGpioPin[6]) ? 1 : 0) << 6;
        usState |= (DL_GPIO_readPins(pstStatic->pstGpioPort[7], pstStatic->ulGpioPin[7]) ? 1 : 0) << 7;
    #else
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[0], pstStatic->usGpioPin[0]) ? 1 : 0) << 0;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[1], pstStatic->usGpioPin[1]) ? 1 : 0) << 1;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[2], pstStatic->usGpioPin[2]) ? 1 : 0) << 2;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[3], pstStatic->usGpioPin[3]) ? 1 : 0) << 3;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[4], pstStatic->usGpioPin[4]) ? 1 : 0) << 4;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[5], pstStatic->usGpioPin[5]) ? 1 : 0) << 5;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[6], pstStatic->usGpioPin[6]) ? 1 : 0) << 6;
        usState |= (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[7], pstStatic->usGpioPin[7]) ? 1 : 0) << 7;
    #endif
    
    return usState;
}

/**
 * @brief 灰度传感器状态分析与处理
 * @param emDevNum 设备号
 */
void vGraySensorCheck(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return;
    
    stGraySensorStaticParamTdf *pstStatic = &astGraySensorDeviceParam[emDevNum].stStaticParam;
    stGraySensorRunningParamTdf *pstRunning = &astGraySensorDeviceParam[emDevNum].stRunningParam;
    
    if (pstRunning->ucEnableSensor == 0) return;
    
    int16_t sTempStatus;
    pstRunning->usGrayState = usGraySensorGetState(emDevNum);
    
    switch (pstRunning->usGrayState) {
        case 0x01: sTempStatus = 7;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x03: sTempStatus = 6;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x02: sTempStatus = 5;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x06: sTempStatus = 4;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x04: sTempStatus = 3;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x0C: sTempStatus = 2;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x08: sTempStatus = 1;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x18: sTempStatus = 0;    pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x10: sTempStatus = -1;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x30: sTempStatus = -2;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x20: sTempStatus = -3;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x60: sTempStatus = -4;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x40: sTempStatus = -5;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0xC0: sTempStatus = -6;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x80: sTempStatus = -7;   pstRunning->sLastGrayStatus = sTempStatus; break;
        case 0x00:
            sTempStatus = pstRunning->sLastGrayStatus;
            vGraySensorAddWorse(emDevNum);
            break;
        default:
            sTempStatus = pstRunning->sLastGrayStatus;
            vGraySensorAddWorse(emDevNum);
            break;
    }
    
    if (pstRunning->usGrayState != 0x00 && 
        (pstRunning->usGrayState & 0xFF) != 0xFF && 
        pstRunning->usGrayState != 0x0F && 
        pstRunning->usGrayState != 0xF0) {
        pstRunning->ulStatusWorse >>= 1;
    }
    
    pstRunning->ucBackupIdx = (pstRunning->ucBackupIdx + 1) % pstStatic->ucBackupLength;
    
    if (pstStatic->ucGrayDirection) {
        pstRunning->psStatusBackup[pstRunning->ucBackupIdx] = sTempStatus;
    } else {
        pstRunning->psStatusBackup[pstRunning->ucBackupIdx] = -sTempStatus;
    }
    
    pstRunning->sGrayStatus = sGraySensorCalcAverage(emDevNum);
    
    if (!pstStatic->ucEnableStopLineDetect) return;
}

/**
 * @brief 停止线/拐角检测
 * @param emDevNum 设备号
 */
void vGraySensorCheckStop(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return;
    
    stGraySensorStaticParamTdf *pstStatic = &astGraySensorDeviceParam[emDevNum].stStaticParam;
    stGraySensorRunningParamTdf *pstRunning = &astGraySensorDeviceParam[emDevNum].stRunningParam;
    
    if (!pstStatic->ucEnableStopLineDetect) return;
    
    uint8_t ucCurr[3];
    #if (QEPACK_PLATFORM == TI)
        ucCurr[0] = (DL_GPIO_readPins(pstStatic->pstGpioPort[5], pstStatic->ulGpioPin[5]) ? 1 : 0);
        ucCurr[1] = (DL_GPIO_readPins(pstStatic->pstGpioPort[6], pstStatic->ulGpioPin[6]) ? 1 : 0);
        ucCurr[2] = (DL_GPIO_readPins(pstStatic->pstGpioPort[7], pstStatic->ulGpioPin[7]) ? 1 : 0);
    #else
        ucCurr[0] = (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[5], pstStatic->usGpioPin[5]) ? 1 : 0);
        ucCurr[1] = (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[6], pstStatic->usGpioPin[6]) ? 1 : 0);
        ucCurr[2] = (HAL_GPIO_ReadPin(pstStatic->pstGpioPort[7], pstStatic->usGpioPin[7]) ? 1 : 0);
    #endif
    
    for (uint8_t i = 0; i < 3; i++) {
        if (ucCurr[i]) {
            pstRunning->stTrack[i].ucTriggered = 1;
            pstRunning->stTrack[i].ucValidCnt = pstStatic->ucTrackValidThreshold;
        } else {
            if (pstRunning->stTrack[i].ucValidCnt > 0) {
                pstRunning->stTrack[i].ucValidCnt--;
            } else {
                pstRunning->stTrack[i].ucTriggered = 0;
            }
        }
    }
    
    if (pstRunning->stTrack[0].ucTriggered && 
        pstRunning->stTrack[1].ucTriggered && 
        pstRunning->stTrack[2].ucTriggered) {
        pstRunning->emStopState = emGrayStopLeftCorner;
    }
    
    if (ucCurr[0] && ucCurr[1] && ucCurr[2]) {
        pstRunning->emStopState = emGrayStopLeftCorner;
    }
}

/**
 * @brief 获取滤波后的偏移量
 * @param emDevNum 设备号
 * @return int16_t 偏移量
 */
int16_t sGraySensorGetStatus(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return 0;
    return astGraySensorDeviceParam[emDevNum].stRunningParam.sGrayStatus;
}

/**
 * @brief 获取异常状态计数器
 * @param emDevNum 设备号
 * @return uint32_t 异常状态计数值
 */
uint32_t ulGraySensorGetWorse(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return 0;
    return astGraySensorDeviceParam[emDevNum].stRunningParam.ulStatusWorse;
}

/**
 * @brief 设置异常状态计数器
 * @param emDevNum 设备号
 * @param ulGrayWorse 新的计数值
 */
void vGraySensorSetWorse(emGraySensorDevNumTdf emDevNum, uint32_t ulGrayWorse)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return;
    astGraySensorDeviceParam[emDevNum].stRunningParam.ulStatusWorse = ulGrayWorse;
}

/**
 * @brief 获取停止状态
 * @param emDevNum 设备号
 * @return emGrayStopStateTdf 停止状态
 */
emGrayStopStateTdf emGraySensorGetStopState(emGraySensorDevNumTdf emDevNum)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return emGrayStopNormal;
    return astGraySensorDeviceParam[emDevNum].stRunningParam.emStopState;
}

/**
 * @brief 设置停止状态
 * @param emDevNum 设备号
 * @param emStopState 新的停止状态
 */
void vGraySensorSetStopState(emGraySensorDevNumTdf emDevNum, emGrayStopStateTdf emStopState)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return;
    astGraySensorDeviceParam[emDevNum].stRunningParam.emStopState = emStopState;
}

/**
 * @brief 设置停止检测冻结阈值
 * @param emDevNum 设备号
 * @param usTime 冻结时间
 */
void vGraySensorSetFreezeStopThreshold(emGraySensorDevNumTdf emDevNum, uint16_t usTime)
{
    (void)emDevNum;
    (void)usTime;
}

/**
 * @brief 设置编码器目标值（停止检测用）
 * @param emDevNum 设备号
 * @param lEncoderTarget 编码器目标值
 */
void vGraySensorSetEncoderTarget(emGraySensorDevNumTdf emDevNum, int32_t lEncoderTarget)
{
    if (emDevNum >= GRAY_SENSOR_DEV_NUM) return;
    slEncoderTarget[emDevNum] = lEncoderTarget;
}

#endif
