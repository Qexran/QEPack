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

    if (astGraySensorDeviceParam[emDevNum].stStaticParam.ucBackupLength > GRAY_SENSOR_BACKUP_MAX_LEN) {
        astGraySensorDeviceParam[emDevNum].stStaticParam.ucBackupLength = GRAY_SENSOR_BACKUP_MAX_LEN;
    }
    if (astGraySensorDeviceParam[emDevNum].stStaticParam.ucBackupLength == 0) {
        astGraySensorDeviceParam[emDevNum].stStaticParam.ucBackupLength = 1;
    }
}

/* 灰度传感器状态到偏移量的查找表（-128 表示使用上一次值） */
static const int8_t scGraySensorOffsetTable[256] = {
    -128,   7,   5,   6,   3, -128,   4, -128,  /* 0x00-0x07 */
       1, -128, -128, -128,   2, -128, -128, -128,  /* 0x08-0x0F */
      -1, -128, -128, -128, -128, -128, -128, -128,  /* 0x10-0x17 */
       0, -128, -128, -128, -128, -128, -128, -128,  /* 0x18-0x1F */
      -3, -128, -128, -128, -128, -128, -128, -128,  /* 0x20-0x27 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x28-0x2F */
      -2, -128, -128, -128, -128, -128, -128, -128,  /* 0x30-0x37 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x38-0x3F */
      -5, -128, -128, -128, -128, -128, -128, -128,  /* 0x40-0x47 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x48-0x4F */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x50-0x57 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x58-0x5F */
      -4, -128, -128, -128, -128, -128, -128, -128,  /* 0x60-0x67 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x68-0x6F */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x70-0x77 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x78-0x7F */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x80-0x87 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x88-0x8F */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x90-0x97 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0x98-0x9F */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xA0-0xA7 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xA8-0xAF */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xB0-0xB7 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xB8-0xBF */
      -6, -128, -128, -128, -128, -128, -128, -128,  /* 0xC0-0xC7 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xC8-0xCF */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xD0-0xD7 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xD8-0xDF */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xE0-0xE7 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xE8-0xEF */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xF0-0xF7 */
    -128, -128, -128, -128, -128, -128, -128, -128,  /* 0xF8-0xFF */
};

/**
 * @brief 计算历史数据的均值（均值滤波核心）
 * @param emDevNum 设备号
 * @return int16_t 历史数据平均值
 */
static int16_t sGraySensorCalcAverage(emGraySensorDevNumTdf emDevNum)
{
    stGraySensorStaticParamTdf *pstStatic = &astGraySensorDeviceParam[emDevNum].stStaticParam;
    stGraySensorRunningParamTdf *pstRunning = &astGraySensorDeviceParam[emDevNum].stRunningParam;

    if (pstStatic->ucBackupLength == 0) return 0;

    int32_t lSum = 0;
    for (uint8_t i = 0; i < pstStatic->ucBackupLength; i++) {
        lSum += pstRunning->asStatusBackup[i];
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
    uint16_t usState = usGraySensorGetState(emDevNum);
    pstRunning->usGrayState = usState;

    int8_t cOffset = scGraySensorOffsetTable[usState & 0xFF];
    if (cOffset != -128) {
        sTempStatus = cOffset;
        pstRunning->sLastGrayStatus = cOffset;
    } else {
        sTempStatus = pstRunning->sLastGrayStatus;
        if (pstRunning->ulStatusWorse <= 20) {
            pstRunning->ulStatusWorse++;
        }
    }

    if (usState != 0x00 && usState != 0xFF && usState != 0x0F && usState != 0xF0) {
        pstRunning->ulStatusWorse >>= 1;
    }
    
    pstRunning->ucBackupIdx = (pstRunning->ucBackupIdx + 1) % pstStatic->ucBackupLength;
    
    if (pstStatic->ucGrayDirection) {
        pstRunning->asStatusBackup[pstRunning->ucBackupIdx] = sTempStatus;
    } else {
        pstRunning->asStatusBackup[pstRunning->ucBackupIdx] = -sTempStatus;
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

    uint16_t usState = pstRunning->usGrayState;
    uint8_t ucCurr[3];
    ucCurr[0] = (usState >> 5) & 1;
    ucCurr[1] = (usState >> 6) & 1;
    ucCurr[2] = (usState >> 7) & 1;

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

/* ===================== 传感器基类适配 ===================== */

#if SENSOR_IS_ENABLE

#include "sensor_device.h"

#define GRAY_SENSOR_WRAPPER_LOCAL_MAX  3
#define GRAY_SENSOR_WRAPPER_TO_LOCAL(dev)  ((uint8_t)((dev) - emSensorGrayDevNum0))

typedef struct {
    stSensorDeviceTdf        stBase;
    emSensorDevNumTdf        emSensorDevNum;
    fix32_t                  fTargetValue;
} stGraySensorWrapperDeviceTdf;

static void vGraySensorWrapperInit(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    (void)pstGray;
}

static void vGraySensorWrapperPeriodExecute(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    emGraySensorDevNumTdf emGrayDev = (emGraySensorDevNumTdf)GRAY_SENSOR_WRAPPER_TO_LOCAL(pstGray->emSensorDevNum);
    vGraySensorCheck(emGrayDev);
    vGraySensorCheckStop(emGrayDev);
}

static fix32_t fGraySensorWrapperGetValue(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    emGraySensorDevNumTdf emGrayDev = (emGraySensorDevNumTdf)GRAY_SENSOR_WRAPPER_TO_LOCAL(pstGray->emSensorDevNum);
    return (fix32_t)((int64_t)(sGraySensorGetStatus(emGrayDev)) * 65536);
}

static void vGraySensorWrapperReset(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    emGraySensorDevNumTdf emGrayDev = (emGraySensorDevNumTdf)GRAY_SENSOR_WRAPPER_TO_LOCAL(pstGray->emSensorDevNum);
    vGraySensorSetWorse(emGrayDev, 0);
    pstGray->fTargetValue = FIX32_ZERO;
}

static void vGraySensorWrapperSetTarget(void *pstSensor, fix32_t fTarget)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    pstGray->fTargetValue = fTarget;
}

static fix32_t fGraySensorWrapperGetTarget(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    return pstGray->fTargetValue;
}

static stSensorVTableTdf g_stGraySensorWrapperVTable = {
    vGraySensorWrapperInit,
    vGraySensorWrapperPeriodExecute,
    fGraySensorWrapperGetValue,
    vGraySensorWrapperReset,
    vGraySensorWrapperSetTarget,
    fGraySensorWrapperGetTarget,
};

static stGraySensorWrapperDeviceTdf g_astGraySensorWrapperDevices[GRAY_SENSOR_WRAPPER_LOCAL_MAX];

void vGraySensorWrapperRegister(emSensorDevNumTdf emSensorDevNum)
{
    uint8_t ucLocalIdx = GRAY_SENSOR_WRAPPER_TO_LOCAL(emSensorDevNum);
    if (ucLocalIdx >= GRAY_SENSOR_WRAPPER_LOCAL_MAX) {
        return;
    }

    stGraySensorWrapperDeviceTdf *pstGray = &g_astGraySensorWrapperDevices[ucLocalIdx];
    memset(pstGray, 0, sizeof(stGraySensorWrapperDeviceTdf));

    pstGray->stBase.emType = emSensorTypeGray;
    pstGray->stBase.pstVTable = &g_stGraySensorWrapperVTable;
    pstGray->stBase.ucEnable = 1;
    pstGray->stBase.fWeight = FIX32_ONE;
    pstGray->stBase.emPidDevNum     = emNoPid;
    pstGray->stBase.usPidPeriodMs   = 0;
    pstGray->stBase.ulPidLastTickMs = 0;
    pstGray->emSensorDevNum = emSensorDevNum;

    vSensorRegisterDevice(emSensorDevNum, &pstGray->stBase);
}

#endif /* SENSOR_IS_ENABLE */

#endif
