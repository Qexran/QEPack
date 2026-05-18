/**
  * @file       sensor_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/10
  * @brief      传感器抽象基类实现 + 各传感器 wrapper
  */

#include "sensor_device.h"
#include "arithmetic.h"

#if SENSOR_IS_ENABLE



/* ===================== 基类实现 ===================== */

stSensorDeviceTdf *g_astSensorDevices[emSensorMaxDevNum];

/**
 * @brief 获取传感器设备指针
 * @param emDevNum 设备号
 * @return 设备指针，越界返回 NULL
 */
stSensorDeviceTdf *pstSensorGetDevice(emSensorDevNumTdf emDevNum)
{
    if (emDevNum >= emSensorMaxDevNum) {
        return NULL;
    }
    return g_astSensorDevices[emDevNum];
}

/**
 * @brief 注册传感器设备
 */
void vSensorRegisterDevice(emSensorDevNumTdf emDevNum, stSensorDeviceTdf *pstSensor)
{
    if (emDevNum < emSensorMaxDevNum && pstSensor != NULL) {
        g_astSensorDevices[emDevNum] = pstSensor;
    }
}

/**
 * @brief 传感器初始化
 */
void vSensorInit(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL && pstSensor->pstVTable->vInit != NULL) {
        pstSensor->pstVTable->vInit(pstSensor);
    }
}

/**
 * @brief 传感器周期执行
 */
void vSensorPeriodExecute(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->ucEnable && pstSensor->pstVTable != NULL
        && pstSensor->pstVTable->vPeriodExecute != NULL) {
        pstSensor->pstVTable->vPeriodExecute(pstSensor);
    }
}

/**
 * @brief 获取传感器当前值
 */
fix32_t fSensorGetValue(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL && pstSensor->pstVTable->fGetValue != NULL) {
        return pstSensor->pstVTable->fGetValue(pstSensor);
    }
    return FIX32_ZERO;
}

/**
 * @brief 重置传感器
 */
void vSensorReset(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL && pstSensor->pstVTable->vReset != NULL) {
        pstSensor->pstVTable->vReset(pstSensor);
    }
}

/**
 * @brief 设置传感器目标值
 */
void vSensorSetTarget(emSensorDevNumTdf emDevNum, fix32_t fTarget)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL && pstSensor->pstVTable->vSetTarget != NULL) {
        pstSensor->pstVTable->vSetTarget(pstSensor, fTarget);
    }
}

/**
 * @brief 获取传感器目标值
 */
fix32_t fSensorGetTarget(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL && pstSensor->pstVTable->fGetTarget != NULL) {
        return pstSensor->pstVTable->fGetTarget(pstSensor);
    }
    return FIX32_ZERO;
}

/**
 * @brief 使能/禁用传感器
 */
void vSensorSetEnable(emSensorDevNumTdf emDevNum, uint8_t bEnable)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL) {
        pstSensor->ucEnable = bEnable;
    }
}

/**
 * @brief 设置传感器互补滤波权重
 */
void vSensorSetWeight(emSensorDevNumTdf emDevNum, fix32_t fWeight)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL) {
        pstSensor->fWeight = fWeight;
    }
}

/**
 * @brief 传感器融合（互补滤波）
 * @return 融合后的值 = A*fWeightA + B*fWeightB
 */
fix32_t fSensorFuseValue(emSensorDevNumTdf emDevNumA, emSensorDevNumTdf emDevNumB)
{
    stSensorDeviceTdf *pstA = pstSensorGetDevice(emDevNumA);
    stSensorDeviceTdf *pstB = pstSensorGetDevice(emDevNumB);

    uint8_t bAValid = (pstA != NULL && pstA->ucEnable && pstA->pstVTable != NULL
                       && pstA->pstVTable->fGetValue != NULL);
    uint8_t bBValid = (pstB != NULL && pstB->ucEnable && pstB->pstVTable != NULL
                       && pstB->pstVTable->fGetValue != NULL);

    if (!bAValid && !bBValid) return FIX32_ZERO;
    if (bAValid && !bBValid) return pstA->pstVTable->fGetValue(pstA);
    if (!bAValid && bBValid) return pstB->pstVTable->fGetValue(pstB);

    fix32_t fValA = pstA->pstVTable->fGetValue(pstA);
    fix32_t fValB = pstB->pstVTable->fGetValue(pstB);
    fix32_t fWeightA = pstA->fWeight;
    fix32_t fWeightB = pstB->fWeight;

    fix32_t fTotalWeight = fWeightA + fWeightB;
    if (fTotalWeight < FIX32_EPSILON) {
        return fValA;
    }

    /* 使用 int64_t 避免加权和溢出 */
    int64_t llWeightedSum = (int64_t)fix32_mul(fValA, fWeightA) + (int64_t)fix32_mul(fValB, fWeightB);
    fix32_t fSum = fix32_sat((fix32_t)llWeightedSum, FIX32_MIN, FIX32_MAX);
    return fix32_div(fSum, fTotalWeight);
}

/* ===================== 陀螺仪传感器 wrapper ===================== */

#define GYRO_SENSOR_GLOBAL_BASE 10   /* emSensorAtkMs901MDevNum0 */

#if ATK_MS901M_IS_ENABLE

/** @brief 陀螺仪传感器设备结构体 */
typedef struct {
    stSensorDeviceTdf      stBase;           /* 基类（第一个成员） */
    emAtkMs901mDevNumTdf   emAtkDevNum;      /* ATK-MS901M 设备号（流式 API） */
    fix32_t                fCurrentYaw;      /* 当前读到的 yaw 值（-180~180） */
    fix32_t                fLastYaw;         /* 上一次 yaw 值 */
    fix32_t                fAccumulatedYaw;  /* 累积角度（处理跳变） */
    fix32_t                fTargetYaw;       /* 目标角度 */
    int32_t                lTurnCount;       /* 圈数计数 */
} stGyroSensorDeviceTdf;

static void vGyroSensorInit(void *pstSensor)
{
    stGyroSensorDeviceTdf *pstGyro = (stGyroSensorDeviceTdf *)pstSensor;
    (void)pstGyro;
}

static void vGyroSensorPeriodExecute(void *pstSensor)
{
    stGyroSensorDeviceTdf *pstGyro = (stGyroSensorDeviceTdf *)pstSensor;
    atk_ms901m_attitude_data_t stAttitude;

    if (atk_ms901m_read_attitude(pstGyro->emAtkDevNum, &stAttitude) != ATK_MS901M_EOK) {
        return;
    }

    pstGyro->fLastYaw = pstGyro->fCurrentYaw;
    pstGyro->fCurrentYaw = fix32_from_float(stAttitude.yaw);

    fix32_t fDelta = pstGyro->fCurrentYaw - pstGyro->fLastYaw;
    if (fDelta > ((fix32_t)(180 * 65536))) {
        pstGyro->lTurnCount--;
    } else if (fDelta < ((fix32_t)(-180 * 65536))) {
        pstGyro->lTurnCount++;
    }

    pstGyro->fAccumulatedYaw = pstGyro->fCurrentYaw + (fix32_t)((int64_t)(pstGyro->lTurnCount) * 360 * 65536);
}

static fix32_t fGyroSensorGetValue(void *pstSensor)
{
    stGyroSensorDeviceTdf *pstGyro = (stGyroSensorDeviceTdf *)pstSensor;
    return pstGyro->fAccumulatedYaw;
}

static void vGyroSensorReset(void *pstSensor)
{
    stGyroSensorDeviceTdf *pstGyro = (stGyroSensorDeviceTdf *)pstSensor;
    pstGyro->fAccumulatedYaw = FIX32_ZERO;
    pstGyro->fCurrentYaw = FIX32_ZERO;
    pstGyro->fLastYaw = FIX32_ZERO;
    pstGyro->lTurnCount = 0;
    pstGyro->fTargetYaw = FIX32_ZERO;
}

static void vGyroSensorSetTarget(void *pstSensor, fix32_t fTarget)
{
    stGyroSensorDeviceTdf *pstGyro = (stGyroSensorDeviceTdf *)pstSensor;
    pstGyro->fTargetYaw = fTarget;
}

static fix32_t fGyroSensorGetTarget(void *pstSensor)
{
    stGyroSensorDeviceTdf *pstGyro = (stGyroSensorDeviceTdf *)pstSensor;
    return pstGyro->fTargetYaw;
}

static stSensorVTableTdf g_stGyroSensorVTable = {
    vGyroSensorInit,
    vGyroSensorPeriodExecute,
    fGyroSensorGetValue,
    vGyroSensorReset,
    vGyroSensorSetTarget,
    fGyroSensorGetTarget,
};

static stGyroSensorDeviceTdf g_astGyroSensorDevices[3];

void vGyroSensorRegister(uint8_t ucLocalIdx, emAtkMs901mDevNumTdf emAtkDevNum)
{
    if (ucLocalIdx >= 3) {
        return;
    }

    stGyroSensorDeviceTdf *pstGyro = &g_astGyroSensorDevices[ucLocalIdx];
    memset(pstGyro, 0, sizeof(stGyroSensorDeviceTdf));

    pstGyro->stBase.emType = emSensorTypeGyro;
    pstGyro->stBase.pstVTable = &g_stGyroSensorVTable;
    pstGyro->stBase.ucEnable = 1;
    pstGyro->stBase.fWeight = FIX32_ONE;
    pstGyro->emAtkDevNum = emAtkDevNum;

    vSensorRegisterDevice((emSensorDevNumTdf)(GYRO_SENSOR_GLOBAL_BASE + ucLocalIdx), &pstGyro->stBase);
}

#endif /* ATK_MS901M_IS_ENABLE */

/* ===================== 灰度传感器 wrapper ===================== */

#define GRAY_SENSOR_GLOBAL_BASE 0    /* emSensorGrayDevNum0 */

#if GRAY_SENSOR_IS_ENABLE

/** @brief 灰度传感器 wrapper 设备结构体 */
typedef struct {
    stSensorDeviceTdf        stBase;         /* 基类（第一个成员） */
    emGraySensorDevNumTdf    emGrayDevNum;   /* 灰度传感器设备号 */
    fix32_t                  fTargetValue;   /* 目标值 */
} stGraySensorWrapperDeviceTdf;

static void vGraySensorWrapperInit(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    (void)pstGray;
}

static void vGraySensorWrapperPeriodExecute(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    vGraySensorCheck(pstGray->emGrayDevNum);
    vGraySensorCheckStop(pstGray->emGrayDevNum);
}

static fix32_t fGraySensorWrapperGetValue(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    return (fix32_t)((int64_t)(sGraySensorGetStatus(pstGray->emGrayDevNum)) * 65536);
}

static void vGraySensorWrapperReset(void *pstSensor)
{
    stGraySensorWrapperDeviceTdf *pstGray = (stGraySensorWrapperDeviceTdf *)pstSensor;
    vGraySensorSetWorse(pstGray->emGrayDevNum, 0);
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

static stGraySensorWrapperDeviceTdf g_astGraySensorWrapperDevices[3];

void vGraySensorWrapperRegister(uint8_t ucLocalIdx, emGraySensorDevNumTdf emGrayDevNum)
{
    if (ucLocalIdx >= 3) {
        return;
    }

    stGraySensorWrapperDeviceTdf *pstGray = &g_astGraySensorWrapperDevices[ucLocalIdx];
    memset(pstGray, 0, sizeof(stGraySensorWrapperDeviceTdf));

    pstGray->stBase.emType = emSensorTypeGray;
    pstGray->stBase.pstVTable = &g_stGraySensorWrapperVTable;
    pstGray->stBase.ucEnable = 1;
    pstGray->stBase.fWeight = FIX32_ONE;
    pstGray->emGrayDevNum = emGrayDevNum;

    vSensorRegisterDevice((emSensorDevNumTdf)(GRAY_SENSOR_GLOBAL_BASE + ucLocalIdx), &pstGray->stBase);
}

#endif /* GRAY_SENSOR_IS_ENABLE */

/* ===================== CCD 传感器 wrapper ===================== */

#define CCD_SENSOR_GLOBAL_BASE 3     /* emSensorCCDDevNum0 */

#if LINEAR_CCD_IS_ENABLE

/** @brief CCD 传感器 wrapper 设备结构体 */
typedef struct {
    stSensorDeviceTdf        stBase;         /* 基类（第一个成员） */
    emLinerCcdDevNumTdf      emCCDDevNum;    /* CCD 设备号 */
    fix32_t                  fTargetValue;   /* 目标值 */
} stCCDSensorDeviceTdf;

static void vCCDSensorInit(void *pstSensor)
{
    stCCDSensorDeviceTdf *pstCCD = (stCCDSensorDeviceTdf *)pstSensor;
    (void)pstCCD;
}

static void vCCDSensorPeriodExecute(void *pstSensor)
{
    stCCDSensorDeviceTdf *pstCCD = (stCCDSensorDeviceTdf *)pstSensor;
    vLinerCcdReadData(pstCCD->emCCDDevNum);
    vLinerCcdCalculateThreshold(pstCCD->emCCDDevNum);
    vLinerCcdFindCenterLine(pstCCD->emCCDDevNum);
}

static fix32_t fCCDSensorGetValue(void *pstSensor)
{
    stCCDSensorDeviceTdf *pstCCD = (stCCDSensorDeviceTdf *)pstSensor;
    return (fix32_t)((int64_t)(sLinerCcdGetCenterLine(pstCCD->emCCDDevNum)) * 65536);
}

static void vCCDSensorReset(void *pstSensor)
{
    stCCDSensorDeviceTdf *pstCCD = (stCCDSensorDeviceTdf *)pstSensor;
    pstCCD->fTargetValue = FIX32_ZERO;
}

static void vCCDSensorSetTarget(void *pstSensor, fix32_t fTarget)
{
    stCCDSensorDeviceTdf *pstCCD = (stCCDSensorDeviceTdf *)pstSensor;
    pstCCD->fTargetValue = fTarget;
}

static fix32_t fCCDSensorGetTarget(void *pstSensor)
{
    stCCDSensorDeviceTdf *pstCCD = (stCCDSensorDeviceTdf *)pstSensor;
    return pstCCD->fTargetValue;
}

static stSensorVTableTdf g_stCCDSensorVTable = {
    vCCDSensorInit,
    vCCDSensorPeriodExecute,
    fCCDSensorGetValue,
    vCCDSensorReset,
    vCCDSensorSetTarget,
    fCCDSensorGetTarget,
};

static stCCDSensorDeviceTdf g_astCCDSensorDevices[4];

void vCCDSensorRegister(uint8_t ucLocalIdx, emLinerCcdDevNumTdf emCCDDevNum)
{
    if (ucLocalIdx >= 4) {
        return;
    }

    stCCDSensorDeviceTdf *pstCCD = &g_astCCDSensorDevices[ucLocalIdx];
    memset(pstCCD, 0, sizeof(stCCDSensorDeviceTdf));

    pstCCD->stBase.emType = emSensorTypeCCD;
    pstCCD->stBase.pstVTable = &g_stCCDSensorVTable;
    pstCCD->stBase.ucEnable = 1;
    pstCCD->stBase.fWeight = FIX32_ONE;
    pstCCD->emCCDDevNum = emCCDDevNum;

    vSensorRegisterDevice((emSensorDevNumTdf)(CCD_SENSOR_GLOBAL_BASE + ucLocalIdx), &pstCCD->stBase);
}

#endif /* LINEAR_CCD_IS_ENABLE */

#endif /* SENSOR_IS_ENABLE */
