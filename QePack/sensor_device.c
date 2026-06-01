/**
  * @file       sensor_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/10
  * @brief      传感器抽象基类实现
  */

#include "sensor_device.h"
#include "arithmetic.h"

#if SENSOR_IS_ENABLE

/* ===================== 基类实现 ===================== */

stSensorDeviceTdf *g_astSensorDevices[emSensorMaxDevNum];

/* 子模块注册函数声明（由 vSensorInit 根据设备号区间自动调用） */
#if GRAY_SENSOR_IS_ENABLE
    extern void vGraySensorWrapperRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if LINEAR_CCD_IS_ENABLE
    extern void vCCDSensorRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if ATK_MS901M_IS_ENABLE
    extern void vGyroSensorRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if HWT101_IS_ENABLE
    extern void vHwt101Register(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if MPU6050_IS_ENABLE
    extern void vMpu6050SensorRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if IMU660RB_IS_ENABLE
    extern void vImu660rbSensorRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if BNO08X_IS_ENABLE
    extern void vBno08xSensorRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif
#if ULTRASONIC_IS_ENABLE
    extern void vUltrasonicSensorRegister(emSensorDevNumTdf emDevNum, void *pstInit);
#endif

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
 * @brief 传感器初始化（注册 + 硬件初始化一步完成）
 * @param emDevNum     传感器设备号
 * @param pstInitParams 子模块静态参数指针（可为 NULL，此时仅做硬件初始化）
 */
void vSensorInit(emSensorDevNumTdf emDevNum, void *pstInitParams)
{
    /* 未注册时，根据设备号区间自动调用子模块注册函数 */
    if (pstSensorGetDevice(emDevNum) == NULL && pstInitParams != NULL) {
        #if GRAY_SENSOR_IS_ENABLE
        if (emDevNum >= emSensorGrayDevNum0 && emDevNum <= emSensorGrayDevNum2) {
            vGraySensorWrapperRegister(emDevNum, pstInitParams);
        }
        #endif
        #if LINEAR_CCD_IS_ENABLE
        if (emDevNum >= emSensorCCDDevNum0 && emDevNum <= emSensorCCDDevNum3) {
            vCCDSensorRegister(emDevNum, pstInitParams);
        }
        #endif
        #if ATK_MS901M_IS_ENABLE
        if (emDevNum >= emSensorAtkMs901MDevNum0 && emDevNum <= emSensorAtkMs901MDevNum2) {
            vGyroSensorRegister(emDevNum, pstInitParams);
        }
        #endif
        #if HWT101_IS_ENABLE
        if (emDevNum >= emSensorHWT101DevNum0 && emDevNum <= emSensorHWT101DevNum2) {
            vHwt101Register(emDevNum, pstInitParams);
        }
        #endif
        #if MPU6050_IS_ENABLE
        if (emDevNum >= emSensorMPU6050DevNum0 && emDevNum <= emSensorMPU6050DevNum2) {
            vMpu6050SensorRegister(emDevNum, pstInitParams);
        }
        #endif
        #if IMU660RB_IS_ENABLE
        if (emDevNum >= emSensor660RBDevNum0 && emDevNum <= emSensor660RBDevNum2) {
            vImu660rbSensorRegister(emDevNum, pstInitParams);
        }
        #endif
        #if BNO08X_IS_ENABLE
        if (emDevNum >= emSensorNBO08XDevNum0 && emDevNum <= emSensorNBO08XDevNum2) {
            vBno08xSensorRegister(emDevNum, pstInitParams);
        }
        #endif
        #if ULTRASONIC_IS_ENABLE
        if (emDevNum >= emSensorUltrasonicDevNum0 && emDevNum <= emSensorUltrasonicDevNum3) {
            vUltrasonicSensorRegister(emDevNum, pstInitParams);
        }
        #endif
    }

    /* 走 VTable 初始化硬件 */
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL
        && pstSensor->pstVTable->vInit != NULL) {
        pstSensor->pstVTable->vInit(pstSensor);
    }
}

/**
 * @brief 传感器周期执行
 */
void vSensorPeriodExecute(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor == NULL || pstSensor->pstVTable == NULL) {
        return;
    }

    /* 始终调用 vPeriodExecute（保证原始值和累加值更新） */
    if (pstSensor->pstVTable->vPeriodExecute != NULL) {
        pstSensor->pstVTable->vPeriodExecute(pstSensor);
    }

    /* PID 纠偏计算（ucEnable=0 时跳过）
       反馈值使用累加值（跨圈连续），避免 ±180° 跳变导致 PID 计算混乱 */
    if (pstSensor->ucEnable
        && pstSensor->emPidDevNum != emNoPid && pstSensor->usPidPeriodMs > 0
        && pstSensor->pstVTable->fGetTarget != NULL
        && pstSensor->pstVTable->fGetAccumulatedValue != NULL) {
        uint32_t ulNow = QE_GET_TICK();
        if ((ulNow - pstSensor->ulPidLastTickMs) >= pstSensor->usPidPeriodMs) {
            pstSensor->ulPidLastTickMs = ulNow;
            fix32_t fTarget   = pstSensor->pstVTable->fGetTarget(pstSensor);
            fix32_t fFeedback = pstSensor->pstVTable->fGetAccumulatedValue(pstSensor);
            vPidCalc(pstSensor->emPidDevNum, fTarget, fFeedback);
            ePidGetOutput(pstSensor->emPidDevNum, &pstSensor->fPidOutput);
        }
    } else if (!pstSensor->ucEnable) {
        pstSensor->fPidOutput = FIX32_ZERO;
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
 * @brief 获取传感器累加值（跨圈连续）
 */
fix32_t fSensorGetAccumulatedValue(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL && pstSensor->pstVTable != NULL && pstSensor->pstVTable->fGetAccumulatedValue != NULL) {
        return pstSensor->pstVTable->fGetAccumulatedValue(pstSensor);
    }
    return FIX32_ZERO;
}

/**
 * @brief 获取传感器 PID 输出值
 */
fix32_t fSensorGetPidOutput(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL) {
        return pstSensor->fPidOutput;
    }
    return FIX32_ZERO;
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
 * @brief 设置陀螺仪类传感器轴选择
 */
void vSensorSetAxis(emSensorDevNumTdf emDevNum, emSensorAxisTdf emAxis)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL) {
        pstSensor->emAxis = emAxis;
    }
}

/**
 * @brief 获取陀螺仪类传感器轴选择
 */
emSensorAxisTdf emSensorGetAxis(emSensorDevNumTdf emDevNum)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL) {
        return pstSensor->emAxis;
    }
    return emSensorAxisYaw;
}

/**
 * @brief 配置传感器 PID 纠偏
 * @param emDevNum    传感器设备号
 * @param emPidDevNum PID 设备号（emNoPid 表示禁用）
 * @param usPeriodMs  PID 计算周期 (ms)
 */
void vSensorSetPidConfig(emSensorDevNumTdf emDevNum, emPidDevNumTdf emPidDevNum, uint16_t usPeriodMs)
{
    stSensorDeviceTdf *pstSensor = pstSensorGetDevice(emDevNum);
    if (pstSensor != NULL) {
        pstSensor->emPidDevNum     = emPidDevNum;
        pstSensor->usPidPeriodMs   = usPeriodMs;
        pstSensor->ulPidLastTickMs = QE_GET_TICK();
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

    uint8_t bAValid = (pstA != NULL && pstA->pstVTable != NULL
                       && pstA->pstVTable->fGetValue != NULL);
    uint8_t bBValid = (pstB != NULL && pstB->pstVTable != NULL
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

    int64_t llWeightedSum = (int64_t)fix32_mul(fValA, fWeightA) + (int64_t)fix32_mul(fValB, fWeightB);
    fix32_t fSum;
    if (llWeightedSum > (int64_t)FIX32_MAX) {
        fSum = FIX32_MAX;
    } else if (llWeightedSum < (int64_t)FIX32_MIN) {
        fSum = FIX32_MIN;
    } else {
        fSum = (fix32_t)llWeightedSum;
    }
    return fix32_div(fSum, fTotalWeight);
}

#endif /* SENSOR_IS_ENABLE */
