/**
 * @file    bno08x_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/05/15
 * @brief   BNO08x 九轴 IMU 驱动模块实现 (UART RVC 模式)
 */

#include "bno08x_device.h"

#if BNO08X_IS_ENABLE

#include "ti_msp_dl_config.h"

stBno08xDeviceParamTdf g_astBno08xDeviceParam[BNO08X_DEV_NUM];

/* BNO08x RVC 数据包固定长度 (含 2 字节头 + 16 字节数据 + 1 字节校验) */
#define BNO08X_PACKET_SIZE  19

/**
 * @brief  初始化 BNO08x 设备
 * @param  emDevNum     BNO08x 设备号
 * @param  emUartDevNum 绑定的 UART 设备号
 */
void vBno08xDeviceInit(emBno08xDevNumTdf emDevNum, emUartDevNumTdf emUartDevNum)
{
    if (emDevNum >= BNO08X_DEV_NUM) return;

    memset(&g_astBno08xDeviceParam[emDevNum], 0, sizeof(stBno08xDeviceParamTdf));
    g_astBno08xDeviceParam[emDevNum].stStaticParam.emUartDevNum = emUartDevNum;
}

/**
 * @brief  UART 中断服务例程 — 解析 BNO08x RVC 数据包
 * @note   由 ti_interrupt.c 中的 UART ISR 调用
 * @param  emDevNum BNO08x 设备号
 */
void vBno08xUartIsr(emBno08xDevNumTdf emDevNum)
{
    if (emDevNum >= BNO08X_DEV_NUM) return;

    stBno08xRunningParamTdf *pstRun = &g_astBno08xDeviceParam[emDevNum].stRunningParam;

    /* 禁止 DMA，计算已接收字节数 */
    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t ucRxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    /* 若有 FIFO 残留字节则一并读取 */
    if (DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false) {
        pstRun->aucDmaBuf[ucRxSize++] = DL_UART_receiveData(UART_BNO08X_INST);
    }

    /* 校验数据包：长度 19、帧头 0xAA 0xAA、校验和 */
    if (ucRxSize == BNO08X_PACKET_SIZE
        && pstRun->aucDmaBuf[0] == 0xAA
        && pstRun->aucDmaBuf[1] == 0xAA)
    {
        uint8_t ucSum = 0;
        for (uint8_t i = 2; i <= 14; i++) {
            ucSum += pstRun->aucDmaBuf[i];
        }
        if (ucSum == pstRun->aucDmaBuf[18])
        {
            stBno08xDataTdf *pstData = &pstRun->stData;
            pstData->ucIndex = pstRun->aucDmaBuf[2];
            /* 数据格式: int16 × 0.01° */
            pstData->stAttitude.fYaw   = (int16_t)((pstRun->aucDmaBuf[4]  << 8) | pstRun->aucDmaBuf[3])  * 0.01f;
            pstData->stAttitude.fPitch = (int16_t)((pstRun->aucDmaBuf[6]  << 8) | pstRun->aucDmaBuf[5])  * 0.01f;
            pstData->stAttitude.fRoll  = (int16_t)((pstRun->aucDmaBuf[8]  << 8) | pstRun->aucDmaBuf[7])  * 0.01f;
            pstData->stAccel.sX        = (int16_t)((pstRun->aucDmaBuf[10] << 8) | pstRun->aucDmaBuf[9]);
            pstData->stAccel.sY        = (int16_t)((pstRun->aucDmaBuf[12] << 8) | pstRun->aucDmaBuf[11]);
            pstData->stAccel.sZ        = (int16_t)((pstRun->aucDmaBuf[14] << 8) | pstRun->aucDmaBuf[13]);
            pstData->ucUpdated = 1;
        }
    }

    /* 清空 FIFO 残留 */
    {
        uint8_t aucDummy[4];
        DL_UART_drainRXFIFO(UART_BNO08X_INST, aucDummy, 4);
    }

    /* 重新启动 DMA */
    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t)&pstRun->aucDmaBuf[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}

/**
 * @brief  读取最新传感器数据（非阻塞）
 * @param  emDevNum BNO08x 设备号
 * @return 数据指针，未收到数据时返回 NULL
 */
const stBno08xDataTdf *pstBno08xReadData(emBno08xDevNumTdf emDevNum)
{
    if (emDevNum >= BNO08X_DEV_NUM) return NULL;
    return &g_astBno08xDeviceParam[emDevNum].stRunningParam.stData;
}

/**
 * @brief  获取姿态角
 */
void vBno08xGetAttitude(emBno08xDevNumTdf emDevNum, stBno08xAttitudeTdf *pstAttitude)
{
    if (emDevNum >= BNO08X_DEV_NUM || pstAttitude == NULL) return;
    stBno08xDataTdf *pstData = &g_astBno08xDeviceParam[emDevNum].stRunningParam.stData;
    pstAttitude->fPitch = pstData->stAttitude.fPitch;
    pstAttitude->fRoll  = pstData->stAttitude.fRoll;
    pstAttitude->fYaw   = pstData->stAttitude.fYaw;
}

float fBno08xGetPitch(emBno08xDevNumTdf emDevNum)
{
    if (emDevNum >= BNO08X_DEV_NUM) return 0.0f;
    return g_astBno08xDeviceParam[emDevNum].stRunningParam.stData.stAttitude.fPitch;
}

float fBno08xGetRoll(emBno08xDevNumTdf emDevNum)
{
    if (emDevNum >= BNO08X_DEV_NUM) return 0.0f;
    return g_astBno08xDeviceParam[emDevNum].stRunningParam.stData.stAttitude.fRoll;
}

float fBno08xGetYaw(emBno08xDevNumTdf emDevNum)
{
    if (emDevNum >= BNO08X_DEV_NUM) return 0.0f;
    return g_astBno08xDeviceParam[emDevNum].stRunningParam.stData.stAttitude.fYaw;
}

/**
 * @brief  获取加速度原始值
 */
void vBno08xGetAccel(emBno08xDevNumTdf emDevNum, stBno08xAccelTdf *pstAccel)
{
    if (emDevNum >= BNO08X_DEV_NUM || pstAccel == NULL) return;
    stBno08xDataTdf *pstData = &g_astBno08xDeviceParam[emDevNum].stRunningParam.stData;
    pstAccel->sX = pstData->stAccel.sX;
    pstAccel->sY = pstData->stAccel.sY;
    pstAccel->sZ = pstData->stAccel.sZ;
}

/* ==================== 传感器基类适配 ==================== */

#if SENSOR_IS_ENABLE

#include "sensor_device.h"

#define BNO08X_SENSOR_LOCAL_MAX  3
#define BNO08X_SENSOR_TO_LOCAL(dev)  ((uint8_t)((dev) - emSensorNBO08XDevNum0))

typedef struct {
    stSensorDeviceTdf      stBase;
    emSensorDevNumTdf      emSensorDevNum;
    fix32_t                fCurrentValue;
    fix32_t                fLastYaw;
    fix32_t                fAccumulatedYaw;
    fix32_t                fTargetValue;
    int32_t                lTurnCount;
} stBno08xSensorWrapperTdf;

static void vBno08xSensorInit(void *pstSensor)
{
    (void)pstSensor;
}

static void vBno08xSensorPeriodExecute(void *pstSensor)
{
    stBno08xSensorWrapperTdf *pstWrapper = (stBno08xSensorWrapperTdf *)pstSensor;
    emBno08xDevNumTdf emLocalDev = (emBno08xDevNumTdf)BNO08X_SENSOR_TO_LOCAL(pstWrapper->emSensorDevNum);
    if (emLocalDev >= BNO08X_DEV_NUM) return;

    if (pstWrapper->stBase.emAxis == emSensorAxisYaw) {
        pstWrapper->fLastYaw = pstWrapper->fCurrentValue;
        pstWrapper->fCurrentValue = fix32_from_float(g_astBno08xDeviceParam[emLocalDev].stRunningParam.stData.stAttitude.fYaw);

        fix32_t fDelta = pstWrapper->fCurrentValue - pstWrapper->fLastYaw;
        if (fDelta > (fix32_t)(180 * 65536)) {
            pstWrapper->lTurnCount--;
        } else if (fDelta < (fix32_t)(-180 * 65536)) {
            pstWrapper->lTurnCount++;
        }
        pstWrapper->fAccumulatedYaw = pstWrapper->fCurrentValue
            + (fix32_t)((int64_t)pstWrapper->lTurnCount * 360 * 65536);
    } else {
        float fVal;
        switch (pstWrapper->stBase.emAxis) {
            case emSensorAxisPitch:
                fVal = g_astBno08xDeviceParam[emLocalDev].stRunningParam.stData.stAttitude.fPitch;
                break;
            case emSensorAxisRoll:
                fVal = g_astBno08xDeviceParam[emLocalDev].stRunningParam.stData.stAttitude.fRoll;
                break;
            default:
                fVal = g_astBno08xDeviceParam[emLocalDev].stRunningParam.stData.stAttitude.fYaw;
                break;
        }
        pstWrapper->fCurrentValue = fix32_from_float(fVal);
        pstWrapper->fAccumulatedYaw = pstWrapper->fCurrentValue;
    }
}

static fix32_t fBno08xSensorGetValue(void *pstSensor)
{
    stBno08xSensorWrapperTdf *pstWrapper = (stBno08xSensorWrapperTdf *)pstSensor;
    return pstWrapper->fCurrentValue;
}

static fix32_t fBno08xSensorGetAccumulatedValue(void *pstSensor)
{
    stBno08xSensorWrapperTdf *pstWrapper = (stBno08xSensorWrapperTdf *)pstSensor;
    return pstWrapper->fAccumulatedYaw;
}

static void vBno08xSensorReset(void *pstSensor)
{
    stBno08xSensorWrapperTdf *pstWrapper = (stBno08xSensorWrapperTdf *)pstSensor;
    pstWrapper->fCurrentValue    = FIX32_ZERO;
    pstWrapper->fLastYaw         = FIX32_ZERO;
    pstWrapper->fAccumulatedYaw  = FIX32_ZERO;
    pstWrapper->fTargetValue     = FIX32_ZERO;
    pstWrapper->lTurnCount       = 0;
}

static void vBno08xSensorSetTarget(void *pstSensor, fix32_t fTarget)
{
    stBno08xSensorWrapperTdf *pstWrapper = (stBno08xSensorWrapperTdf *)pstSensor;
    pstWrapper->fTargetValue = fTarget;
}

static fix32_t fBno08xSensorGetTarget(void *pstSensor)
{
    stBno08xSensorWrapperTdf *pstWrapper = (stBno08xSensorWrapperTdf *)pstSensor;
    return pstWrapper->fTargetValue;
}

static stSensorVTableTdf g_stBno08xSensorVTable = {
    vBno08xSensorInit,
    vBno08xSensorPeriodExecute,
    fBno08xSensorGetValue,
    fBno08xSensorGetAccumulatedValue,
    vBno08xSensorReset,
    vBno08xSensorSetTarget,
    fBno08xSensorGetTarget,
};

static stBno08xSensorWrapperTdf g_astBno08xSensorDevices[BNO08X_SENSOR_LOCAL_MAX];

void vBno08xSensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit)
{
    uint8_t ucLocalIdx = BNO08X_SENSOR_TO_LOCAL(emSensorDevNum);
    if (ucLocalIdx >= BNO08X_SENSOR_LOCAL_MAX) return;

    /* 在注册阶段完成硬件初始化（pstInit 指向 emUartDevNumTdf） */
    if (pstInit != NULL) {
        emUartDevNumTdf emUartDevNum = *(emUartDevNumTdf *)pstInit;
        vBno08xDeviceInit((emBno08xDevNumTdf)ucLocalIdx, emUartDevNum);
    }

    stBno08xSensorWrapperTdf *pstWrapper = &g_astBno08xSensorDevices[ucLocalIdx];
    memset(pstWrapper, 0, sizeof(stBno08xSensorWrapperTdf));

    pstWrapper->stBase.emType          = emSensorTypeNBO08XGyro;
    pstWrapper->stBase.pstVTable       = &g_stBno08xSensorVTable;
    pstWrapper->stBase.ucEnable        = 1;
    pstWrapper->stBase.fWeight         = FIX32_ONE;
    pstWrapper->stBase.emAxis          = emSensorAxisYaw;
    pstWrapper->stBase.emPidDevNum     = emNoPid;
    pstWrapper->stBase.usPidPeriodMs   = 0;
    pstWrapper->stBase.ulPidLastTickMs = 0;
    pstWrapper->emSensorDevNum         = emSensorDevNum;

    vSensorRegisterDevice(emSensorDevNum, &pstWrapper->stBase);
}

#endif /* SENSOR_IS_ENABLE */

#endif
