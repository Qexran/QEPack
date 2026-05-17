/**
 * @file    imu660rb_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/05/15
 * @brief   IMU660RB 六轴 IMU 驱动模块实现 (SPI + LSM6DSR + Fusion AHRS)
 *
 * 依赖 ST lsm6dsr_reg 寄存器库和 Fusion AHRS 传感器融合库。
 */

#include "imu660rb_device.h"

#if IMU660RB_IS_ENABLE

#include "ti_msp_dl_config.h"

/* 第三方库 — 需将以下文件加入 CCS 工程：
 *   Drivers/IMU660RB/lsm6dsr_reg.c
 *   Drivers/IMU660RB/Fusion/FusionAhrs.c
 *   Drivers/IMU660RB/Fusion/FusionOffset.c
 */
#include "lsm6dsr_reg.h"
#include "Fusion.h"

/* ==================== 校准常量 ==================== */

#define BOOT_TIME          10
#define OFFSET_CAL_TIME    50
#define ODR_COEFF_52Hz     128

/* ==================== 全局设备参数 ==================== */

stImu660rbDeviceParamTdf g_astImu660rbDeviceParam[IMU660RB_DEV_NUM];

/* ==================== 内部状态（不对外暴露） ==================== */

typedef struct {
    stmdev_ctx_t      devCtx;            /* lsm6dsr 设备上下文 */
    FusionAhrs        ahrs;              /* AHRS 融合实例 */
    FusionOffset      offset;            /* 陀螺仪零偏校准 */
    FusionEuler       euler;             /* 欧拉角输出 */
    FusionMatrix      gyroMisalignment;  /* 陀螺仪安装误差矩阵 */
    FusionVector      gyroSensitivity;   /* 陀螺仪灵敏度 */
    FusionVector      gyroOffset;        /* 陀螺仪零偏 */
    int16_t           sAccelRaw[3];      /* 加速度原始值 */
    int16_t           sGyroRaw[3];       /* 陀螺仪原始值 */
    float             fSampleRate;       /* 实际采样率 (Hz) */
    float             fSamplePeriod;     /* 采样周期 (s) */
} stImu660rbInternalTdf;

static stImu660rbInternalTdf g_astInternal[IMU660RB_DEV_NUM];

/* ==================== SPI 底层操作 ==================== */

/**
 * @brief  SPI 单字节收发
 */
static uint8_t ucSpiTransferByte(SPI_Regs *pstSpiInst, uint8_t ucData)
{
    DL_SPI_transmitData8(pstSpiInst, ucData);
    while (DL_SPI_isRXFIFOEmpty(pstSpiInst));
    uint8_t ucRx = DL_SPI_receiveData8(pstSpiInst);
    while (DL_SPI_isBusy(pstSpiInst));
    return ucRx;
}

/**
 * @brief  平台延时 (ms)
 */
static void vPlatformDelay(uint32_t ulMs)
{
    DL_Common_delayCycles(ulMs * CPUCLK_FREQ / 1000);
}

/* ==================== lsm6dsr_reg 平台接口 ==================== */

/**
 * @brief  SPI 写寄存器（实现 stmdev_write_ptr 接口）
 * @param  handle  设备号 (emImu660rbDevNumTdf 转为 void*)
 */
static int32_t lPlatformWrite(void *handle, uint8_t ucReg, const uint8_t *pucBuf, uint16_t usLen)
{
    emImu660rbDevNumTdf emDevNum = (emImu660rbDevNumTdf)(uintptr_t)handle;
    stImu660rbSpiTdf *pstSpi = &g_astImu660rbDeviceParam[emDevNum].stStaticParam.stSpi;

    DL_GPIO_clearPins(pstSpi->pstCsPort, pstSpi->ulCsPin);
    ucSpiTransferByte(pstSpi->spi_inst, ucReg);
    while (usLen--) {
        ucSpiTransferByte(pstSpi->spi_inst, *pucBuf++);
    }
    DL_GPIO_setPins(pstSpi->pstCsPort, pstSpi->ulCsPin);
    return 0;
}

/**
 * @brief  SPI 读寄存器（实现 stmdev_read_ptr 接口）
 * @param  handle  设备号 (emImu660rbDevNumTdf 转为 void*)
 */
static int32_t lPlatformRead(void *handle, uint8_t ucReg, uint8_t *pucBuf, uint16_t usLen)
{
    emImu660rbDevNumTdf emDevNum = (emImu660rbDevNumTdf)(uintptr_t)handle;
    stImu660rbSpiTdf *pstSpi = &g_astImu660rbDeviceParam[emDevNum].stStaticParam.stSpi;

    DL_GPIO_clearPins(pstSpi->pstCsPort, pstSpi->ulCsPin);
    ucSpiTransferByte(pstSpi->spi_inst, ucReg | 0x80);
    while (usLen--) {
        *pucBuf++ = ucSpiTransferByte(pstSpi->spi_inst, 0x00);
    }
    DL_GPIO_setPins(pstSpi->pstCsPort, pstSpi->ulCsPin);
    return 0;
}

/* ==================== QEPack API 实现 ==================== */

/**
 * @brief  初始化 IMU660RB 设备（阻塞，含陀螺仪零偏校准）
 */
QE_StatusTypeDef emImu660rbDeviceInit(emImu660rbDevNumTdf emDevNum, const stImu660rbStaticParamTdf *pstInit)
{
    if (emDevNum >= IMU660RB_DEV_NUM || pstInit == NULL) return QE_ERROR;

    memset(&g_astImu660rbDeviceParam[emDevNum], 0, sizeof(stImu660rbDeviceParamTdf));
    memset(&g_astInternal[emDevNum], 0, sizeof(stImu660rbInternalTdf));
    g_astImu660rbDeviceParam[emDevNum].stStaticParam = *pstInit;

    stImu660rbInternalTdf *pstInt = &g_astInternal[emDevNum];

    /* 初始化 dev_ctx */
    pstInt->devCtx.write_reg = lPlatformWrite;
    pstInt->devCtx.read_reg  = lPlatformRead;
    pstInt->devCtx.mdelay    = vPlatformDelay;
    pstInt->devCtx.handle    = (void *)(uintptr_t)emDevNum;

    /* 等待传感器启动 */
    vPlatformDelay(BOOT_TIME);

    /* 检查设备 ID */
    uint8_t ucWhoAmI;
    lsm6dsr_device_id_get(&pstInt->devCtx, &ucWhoAmI);
    if (ucWhoAmI != LSM6DSR_ID) return QE_ERROR;

    /* 复位设备 */
    uint8_t ucRst;
    lsm6dsr_reset_set(&pstInt->devCtx, PROPERTY_ENABLE);
    do {
        lsm6dsr_reset_get(&pstInt->devCtx, &ucRst);
    } while (ucRst);

    /* 禁用 I3C */
    lsm6dsr_i3c_disable_set(&pstInt->devCtx, LSM6DSR_I3C_DISABLE);

    /* 启用 Block Data Update */
    lsm6dsr_block_data_update_set(&pstInt->devCtx, PROPERTY_ENABLE);

    /* 设置输出数据率 (52Hz) */
    lsm6dsr_xl_data_rate_set(&pstInt->devCtx, LSM6DSR_XL_ODR_52Hz);
    lsm6dsr_gy_data_rate_set(&pstInt->devCtx, LSM6DSR_GY_ODR_52Hz);

    /* 设置满量程: ±2g 加速度, ±2000dps 陀螺仪 */
    lsm6dsr_xl_full_scale_set(&pstInt->devCtx, LSM6DSR_2g);
    lsm6dsr_gy_full_scale_set(&pstInt->devCtx, LSM6DSR_2000dps);

    /* 启用陀螺仪低通滤波 */
    lsm6dsr_gy_filter_lp1_set(&pstInt->devCtx, 1);

    /* 配置 INT1 引脚输出数据就绪信号 */
    lsm6dsr_pin_int1_route_t int1Route;
    lsm6dsr_pin_int1_route_get(&pstInt->devCtx, &int1Route);
    int1Route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    lsm6dsr_pin_int1_route_set(&pstInt->devCtx, &int1Route);
    lsm6dsr_data_ready_mode_set(&pstInt->devCtx, LSM6DSR_DRDY_PULSED);

    /* 计算实际采样率 */
    int8_t cFreqFine;
    lsm6dsr_odr_cal_reg_get(&pstInt->devCtx, &cFreqFine);
    pstInt->fSampleRate = (6667.0f + (0.0015f * cFreqFine) * 6667.0f) / ODR_COEFF_52Hz;
    pstInt->fSamplePeriod = 1.0f / pstInt->fSampleRate;

    /* 初始化校准矩阵（默认单位矩阵） */
    pstInt->gyroMisalignment = (FusionMatrix){.array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    pstInt->gyroSensitivity  = (FusionVector){.array = {1.0f, 1.0f, 1.0f}};
    pstInt->gyroOffset       = (FusionVector){.array = {0.0f, 0.0f, 0.0f}};

    /* 初始化 Fusion AHRS 和零偏校准 */
    FusionAhrsInitialise(&pstInt->ahrs);
    FusionOffsetInitialise(&pstInt->offset, (unsigned int)pstInt->fSampleRate);

    vPlatformDelay(200);

    /* 陀螺仪零偏校准（采集 OFFSET_CAL_TIME 个样本取平均） */
    uint8_t ucOffsetCnt = OFFSET_CAL_TIME;
    while (ucOffsetCnt) {
        uint8_t ucDataReady;
        lsm6dsr_gy_flag_data_ready_get(&pstInt->devCtx, &ucDataReady);
        if (ucDataReady) {
            ucOffsetCnt--;
            lsm6dsr_angular_rate_raw_get(&pstInt->devCtx, pstInt->sGyroRaw);
            pstInt->gyroOffset.array[0] += lsm6dsr_from_fs2000dps_to_mdps(pstInt->sGyroRaw[0]) / 1000.0f;
            pstInt->gyroOffset.array[1] += lsm6dsr_from_fs2000dps_to_mdps(pstInt->sGyroRaw[1]) / 1000.0f;
            pstInt->gyroOffset.array[2] += lsm6dsr_from_fs2000dps_to_mdps(pstInt->sGyroRaw[2]) / 1000.0f;
        }
    }
    pstInt->gyroOffset.array[0] /= OFFSET_CAL_TIME;
    pstInt->gyroOffset.array[1] /= OFFSET_CAL_TIME;
    pstInt->gyroOffset.array[2] /= OFFSET_CAL_TIME;

    return QE_OK;
}

/**
 * @brief  读取传感器数据并更新姿态
 * @note   在 GPIO INT1 中断回调中调用（GROUP1_IRQHandler）
 */
void vImu660rbReadData(emImu660rbDevNumTdf emDevNum)
{
    if (emDevNum >= IMU660RB_DEV_NUM) return;

    stImu660rbInternalTdf *pstInt = &g_astInternal[emDevNum];
    stImu660rbRunningParamTdf *pstRun = &g_astImu660rbDeviceParam[emDevNum].stRunningParam;

    /* 读取加速度原始值并转换为 mg */
    lsm6dsr_acceleration_raw_get(&pstInt->devCtx, pstInt->sAccelRaw);
    float fAccelMg[3];
    fAccelMg[0] = lsm6dsr_from_fs2g_to_mg(pstInt->sAccelRaw[0]);
    fAccelMg[1] = lsm6dsr_from_fs2g_to_mg(pstInt->sAccelRaw[1]);
    fAccelMg[2] = lsm6dsr_from_fs2g_to_mg(pstInt->sAccelRaw[2]);

    /* 读取陀螺仪原始值并转换为 mdps */
    lsm6dsr_angular_rate_raw_get(&pstInt->devCtx, pstInt->sGyroRaw);
    float fGyroMdps[3];
    fGyroMdps[0] = lsm6dsr_from_fs2000dps_to_mdps(pstInt->sGyroRaw[0]);
    fGyroMdps[1] = lsm6dsr_from_fs2000dps_to_mdps(pstInt->sGyroRaw[1]);
    fGyroMdps[2] = lsm6dsr_from_fs2000dps_to_mdps(pstInt->sGyroRaw[2]);

    /* 保存原始值到运行参数 */
    pstRun->stAccel.fX = fAccelMg[0];
    pstRun->stAccel.fY = fAccelMg[1];
    pstRun->stAccel.fZ = fAccelMg[2];
    pstRun->stGyro.fX = fGyroMdps[0];
    pstRun->stGyro.fY = fGyroMdps[1];
    pstRun->stGyro.fZ = fGyroMdps[2];

    /* Fusion AHRS 传感器融合 */
    FusionVector vAccel = {.array = {fAccelMg[0] / 1000.0f, fAccelMg[1] / 1000.0f, fAccelMg[2] / 1000.0f}};
    FusionVector vGyro  = {.array = {fGyroMdps[0] / 1000.0f, fGyroMdps[1] / 1000.0f, fGyroMdps[2] / 1000.0f}};

    vGyro = FusionCalibrationInertial(vGyro, pstInt->gyroMisalignment, pstInt->gyroSensitivity, pstInt->gyroOffset);
    vGyro = FusionOffsetUpdate(&pstInt->offset, vGyro);

    FusionAhrsUpdateNoMagnetometer(&pstInt->ahrs, vGyro, vAccel, pstInt->fSamplePeriod);
    pstInt->euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&pstInt->ahrs));

    /* 更新姿态输出 */
    pstRun->stAttitude.fPitch = pstInt->euler.angle.pitch;
    pstRun->stAttitude.fRoll  = pstInt->euler.angle.roll;
    pstRun->stAttitude.fYaw   = pstInt->euler.angle.yaw;

    pstRun->ucDataReady = 1;
    pstRun->ucAhrsInit  = pstInt->ahrs.initialising ? 1 : 0;
}

/* ==================== 数据读取 API ==================== */

void vImu660rbGetAttitude(emImu660rbDevNumTdf emDevNum, stImu660rbAttitudeTdf *pstAttitude)
{
    if (emDevNum >= IMU660RB_DEV_NUM || pstAttitude == NULL) return;
    *pstAttitude = g_astImu660rbDeviceParam[emDevNum].stRunningParam.stAttitude;
}

float fImu660rbGetPitch(emImu660rbDevNumTdf emDevNum)
{
    if (emDevNum >= IMU660RB_DEV_NUM) return 0.0f;
    return g_astImu660rbDeviceParam[emDevNum].stRunningParam.stAttitude.fPitch;
}

float fImu660rbGetRoll(emImu660rbDevNumTdf emDevNum)
{
    if (emDevNum >= IMU660RB_DEV_NUM) return 0.0f;
    return g_astImu660rbDeviceParam[emDevNum].stRunningParam.stAttitude.fRoll;
}

float fImu660rbGetYaw(emImu660rbDevNumTdf emDevNum)
{
    if (emDevNum >= IMU660RB_DEV_NUM) return 0.0f;
    return g_astImu660rbDeviceParam[emDevNum].stRunningParam.stAttitude.fYaw;
}

void vImu660rbGetAccel(emImu660rbDevNumTdf emDevNum, stImu660rbAccelTdf *pstAccel)
{
    if (emDevNum >= IMU660RB_DEV_NUM || pstAccel == NULL) return;
    *pstAccel = g_astImu660rbDeviceParam[emDevNum].stRunningParam.stAccel;
}

void vImu660rbGetGyro(emImu660rbDevNumTdf emDevNum, stImu660rbGyroTdf *pstGyro)
{
    if (emDevNum >= IMU660RB_DEV_NUM || pstGyro == NULL) return;
    *pstGyro = g_astImu660rbDeviceParam[emDevNum].stRunningParam.stGyro;
}

#endif
