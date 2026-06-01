/**
 * @file    mpu6050_device.c
 * @author  Qe_xr
 * @version V1.0.0
 * @date    2026/05/15
 * @brief   MPU6050 六轴 IMU 驱动模块实现 (I2C + DMP)
 *
 * 依赖 InvenSense inv_mpu 库，提供 I2C 读写和定时接口。
 * 使用 MPU6050 内置 DMP 实现传感器融合。
 */

#include "mpu6050_device.h"

#if MPU6050_IS_ENABLE

#include "ti_msp_dl_config.h"

/* ==================== InvenSense 库引用 ====================
 * 将以下文件加入工程：
 *   Drivers/MPU6050/inv_mpu.c
 *   Drivers/MPU6050/inv_mpu_dmp_motion_driver.c
 */
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* inv_mpu 库的平台宏映射 */
#define i2c_write   mspm0_i2c_write
#define i2c_read    mspm0_i2c_read
#define delay_ms    mspm0_delay_ms
#define get_ms      mspm0_get_clock_ms

/* ==================== 硬件配置 ==================== */

#define I2C_TIMEOUT_MS  10
#define Q30             (1073741824.0f)
#define RAD_TO_DEG      (180.0f / 3.14159265f)

static stMpu6050DeviceParamTdf g_astMpu6050DeviceParam[MPU6050_DEV_NUM];
static const signed char g_scGyroOrientation[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};

/* inv_mpu 库通过全局宏（i2c_write/i2c_read）回调，无法传递设备号。
 * 通过此变量在调用 inv_mpu 接口前设置当前活跃设备，实现多设备支持。 */
static emMpu6050DevNumTdf g_emActiveDevNum = emMpu6050DevNum0;

/** @brief 获取当前设备 I2C 实例指针（内部） */
static inline I2C_Regs *pstGetI2cInst(emMpu6050DevNumTdf emDevNum)
{
    return g_astMpu6050DeviceParam[emDevNum].stStaticParam.stI2c.i2c_inst;
}

/* ==================== inv_mpu 平台接口实现 ==================== */
/* mspm0_delay_ms 和 mspm0_get_clock_ms 由 ti_platform.c 提供 */

/**
 * @brief  I2C SDA 解锁（时钟脉冲恢复）
 */
static void vI2cSdaUnlock(I2C_Regs *pstI2cInst, GPIO_Regs *pstSclPort, uint32_t ulSclPin,
                          GPIO_Regs *pstSdaPort, uint32_t ulSdaPin,
                          uint32_t ulIomuxSda, uint32_t ulIomuxSdaFunc,
                          uint32_t ulIomuxScl, uint32_t ulIomuxSclFunc)
{
    /* 释放 I2C，将 SCL/SDA 临时切换为 GPIO */
    DL_I2C_reset(pstI2cInst);
    DL_GPIO_initDigitalOutputFeatures(ulIomuxScl, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, DL_GPIO_DRIVE_STRENGTH_HIGH);
    DL_GPIO_initDigitalInputFeatures(ulIomuxSda, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(pstSclPort, ulSclPin);
    DL_GPIO_enableOutput(pstSclPort, ulSclPin);

    uint8_t ucCycle = 0;
    do {
        DL_GPIO_clearPins(pstSclPort, ulSclPin);
        mspm0_delay_ms(1);
        DL_GPIO_setPins(pstSclPort, ulSclPin);
        mspm0_delay_ms(1);
        if (DL_GPIO_readPins(pstSdaPort, ulSdaPin)) break;
    } while (++ucCycle < 100);

    /* 恢复 I2C 功能 */
    DL_I2C_reset(pstI2cInst);
    DL_GPIO_initPeripheralInputFunctionFeatures(ulIomuxSda, ulIomuxSdaFunc, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(ulIomuxScl, ulIomuxSclFunc, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(ulIomuxSda);
    DL_GPIO_enableHiZ(ulIomuxScl);
    DL_I2C_enablePower(pstI2cInst);
    /* 重新调用 SysConfig 初始化 */
    g_astMpu6050DeviceParam[g_emActiveDevNum].stStaticParam.stI2c.vI2cInitFunc();
}

/**
 * @brief  I2C 写寄存器（多字节）
 */
int mspm0_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    I2C_Regs *pstI2c = pstGetI2cInst(g_emActiveDevNum);
    unsigned int cnt = length;
    unsigned char const *ptr = data;
    unsigned long start;

    if (!length) return 0;
    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(pstI2c, reg_addr);
    DL_I2C_clearInterruptStatus(pstI2c, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(pstI2c) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(pstI2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, length + 1);

    do {
        unsigned fillcnt = DL_I2C_fillControllerTXFIFO(pstI2c, ptr, cnt);
        cnt -= fillcnt;
        ptr += fillcnt;

        unsigned long cur;
        mspm0_get_clock_ms(&cur);
        if ((cur - start) >= I2C_TIMEOUT_MS) {
            stMpu6050StaticParamTdf *pstSt = &g_astMpu6050DeviceParam[g_emActiveDevNum].stStaticParam;
            vI2cSdaUnlock(pstI2c, pstSt->stI2c.pstSclGpioPort, pstSt->stI2c.usSclPin,
                          pstSt->stI2c.pstSdaGpioPort, pstSt->stI2c.usSdaPin,
                          pstSt->stI2c.ulIOMuxSda, pstSt->stI2c.ulIOMuxSdaFunc,
                          pstSt->stI2c.ulIOMuxScl, pstSt->stI2c.ulIOMuxSclFunc);
            return -1;
        }
    } while (!DL_I2C_getRawInterruptStatus(pstI2c, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

    return 0;
}

/**
 * @brief  I2C 读寄存器（多字节）
 */
int mspm0_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    I2C_Regs *pstI2c = pstGetI2cInst(g_emActiveDevNum);
    unsigned i = 0;
    unsigned long start;

    if (!length) return 0;
    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(pstI2c, reg_addr);
    pstI2c->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(pstI2c, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    while (!(DL_I2C_getControllerStatus(pstI2c) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(pstI2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, length);

    do {
        if (!DL_I2C_isControllerRXFIFOEmpty(pstI2c)) {
            uint8_t c = DL_I2C_receiveControllerData(pstI2c);
            if (i < length) { data[i] = c; ++i; }
        }
        unsigned long cur;
        mspm0_get_clock_ms(&cur);
        if ((cur - start) >= I2C_TIMEOUT_MS) {
            stMpu6050StaticParamTdf *pstSt = &g_astMpu6050DeviceParam[g_emActiveDevNum].stStaticParam;
            vI2cSdaUnlock(pstI2c, pstSt->stI2c.pstSclGpioPort, pstSt->stI2c.usSclPin,
                          pstSt->stI2c.pstSdaGpioPort, pstSt->stI2c.usSdaPin,
                          pstSt->stI2c.ulIOMuxSda, pstSt->stI2c.ulIOMuxSdaFunc,
                          pstSt->stI2c.ulIOMuxScl, pstSt->stI2c.ulIOMuxSclFunc);
            return -1;
        }
    } while (!DL_I2C_getRawInterruptStatus(pstI2c, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    if (!DL_I2C_isControllerRXFIFOEmpty(pstI2c)) {
        uint8_t c = DL_I2C_receiveControllerData(pstI2c);
        if (i < length) { data[i] = c; ++i; }
    }
    pstI2c->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(pstI2c);
    return (i == length) ? 0 : -1;
}

/* ==================== 方向矩阵辅助函数 ==================== */

static inline unsigned short usInvRow2Scale(const signed char *row)
{
    if (row[0] > 0)      return 0;
    else if (row[0] < 0) return 4;
    else if (row[1] > 0) return 1;
    else if (row[1] < 0) return 5;
    else if (row[2] > 0) return 2;
    else if (row[2] < 0) return 6;
    return 7;
}

static inline unsigned short usInvOrientationToScalar(const signed char *mtx)
{
    unsigned short scalar = usInvRow2Scale(mtx);
    scalar |= usInvRow2Scale(mtx + 3) << 3;
    scalar |= usInvRow2Scale(mtx + 6) << 6;
    return scalar;
}

/* ==================== DMP 回调（空实现） ==================== */

static void vDmpTapCb(unsigned char direction, unsigned char count) { (void)direction; (void)count; }
static void vDmpOrientCb(unsigned char orientation) { (void)orientation; }

/* ==================== QEPack API 实现 ==================== */

/**
 * @brief  初始化 MPU6050 设备（阻塞，加载 DMP 固件）
 */
QE_StatusTypeDef emMpu6050DeviceInit(emMpu6050DevNumTdf emDevNum, const stMpu6050StaticParamTdf *pstInit)
{
    if (emDevNum >= MPU6050_DEV_NUM || pstInit == NULL) return QE_ERROR;

    memset(&g_astMpu6050DeviceParam[emDevNum], 0, sizeof(stMpu6050DeviceParamTdf));
    g_astMpu6050DeviceParam[emDevNum].stStaticParam = *pstInit;

    I2C_Regs *pstI2c = pstGetI2cInst(emDevNum);

    /* SDA 锁死检测与恢复 */
    if (DL_I2C_getSDAStatus(pstI2c) == DL_I2C_CONTROLLER_SDA_LOW) {
        stMpu6050StaticParamTdf *pstSt = &g_astMpu6050DeviceParam[emDevNum].stStaticParam;
        vI2cSdaUnlock(pstI2c, pstSt->stI2c.pstSclGpioPort, pstSt->stI2c.usSclPin,
                      pstSt->stI2c.pstSdaGpioPort, pstSt->stI2c.usSdaPin,
                      pstSt->stI2c.ulIOMuxSda, pstSt->stI2c.ulIOMuxSdaFunc,
                      pstSt->stI2c.ulIOMuxScl, pstSt->stI2c.ulIOMuxSclFunc);
    }

    /* mpu_init 会调用 i2c_write 读取 WHO_AM_I 进行验证 */
    /* 禁用中断防止 ISR 中的 emMpu6050ReadDmp 修改 g_emActiveDevNum 导致竞态 */
    __disable_irq();
    g_emActiveDevNum = emDevNum;
    if (mpu_init() != 0) { __enable_irq(); return QE_ERROR; }
    __enable_irq();

    int result = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned char accel_fsr;
    uint16_t usRate = pstInit->usSampleRateHz ? pstInit->usSampleRateHz : 50;

    __disable_irq();
    g_emActiveDevNum = emDevNum;
    result |= mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    __enable_irq();

    __disable_irq();
    g_emActiveDevNum = emDevNum;
    result |= mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    __enable_irq();

    __disable_irq();
    g_emActiveDevNum = emDevNum;
    result |= mpu_set_sample_rate(usRate);
    result |= mpu_get_sample_rate(&gyro_rate);
    result |= mpu_get_gyro_fsr(&gyro_fsr);
    result |= mpu_get_accel_fsr(&accel_fsr);
    __enable_irq();

    /* 加载 DMP 固件 */
    __disable_irq();
    g_emActiveDevNum = emDevNum;
    result |= dmp_load_motion_driver_firmware();
    result |= dmp_set_orientation(usInvOrientationToScalar(g_scGyroOrientation));
    result |= dmp_register_tap_cb(vDmpTapCb);
    result |= dmp_register_android_orient_cb(vDmpOrientCb);
    result |= dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                  DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
                                  DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    result |= dmp_set_fifo_rate(usRate);
    result |= mpu_set_dmp_state(1);
    __enable_irq();

    if (result != 0) return QE_ERROR;

    g_astMpu6050DeviceParam[emDevNum].stRunningParam.ucDmpReady = 1;
    return QE_OK;
}

/**
 * @brief  读取 DMP FIFO 并更新姿态数据
 * @note   在 MPU6050 INT 引脚 GPIO 中断回调中调用
 *         ISR 仅缓存原始数据，浮点/三角运算延迟到主循环 getter 中执行
 */
QE_StatusTypeDef emMpu6050ReadDmp(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return QE_ERROR;

    stMpu6050RunningParamTdf *pstRun = &g_astMpu6050DeviceParam[emDevNum].stRunningParam;
    if (!pstRun->ucDmpReady) return QE_ERROR;

    /* 保存并设置当前活跃设备，ISR 退出前恢复，防止破坏主循环正在进行的 I2C 事务 */
    emMpu6050DevNumTdf emPrevDev = g_emActiveDevNum;
    g_emActiveDevNum = emDevNum;

    short sGyro[3], sAccel[3], sSensors;
    long lQuat[4];
    unsigned long ulTimestamp;
    unsigned char ucMore;

    int result = dmp_read_fifo(sGyro, sAccel, lQuat, &ulTimestamp, &sSensors, &ucMore);

    g_emActiveDevNum = emPrevDev;

    if (result != 0) return QE_ERROR;

    /* 保存原始值 */
    pstRun->stGyro.sX  = sGyro[0];
    pstRun->stGyro.sY  = sGyro[1];
    pstRun->stGyro.sZ  = sGyro[2];
    pstRun->stAccel.sX = sAccel[0];
    pstRun->stAccel.sY = sAccel[1];
    pstRun->stAccel.sZ = sAccel[2];

    /* 缓存原始四元数（Q30 定点），浮点转换和欧拉角计算延迟到主循环 */
    pstRun->alQuatRaw[0] = lQuat[0];
    pstRun->alQuatRaw[1] = lQuat[1];
    pstRun->alQuatRaw[2] = lQuat[2];
    pstRun->alQuatRaw[3] = lQuat[3];
    pstRun->ucDataUpdated = 1;
    pstRun->ucAttitudeDirty = 1;

    return QE_OK;
}

/* ==================== 数据读取 API ==================== */

/**
 * @brief  从缓存的原始四元数计算欧拉角（主循环调用）
 * @note   包含 asinf/atan2f 等软浮点三角运算，绝不可在 ISR 中调用
 */
static void vMpu6050ComputeAttitude(emMpu6050DevNumTdf emDevNum)
{
    stMpu6050RunningParamTdf *pstRun = &g_astMpu6050DeviceParam[emDevNum].stRunningParam;
    if (!pstRun->ucAttitudeDirty) return;

    float fQ0 = pstRun->alQuatRaw[0] / Q30;
    float fQ1 = pstRun->alQuatRaw[1] / Q30;
    float fQ2 = pstRun->alQuatRaw[2] / Q30;
    float fQ3 = pstRun->alQuatRaw[3] / Q30;

    pstRun->stQuat.fQ0 = fQ0;
    pstRun->stQuat.fQ1 = fQ1;
    pstRun->stQuat.fQ2 = fQ2;
    pstRun->stQuat.fQ3 = fQ3;

    pstRun->stAttitude.fPitch = asinf(-2.0f * fQ1 * fQ3 + 2.0f * fQ0 * fQ2) * RAD_TO_DEG;
    pstRun->stAttitude.fRoll  = atan2f(2.0f * fQ2 * fQ3 + 2.0f * fQ0 * fQ1, -2.0f * fQ1 * fQ1 - 2.0f * fQ2 * fQ2 + 1.0f) * RAD_TO_DEG;
    pstRun->stAttitude.fYaw   = atan2f(2.0f * (fQ1 * fQ2 + fQ0 * fQ3), fQ0 * fQ0 + fQ1 * fQ1 - fQ2 * fQ2 - fQ3 * fQ3) * RAD_TO_DEG;
    pstRun->ucAttitudeDirty = 0;
}

void vMpu6050GetAttitude(emMpu6050DevNumTdf emDevNum, stMpu6050AttitudeTdf *pstAttitude)
{
    if (emDevNum >= MPU6050_DEV_NUM || pstAttitude == NULL) return;
    vMpu6050ComputeAttitude(emDevNum);
    *pstAttitude = g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude;
}

float fMpu6050GetPitch(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return 0.0f;
    vMpu6050ComputeAttitude(emDevNum);
    return g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude.fPitch;
}

float fMpu6050GetRoll(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return 0.0f;
    vMpu6050ComputeAttitude(emDevNum);
    return g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude.fRoll;
}

float fMpu6050GetYaw(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return 0.0f;
    vMpu6050ComputeAttitude(emDevNum);
    return g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude.fYaw;
}

void vMpu6050GetGyroRaw(emMpu6050DevNumTdf emDevNum, stMpu6050GyroRawTdf *pstGyro)
{
    if (emDevNum >= MPU6050_DEV_NUM || pstGyro == NULL) return;
    *pstGyro = g_astMpu6050DeviceParam[emDevNum].stRunningParam.stGyro;
}

void vMpu6050GetAccelRaw(emMpu6050DevNumTdf emDevNum, stMpu6050AccelRawTdf *pstAccel)
{
    if (emDevNum >= MPU6050_DEV_NUM || pstAccel == NULL) return;
    *pstAccel = g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAccel;
}

/* ==================== 传感器基类适配 ==================== */

#if SENSOR_IS_ENABLE

#include "sensor_device.h"

#define MPU6050_SENSOR_LOCAL_MAX  3
#define MPU6050_SENSOR_TO_LOCAL(dev)  ((uint8_t)((dev) - emSensorMPU6050DevNum0))

typedef struct {
    stSensorDeviceTdf      stBase;
    emSensorDevNumTdf      emSensorDevNum;
    fix32_t                fCurrentValue;
    fix32_t                fLastYaw;
    fix32_t                fAccumulatedYaw;
    fix32_t                fTargetValue;
    int32_t                lTurnCount;
} stMpu6050SensorWrapperTdf;

static void vMpu6050SensorInit(void *pstSensor)
{
    (void)pstSensor;
}

static void vMpu6050SensorPeriodExecute(void *pstSensor)
{
    stMpu6050SensorWrapperTdf *pstWrapper = (stMpu6050SensorWrapperTdf *)pstSensor;
    emMpu6050DevNumTdf emLocalDev = (emMpu6050DevNumTdf)MPU6050_SENSOR_TO_LOCAL(pstWrapper->emSensorDevNum);
    if (emLocalDev >= MPU6050_DEV_NUM) return;

    vMpu6050ComputeAttitude(emLocalDev);

    if (pstWrapper->stBase.emAxis == emSensorAxisYaw) {
        pstWrapper->fLastYaw = pstWrapper->fCurrentValue;
        pstWrapper->fCurrentValue = fix32_from_float(g_astMpu6050DeviceParam[emLocalDev].stRunningParam.stAttitude.fYaw);

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
                fVal = g_astMpu6050DeviceParam[emLocalDev].stRunningParam.stAttitude.fPitch;
                break;
            case emSensorAxisRoll:
                fVal = g_astMpu6050DeviceParam[emLocalDev].stRunningParam.stAttitude.fRoll;
                break;
            default:
                fVal = g_astMpu6050DeviceParam[emLocalDev].stRunningParam.stAttitude.fYaw;
                break;
        }
        pstWrapper->fCurrentValue = fix32_from_float(fVal);
        pstWrapper->fAccumulatedYaw = pstWrapper->fCurrentValue;
    }
}

static fix32_t fMpu6050SensorGetValue(void *pstSensor)
{
    stMpu6050SensorWrapperTdf *pstWrapper = (stMpu6050SensorWrapperTdf *)pstSensor;
    return pstWrapper->fCurrentValue;
}

static fix32_t fMpu6050SensorGetAccumulatedValue(void *pstSensor)
{
    stMpu6050SensorWrapperTdf *pstWrapper = (stMpu6050SensorWrapperTdf *)pstSensor;
    return pstWrapper->fAccumulatedYaw;
}

static void vMpu6050SensorReset(void *pstSensor)
{
    stMpu6050SensorWrapperTdf *pstWrapper = (stMpu6050SensorWrapperTdf *)pstSensor;
    pstWrapper->fCurrentValue    = FIX32_ZERO;
    pstWrapper->fLastYaw         = FIX32_ZERO;
    pstWrapper->fAccumulatedYaw  = FIX32_ZERO;
    pstWrapper->fTargetValue     = FIX32_ZERO;
    pstWrapper->lTurnCount       = 0;
}

static void vMpu6050SensorSetTarget(void *pstSensor, fix32_t fTarget)
{
    stMpu6050SensorWrapperTdf *pstWrapper = (stMpu6050SensorWrapperTdf *)pstSensor;
    pstWrapper->fTargetValue = fTarget;
}

static fix32_t fMpu6050SensorGetTarget(void *pstSensor)
{
    stMpu6050SensorWrapperTdf *pstWrapper = (stMpu6050SensorWrapperTdf *)pstSensor;
    return pstWrapper->fTargetValue;
}

static stSensorVTableTdf g_stMpu6050SensorVTable = {
    vMpu6050SensorInit,
    vMpu6050SensorPeriodExecute,
    fMpu6050SensorGetValue,
    fMpu6050SensorGetAccumulatedValue,
    vMpu6050SensorReset,
    vMpu6050SensorSetTarget,
    fMpu6050SensorGetTarget,
};

static stMpu6050SensorWrapperTdf g_astMpu6050SensorDevices[MPU6050_SENSOR_LOCAL_MAX];

void vMpu6050SensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit)
{
    uint8_t ucLocalIdx = MPU6050_SENSOR_TO_LOCAL(emSensorDevNum);
    if (ucLocalIdx >= MPU6050_SENSOR_LOCAL_MAX) return;

    /* 在注册阶段完成硬件初始化 */
    if (pstInit != NULL) {
        emMpu6050DeviceInit((emMpu6050DevNumTdf)ucLocalIdx, (const stMpu6050StaticParamTdf *)pstInit);
    }

    stMpu6050SensorWrapperTdf *pstWrapper = &g_astMpu6050SensorDevices[ucLocalIdx];
    memset(pstWrapper, 0, sizeof(stMpu6050SensorWrapperTdf));

    pstWrapper->stBase.emType          = emSensorTypeMPU6050Gyro;
    pstWrapper->stBase.pstVTable       = &g_stMpu6050SensorVTable;
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
