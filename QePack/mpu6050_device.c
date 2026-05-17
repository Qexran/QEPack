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

/**
 * @brief  平台延时 (ms)
 */
void mspm0_delay_ms(unsigned long num_ms)
{
    DL_Common_delayCycles(num_ms * CPUCLK_FREQ / 1000);
}

/**
 * @brief  获取系统时钟 (ms)
 */
unsigned long mspm0_get_clock_ms(unsigned long *count)
{
    extern uint32_t tick_ms;
    *count = tick_ms;
    return *count;
}

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
        if (cur >= (start + I2C_TIMEOUT_MS)) {
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
        if (cur >= (start + I2C_TIMEOUT_MS)) {
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
    g_emActiveDevNum = emDevNum;
    if (mpu_init() != 0) return QE_ERROR;

    int result = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned char accel_fsr;
    uint16_t usRate = pstInit->usSampleRateHz ? pstInit->usSampleRateHz : 50;

    result |= mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    result |= mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    result |= mpu_set_sample_rate(usRate);
    result |= mpu_get_sample_rate(&gyro_rate);
    result |= mpu_get_gyro_fsr(&gyro_fsr);
    result |= mpu_get_accel_fsr(&accel_fsr);

    /* 加载 DMP 固件 */
    result |= dmp_load_motion_driver_firmware();
    result |= dmp_set_orientation(usInvOrientationToScalar(g_scGyroOrientation));
    result |= dmp_register_tap_cb(vDmpTapCb);
    result |= dmp_register_android_orient_cb(vDmpOrientCb);
    result |= dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                  DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
                                  DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    result |= dmp_set_fifo_rate(usRate);
    result |= mpu_set_dmp_state(1);

    if (result != 0) return QE_ERROR;

    g_astMpu6050DeviceParam[emDevNum].stRunningParam.ucDmpReady = 1;
    return QE_OK;
}

/**
 * @brief  读取 DMP FIFO 并更新姿态数据
 * @note   在 MPU6050 INT 引脚 GPIO 中断回调中调用
 */
QE_StatusTypeDef emMpu6050ReadDmp(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return QE_ERROR;

    stMpu6050RunningParamTdf *pstRun = &g_astMpu6050DeviceParam[emDevNum].stRunningParam;
    if (!pstRun->ucDmpReady) return QE_ERROR;

    g_emActiveDevNum = emDevNum;

    short sGyro[3], sAccel[3], sSensors;
    long lQuat[4];
    unsigned long ulTimestamp;
    unsigned char ucMore;

    if (dmp_read_fifo(sGyro, sAccel, lQuat, &ulTimestamp, &sSensors, &ucMore) != 0)
        return QE_ERROR;

    /* 保存原始值 */
    pstRun->stGyro.sX  = sGyro[0];
    pstRun->stGyro.sY  = sGyro[1];
    pstRun->stGyro.sZ  = sGyro[2];
    pstRun->stAccel.sX = sAccel[0];
    pstRun->stAccel.sY = sAccel[1];
    pstRun->stAccel.sZ = sAccel[2];

    /* 四元数归一化 -> 欧拉角 */
    float fQ0 = lQuat[0] / Q30;
    float fQ1 = lQuat[1] / Q30;
    float fQ2 = lQuat[2] / Q30;
    float fQ3 = lQuat[3] / Q30;

    pstRun->stQuat.fQ0 = fQ0;
    pstRun->stQuat.fQ1 = fQ1;
    pstRun->stQuat.fQ2 = fQ2;
    pstRun->stQuat.fQ3 = fQ3;

    pstRun->stAttitude.fPitch = asinf(-2.0f * fQ1 * fQ3 + 2.0f * fQ0 * fQ2) * 57.3f;
    pstRun->stAttitude.fRoll  = atan2f(2.0f * fQ2 * fQ3 + 2.0f * fQ0 * fQ1, -2.0f * fQ1 * fQ1 - 2.0f * fQ2 * fQ2 + 1.0f) * 57.3f;
    pstRun->stAttitude.fYaw   = atan2f(2.0f * (fQ1 * fQ2 + fQ0 * fQ3), fQ0 * fQ0 + fQ1 * fQ1 - fQ2 * fQ2 - fQ3 * fQ3) * 57.3f;
    pstRun->ucDataUpdated = 1;

    return QE_OK;
}

/* ==================== 数据读取 API ==================== */

void vMpu6050GetAttitude(emMpu6050DevNumTdf emDevNum, stMpu6050AttitudeTdf *pstAttitude)
{
    if (emDevNum >= MPU6050_DEV_NUM || pstAttitude == NULL) return;
    *pstAttitude = g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude;
}

float fMpu6050GetPitch(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return 0.0f;
    return g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude.fPitch;
}

float fMpu6050GetRoll(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return 0.0f;
    return g_astMpu6050DeviceParam[emDevNum].stRunningParam.stAttitude.fRoll;
}

float fMpu6050GetYaw(emMpu6050DevNumTdf emDevNum)
{
    if (emDevNum >= MPU6050_DEV_NUM) return 0.0f;
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

#endif
