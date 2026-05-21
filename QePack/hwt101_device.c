/**
  * @file       hwt101_device.c
  * @author     Qe_xr
  * @version    V3.0.0
  * @date       2026/5/21
  * @brief      HWT101 Z轴陀螺仪/角度传感器驱动实现
  *
  * 支持 UART (TTL) 和 I2C 两种通信模式。
  * 数据以 Q16.16 定点数存储，无浮点运算依赖。
  * 继承自 sensor_device 基类，统一使用 emSensorDevNumTdf 管理。
  */

#include "hwt101_device.h"
#if HWT101_IS_ENABLE

/* 全局设备数组（以 sensor 设备号偏移为索引） */
stHwt101DeviceParamTdf gastHwt101DeviceParam[HWT101_DEV_NUM];


/** @brief HWT101 虚方法表 */
static stSensorVTableTdf g_stHwt101VTable = {
    vHwt101Init,
    vHwt101PeriodExecute,
    fHwt101GetValue,
    fHwt101GetAccumulatedValue,
    vHwt101Reset,
    vHwt101SetTarget,
    fHwt101GetTarget,
};


/* ==================== HWT101 协议常量 ==================== */

#define HWT101_PKT_HEADER       0x55        // 数据包帧头
#define HWT101_PKT_LEN          11          // 数据包长度

#define HWT101_TYPE_GYRO        0x52        // 角速度包
#define HWT101_TYPE_ANGLE       0x53        // 角度包

#define HWT101_CMD_HEADER1      0xFF        // 命令帧头1
#define HWT101_CMD_HEADER2      0xAA        // 命令帧头2
#define HWT101_CMD_LEN          5           // 命令帧长度

/* 寄存器地址 */
#define HWT101_REG_SAVE         0x00
#define HWT101_REG_CALSW        0x01
#define HWT101_REG_RRATE        0x03
#define HWT101_REG_BAUD         0x04
#define HWT101_REG_IICADDR      0x1A
#define HWT101_REG_READADDR     0x27
#define HWT101_REG_VERSION      0x2E
#define HWT101_REG_GZ           0x39
#define HWT101_REG_YAW          0x3F
#define HWT101_REG_WORKMODE     0x48
#define HWT101_REG_KEY          0x69
#define HWT101_REG_CALIYAW      0x76
#define HWT101_REG_MANUALCALI   0xA6
#define HWT101_REG_NOAUTOCALI   0xA7

#define HWT101_KEY_UNLOCK       0xB588      // 解锁密钥


/* ==================== 内部辅助函数 ==================== */

/**
 * @brief sensor 设备号转数组索引
 */
static uint8_t u8Hwt101GetLocalIdx(emSensorDevNumTdf emSensorDevNum)
{
    return (uint8_t)(emSensorDevNum - emSensorHWT101DevNum0);
}


/**
 * @brief 通过 UART 设备号查找对应的 sensor 设备号
 */
static emSensorDevNumTdf emHwt101FindByUartDev(emUartDevNumTdf emUartDev)
{
    uint8_t i;
    for (i = 0; i < HWT101_DEV_NUM; i++)
    {
        if (gastHwt101DeviceParam[i].stStaticParam.emComMode == emHwt101ComModeUart
            && gastHwt101DeviceParam[i].stStaticParam.emUartDev == emUartDev)
        {
            return (emSensorDevNumTdf)(emSensorHWT101DevNum0 + i);
        }
    }
    return emNoSensor;
}


/**
 * @brief UART 模式：发送 5 字节命令
 */
static void vHwt101UartSendCmd(uint8_t ucLocalIdx, uint8_t u8Addr, int16_t s16Data)
{
    uint8_t aucCmd[5];
    aucCmd[0] = HWT101_CMD_HEADER1;
    aucCmd[1] = HWT101_CMD_HEADER2;
    aucCmd[2] = u8Addr;
    aucCmd[3] = (uint8_t)(s16Data & 0xFF);
    aucCmd[4] = (uint8_t)((s16Data >> 8) & 0xFF);
    vUartSendArray(gastHwt101DeviceParam[ucLocalIdx].stStaticParam.emUartDev, aucCmd, 5);
}


/**
 * @brief UART 模式：发送读寄存器命令（通过 READADDR 0x27）
 */
static void vHwt101UartReadCmd(uint8_t ucLocalIdx, uint8_t u8Addr)
{
    uint8_t aucCmd[5];
    aucCmd[0] = HWT101_CMD_HEADER1;
    aucCmd[1] = HWT101_CMD_HEADER2;
    aucCmd[2] = HWT101_REG_READADDR;
    aucCmd[3] = u8Addr;
    aucCmd[4] = 0x00;
    vUartSendArray(gastHwt101DeviceParam[ucLocalIdx].stStaticParam.emUartDev, aucCmd, 5);
}


/**
 * @brief I2C 模式：写 2 字节数据到寄存器
 */
static QE_StatusTypeDef emHwt101I2cWriteReg2(uint8_t ucLocalIdx, uint8_t u8Addr, int16_t s16Data)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[ucLocalIdx].stStaticParam;
    uint8_t aucTx[2];
    aucTx[0] = (uint8_t)(s16Data & 0xFF);
    aucTx[1] = (uint8_t)((s16Data >> 8) & 0xFF);
#if (QEPACK_PLATFORM == TI)
    return TI_I2C_Mem_Write(pStatic->pstI2cHandle, pStatic->u8I2cAddr, u8Addr, aucTx, 2, 10);
#else
    if (HAL_I2C_Mem_Write(pStatic->pstI2cHandle, (uint16_t)(pStatic->u8I2cAddr << 1),
        u8Addr, I2C_MEMADD_SIZE_8BIT, aucTx, 2, 10) == HAL_OK)
    {
        return QE_OK;
    }
    return QE_ERROR;
#endif
}


/**
 * @brief I2C 模式：从寄存器读取 2 字节数据
 */
#if (QEPACK_PLATFORM == TI)
static QE_StatusTypeDef emHwt101I2cReadReg2(uint8_t ucLocalIdx, uint8_t u8Addr, uint8_t *pucBuf)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[ucLocalIdx].stStaticParam;
    return TI_I2C_Mem_Read(pStatic->pstI2cHandle, pStatic->u8I2cAddr, u8Addr, pucBuf, 2, 10);
}
#else
static QE_StatusTypeDef emHwt101I2cReadReg2(uint8_t ucLocalIdx, uint8_t u8Addr, uint8_t *pucBuf)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[ucLocalIdx].stStaticParam;
    if (HAL_I2C_Mem_Read(pStatic->pstI2cHandle, (uint16_t)(pStatic->u8I2cAddr << 1),
        u8Addr, I2C_MEMADD_SIZE_8BIT, pucBuf, 2, 10) == HAL_OK)
    {
        return QE_OK;
    }
    return QE_ERROR;
}
#endif


/**
 * @brief 计算 11 字节数据包的校验和
 */
static uint8_t u8Hwt101CalcChecksum(uint8_t *pucData)
{
    uint8_t u8Sum = 0;
    uint8_t i;
    for (i = 0; i < 10; i++)
    {
        u8Sum += pucData[i];
    }
    return u8Sum;
}


/**
 * @brief 角度累加：检测跨 ±180° 边界并更新 lTurnCount
 */
static void vHwt101UpdateAccumulation(stHwt101RunningParamTdf *pRunning)
{
    fix32_t f180  = (fix32_t)180 * FIX32_ONE;
    fix32_t fDiff = pRunning->s32AngleZ - pRunning->fLastAngleZ;

    if (fDiff > f180)
    {
        pRunning->lTurnCount--;
    }
    else if (fDiff < -f180)
    {
        pRunning->lTurnCount++;
    }

    pRunning->fLastAngleZ = pRunning->s32AngleZ;
}


/**
 * @brief 解析完整的 11 字节 UART 数据包并更新运行参数
 */
static void vHwt101ParsePacket(uint8_t ucLocalIdx, uint8_t *pucPkt)
{
    stHwt101RunningParamTdf *pRunning = &gastHwt101DeviceParam[ucLocalIdx].stRunningParam;

    /* 校验 */
    if (u8Hwt101CalcChecksum(pucPkt) != pucPkt[10])
    {
        return;
    }

    switch (pucPkt[1])
    {
        case HWT101_TYPE_GYRO:  /* 角速度包 0x52 */
        {
            /* 校准角速度 Wz = pucPkt[6] | (pucPkt[7] << 8)，0.01°/s/LSB */
            pRunning->s16GzRaw = (int16_t)(pucPkt[6] | ((uint16_t)pucPkt[7] << 8));
            pRunning->s32AngularVelZ = (fix32_t)pRunning->s16GzRaw * 4000;
            pRunning->u8DataFlags |= HWT101_DATA_GYRO_UPDATE;
            pRunning->u8IsOnline = 1;
            pRunning->u32LastRxTick = QE_GET_TICK();
            break;
        }

        case HWT101_TYPE_ANGLE:  /* 角度包 0x53 */
        {
            /* 偏航角 Yaw = pucPkt[6] | (pucPkt[7] << 8)，0.01°/LSB */
            pRunning->s16YawRaw = (int16_t)(pucPkt[6] | ((uint16_t)pucPkt[7] << 8));
            pRunning->s32AngleZ = (fix32_t)pRunning->s16YawRaw * 360;
            pRunning->u16Version = (uint16_t)(pucPkt[8] | ((uint16_t)pucPkt[9] << 8));
            /* 角度累加：检测跨 ±180° 边界 */
            vHwt101UpdateAccumulation(pRunning);
            pRunning->u8DataFlags |= HWT101_DATA_ANGLE_UPDATE;
            pRunning->u8IsOnline = 1;
            pRunning->u32LastRxTick = QE_GET_TICK();
            break;
        }

        default:
            break;
    }
}


/**
 * @brief uart_device 回调函数（非帧模式）
 * @note  由 uart_device 在 vUartDevicePeriodExecute 中调用
 */
static void vHwt101UartCallback(emUartDevNumTdf emUartDev, stUartRunningParamTdf *pstUartRunning)
{
    emSensorDevNumTdf emSensorDev = emHwt101FindByUartDev(emUartDev);
    if (emSensorDev == emNoSensor)
    {
        return;
    }

    if (pstUartRunning->ulFrameDataCount > 0)
    {
        vHwt101RxDataInput(emSensorDev, pstUartRunning->aucFrameDataBuf,
                           pstUartRunning->ulFrameDataCount);
    }
}


/* ==================== 直接访问 API ==================== */

/**
 * @brief  输入接收到的原始字节
 */
void vHwt101RxDataInput(emSensorDevNumTdf emSensorDevNum, uint8_t *pucData, uint32_t ulLen)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return; }

    stHwt101RunningParamTdf *pRunning = &gastHwt101DeviceParam[ucLocalIdx].stRunningParam;
    uint32_t i;

    for (i = 0; i < ulLen; i++)
    {
        uint8_t ucByte = pucData[i];

        switch (pRunning->u8PktState)
        {
            case 0:  /* 找帧头 0x55 */
                if (ucByte == HWT101_PKT_HEADER)
                {
                    pRunning->au8PktBuf[0] = ucByte;
                    pRunning->u8PktIdx = 1;
                    pRunning->u8PktState = 1;
                }
                break;

            case 1:  /* 累积数据 */
                pRunning->au8PktBuf[pRunning->u8PktIdx++] = ucByte;
                if (pRunning->u8PktIdx >= HWT101_PKT_LEN)
                {
                    vHwt101ParsePacket(ucLocalIdx, pRunning->au8PktBuf);
                    pRunning->u8PktState = 0;
                    pRunning->u8PktIdx = 0;
                }
                break;

            default:
                pRunning->u8PktState = 0;
                pRunning->u8PktIdx = 0;
                break;
        }
    }
}


/* ==================== 数据获取 ==================== */

fix32_t s32Hwt101GetAngleZ(emSensorDevNumTdf emSensorDevNum)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return FIX32_ZERO; }
    return gastHwt101DeviceParam[ucLocalIdx].stRunningParam.s32AngleZ;
}

fix32_t s32Hwt101GetAngularVelZ(emSensorDevNumTdf emSensorDevNum)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return FIX32_ZERO; }
    return gastHwt101DeviceParam[ucLocalIdx].stRunningParam.s32AngularVelZ;
}

uint8_t u8Hwt101IsOnline(emSensorDevNumTdf emSensorDevNum)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return 0; }
    return gastHwt101DeviceParam[ucLocalIdx].stRunningParam.u8IsOnline;
}

uint16_t u16Hwt101GetVersion(emSensorDevNumTdf emSensorDevNum)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return 0; }
    return gastHwt101DeviceParam[ucLocalIdx].stRunningParam.u16Version;
}

uint8_t u8Hwt101GetDataFlags(emSensorDevNumTdf emSensorDevNum)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return 0; }

    uint8_t u8Flags;
    __disable_irq();
    u8Flags = gastHwt101DeviceParam[ucLocalIdx].stRunningParam.u8DataFlags;
    gastHwt101DeviceParam[ucLocalIdx].stRunningParam.u8DataFlags = 0;
    __enable_irq();
    return u8Flags;
}


/* ==================== 寄存器操作 ==================== */

/**
 * @brief  读取传感器寄存器
 */
QE_StatusTypeDef emHwt101ReadReg(emSensorDevNumTdf emSensorDevNum, uint8_t u8Addr, int16_t *ps16Val)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return QE_ERROR; }
    if (ps16Val == NULL) { return QE_ERROR; }

    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[ucLocalIdx].stStaticParam;

    if (pStatic->emComMode == emHwt101ComModeI2C)
    {
        uint8_t aucBuf[2];
        QE_StatusTypeDef emRet = emHwt101I2cReadReg2(ucLocalIdx, u8Addr, aucBuf);
        if (emRet == QE_OK)
        {
            *ps16Val = (int16_t)(aucBuf[0] | ((uint16_t)aucBuf[1] << 8));
        }
        return emRet;
    }
    else
    {
        /* UART 模式：发送读寄存器命令，结果通过回调返回 */
        vHwt101UartReadCmd(ucLocalIdx, u8Addr);
        return QE_OK;
    }
}


/**
 * @brief  写入传感器寄存器
 */
QE_StatusTypeDef emHwt101WriteReg(emSensorDevNumTdf emSensorDevNum, uint8_t u8Addr, int16_t s16Data)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return QE_ERROR; }

    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[ucLocalIdx].stStaticParam;

    if (pStatic->emComMode == emHwt101ComModeI2C)
    {
        return emHwt101I2cWriteReg2(ucLocalIdx, u8Addr, s16Data);
    }
    else
    {
        vHwt101UartSendCmd(ucLocalIdx, u8Addr, s16Data);
        return QE_OK;
    }
}


/* ==================== 控制命令 ==================== */

/**
 * @brief  解锁传感器
 */
void vHwt101Unlock(emSensorDevNumTdf emSensorDevNum)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_KEY, HWT101_KEY_UNLOCK);
}


/**
 * @brief  保存配置到传感器 Flash
 */
void vHwt101Save(emSensorDevNumTdf emSensorDevNum)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_SAVE, 0x0000);
}


/**
 * @brief  Z 轴角度归零
 */
void vHwt101SetZero(emSensorDevNumTdf emSensorDevNum)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    vHwt101Unlock(emSensorDevNum);
    QE_DELAY(200);
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_CALIYAW, 0x0000);
    QE_DELAY(500);
    vHwt101Save(emSensorDevNum);
}


/**
 * @brief  启动自动零偏校准
 */
void vHwt101StartAutoCali(emSensorDevNumTdf emSensorDevNum)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    vHwt101Unlock(emSensorDevNum);
    QE_DELAY(200);
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_CALSW, 0x0001);
    QE_DELAY(500);
}


/**
 * @brief  设置输出速率
 */
void vHwt101SetOutputRate(emSensorDevNumTdf emSensorDevNum, emHwt101RateTdf emRate)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    vHwt101Unlock(emSensorDevNum);
    QE_DELAY(200);
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_RRATE, (int16_t)emRate);
    QE_DELAY(100);
    vHwt101Save(emSensorDevNum);
}


/**
 * @brief  关闭/打开陀螺仪自动校准
 */
void vHwt101SetAutoCali(emSensorDevNumTdf emSensorDevNum, uint8_t ucEnable)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    vHwt101Unlock(emSensorDevNum);
    QE_DELAY(200);
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_NOAUTOCALI, ucEnable ? 0x0001 : 0x0000);
    QE_DELAY(100);
    vHwt101Save(emSensorDevNum);
}


/**
 * @brief  手动获取零偏（MANUALCALI）
 */
void vHwt101ManualCali(emSensorDevNumTdf emSensorDevNum, uint8_t ucStart)
{
    if (u8Hwt101GetLocalIdx(emSensorDevNum) >= HWT101_DEV_NUM) { return; }
    vHwt101Unlock(emSensorDevNum);
    QE_DELAY(200);
    emHwt101WriteReg(emSensorDevNum, HWT101_REG_MANUALCALI, ucStart ? 0x0001 : 0x0004);
    QE_DELAY(100);
    vHwt101Save(emSensorDevNum);
}


/* ===================== 传感器基类 VTable 实现 ===================== */

#if SENSOR_IS_ENABLE

/** @brief HWT101 VTable 实现函数 */

void vHwt101Init(void *pstSensor)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return; }

    /* 清零运行参数 */
    memset(&pstHwt->stRunningParam, 0, sizeof(stHwt101RunningParamTdf));

    if (pstHwt->stStaticParam.emComMode == emHwt101ComModeUart)
    {
        /* UART 模式：注册回调到 uart_device */
        vUartSetCallback(pstHwt->stStaticParam.emUartDev, vHwt101UartCallback);
    }
    else /* I2C 模式 */
    {
        uint8_t ucIdx = (uint8_t)(pstHwt - gastHwt101DeviceParam);
        emSensorDevNumTdf emDev = (emSensorDevNumTdf)(emSensorHWT101DevNum0 + ucIdx);

        /* 等待传感器上电稳定 */
        QE_DELAY(100);

        /* 读取版本号验证 I2C 通信是否正常 */
        uint8_t aucVer[2];
        QE_StatusTypeDef emRet = emHwt101I2cReadReg2(ucIdx, HWT101_REG_VERSION, aucVer);

        if (emRet == QE_OK)
        {
            pstHwt->stRunningParam.u16Version = (uint16_t)(aucVer[0] | ((uint16_t)aucVer[1] << 8));

            /* I2C 通信正常，解锁并配置传感器（按协议文档要求的延时） */
            vHwt101Unlock(emDev);
            QE_DELAY(200);
            emHwt101WriteReg(emDev, HWT101_REG_WORKMODE, 0x0000);  /* 正常数据模式 */
            QE_DELAY(100);
            emHwt101WriteReg(emDev, HWT101_REG_RRATE, 0x0006);     /* 10Hz */
            QE_DELAY(100);
            vHwt101Save(emDev);
            QE_DELAY(100);
        }
    }
}

void vHwt101PeriodExecute(void *pstSensor)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return; }

    /* 计算局部索引 */
    uint8_t ucLocalIdx = (uint8_t)(pstHwt - gastHwt101DeviceParam);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return; }

    stHwt101StaticParamTdf  *pStatic  = &pstHwt->stStaticParam;
    stHwt101RunningParamTdf *pRunning = &pstHwt->stRunningParam;

    if (pStatic->emComMode == emHwt101ComModeUart)
    {
        /* UART 模式：检查离线超时 */
        if (pRunning->u8IsOnline)
        {
            uint32_t ulNow = QE_GET_TICK();
            if ((ulNow - pRunning->u32LastRxTick) >= HWT101_OFFLINE_TIMEOUT_MS)
            {
                pRunning->u8IsOnline = 0;
            }
        }
    }
    else /* I2C 模式 */
    {
        uint32_t ulNow = QE_GET_TICK();

        /* 频率控制 */
        if ((ulNow - pRunning->u32LastPollTick) < HWT101_I2C_POLL_INTERVAL_MS)
        {
            return;
        }
        pRunning->u32LastPollTick = ulNow;

        /* 分时轮询寄存器 */
        switch (pRunning->u8PollPhase)
        {
            case 0:  /* 读 GZ (0x39) */
            {
                uint8_t aucBuf[2];
                if (emHwt101I2cReadReg2(ucLocalIdx, HWT101_REG_GZ, aucBuf) == QE_OK)
                {
                    pRunning->s16GzRaw = (int16_t)(aucBuf[0] | ((uint16_t)aucBuf[1] << 8));
                    pRunning->s32AngularVelZ = (fix32_t)pRunning->s16GzRaw * 4000;
                    pRunning->u8DataFlags |= HWT101_DATA_GYRO_UPDATE;
                    pRunning->u8IsOnline = 1;
                }
                else
                {
                    pRunning->u8IsOnline = 0;
                }
                break;
            }

            case 1:  /* 读 Yaw (0x3F) */
            {
                uint8_t aucBuf[2];
                if (emHwt101I2cReadReg2(ucLocalIdx, HWT101_REG_YAW, aucBuf) == QE_OK)
                {
                    pRunning->s16YawRaw = (int16_t)(aucBuf[0] | ((uint16_t)aucBuf[1] << 8));
                    pRunning->s32AngleZ = (fix32_t)pRunning->s16YawRaw * 360;
                    vHwt101UpdateAccumulation(pRunning);
                    pRunning->u8DataFlags |= HWT101_DATA_ANGLE_UPDATE;
                    pRunning->u8IsOnline = 1;
                }
                break;
            }

            default:
                break;
        }

        pRunning->u8PollPhase = (pRunning->u8PollPhase + 1) & 0x03;
    }
}

fix32_t fHwt101GetValue(void *pstSensor)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return FIX32_ZERO; }
    return pstHwt->stRunningParam.s32AngleZ;
}

void vHwt101Reset(void *pstSensor)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return; }

    uint8_t ucLocalIdx = (uint8_t)(pstHwt - gastHwt101DeviceParam);
    if (ucLocalIdx >= HWT101_DEV_NUM) { return; }

    emSensorDevNumTdf emSensorDev = (emSensorDevNumTdf)(emSensorHWT101DevNum0 + ucLocalIdx);
    vHwt101SetZero(emSensorDev);
    pstHwt->stRunningParam.fTargetValue = FIX32_ZERO;
    pstHwt->stRunningParam.fLastAngleZ = FIX32_ZERO;
    pstHwt->stRunningParam.lTurnCount   = 0;
}

void vHwt101SetTarget(void *pstSensor, fix32_t fTarget)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return; }
    pstHwt->stRunningParam.fTargetValue = fTarget;
}

fix32_t fHwt101GetTarget(void *pstSensor)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return FIX32_ZERO; }
    return pstHwt->stRunningParam.fTargetValue;
}

fix32_t fHwt101GetAccumulatedValue(void *pstSensor)
{
    stHwt101DeviceParamTdf *pstHwt = (stHwt101DeviceParamTdf *)pstSensor;
    if (pstHwt == NULL) { return FIX32_ZERO; }
    stHwt101RunningParamTdf *pRunning = &pstHwt->stRunningParam;
    fix32_t f360 = (fix32_t)360 * FIX32_ONE;
    int64_t llAccum = (int64_t)pRunning->s32AngleZ + (int64_t)pRunning->lTurnCount * f360;
    if (llAccum > (int64_t)FIX32_MAX)  return FIX32_MAX;
    if (llAccum < (int64_t)FIX32_MIN)  return FIX32_MIN;
    return (fix32_t)llAccum;
}


/* ===================== 注册函数 ===================== */

void vHwt101Register(emSensorDevNumTdf emSensorDevNum, stHwt101StaticParamTdf *pstInit)
{
    uint8_t ucLocalIdx = u8Hwt101GetLocalIdx(emSensorDevNum);
    if (ucLocalIdx >= HWT101_DEV_NUM || pstInit == NULL) { return; }

    stHwt101DeviceParamTdf *pstHwt = &gastHwt101DeviceParam[ucLocalIdx];
    memset(pstHwt, 0, sizeof(stHwt101DeviceParamTdf));

    /* 初始化基类 */
    pstHwt->stBase.emType          = emSensorTypeHWT101Gyro;
    pstHwt->stBase.pstVTable       = &g_stHwt101VTable;
    pstHwt->stBase.ucEnable        = 1;
    pstHwt->stBase.fWeight         = pstInit->fWeight;
    pstHwt->stBase.emPidDevNum     = pstInit->emPidDevNum;
    pstHwt->stBase.usPidPeriodMs   = pstInit->usPidPeriodMs;
    pstHwt->stBase.ulPidLastTickMs = 0;

    /* 拷贝静态参数 */
    memcpy(&pstHwt->stStaticParam, pstInit, sizeof(stHwt101StaticParamTdf));

    /* 注册到 sensor 基类 */
    vSensorRegisterDevice(emSensorDevNum, &pstHwt->stBase);
}

#endif /* SENSOR_IS_ENABLE */

#endif
