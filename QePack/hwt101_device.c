/**
  * @file       hwt101_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/19
  * @brief      HWT101 Z轴陀螺仪/角度传感器驱动实现
  *
  * 支持 UART (TTL) 和 I2C 两种通信模式。
  * 数据以 Q16.16 定点数存储，无浮点运算依赖。
  */

#include "hwt101_device.h"
#if HWT101_IS_ENABLE

/* 全局设备数组 */
stHwt101DeviceParamTdf gastHwt101DeviceParam[HWT101_DEV_NUM];


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
 * @brief 通过 UART 设备号查找对应的 HWT101 设备号
 */
static emHwt101DevNumTdf emHwt101FindByUartDev(emUartDevNumTdf emUartDev)
{
    uint8_t i;
    for (i = 0; i < HWT101_DEV_NUM; i++)
    {
        if (gastHwt101DeviceParam[i].stStaticParam.emComMode == emHwt101ComModeUart
            && gastHwt101DeviceParam[i].stStaticParam.emUartDev == emUartDev)
        {
            return (emHwt101DevNumTdf)i;
        }
    }
    return (emHwt101DevNumTdf)HWT101_DEV_NUM;
}


/**
 * @brief UART 模式：发送 5 字节命令
 */
static void vHwt101UartSendCmd(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t s16Data)
{
    uint8_t aucCmd[5];
    aucCmd[0] = HWT101_CMD_HEADER1;
    aucCmd[1] = HWT101_CMD_HEADER2;
    aucCmd[2] = u8Addr;
    aucCmd[3] = (uint8_t)(s16Data & 0xFF);
    aucCmd[4] = (uint8_t)((s16Data >> 8) & 0xFF);
    vUartSendArray(gastHwt101DeviceParam[emDevNum].stStaticParam.emUartDev, aucCmd, 5);
}


/**
 * @brief UART 模式：发送读寄存器命令（通过 READADDR 0x27）
 */
static void vHwt101UartReadCmd(emHwt101DevNumTdf emDevNum, uint8_t u8Addr)
{
    uint8_t aucCmd[5];
    aucCmd[0] = HWT101_CMD_HEADER1;
    aucCmd[1] = HWT101_CMD_HEADER2;
    aucCmd[2] = HWT101_REG_READADDR;
    aucCmd[3] = u8Addr;
    aucCmd[4] = 0x00;
    vUartSendArray(gastHwt101DeviceParam[emDevNum].stStaticParam.emUartDev, aucCmd, 5);
}


/**
 * @brief I2C 模式：写 2 字节数据到寄存器（单次 I2C 事务）
 */
#if (QEPACK_PLATFORM == TI)
static QE_StatusTypeDef emHwt101I2cWriteReg2(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t s16Data)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[emDevNum].stStaticParam;
    stI2CTdf *pstI2c = pStatic->pstI2cHandle;

    uint8_t aucTx[3];
    unsigned long start, cur;

    aucTx[0] = u8Addr;
    aucTx[1] = (uint8_t)(s16Data & 0xFF);
    aucTx[2] = (uint8_t)((s16Data >> 8) & 0xFF);

    DL_I2C_fillControllerTXFIFO(pstI2c->i2c_inst, aucTx, 3);
    DL_I2C_clearInterruptStatus(pstI2c->i2c_inst, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(pstI2c->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(pstI2c->i2c_inst, pStatic->u8I2cAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 3);

    mspm0_get_clock_ms(&start);
    while (!DL_I2C_getRawInterruptStatus(pstI2c->i2c_inst, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        mspm0_get_clock_ms(&cur);
        if ((cur - start) >= 10)  /* 10ms 超时 */
        {
            return QE_TIMEOUT;
        }
    }
    return QE_OK;
}
#else
static QE_StatusTypeDef emHwt101I2cWriteReg2(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t s16Data)
{
    /* STM32 平台暂未实现 I2C 模式 */
    (void)emDevNum; (void)u8Addr; (void)s16Data;
    return QE_ERROR;
}
#endif


/**
 * @brief I2C 模式：从寄存器读取 2 字节数据
 */
#if (QEPACK_PLATFORM == TI)
static QE_StatusTypeDef emHwt101I2cReadReg2(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, uint8_t *pucBuf)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[emDevNum].stStaticParam;
    return TI_I2C_Mem_Read(pStatic->pstI2cHandle, pStatic->u8I2cAddr, u8Addr, pucBuf, 2, 10);
}
#else
static QE_StatusTypeDef emHwt101I2cReadReg2(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, uint8_t *pucBuf)
{
    (void)emDevNum; (void)u8Addr; (void)pucBuf;
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
 * @brief 解析完整的 11 字节 UART 数据包并更新运行参数
 */
static void vHwt101ParsePacket(emHwt101DevNumTdf emDevNum, uint8_t *pucPkt)
{
    stHwt101RunningParamTdf *pRunning = &gastHwt101DeviceParam[emDevNum].stRunningParam;

    /* 校验 */
    if (u8Hwt101CalcChecksum(pucPkt) != pucPkt[10])
    {
        return;
    }

    switch (pucPkt[1])
    {
        case HWT101_TYPE_GYRO:  /* 角速度包 0x52 */
        {
            /* 校准角速度 Wz = pucPkt[4] | (pucPkt[5] << 8) */
            pRunning->s16GzRaw = (int16_t)(pucPkt[4] | ((uint16_t)pucPkt[5] << 8));
            pRunning->s32AngularVelZ = (fix32_t)pRunning->s16GzRaw * 4000;
            pRunning->u8DataFlags |= HWT101_DATA_GYRO_UPDATE;
            pRunning->u8IsOnline = 1;
            pRunning->u32LastRxTick = TI_GetTick();
            break;
        }

        case HWT101_TYPE_ANGLE:  /* 角度包 0x53 */
        {
            /* 偏航角 Yaw = pucPkt[6] | (pucPkt[7] << 8) */
            pRunning->s16YawRaw = (int16_t)(pucPkt[6] | ((uint16_t)pucPkt[7] << 8));
            pRunning->s32AngleZ = (fix32_t)pRunning->s16YawRaw * 360;
            pRunning->u16Version = (uint16_t)(pucPkt[8] | ((uint16_t)pucPkt[9] << 8));
            pRunning->u8DataFlags |= HWT101_DATA_ANGLE_UPDATE;
            pRunning->u8IsOnline = 1;
            pRunning->u32LastRxTick = TI_GetTick();
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
    emHwt101DevNumTdf emDev = emHwt101FindByUartDev(emUartDev);
    if (emDev >= (emHwt101DevNumTdf)HWT101_DEV_NUM)
    {
        return;
    }

    if (pstUartRunning->ulFrameDataCount > 0)
    {
        vHwt101RxDataInput(emDev, pstUartRunning->aucFrameDataBuf,
                           pstUartRunning->ulFrameDataCount);
    }
}


/* ==================== 公共 API ==================== */

/**
 * @brief  HWT101 设备初始化
 */
void vHwt101DeviceInit(stHwt101StaticParamTdf *pstInit, emHwt101DevNumTdf emDevNum)
{
    stHwt101StaticParamTdf  *pStatic  = &gastHwt101DeviceParam[emDevNum].stStaticParam;
    stHwt101RunningParamTdf *pRunning = &gastHwt101DeviceParam[emDevNum].stRunningParam;

    /* 拷贝静态参数 */
    memcpy(pStatic, pstInit, sizeof(stHwt101StaticParamTdf));

    /* 清零运行参数 */
    memset(pRunning, 0, sizeof(stHwt101RunningParamTdf));

    if (pstInit->emComMode == emHwt101ComModeUart)
    {
        /* UART 模式：注册回调到 uart_device */
        vUartSetCallback(pstInit->emUartDev, vHwt101UartCallback);
    }
    /* I2C 模式：无需额外初始化，轮询在 vHwt101DevicePeriodExecute 中进行 */
}


/**
 * @brief  HWT101 周期执行
 */
void vHwt101DevicePeriodExecute(emHwt101DevNumTdf emDevNum)
{
    stHwt101StaticParamTdf  *pStatic  = &gastHwt101DeviceParam[emDevNum].stStaticParam;
    stHwt101RunningParamTdf *pRunning = &gastHwt101DeviceParam[emDevNum].stRunningParam;

    if (pStatic->emComMode == emHwt101ComModeUart)
    {
        /* UART 模式：检查离线超时 */
        if (pRunning->u8IsOnline)
        {
            uint32_t ulNow = TI_GetTick();
            if ((ulNow - pRunning->u32LastRxTick) >= HWT101_OFFLINE_TIMEOUT_MS)
            {
                pRunning->u8IsOnline = 0;
            }
        }
    }
    else /* I2C 模式 */
    {
        #if (QEPACK_PLATFORM == TI)
        {
            uint32_t ulNow = TI_GetTick();

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
                    if (emHwt101I2cReadReg2(emDevNum, HWT101_REG_GZ, aucBuf) == QE_OK)
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
                    if (emHwt101I2cReadReg2(emDevNum, HWT101_REG_YAW, aucBuf) == QE_OK)
                    {
                        pRunning->s16YawRaw = (int16_t)(aucBuf[0] | ((uint16_t)aucBuf[1] << 8));
                        pRunning->s32AngleZ = (fix32_t)pRunning->s16YawRaw * 360;
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
        #endif
    }
}


/**
 * @brief  输入接收到的原始字节
 */
void vHwt101RxDataInput(emHwt101DevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen)
{
    stHwt101RunningParamTdf *pRunning = &gastHwt101DeviceParam[emDevNum].stRunningParam;
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
                    vHwt101ParsePacket(emDevNum, pRunning->au8PktBuf);
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

fix32_t s32Hwt101GetAngleZ(emHwt101DevNumTdf emDevNum)
{
    return gastHwt101DeviceParam[emDevNum].stRunningParam.s32AngleZ;
}

fix32_t s32Hwt101GetAngularVelZ(emHwt101DevNumTdf emDevNum)
{
    return gastHwt101DeviceParam[emDevNum].stRunningParam.s32AngularVelZ;
}

uint8_t u8Hwt101IsOnline(emHwt101DevNumTdf emDevNum)
{
    return gastHwt101DeviceParam[emDevNum].stRunningParam.u8IsOnline;
}

uint16_t u16Hwt101GetVersion(emHwt101DevNumTdf emDevNum)
{
    return gastHwt101DeviceParam[emDevNum].stRunningParam.u16Version;
}

uint8_t u8Hwt101GetDataFlags(emHwt101DevNumTdf emDevNum)
{
    uint8_t u8Flags = gastHwt101DeviceParam[emDevNum].stRunningParam.u8DataFlags;
    gastHwt101DeviceParam[emDevNum].stRunningParam.u8DataFlags = 0;
    return u8Flags;
}


/* ==================== 寄存器操作 ==================== */

/**
 * @brief  读取传感器寄存器
 */
QE_StatusTypeDef emHwt101ReadReg(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t *ps16Val)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[emDevNum].stStaticParam;

    if (ps16Val == NULL)
    {
        return QE_ERROR;
    }

    if (pStatic->emComMode == emHwt101ComModeI2C)
    {
        uint8_t aucBuf[2];
        QE_StatusTypeDef emRet = emHwt101I2cReadReg2(emDevNum, u8Addr, aucBuf);
        if (emRet == QE_OK)
        {
            *ps16Val = (int16_t)(aucBuf[0] | ((uint16_t)aucBuf[1] << 8));
        }
        return emRet;
    }
    else
    {
        /* UART 模式：发送读寄存器命令，结果通过回调返回 */
        vHwt101UartReadCmd(emDevNum, u8Addr);
        return QE_OK;
    }
}


/**
 * @brief  写入传感器寄存器
 */
QE_StatusTypeDef emHwt101WriteReg(emHwt101DevNumTdf emDevNum, uint8_t u8Addr, int16_t s16Data)
{
    stHwt101StaticParamTdf *pStatic = &gastHwt101DeviceParam[emDevNum].stStaticParam;

    if (pStatic->emComMode == emHwt101ComModeI2C)
    {
        return emHwt101I2cWriteReg2(emDevNum, u8Addr, s16Data);
    }
    else
    {
        vHwt101UartSendCmd(emDevNum, u8Addr, s16Data);
        return QE_OK;
    }
}


/* ==================== 控制命令 ==================== */

/**
 * @brief  解锁传感器
 */
void vHwt101Unlock(emHwt101DevNumTdf emDevNum)
{
    emHwt101WriteReg(emDevNum, HWT101_REG_KEY, HWT101_KEY_UNLOCK);
}


/**
 * @brief  保存配置到传感器 Flash
 */
void vHwt101Save(emHwt101DevNumTdf emDevNum)
{
    emHwt101WriteReg(emDevNum, HWT101_REG_SAVE, 0x0000);
}


/**
 * @brief  Z 轴角度归零
 */
void vHwt101SetZero(emHwt101DevNumTdf emDevNum)
{
    vHwt101Unlock(emDevNum);
    TI_Delay(200);
    emHwt101WriteReg(emDevNum, HWT101_REG_CALIYAW, 0x0000);
    TI_Delay(500);
    vHwt101Save(emDevNum);
}


/**
 * @brief  启动自动零偏校准
 */
void vHwt101StartAutoCali(emHwt101DevNumTdf emDevNum)
{
    vHwt101Unlock(emDevNum);
    TI_Delay(200);
    emHwt101WriteReg(emDevNum, HWT101_REG_WORKMODE, 0x0001);
    TI_Delay(500);
    vHwt101Save(emDevNum);
}


/**
 * @brief  设置输出速率
 */
void vHwt101SetOutputRate(emHwt101DevNumTdf emDevNum, emHwt101RateTdf emRate)
{
    vHwt101Unlock(emDevNum);
    TI_Delay(200);
    emHwt101WriteReg(emDevNum, HWT101_REG_RRATE, (int16_t)emRate);
    TI_Delay(100);
    vHwt101Save(emDevNum);
}


/**
 * @brief  关闭/打开陀螺仪自动校准
 */
void vHwt101SetAutoCali(emHwt101DevNumTdf emDevNum, uint8_t ucEnable)
{
    vHwt101Unlock(emDevNum);
    TI_Delay(200);
    emHwt101WriteReg(emDevNum, HWT101_REG_NOAUTOCALI, ucEnable ? 0x0001 : 0x0000);
    TI_Delay(100);
    vHwt101Save(emDevNum);
}


/**
 * @brief  手动获取零偏（MANUALCALI）
 */
void vHwt101ManualCali(emHwt101DevNumTdf emDevNum, uint8_t ucStart)
{
    vHwt101Unlock(emDevNum);
    TI_Delay(200);
    emHwt101WriteReg(emDevNum, HWT101_REG_MANUALCALI, ucStart ? 0x0001 : 0x0004);
    TI_Delay(100);
    vHwt101Save(emDevNum);
}

#endif
