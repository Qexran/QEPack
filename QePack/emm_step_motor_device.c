/**
  * @file       emm_step_motor_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm步进闭环电机驱动，基于 STM32 HAL 库
  *
  */
#include "emm_step_motor_device.h"
#include "arithmetic.h"

#if EMM_MOTOR_IS_ENABLE

stEmmMotorDeviceParamTdf astEmmMotorDeviceParam[EMM_MOTOR_DEV_NUM];

#define EMM_MOTOR_CMD_END_MARK 0x6B     /* 固定帧尾模式控制指令结束标志 */

#define EMM_MOTOR_CMD_GAP_MS      10    /* 控制指令发送间隔(ms) */
#define EMM_MOTOR_SYNC_GAP_MS     10    /* 同步指令间隔(ms) */

/* 重试参数 */
#define EMM_MOTOR_DEFAULT_MAX_RETRY       3   /* 最大重试次数 */
#define EMM_MOTOR_RETRY_TIMEOUT_THRESHOLD 50  /* 超时阈值（50 * 30ms = 1500ms），避免正常运动中误触发重试 */

/* 解析器参数 */
#define EMM_PARSER_BUF_SIZE        64       /* 解析器缓冲区大小 */
#define EMM_PARSER_TIMEOUT_MS      5        /* 解析器超时复位阈值(ms) */

static uint8_t g_ucEmmCallbackRegistered = 0;
static uint16_t g_aucEmmFlagRxCnt[EMM_MOTOR_DEV_NUM];

/* ==================== CRC-8 校验表 ==================== */

static const uint8_t g_aucCRC8Table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

/* ==================== 校验计算 ==================== */

static uint8_t ucEmmCRC8(const uint8_t *pucData, uint8_t ucLen)
{
    uint8_t ucCRC = pucData[0];
    for (uint8_t i = 1; i < ucLen; i++) {
        ucCRC = g_aucCRC8Table[ucCRC ^ pucData[i]];
    }
    return ucCRC;
}

static uint8_t ucEmmXOR(const uint8_t *pucData, uint8_t ucLen)
{
    uint8_t ucXOR = pucData[0];
    for (uint8_t i = 1; i < ucLen; i++) {
        ucXOR ^= pucData[i];
    }
    return ucXOR;
}

static uint8_t ucEmmComputeChecksum(emEmmMotorChecksumTdf emMode,
    const uint8_t *pucFrame, uint8_t ucFrameLen)
{
    switch (emMode) {
        case emEmmMotorChecksum_CRC8:
            return ucEmmCRC8(pucFrame, ucFrameLen);
        case emEmmMotorChecksum_XOR:
            return ucEmmXOR(pucFrame, ucFrameLen);
        case emEmmMotorChecksum_Fixed6B:
        default:
            return EMM_MOTOR_CMD_END_MARK;
    }
}

/** 根据响应首字节获取期望响应长度（aucBuf 中数据+校验字节总数），0=未知 */
static uint8_t ucGetCmdResponseLen(uint8_t ucCmdByte)
{
    switch (ucCmdByte) {
        /* 查询响应 */
        case 0x1F: return 3;   /* Ver */
        case 0x20: return 6;   /* RL */
        case 0x21: return 14;  /* PID */
        case 0x24: return 4;   /* VBus */
        case 0x27: return 4;   /* Cpha */
        case 0x31: return 4;   /* Encl */
        case 0x33: return 4;   /* TPos */
        case 0x35: return 4;   /* Vel */
        case 0x36: return 4;   /* CPos */
        case 0x37: return 4;   /* PErr */
        case 0x3A: return 3;   /* Flag */
        case 0x3B: return 3;   /* Org */
        case 0x42: return 5;   /* Conf */
        case 0x43: return 5;   /* State */
        /* 命令 ACK */
        case 0x0A: return 3;   /* ResetCurPos */
        case 0x0E: return 3;   /* ResetClog */
        case 0x46: return 3;   /* ModifyCtrlMode */
        case 0x4C: return 3;   /* OriginModifyParams */
        case 0x93: return 3;   /* OriginSetO */
        case 0x9A: return 3;   /* OriginTrigger */
        case 0x9C: return 3;   /* OriginInterrupt */
        case 0xF3: return 3;   /* Enable */
        case 0xF6: return 3;   /* VelControl */
        case 0xFD: return 3;   /* PosControl */
        case 0xFE: return 3;   /* Stop */
        case 0xFF: return 0;   /* SyncMotion：同步触发不产生ACK应答 */
        default:  return 0;
    }
}

/* ==================== UART 回调 — 响应帧解析 ==================== */

/**
 * @brief  EMM 响应帧解析状态
 * @note   所有电机共用一条 UART，在回调中解析响应并分发到对应地址的电机
 */
typedef enum {
    kEmmParseIdle = 0,       /* 等待地址字节 */
    kEmmParseCollect,        /* 收集数据直到 0x6B 结束标志 */
} emEmmParseStateTdf;

static struct {
    emEmmParseStateTdf emState;
    uint8_t            ucAddr;
    uint8_t            aucBuf[EMM_PARSER_BUF_SIZE];
    uint8_t            ucIdx;
    uint8_t            ucExpectedLen;     /* 0=使用0x6B终止符; >0=收集N字节后结束（CRC-8/XOR模式） */
    uint8_t            ucChecksumMode;    /* 当前帧的校验模式 */
    uint32_t           ulEnterTick;       /* 进入收集态的时刻（用于超时复位） */
} g_stEmmParser;

/**
 * @brief  根据地址查找对应电机，更新缓存的响应数据
 */
static void vEmmMotorDispatchResponse(uint8_t ucAddr, const uint8_t *pucData, uint8_t ucLen)
{
    uint8_t i;

    /* 最小长度校验：至少需要 命令字节 + 校验字节 */
    if (ucLen < 2) return;

    for (i = 0; i < EMM_MOTOR_DEV_NUM; i++) {
        if (astEmmMotorDeviceParam[i].stStaticParam.ucAddr != ucAddr) continue;

        emEmmMotorChecksumTdf emMode = astEmmMotorDeviceParam[i].stStaticParam.emChecksumMode;

        /* 固定0x6B模式：校验帧尾标记 */
        if (emMode == emEmmMotorChecksum_Fixed6B) {
            if (pucData[ucLen - 1] != EMM_MOTOR_CMD_END_MARK) return;
        }

        /* 响应长度与首字节类型匹配校验 */
        {
            uint8_t ucExpected = ucGetCmdResponseLen(pucData[0]);
            if (ucExpected > 0 && ucLen != ucExpected) {
                return;
            }
        }

        /* 响应首字节校验：必须为已知的命令/查询字节 */
        if (ucGetCmdResponseLen(pucData[0]) == 0) return;

        /* Flag 响应: [0x3A] [flag] [cs] */
        if (pucData[0] == 0x3A && ucLen >= 3) {
            astEmmMotorDeviceParam[i].stRunningParam.ucLastFlag    = pucData[1];
            astEmmMotorDeviceParam[i].stRunningParam.ucFlagUpdated = 1;
            g_aucEmmFlagRxCnt[i]++;
        }

        astEmmMotorDeviceParam[i].stRunningParam.ucCmdAckReceived = 1;
        return;
    }
}

/**
 * @brief  UART 回调 — 由 vUartDevicePeriodExecute (1ms ISR) 触发
 * @note   将接收字节喂入解析状态机，识别 [addr] [data...] [checksum] 格式的响应帧。
 *         固定0x6B模式以 0x6B 为帧尾；CRC-8/XOR 模式按期望长度收帧并校验。
 */
static void vEmmMotorUartCallback(emUartDevNumTdf emUartDevNum, stUartRunningParamTdf *pstRunning)
{
    (void)emUartDevNum;
    uint32_t i;

    for (i = 0; i < pstRunning->ulFrameDataCount; i++) {
        uint8_t ucByte = pstRunning->aucFrameDataBuf[i];

        switch (g_stEmmParser.emState) {
            case kEmmParseIdle:
                /* 查找是否匹配某个已注册电机的地址 */
                {
                    uint8_t m;
                    for (m = 0; m < EMM_MOTOR_DEV_NUM; m++) {
                        if (astEmmMotorDeviceParam[m].stStaticParam.ucAddr == ucByte
                            && astEmmMotorDeviceParam[m].stStaticParam.ucAddr != 0) {
                            g_stEmmParser.ucAddr  = ucByte;
                            g_stEmmParser.ucIdx   = 0;
                            g_stEmmParser.ulEnterTick = QE_GET_TICK();
                            g_stEmmParser.emState = kEmmParseCollect;
                            /* 查表获取该校验模式下的期望响应长度 */
                            g_stEmmParser.ucChecksumMode = (uint8_t)astEmmMotorDeviceParam[m].stStaticParam.emChecksumMode;
                            if (g_stEmmParser.ucChecksumMode != emEmmMotorChecksum_Fixed6B) {
                                g_stEmmParser.ucExpectedLen = astEmmMotorDeviceParam[m].stRunningParam.ucExpectedRespLen;
                            } else {
                                g_stEmmParser.ucExpectedLen = 0;
                            }
                            break;
                        }
                    }
                }
                break;

            case kEmmParseCollect:
                /* 缓冲区溢出保护：超过上限则丢弃当前帧 */
                if (g_stEmmParser.ucIdx >= EMM_PARSER_BUF_SIZE) {
                    g_stEmmParser.emState = kEmmParseIdle;
                    break;
                }

                g_stEmmParser.aucBuf[g_stEmmParser.ucIdx++] = ucByte;

                if (g_stEmmParser.ucExpectedLen > 0) {
                    /* CRC-8/XOR 模式：按期望长度收帧，校验通过后派发 */
                    if (g_stEmmParser.ucIdx >= g_stEmmParser.ucExpectedLen) {
                        uint8_t aucFrame[EMM_PARSER_BUF_SIZE + 1];
                        aucFrame[0] = g_stEmmParser.ucAddr;
                        memcpy(&aucFrame[1], g_stEmmParser.aucBuf, g_stEmmParser.ucIdx - 1);
                        uint8_t ucComputed = ucEmmComputeChecksum(
                            (emEmmMotorChecksumTdf)g_stEmmParser.ucChecksumMode,
                            aucFrame, g_stEmmParser.ucIdx);
                        if (ucComputed == g_stEmmParser.aucBuf[g_stEmmParser.ucIdx - 1]) {
                            vEmmMotorDispatchResponse(g_stEmmParser.ucAddr,
                                                      g_stEmmParser.aucBuf,
                                                      g_stEmmParser.ucIdx);
                        }
                        g_stEmmParser.emState = kEmmParseIdle;
                    }
                } else {
                    /* 固定0x6B 模式：以 0x6B 为帧尾。
                       但若已收集的命令字节在查表中且长度未达期望值，
                       说明此 0x6B 是数据字节而非终止符，继续收集。 */
                    if (ucByte == EMM_MOTOR_CMD_END_MARK) {
                        uint8_t ucExp = 0;
                        if (g_stEmmParser.ucIdx >= 2) {
                            ucExp = ucGetCmdResponseLen(g_stEmmParser.aucBuf[0]);
                        }
                        if (ucExp > 0 && g_stEmmParser.ucIdx < ucExp) {
                            /* 0x6B 是数据字节，不是帧尾，继续收集 */
                        } else {
                            vEmmMotorDispatchResponse(g_stEmmParser.ucAddr,
                                                      g_stEmmParser.aucBuf,
                                                      g_stEmmParser.ucIdx);
                            g_stEmmParser.emState = kEmmParseIdle;
                        }
                    }
                }
                break;

            default:
                g_stEmmParser.emState = kEmmParseIdle;
                break;
        }
    }
}

/* ==================== 指令发送 ==================== */

/**
 * @brief  通用命令发送辅助函数（内部）
 * @param  pstEmmMotor ：EmmMotor设备指针
 * @param  pucCmdData  ：命令数据指针
 * @param  ucDataLen   ：命令数据长度（不含地址字节和结束标志）
 * @param  usDelayMs   ：发送后等待时间(ms)，0 = 不等
 */
static void vEmmMotorSendCmdEx(
    stEmmMotorDeviceParamTdf *pstEmmMotor,
    const uint8_t *pucCmdData, uint8_t ucDataLen,
    uint16_t usDelayMs
) {
    uint8_t aucCmd[32];
    uint8_t ucIndex = 0;

    /* 保留 2 字节给地址和校验字节 */
    if (ucDataLen > sizeof(aucCmd) - 2) ucDataLen = sizeof(aucCmd) - 2;

    aucCmd[ucIndex++] = pstEmmMotor->stStaticParam.ucAddr;
    for (uint8_t i = 0; i < ucDataLen; i++) {
        aucCmd[ucIndex++] = pucCmdData[i];
    }
    aucCmd[ucIndex] = ucEmmComputeChecksum(
        pstEmmMotor->stStaticParam.emChecksumMode,
        aucCmd, ucIndex);
    ucIndex++;

    /* 记录期望响应长度，供解析器在 CRC-8/XOR 模式下确定收帧长度 */
    if (ucDataLen > 0) {
        pstEmmMotor->stRunningParam.ucExpectedRespLen = ucGetCmdResponseLen(pucCmdData[0]);
    }

    vUartSendArray(pstEmmMotor->stStaticParam.emUartDevNum, aucCmd, ucIndex);
    if (usDelayMs > 0) {
        QE_DELAY(usDelayMs);
    }
}

/**
 * @brief  保存上一次运动指令参数，供超时重试使用
 */
static void vEmmMotorSaveLastCmd(
    stEmmMotorDeviceParamTdf *pstEmmMotor,
    emEmmMotorLastCmdTdf emCmd,
    const uint8_t *pucCmdData, uint8_t ucDataLen
) {
    pstEmmMotor->stRunningParam.emLastCmd = emCmd;
    pstEmmMotor->stRunningParam.ucLastCmdDataLen = (ucDataLen < sizeof(pstEmmMotor->stRunningParam.aucLastCmdData))
                                                 ? ucDataLen
                                                 : sizeof(pstEmmMotor->stRunningParam.aucLastCmdData);
    memcpy(pstEmmMotor->stRunningParam.aucLastCmdData, pucCmdData,
           pstEmmMotor->stRunningParam.ucLastCmdDataLen);
}

/**
 * @brief EmmMotor虚方法表
 */
static stMotorVTableTdf g_stEmmMotorVTable = {
    vEmmMotorInit,              /* 对应vInit(void *pstInit); */
    vEmmMotorPeriodExecute,     /* 对应vPeriodExecute(void *pstPeriodExecute); */
    vEmmMotorStop,              /* 对应vStop(void *pstMotor, uint8_t bSyncFlag); */
    vEmmMotorEnable,            /* 对应vEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag); */
    emGetEmmMotorState,         /* 对应emGetEmmMotorState(void *pstMotor); */
    vEmmMotorPosControl,        /* 对应vPosControl(...); */
    vEmmMotorVelControl,        /* 对应vVelControl(...); */
    vEmmMotorSynchronousMotion, /* 对应vSynchronousMotion(...); */
};

/* 虚方法实现 ************************************* */

/**
 * @brief  EmmMotor初始化
 * @param  pstInit ：EmmMotor设备指针
 * @note   实现父类方法 vInit(void *pstInit);
 */
void vEmmMotorInit(void *pstInit)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstInit;
    if (pstEmmMotor == NULL) {
        return;
    }

    pstEmmMotor->stRunningParam.ucRetryMax = EMM_MOTOR_DEFAULT_MAX_RETRY;
    pstEmmMotor->stRunningParam.emLastCmd = emEmmMotorLastCmd_None;

    /* 首次初始化时注册 UART 回调（4个电机共用一条 UART，只注册一次） */
    if (!g_ucEmmCallbackRegistered) {
        g_ucEmmCallbackRegistered = 1;
        g_stEmmParser.emState = kEmmParseIdle;
        vUartSetCallback(pstEmmMotor->stStaticParam.emUartDevNum, vEmmMotorUartCallback);
    }
}

/**
 * @brief  EmmMotor周期执行 — UART 回调消费 + 超时重试 + 错误记录
 * @param  pstMotor ：EmmMotor设备指针
 * @note   每 30ms 发送一次 Flag 查询；300ms 无应答触发重试；
 *         3 次重试耗尽（总计 < 1s）记录 emEmmMotorErr_NoResponse 并强制切 Stop。
 *         检测到堵转（Flag bit2）记录 emEmmMotorErr_Stall。
 */
void vEmmMotorPeriodExecute(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) return;

    /* 解析器超时复位：若停留在收集态超过阈值，强制复位以防卡死 */
    if (g_stEmmParser.emState == kEmmParseCollect) {
        if ((QE_GET_TICK() - g_stEmmParser.ulEnterTick) >= EMM_PARSER_TIMEOUT_MS) {
            g_stEmmParser.emState = kEmmParseIdle;
        }
    }

    if (pstEmmMotor->stBase.emMotorState != emMotorStateRunning) {
        pstEmmMotor->stRunningParam.usPollTimeout = 0;
        pstEmmMotor->stRunningParam.ucRetryCnt  = 0;
        return;
    }

    /* 消费 UART 回调更新的 Flag 数据 */
    if (pstEmmMotor->stRunningParam.ucFlagUpdated) {
        pstEmmMotor->stRunningParam.ucFlagUpdated = 0;
        pstEmmMotor->stRunningParam.usPollTimeout = 0;
        pstEmmMotor->stRunningParam.ucRetryCnt   = 0;

        /* 检测堵转 */
        if (pstEmmMotor->stRunningParam.ucLastFlag & 0x04) {
            pstEmmMotor->stRunningParam.emLastError = emEmmMotorErr_Stall;
            pstEmmMotor->stRunningParam.ulErrorTick = QE_GET_TICK();
        }
        /* 检测到位 */
        if (pstEmmMotor->stRunningParam.ucLastFlag & 0x02) {
            pstEmmMotor->stBase.emMotorState = emMotorStateStop;
        }
        return;
    }

    /* 分频发送 Flag 查询 */
    if (++pstEmmMotor->stRunningParam.ucPollCnt >= 30) {
        pstEmmMotor->stRunningParam.ucPollCnt = 0;
        pstEmmMotor->stRunningParam.usPollTimeout++;

        /* 超时 → 重试或放弃 */
        if (pstEmmMotor->stRunningParam.usPollTimeout >= EMM_MOTOR_RETRY_TIMEOUT_THRESHOLD) {
            pstEmmMotor->stRunningParam.usPollTimeout = 0;
            if (pstEmmMotor->stRunningParam.ucRetryCnt < pstEmmMotor->stRunningParam.ucRetryMax
                && pstEmmMotor->stRunningParam.emLastCmd != emEmmMotorLastCmd_None
                && pstEmmMotor->stRunningParam.ucLastCmdDataLen > 0) {
                pstEmmMotor->stRunningParam.ucRetryCnt++;
                /* 重试时强制 snF=0（立即执行），避免遗留缓存指令等不到同步广播 */
                uint8_t aucRetry[12];
                memcpy(aucRetry, pstEmmMotor->stRunningParam.aucLastCmdData,
                       pstEmmMotor->stRunningParam.ucLastCmdDataLen);
                aucRetry[pstEmmMotor->stRunningParam.ucLastCmdDataLen - 1] = 0;
                vEmmMotorSendCmdEx(pstEmmMotor, aucRetry,
                                   pstEmmMotor->stRunningParam.ucLastCmdDataLen, 0);
            } else {
                pstEmmMotor->stRunningParam.emLastError = emEmmMotorErr_NoResponse;
                pstEmmMotor->stRunningParam.ulErrorTick = QE_GET_TICK();
                pstEmmMotor->stBase.emMotorState = emMotorStateStop;
                return;
            }
        }
        vEmmMotorReadSysParams(pstMotor, emEmmMotorSysParam_Flag);
    }
}

/**
 * @brief  EmmMotor使能控制
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bEnable ：使能状态
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 * @note   实现父类方法 vEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag);
 */
void vEmmMotorEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0xF3, 0xAB, bEnable, bSyncFlag};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}


/**
 * @brief 获取EmmMotor电机运动状态
 * @param pstMotor EmmMotor电机设备指针
 * @return emMotorStateTdf 电机运动状态
 * @note   实现父类方法 emGetEmmMotorState(void *pstMotor);
 */
emMotorStateTdf emGetEmmMotorState(void *pstMotor) {
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return emMotorStateNULL;
    }
    return pstEmmMotor->stBase.emMotorState;
}

/**
 * @brief  位置模式控制
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emDir ：方向
 * @param  usVel ：速度(RPM)，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  ulClk ：脉冲数，范围0- (2^32 - 1)个
 * @param  bAbsFlag ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 * @note   实现父类方法 vPosControl(...);
 */
void vEmmMotorPosControl(
    void *pstMotor, emMotorDirTdf emDir,
    uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag,
    uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }

    if (pstEmmMotor->stStaticParam.ucReversed) {
        emDir = (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;
    }

    const uint8_t aucCmdData[] = {
        0xFD,
        emDir,
        (uint8_t)(usVel >> 8),
        (uint8_t)(usVel >> 0),
        ucAcc,
        (uint8_t)(ulClk >> 24),
        (uint8_t)(ulClk >> 16),
        (uint8_t)(ulClk >> 8),
        (uint8_t)(ulClk >> 0),
        bAbsFlag,
        bSyncFlag
    };
    vEmmMotorSaveLastCmd(pstEmmMotor, emEmmMotorLastCmd_Pos, aucCmdData, sizeof(aucCmdData));
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
    pstEmmMotor->stBase.emMotorState = emMotorStateRunning;
    /* 按地址错开轮询起点，避免4电机同tick查询导致响应帧交错 */
    pstEmmMotor->stRunningParam.ucPollCnt         = (pstEmmMotor->stStaticParam.ucAddr - 1) * 7;
    pstEmmMotor->stRunningParam.ucPollWait        = 0;
    pstEmmMotor->stRunningParam.usPollTimeout     = 0;
    pstEmmMotor->stRunningParam.ucFlagUpdated     = 0;
    pstEmmMotor->stRunningParam.ucCmdAckReceived  = 0;
    pstEmmMotor->stRunningParam.ucRetryCnt        = 0;
}


/**
 * @brief  速度模式控制
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emDir ：方向
 * @param  usVel ：速度，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 * @note   实现父类方法 vVelControl(...);
 */
void vEmmMotorVelControl(void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }

    if (pstEmmMotor->stStaticParam.ucReversed) {
        emDir = (emDir == emMotorDir_Forward) ? emMotorDir_Backward : emMotorDir_Forward;
    }

    const uint8_t aucCmdData[] = {
        0xF6,
        emDir,
        (uint8_t)(usVel >> 8),
        (uint8_t)(usVel >> 0),
        ucAcc,
        bSyncFlag
    };
    vEmmMotorSaveLastCmd(pstEmmMotor, emEmmMotorLastCmd_Vel, aucCmdData, sizeof(aucCmdData));
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
    pstEmmMotor->stBase.emMotorState = emMotorStateRunning;
    /* 按地址错开轮询起点，避免4电机同tick查询导致响应帧交错 */
    pstEmmMotor->stRunningParam.ucPollCnt         = (pstEmmMotor->stStaticParam.ucAddr - 1) * 7;
    pstEmmMotor->stRunningParam.ucPollWait        = 0;
    pstEmmMotor->stRunningParam.usPollTimeout     = 0;
    pstEmmMotor->stRunningParam.ucFlagUpdated     = 0;
    pstEmmMotor->stRunningParam.ucCmdAckReceived  = 0;
    pstEmmMotor->stRunningParam.ucRetryCnt        = 0;
}

/******************************************************/


/**
 * @brief  设置EmmMotor速度
 * @param  pstMotor ：EmmMotor设备指针
 * @param  speed ：速度值（RPM）
 */
void vEmmMotorSetSpeed(void *pstMotor, int16_t speed)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    emMotorDirTdf emDir = (speed >= 0) ? emMotorDir_Forward : emMotorDir_Backward;
    uint16_t usVel = (speed < 0) ? -speed : speed;
    
    // 调用速度控制函数
    vEmmMotorVelControl(pstMotor, emDir, usVel, 10, 0);
}

/**
 * @brief  停止EmmMotor
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmMotorStop(void *pstMotor, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0xFE, 0x98, bSyncFlag};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
    pstEmmMotor->stBase.emMotorState = emMotorStateStop;
}



/**
 * @brief  注册EmmMotor设备
 * @param  emDevNum ：设备号
 * @param  pstInit ：静态参数
 */
void vEmmMotorRegister(emMotorDevNumTdf emDevNum, stEmmMotorStaticParamTdf *pstInit)
{   
    emMotorDevNumTdf offsetDevNum = (emMotorDevNumTdf) (emDevNum - emEmmMotorDevNum0);
    
    if (offsetDevNum < EMM_MOTOR_DEV_NUM && pstInit != NULL) {

        // 初始化基类
        astEmmMotorDeviceParam[offsetDevNum].stBase.emType = emMotorType_Emm;
        astEmmMotorDeviceParam[offsetDevNum].stBase.pstVTable = &g_stEmmMotorVTable;

        memcpy(&astEmmMotorDeviceParam[offsetDevNum].stStaticParam, 
           pstInit, 
           sizeof(stEmmMotorStaticParamTdf));
    
        memset(&astEmmMotorDeviceParam[offsetDevNum].stRunningParam, 
            0, 
            sizeof(stEmmMotorRunningParamTdf));
        
        // 注册到基类
        vMotorRegisterDevice(emDevNum, &astEmmMotorDeviceParam[offsetDevNum].stBase);

    }
}

const stEmmMotorStaticParamTdf *c_pstGetEmmMotorStaticParam(emMotorDevNumTdf emDevNum)
{
    emMotorDevNumTdf offsetDevNum = (emMotorDevNumTdf)(emDevNum - emEmmMotorDevNum0);
    if (offsetDevNum < EMM_MOTOR_DEV_NUM) {
        return &astEmmMotorDeviceParam[offsetDevNum].stStaticParam;
    }
    return NULL;
}

/**
 * @brief  将当前位置清零
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorResetCurPosToZero(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x0A, 0x6D};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}

/**
 * @brief  解除堵转保护
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorResetClogPro(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x0E, 0x52};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}

/**
 * @brief  读取系统参数
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emSysParam ：系统参数类型
 */
void vEmmMotorReadSysParams(void *pstMotor, emEmmMotorSysParamTdf emSysParam)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    // 参数码表：单字节参数和双字节参数
    static const uint8_t aucParamCode[][2] = {
        [emEmmMotorSysParam_Ver]   = {0x1F, 0x00},
        [emEmmMotorSysParam_RL]    = {0x20, 0x00},
        [emEmmMotorSysParam_PID]   = {0x21, 0x00},
        [emEmmMotorSysParam_VBus]  = {0x24, 0x00},
        [emEmmMotorSysParam_Cpha]  = {0x27, 0x00},
        [emEmmMotorSysParam_Encl]  = {0x31, 0x00},
        [emEmmMotorSysParam_TPos]  = {0x33, 0x00},
        [emEmmMotorSysParam_Vel]   = {0x35, 0x00},
        [emEmmMotorSysParam_CPos]  = {0x36, 0x00},
        [emEmmMotorSysParam_PErr]  = {0x37, 0x00},
        [emEmmMotorSysParam_Flag]  = {0x3A, 0x00},
        [emEmmMotorSysParam_Org]   = {0x3B, 0x00},
        [emEmmMotorSysParam_Conf]  = {0x42, 0x6C},
        [emEmmMotorSysParam_State] = {0x43, 0x7A},
    };
    
    if (emSysParam >= sizeof(aucParamCode) / sizeof(aucParamCode[0])) {
        return;
    }

    uint8_t aucCmdData[2];
    uint8_t ucLen = 0;
    aucCmdData[ucLen++] = aucParamCode[emSysParam][0];
    if (aucParamCode[emSysParam][1] != 0) {
        aucCmdData[ucLen++] = aucParamCode[emSysParam][1];
    }
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, ucLen, 0);
}

/**
 * @brief  修改开环/闭环控制模式
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSave ：存储标志
 * @param  emCtrlMode ：控制模式
 */
void vEmmMotorModifyCtrlMode(void *pstMotor, uint8_t bSave, emEmmMotorCtrlModeTdf emCtrlMode)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x46, 0x69, bSave, emCtrlMode};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}





/**
 * @brief  触发多机同步开始运动
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorSynchronousMotion(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }

    const uint8_t aucCmdData[] = {0xFF, 0x66};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_SYNC_GAP_MS);
}

/**
 * @brief  广播同步触发 — 一条指令同时启动总线上所有 EMM 电机
 * @param  emUartDevNum ：UART 设备号
 * @note   使用地址 0x00 广播，所有已收到 snF=1 指令的电机同时开始运动。
 *         广播指令不期待应答，不经过 vEmmMotorSendCmdEx（无延时）。
 */
void vEmmMotorSyncBroadcast(emUartDevNumTdf emUartDevNum)
{
    uint8_t aucCmd[4];
    aucCmd[0] = 0x00;
    aucCmd[1] = 0xFF;
    aucCmd[2] = 0x66;

    /* 查找该 UART 上任意一个 EMM 电机，获取其校验模式以计算正确校验和 */
    emEmmMotorChecksumTdf emMode = emEmmMotorChecksum_Fixed6B;
    for (uint8_t i = 0; i < EMM_MOTOR_DEV_NUM; i++) {
        if (astEmmMotorDeviceParam[i].stStaticParam.emUartDevNum == emUartDevNum
            && astEmmMotorDeviceParam[i].stStaticParam.ucAddr != 0) {
            emMode = astEmmMotorDeviceParam[i].stStaticParam.emChecksumMode;
            break;
        }
    }

    aucCmd[3] = ucEmmComputeChecksum(emMode, aucCmd, 3);
    vUartSendArray(emUartDevNum, aucCmd, 4);
}

/**
 * @brief  设置单圈回零的零点位置
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 */
void vEmmMotorOriginSetO(void *pstMotor, uint8_t bSave)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x93, 0x88, bSave};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}

/**
 * @brief  修改回零参数
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 * @param  emOrgMode ：回零模式
 * @param  emDir ：回零方向
 * @param  usOrgVel ：回零速度，单位：RPM（转/分钟）
 * @param  ulOrgTm ：回零超时时间，单位：毫秒
 * @param  usSlVel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param  usSlMa ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param  usSlMs ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param  bPotFlag ：上电自动触发回零，false为不使能，true为使能
 */
void vEmmMotorOriginModifyParams(void *pstMotor, uint8_t bSave, emEmmMotorOrgModeTdf emOrgMode, emMotorDirTdf emDir, 
                               uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                               uint16_t usSlMs, uint8_t bPotFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {
        0x4C,
        0xAE,
        bSave,
        emOrgMode,
        emDir,
        (uint8_t)(usOrgVel >> 8),
        (uint8_t)(usOrgVel >> 0),
        (uint8_t)(ulOrgTm >> 24),
        (uint8_t)(ulOrgTm >> 16),
        (uint8_t)(ulOrgTm >> 8),
        (uint8_t)(ulOrgTm >> 0),
        (uint8_t)(usSlVel >> 8),
        (uint8_t)(usSlVel >> 0),
        (uint8_t)(usSlMa >> 8),
        (uint8_t)(usSlMa >> 0),
        (uint8_t)(usSlMs >> 8),
        (uint8_t)(usSlMs >> 0),
        bPotFlag
    };
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}

/**
 * @brief  触发回零
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emOrgMode ：回零模式
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmMotorOriginTriggerReturn(void *pstMotor, emEmmMotorOrgModeTdf emOrgMode, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x9A, emOrgMode, bSyncFlag};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}

/**
 * @brief  强制中断并退出回零
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorOriginInterrupt(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x9C, 0x48};
    vEmmMotorSendCmdEx(pstEmmMotor, aucCmdData, sizeof(aucCmdData), EMM_MOTOR_CMD_GAP_MS);
}

/* ==================== 错误记录 ==================== */

emEmmMotorErrTdf emGetEmmMotorLastError(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) return emEmmMotorErr_None;
    return pstEmmMotor->stRunningParam.emLastError;
}

void vEmmMotorClearError(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) return;
    pstEmmMotor->stRunningParam.emLastError = emEmmMotorErr_None;
    pstEmmMotor->stRunningParam.ulErrorTick = 0;
}

uint8_t ucGetEmmMotorLastFlag(emMotorDevNumTdf emDevNum)
{
    if (emDevNum < emEmmMotorDevNum0 || emDevNum >= emEmmMotorDevMax) return 0;
    uint8_t ucIdx = emDevNum - emEmmMotorDevNum0;
    return astEmmMotorDeviceParam[ucIdx].stRunningParam.ucLastFlag;
}

uint16_t usGetEmmMotorFlagRxCnt(emMotorDevNumTdf emDevNum)
{
    if (emDevNum < emEmmMotorDevNum0 || emDevNum >= emEmmMotorDevMax) return 0;
    uint8_t ucIdx = emDevNum - emEmmMotorDevNum0;
    return g_aucEmmFlagRxCnt[ucIdx];
}

#endif
