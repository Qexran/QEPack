/**
  * @file       atk_ms901m_device.c
  * @author     正点原子团队(ALIENTEK) & Qe_xr
  * @version    V1.3.0
  * @date       2026/05/14
  * @brief      ATK-MS901M 模块驱动，基于 STM32 HAL 库 / TI DriverLib
  *             支持阻塞轮询模式和流式缓存回调模式
  */

#include "atk_ms901m_device.h"

#if ATK_MS901M_IS_ENABLE

/* ATK-MS901M UART通讯帧头 */
#define ATK_MS901M_FRAME_HEAD_L             0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H      0x55
#define ATK_MS901M_FRAME_HEAD_ACK_H         0xAF

#define ATK_MS901M_READ_REG_ID(id)         (id | 0x80)
#define ATK_MS901M_WRITE_REG_ID(id)        (id)

/* ---- 内部帧解析状态 ---- */
typedef enum
{
    kWaitHeadL = 0,
    kWaitHeadH,
    kWaitId,
    kWaitLen,
    kWaitDat,
    kWaitSum,
} atk_ms901m_parse_state_t;

/* ---- 内部帧结构 ---- */
typedef struct
{
    uint8_t head_l;
    uint8_t head_h;
    uint8_t id;
    uint8_t len;
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];
    uint8_t check_sum;
} atk_ms901m_frame_t;

/* 满量程表 */
static const uint16_t g_au16GyroFsrTable[4]   = {250, 500, 1000, 2000};
static const uint8_t  g_au8AccelFsrTable[4]  = {2, 4, 8, 16};

/* ---- 单设备流式运行时数据 ---- */
typedef struct
{
    uint8_t                         ucOnline;               /* 流式模式是否已启动 */
    emUartDevNumTdf                 emUartDevNum;           /* 绑定的 UART 设备号 */

    /* 帧解析状态机 */
    atk_ms901m_parse_state_t        emParseState;
    uint8_t                         ucDatIndex;
    atk_ms901m_frame_t              stFrame;

    /* 满量程（从模块读取） */
    uint8_t                         ucGyroFsr;
    uint8_t                         ucAccelFsr;

    /* 最新传感器数据缓存（嵌入设备参数结构体，用户可通过 getter 只读访问） */
    stAtkMs901mDeviceParamTdf       stDeviceParam;

    /* 数据更新标志（收到帧后置 1，用户读取后可选清除） */
    uint8_t                         ucAttitudeUpdated  : 1;
    uint8_t                         ucQuatUpdated      : 1;
    uint8_t                         ucGyroAccelUpdated : 1;
    uint8_t                         ucMagUpdated       : 1;
    uint8_t                         ucBaroUpdated      : 1;
    uint8_t                         ucPortUpdated      : 1;
} atk_ms901m_runtime_t;

static atk_ms901m_runtime_t g_astRuntime[ATK_MS901M_DEV_NUM];

/* 阻塞模式使用的满量程缓存（兼容旧 API），默认最大量程避免溢出 */
static struct
{
    uint8_t ucGyroFsr;
    uint8_t ucAccelFsr;
} g_stBlockingFsr = {3, 3};  /* 2000dps, 16g */

/* ---- 前向声明 ---- */
static void vAtkMs901mUartCallback(emUartDevNumTdf emUartDevNum, stUartRunningParamTdf *pstRunning);

/* ================================================================ */
/*  内部帧解码                                                       */
/* ================================================================ */

/**
 * @brief  解析 GYRO_ACCE 帧数据（流式/阻塞共用）
 */
static void vAtkMs901mDecodeGyroAccel(const uint8_t *pucDat, uint8_t ucGyroFsr, uint8_t ucAccelFsr,
                                       atk_ms901m_gyro_data_t *pstGyro,
                                       atk_ms901m_accelerometer_data_t *pstAccel)
{
    if (pstGyro != NULL && ucGyroFsr < 4)
    {
        int16_t i16Raw;
        i16Raw = (int16_t)(pucDat[7] << 8) | pucDat[6];
        pstGyro->raw.x = i16Raw;
        pstGyro->x = (float)i16Raw / 32768.0f * (float)g_au16GyroFsrTable[ucGyroFsr];
        i16Raw = (int16_t)(pucDat[9] << 8) | pucDat[8];
        pstGyro->raw.y = i16Raw;
        pstGyro->y = (float)i16Raw / 32768.0f * (float)g_au16GyroFsrTable[ucGyroFsr];
        i16Raw = (int16_t)(pucDat[11] << 8) | pucDat[10];
        pstGyro->raw.z = i16Raw;
        pstGyro->z = (float)i16Raw / 32768.0f * (float)g_au16GyroFsrTable[ucGyroFsr];
    }
    if (pstAccel != NULL && ucAccelFsr < 4)
    {
        int16_t i16Raw;
        i16Raw = (int16_t)(pucDat[1] << 8) | pucDat[0];
        pstAccel->raw.x = i16Raw;
        pstAccel->x = (float)i16Raw / 32768.0f * (float)g_au8AccelFsrTable[ucAccelFsr];
        i16Raw = (int16_t)(pucDat[3] << 8) | pucDat[2];
        pstAccel->raw.y = i16Raw;
        pstAccel->y = (float)i16Raw / 32768.0f * (float)g_au8AccelFsrTable[ucAccelFsr];
        i16Raw = (int16_t)(pucDat[5] << 8) | pucDat[4];
        pstAccel->raw.z = i16Raw;
        pstAccel->z = (float)i16Raw / 32768.0f * (float)g_au8AccelFsrTable[ucAccelFsr];
    }
}

/**
 * @brief  解析单个完成帧并缓存传感器数据
 */
static void vAtkMs901mDecodeFrame(atk_ms901m_runtime_t *pstRt, atk_ms901m_frame_t *pstFrame)
{
    switch (pstFrame->id)
    {
        case ATK_MS901M_FRAME_ID_ATTITUDE:
        {
            pstRt->stDeviceParam.stAttitude.roll  = (float)((int16_t)(pstFrame->dat[1] << 8) | pstFrame->dat[0]) / 32768.0f * 180.0f;
            pstRt->stDeviceParam.stAttitude.pitch = (float)((int16_t)(pstFrame->dat[3] << 8) | pstFrame->dat[2]) / 32768.0f * 180.0f;
            pstRt->stDeviceParam.stAttitude.yaw   = (float)((int16_t)(pstFrame->dat[5] << 8) | pstFrame->dat[4]) / 32768.0f * 180.0f;
            pstRt->ucAttitudeUpdated = 1;
            break;
        }
        case ATK_MS901M_FRAME_ID_QUAT:
        {
            pstRt->stDeviceParam.stQuaternion.q0 = (float)((int16_t)(pstFrame->dat[1] << 8) | pstFrame->dat[0]) / 32768.0f;
            pstRt->stDeviceParam.stQuaternion.q1 = (float)((int16_t)(pstFrame->dat[3] << 8) | pstFrame->dat[2]) / 32768.0f;
            pstRt->stDeviceParam.stQuaternion.q2 = (float)((int16_t)(pstFrame->dat[5] << 8) | pstFrame->dat[4]) / 32768.0f;
            pstRt->stDeviceParam.stQuaternion.q3 = (float)((int16_t)(pstFrame->dat[7] << 8) | pstFrame->dat[6]) / 32768.0f;
            pstRt->ucQuatUpdated = 1;
            break;
        }
        case ATK_MS901M_FRAME_ID_GYRO_ACCE:
        {
            vAtkMs901mDecodeGyroAccel(pstFrame->dat, pstRt->ucGyroFsr, pstRt->ucAccelFsr,
                                       &pstRt->stDeviceParam.stGyro,
                                       &pstRt->stDeviceParam.stAccel);
            pstRt->ucGyroAccelUpdated = 1;
            break;
        }
        case ATK_MS901M_FRAME_ID_MAG:
        {
            pstRt->stDeviceParam.stMag.x = (int16_t)(pstFrame->dat[1] << 8) | pstFrame->dat[0];
            pstRt->stDeviceParam.stMag.y = (int16_t)(pstFrame->dat[3] << 8) | pstFrame->dat[2];
            pstRt->stDeviceParam.stMag.z = (int16_t)(pstFrame->dat[5] << 8) | pstFrame->dat[4];
            pstRt->stDeviceParam.stMag.temperature = (float)((int16_t)(pstFrame->dat[7] << 8) | pstFrame->dat[6]) / 100.0f;
            pstRt->ucMagUpdated = 1;
            break;
        }
        case ATK_MS901M_FRAME_ID_BARO:
        {
            pstRt->stDeviceParam.stBaro.pressure = (int32_t)((uint32_t)pstFrame->dat[3] << 24) | ((uint32_t)pstFrame->dat[2] << 16) | ((uint32_t)pstFrame->dat[1] << 8) | pstFrame->dat[0];
            pstRt->stDeviceParam.stBaro.altitude = (int32_t)((uint32_t)pstFrame->dat[7] << 24) | ((uint32_t)pstFrame->dat[6] << 16) | ((uint32_t)pstFrame->dat[5] << 8) | pstFrame->dat[4];
            pstRt->stDeviceParam.stBaro.temperature = (float)((int16_t)(pstFrame->dat[9] << 8) | pstFrame->dat[8]) / 100.0f;
            pstRt->ucBaroUpdated = 1;
            break;
        }
        case ATK_MS901M_FRAME_ID_PORT:
        {
            pstRt->stDeviceParam.stPort.d0 = (uint16_t)(pstFrame->dat[1] << 8) | pstFrame->dat[0];
            pstRt->stDeviceParam.stPort.d1 = (uint16_t)(pstFrame->dat[3] << 8) | pstFrame->dat[2];
            pstRt->stDeviceParam.stPort.d2 = (uint16_t)(pstFrame->dat[5] << 8) | pstFrame->dat[4];
            pstRt->stDeviceParam.stPort.d3 = (uint16_t)(pstFrame->dat[7] << 8) | pstFrame->dat[6];
            pstRt->ucPortUpdated = 1;
            break;
        }
        default:
            break;
    }
}

/**
 * @brief  向帧解析状态机喂入一个字节
 * @return 1: 帧完成, 0: 继续
 */
static uint8_t ucAtkMs901mFeedByte(atk_ms901m_runtime_t *pstRt, uint8_t ucData)
{
    atk_ms901m_frame_t *pstFrame = &pstRt->stFrame;

    switch (pstRt->emParseState)
    {
        case kWaitHeadL:
        {
            if (ucData == ATK_MS901M_FRAME_HEAD_L)
            {
                pstFrame->head_l = ucData;
                pstFrame->check_sum = ucData;
                pstRt->emParseState = kWaitHeadH;
            }
            break;
        }
        case kWaitHeadH:
        {
            if (ucData == ATK_MS901M_FRAME_HEAD_UPLOAD_H || ucData == ATK_MS901M_FRAME_HEAD_ACK_H)
            {
                pstFrame->head_h = ucData;
                pstFrame->check_sum += ucData;
                pstRt->emParseState = kWaitId;
            }
            else
            {
                pstRt->emParseState = kWaitHeadL;
            }
            break;
        }
        case kWaitId:
        {
            pstFrame->id = ucData;
            pstFrame->check_sum += ucData;
            pstRt->emParseState = kWaitLen;
            break;
        }
        case kWaitLen:
        {
            if (ucData > ATK_MS901M_FRAME_DAT_MAX_SIZE)
            {
                pstRt->emParseState = kWaitHeadL;
            }
            else
            {
                pstFrame->len = ucData;
                pstFrame->check_sum += ucData;
                pstRt->ucDatIndex = 0;
                pstRt->emParseState = (ucData == 0) ? kWaitSum : kWaitDat;
            }
            break;
        }
        case kWaitDat:
        {
            pstFrame->dat[pstRt->ucDatIndex] = ucData;
            pstFrame->check_sum += ucData;
            pstRt->ucDatIndex++;
            if (pstRt->ucDatIndex >= pstFrame->len)
            {
                pstRt->ucDatIndex = 0;
                pstRt->emParseState = kWaitSum;
            }
            break;
        }
        case kWaitSum:
        {
            pstRt->emParseState = kWaitHeadL;
            if (ucData == pstFrame->check_sum)
            {
                return 1;   /* 帧完成 */
            }
            break;
        }
        default:
        {
            pstRt->emParseState = kWaitHeadL;
            break;
        }
    }
    return 0;
}

/**
 * @brief  处理一批接收到的字节（由 UART 回调或 DMA 回调调用）
 */
static void vAtkMs901mProcessBytes(atk_ms901m_runtime_t *pstRt, const uint8_t *pucData, uint16_t usLen)
{
    uint16_t i;
    for (i = 0; i < usLen; i++)
    {
        if (ucAtkMs901mFeedByte(pstRt, pucData[i]))
        {
            vAtkMs901mDecodeFrame(pstRt, &pstRt->stFrame);
        }
    }
}

/* ================================================================ */
/*  UART 回调 — 由 vUartDevicePeriodExecute 触发                     */
/* ================================================================ */

static void vAtkMs901mUartCallback(emUartDevNumTdf emUartDevNum, stUartRunningParamTdf *pstRunning)
{
    uint8_t i;
    /* 查找此 UART 设备号对应的 ATK-MS901M 运行时 */
    for (i = 0; i < ATK_MS901M_DEV_NUM; i++)
    {
        if (g_astRuntime[i].ucOnline && g_astRuntime[i].emUartDevNum == emUartDevNum)
        {
            vAtkMs901mProcessBytes(&g_astRuntime[i],
                                   pstRunning->aucFrameDataBuf,
                                   (uint16_t)pstRunning->ulFrameDataCount);
            break;
        }
    }
}

/* ================================================================ */
/*  流式缓存模式 API                                                 */
/* ================================================================ */

/**
 * @brief  启动流式缓存模式
 */
uint8_t atk_ms901m_start_streaming(emAtkMs901mDevNumTdf emDevNum, emUartDevNumTdf emUartDevNum)
{
    atk_ms901m_runtime_t *pstRt;

    if (emDevNum >= ATK_MS901M_DEV_NUM)
    {
        return ATK_MS901M_ERROR;
    }

    pstRt = &g_astRuntime[emDevNum];
    memset(pstRt, 0, sizeof(atk_ms901m_runtime_t));

    pstRt->emUartDevNum = emUartDevNum;
    pstRt->emParseState = kWaitHeadL;

    /* 同步由 atk_ms901m_init() 读取的满量程 */
    pstRt->ucGyroFsr  = g_stBlockingFsr.ucGyroFsr;
    pstRt->ucAccelFsr = g_stBlockingFsr.ucAccelFsr;

    /* 先标记在线，再注册回调，避免 ISR 在 ucOnline=0 时触发回调导致数据丢失 */
    pstRt->ucOnline = 1;
    vUartSetCallback(emUartDevNum, vAtkMs901mUartCallback);
    return ATK_MS901M_EOK;
}

/**
 * @brief  停止流式缓存模式
 */
void atk_ms901m_stop_streaming(emAtkMs901mDevNumTdf emDevNum)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM)
    {
        return;
    }

    vUartSetCallback(g_astRuntime[emDevNum].emUartDevNum, NULL);
    memset(&g_astRuntime[emDevNum], 0, sizeof(atk_ms901m_runtime_t));
}

/**
 * @brief  获取设备参数只读指针
 * @param  emDevNum: ATK-MS901M 设备号
 * @return 设备参数只读指针，emDevNum 越界返回 NULL
 * @note   流式模式下，缓存数据由 ISR 上下文自动更新
 */
const stAtkMs901mDeviceParamTdf *c_pstGetAtkMs901mDeviceParam(emAtkMs901mDevNumTdf emDevNum)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM)
    {
        return NULL;
    }
    return &g_astRuntime[emDevNum].stDeviceParam;
}

uint8_t atk_ms901m_read_attitude(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_attitude_data_t *attitude_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || attitude_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(attitude_dat, &g_astRuntime[emDevNum].stDeviceParam.stAttitude, sizeof(atk_ms901m_attitude_data_t));
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_read_quaternion(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_quaternion_data_t *quaternion_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || quaternion_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(quaternion_dat, &g_astRuntime[emDevNum].stDeviceParam.stQuaternion, sizeof(atk_ms901m_quaternion_data_t));
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_read_gyro(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_gyro_data_t *gyro_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || gyro_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(gyro_dat, &g_astRuntime[emDevNum].stDeviceParam.stGyro, sizeof(atk_ms901m_gyro_data_t));
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_read_accelerometer(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_accelerometer_data_t *accelerometer_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || accelerometer_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(accelerometer_dat, &g_astRuntime[emDevNum].stDeviceParam.stAccel, sizeof(atk_ms901m_accelerometer_data_t));
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_read_magnetometer(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_magnetometer_data_t *magnetometer_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || magnetometer_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(magnetometer_dat, &g_astRuntime[emDevNum].stDeviceParam.stMag, sizeof(atk_ms901m_magnetometer_data_t));
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_read_barometer(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_barometer_data_t *barometer_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || barometer_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(barometer_dat, &g_astRuntime[emDevNum].stDeviceParam.stBaro, sizeof(atk_ms901m_barometer_data_t));
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_read_port(emAtkMs901mDevNumTdf emDevNum, atk_ms901m_port_data_t *port_dat)
{
    if (emDevNum >= ATK_MS901M_DEV_NUM || port_dat == NULL || !g_astRuntime[emDevNum].ucOnline)
    { return ATK_MS901M_ERROR; }
    memcpy(port_dat, &g_astRuntime[emDevNum].stDeviceParam.stPort, sizeof(atk_ms901m_port_data_t));
    return ATK_MS901M_EOK;
}

/* ================================================================ */
/*  阻塞轮询 API（保留兼容）                                          */
/* ================================================================ */

/**
 * @brief  通过指定帧ID获取接收到的数据帧（阻塞）
 */
static uint8_t atk_ms901m_get_frame_by_id(emUartDevNumTdf emDevNum, atk_ms901m_frame_t *frame, uint8_t id, uint8_t id_type, uint32_t timeout)
{
    uint8_t dat;
    uint8_t ucRet;
    atk_ms901m_parse_state_t parse_state = kWaitHeadL;
    uint8_t dat_index = 0;
    uint16_t timeout_index = 0;
    const stUartDeviceParamTdf *pstDev;
    vUartFrameCallback pfnSavedCallback;

    /* 临时接管 UART 回调，防止 ISR 中回调消费 ring buffer 导致数据丢失 */
    pstDev = c_pstGetUartDeviceParam(emDevNum);
    pfnSavedCallback = pstDev->stStaticParam.vCallbackFcn;
    if (pfnSavedCallback != NULL)
    {
        vUartSetCallback(emDevNum, NULL);
    }

    ucRet = ATK_MS901M_ETIMEOUT;
    while (1)
    {
        if (timeout == 0)
        {
            ucRet = ATK_MS901M_ETIMEOUT;
            break;
        }

        if (ucUartRxAvailable(emDevNum) == 0)
        {
            timeout_index++;
            if (timeout_index == 1000)
            {
                timeout_index = 0;
                timeout--;
            }
            continue;
        }

        /* 关中断读取 ring buffer，防止与 UART RX ISR 中的 count++ 竞态 */
        {
            uint32_t primask;
            primask = __get_PRIMASK();
            __disable_irq();
            dat = ucUartReceiveByte(emDevNum);
            __set_PRIMASK(primask);
        }

        switch (parse_state)
        {
            case kWaitHeadL:
            {
                if (dat == ATK_MS901M_FRAME_HEAD_L)
                {
                    frame->head_l = dat;
                    frame->check_sum = frame->head_l;
                    parse_state = kWaitHeadH;
                }
                break;
            }
            case kWaitHeadH:
            {
                if (id_type == ATK_MS901M_FRAME_ID_TYPE_UPLOAD && dat == ATK_MS901M_FRAME_HEAD_UPLOAD_H)
                {
                    frame->head_h = dat;
                    frame->check_sum += frame->head_h;
                    parse_state = kWaitId;
                }
                else if (id_type == ATK_MS901M_FRAME_ID_TYPE_ACK && dat == ATK_MS901M_FRAME_HEAD_ACK_H)
                {
                    frame->head_h = dat;
                    frame->check_sum += frame->head_h;
                    parse_state = kWaitId;
                }
                else
                {
                    parse_state = kWaitHeadL;
                }
                break;
            }
            case kWaitId:
            {
                if (dat == id)
                {
                    frame->id = dat;
                    frame->check_sum += frame->id;
                    parse_state = kWaitLen;
                }
                else
                {
                    parse_state = kWaitHeadL;
                }
                break;
            }
            case kWaitLen:
            {
                if (dat > ATK_MS901M_FRAME_DAT_MAX_SIZE)
                {
                    parse_state = kWaitHeadL;
                }
                else
                {
                    frame->len = dat;
                    frame->check_sum += frame->len;
                    parse_state = (frame->len == 0) ? kWaitSum : kWaitDat;
                    dat_index = 0;
                }
                break;
            }
            case kWaitDat:
            {
                frame->dat[dat_index] = dat;
                frame->check_sum += frame->dat[dat_index];
                dat_index++;
                if (dat_index == frame->len)
                {
                    dat_index = 0;
                    parse_state = kWaitSum;
                }
                break;
            }
            case kWaitSum:
            {
                if (dat == frame->check_sum)
                {
                    ucRet = ATK_MS901M_EOK;
                    break;
                }
                parse_state = kWaitHeadL;
                break;
            }
            default:
            {
                parse_state = kWaitHeadL;
                break;
            }
        }

        /* 帧匹配成功立即退出 while 循环，否则继续循环直到超时覆盖 ucRet */
        if (ucRet == ATK_MS901M_EOK)
            break;

        timeout_index++;
        if (timeout_index == 1000)
        {
            timeout_index = 0;
            timeout--;
        }
    }

    if (pfnSavedCallback != NULL)
    {
        vUartSetCallback(emDevNum, pfnSavedCallback);
    }
    return ucRet;
}

uint8_t atk_ms901m_read_reg_by_id(emUartDevNumTdf emDevNum, uint8_t id, uint8_t *dat, uint32_t timeout)
{
    uint8_t buf[7];
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    uint8_t dat_index;

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    vUartSendArray(emDevNum, buf, 6);
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, id, ATK_MS901M_FRAME_ID_TYPE_ACK, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return 0;
    }

    {
        uint8_t ucCopyLen = frame.len;
        if (ucCopyLen > ATK_MS901M_FRAME_DAT_MAX_SIZE) ucCopyLen = ATK_MS901M_FRAME_DAT_MAX_SIZE;
        for (dat_index = 0; dat_index < ucCopyLen; dat_index++)
        {
            dat[dat_index] = frame.dat[dat_index];
        }
        return ucCopyLen;
    }
}

uint8_t atk_ms901m_write_reg_by_id(emUartDevNumTdf emDevNum, uint8_t id, uint8_t len, uint8_t *dat)
{
    uint8_t buf[7];

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_WRITE_REG_ID(id);
    buf[3] = len;
    if (len == 1)
    {
        buf[4] = dat[0];
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        vUartSendArray(emDevNum, buf, 6);
    }
    else if (len == 2)
    {
        buf[4] = dat[0];
        buf[5] = dat[1];
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        vUartSendArray(emDevNum, buf, 7);
    }
    else
    {
        return ATK_MS901M_EINVAL;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief  ATK-MS901M初始化
 * @note   若使用流式模式，需在 atk_ms901m_init() 之后再调用 atk_ms901m_start_streaming()
 * @note   执行期间会临时禁用 UART 回调，完成后恢复。
 *         ATK-MS901M 持续上传数据，若有其他回调占用 ring buffer 会导致阻塞 API 超时。
 */
uint8_t atk_ms901m_init(emUartDevNumTdf emDevNum)
{
    uint8_t ret;
    const stUartDeviceParamTdf *pstDev;
    vUartFrameCallback pfnSavedCallback;

    /* 保存并临时禁用 UART 回调，防止 ISR 中回调消费 ring buffer 导致 ACK 丢失 */
    pstDev = c_pstGetUartDeviceParam(emDevNum);
    pfnSavedCallback = pstDev->stStaticParam.vCallbackFcn;
    vUartSetCallback(emDevNum, NULL);

    /* 清空回调期间累积的旧数据 */
    while (ucUartRxAvailable(emDevNum))
    {
        ucUartReceiveByte(emDevNum);
    }

    ret = atk_ms901m_read_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_GYROFSR, &g_stBlockingFsr.ucGyroFsr, 100);
    if (ret == 0)
    {
        vUartSetCallback(emDevNum, pfnSavedCallback);
        return ATK_MS901M_ERROR;
    }

    ret = atk_ms901m_read_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_ACCFSR, &g_stBlockingFsr.ucAccelFsr, 100);
    if (ret == 0)
    {
        vUartSetCallback(emDevNum, pfnSavedCallback);
        return ATK_MS901M_ERROR;
    }

    /* 恢复原回调 */
    vUartSetCallback(emDevNum, pfnSavedCallback);
    return ATK_MS901M_EOK;
}

/**
 * @brief  ATK-MS901M 默认初始化（低配版模块专用）
 * @note   适用于不支持寄存器读写指令的低配版模块。
 *         跳过 FSR 查询，使用最大量程作为安全默认值：
 *         陀螺仪 2000dps，加速度计 16g。
 * @note   提前注册流式回调，防止 init 后到 start_streaming 之间
 *         数据在 ring buffer 中堆积。此时 ucOnline=0，回调为空操作。
 */
uint8_t atk_ms901m_init_default(emUartDevNumTdf emDevNum)
{
    /* 使用最大量程作为安全默认值（避免数据溢出） */
    g_stBlockingFsr.ucGyroFsr  = 3;   /* 2000 dps */
    g_stBlockingFsr.ucAccelFsr = 3;   /* 16g */

    /* 提前注册回调，避免 vUartDevicePeriodExecute 因 vCallbackFcn==NULL
       走轮询分支导致 ring buffer 堆积 */
    vUartSetCallback(emDevNum, vAtkMs901mUartCallback);

    return ATK_MS901M_EOK;
}

/* ---- 阻塞读取 API ---- */

uint8_t atk_ms901m_get_attitude(emUartDevNumTdf emDevNum, atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (attitude_dat == NULL) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_ATTITUDE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    attitude_dat->roll  = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0f * 180.0f;
    attitude_dat->pitch = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0f * 180.0f;
    attitude_dat->yaw   = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0f * 180.0f;

    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_quaternion(emUartDevNumTdf emDevNum, atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (quaternion_dat == NULL) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_QUAT, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    quaternion_dat->q0 = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0f;
    quaternion_dat->q1 = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0f;
    quaternion_dat->q2 = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0f;
    quaternion_dat->q3 = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 32768.0f;

    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_gyro_accelerometer(emUartDevNumTdf emDevNum, atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if ((gyro_dat == NULL) && (accelerometer_dat == NULL)) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_GYRO_ACCE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    vAtkMs901mDecodeGyroAccel(frame.dat, g_stBlockingFsr.ucGyroFsr, g_stBlockingFsr.ucAccelFsr,
                               gyro_dat, accelerometer_dat);

    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_magnetometer(emUartDevNumTdf emDevNum, atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (magnetometer_dat == NULL) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_MAG, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    magnetometer_dat->x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
    magnetometer_dat->y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
    magnetometer_dat->z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];
    magnetometer_dat->temperature = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 100.0f;

    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_barometer(emUartDevNumTdf emDevNum, atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (barometer_dat == NULL) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_BARO, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    barometer_dat->pressure = (int32_t)(frame.dat[3] << 24) | ((int32_t)frame.dat[2] << 16) | ((int32_t)frame.dat[1] << 8) | frame.dat[0];
    barometer_dat->altitude = (int32_t)(frame.dat[7] << 24) | ((int32_t)frame.dat[6] << 16) | ((int32_t)frame.dat[5] << 8) | frame.dat[4];
    barometer_dat->temperature = (float)((int16_t)(frame.dat[9] << 8) | frame.dat[8]) / 100.0f;

    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_port(emUartDevNumTdf emDevNum, atk_ms901m_port_data_t *port_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (port_dat == NULL) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_PORT, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    port_dat->d0 = (uint16_t)(frame.dat[1] << 8) | frame.dat[0];
    port_dat->d1 = (uint16_t)(frame.dat[3] << 8) | frame.dat[2];
    port_dat->d2 = (uint16_t)(frame.dat[5] << 8) | frame.dat[4];
    port_dat->d3 = (uint16_t)(frame.dat[7] << 8) | frame.dat[6];

    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_led_state(emUartDevNumTdf emDevNum, atk_ms901m_led_state_t *state, uint32_t timeout)
{
    uint8_t ret;
    ret = atk_ms901m_read_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_LEDOFF, (uint8_t *)state, timeout);
    if (ret == 0) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_set_led_state(emUartDevNumTdf emDevNum, atk_ms901m_led_state_t state, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_led_state_t state_recv;

    ret = atk_ms901m_write_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_LEDOFF, 1, (uint8_t *)&state);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_led_state(emDevNum, &state_recv, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    if (state_recv != state) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_port_mode(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;

    switch (port)
    {
        case ATK_MS901M_PORT_D0: id = ATK_MS901M_FRAME_ID_REG_D0MODE; break;
        case ATK_MS901M_PORT_D1: id = ATK_MS901M_FRAME_ID_REG_D1MODE; break;
        case ATK_MS901M_PORT_D2: id = ATK_MS901M_FRAME_ID_REG_D2MODE; break;
        case ATK_MS901M_PORT_D3: id = ATK_MS901M_FRAME_ID_REG_D3MODE; break;
        default: return ATK_MS901M_ERROR;
    }

    ret = atk_ms901m_read_reg_by_id(emDevNum, id, (uint8_t *)mode, timeout);
    if (ret == 0) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_set_port_mode(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    atk_ms901m_port_mode_t mode_recv;

    switch (port)
    {
        case ATK_MS901M_PORT_D0:
            if (mode == ATK_MS901M_PORT_MODE_OUTPUT_PWM) { return ATK_MS901M_ERROR; }
            id = ATK_MS901M_FRAME_ID_REG_D0MODE; break;
        case ATK_MS901M_PORT_D1:
            id = ATK_MS901M_FRAME_ID_REG_D1MODE; break;
        case ATK_MS901M_PORT_D2:
            if (mode == ATK_MS901M_PORT_MODE_OUTPUT_PWM) { return ATK_MS901M_ERROR; }
            id = ATK_MS901M_FRAME_ID_REG_D2MODE; break;
        case ATK_MS901M_PORT_D3:
            id = ATK_MS901M_FRAME_ID_REG_D3MODE; break;
        default: return ATK_MS901M_ERROR;
    }

    ret = atk_ms901m_write_reg_by_id(emDevNum, id, 1, (uint8_t *)&mode);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_port_mode(emDevNum, port, &mode_recv, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    if (mode_recv != mode) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_port_pwm_pulse(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout)
{
    uint8_t id;

    switch (port)
    {
        case ATK_MS901M_PORT_D0:
        case ATK_MS901M_PORT_D2:
            return ATK_MS901M_ERROR;
        case ATK_MS901M_PORT_D1: id = ATK_MS901M_FRAME_ID_REG_D1PULSE; break;
        case ATK_MS901M_PORT_D3: id = ATK_MS901M_FRAME_ID_REG_D3PULSE; break;
        default: return ATK_MS901M_ERROR;
    }

    if (atk_ms901m_read_reg_by_id(emDevNum, id, (uint8_t *)pulse, timeout) == 0) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_set_port_pwm_pulse(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    uint16_t pulse_recv;

    switch (port)
    {
        case ATK_MS901M_PORT_D0:
        case ATK_MS901M_PORT_D2:
            return ATK_MS901M_ERROR;
        case ATK_MS901M_PORT_D1: id = ATK_MS901M_FRAME_ID_REG_D1PULSE; break;
        case ATK_MS901M_PORT_D3: id = ATK_MS901M_FRAME_ID_REG_D3PULSE; break;
        default: return ATK_MS901M_ERROR;
    }

    ret = atk_ms901m_write_reg_by_id(emDevNum, id, 2, (uint8_t *)&pulse);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_port_pwm_pulse(emDevNum, port, &pulse_recv, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    if (pulse_recv != pulse) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_get_port_pwm_period(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t *period, uint32_t timeout)
{
    uint8_t id;

    switch (port)
    {
        case ATK_MS901M_PORT_D0:
        case ATK_MS901M_PORT_D2:
            return ATK_MS901M_ERROR;
        case ATK_MS901M_PORT_D1: id = ATK_MS901M_FRAME_ID_REG_D1PERIOD; break;
        case ATK_MS901M_PORT_D3: id = ATK_MS901M_FRAME_ID_REG_D3PERIOD; break;
        default: return ATK_MS901M_ERROR;
    }

    if (atk_ms901m_read_reg_by_id(emDevNum, id, (uint8_t *)period, timeout) == 0) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

uint8_t atk_ms901m_set_port_pwm_period(emUartDevNumTdf emDevNum, atk_ms901m_port_t port, uint16_t period, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    uint16_t period_recv;

    switch (port)
    {
        case ATK_MS901M_PORT_D0:
        case ATK_MS901M_PORT_D2:
            return ATK_MS901M_ERROR;
        case ATK_MS901M_PORT_D1: id = ATK_MS901M_FRAME_ID_REG_D1PERIOD; break;
        case ATK_MS901M_PORT_D3: id = ATK_MS901M_FRAME_ID_REG_D3PERIOD; break;
        default: return ATK_MS901M_ERROR;
    }

    ret = atk_ms901m_write_reg_by_id(emDevNum, id, 2, (uint8_t *)&period);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    ret = atk_ms901m_get_port_pwm_period(emDevNum, port, &period_recv, timeout);
    if (ret != ATK_MS901M_EOK) { return ATK_MS901M_ERROR; }

    if (period_recv != period) { return ATK_MS901M_ERROR; }
    return ATK_MS901M_EOK;
}

/* ===================== 传感器基类适配 ===================== */

#if SENSOR_IS_ENABLE

#include "sensor_device.h"

#define GYRO_SENSOR_LOCAL_MAX  3

#define GYRO_SENSOR_TO_LOCAL(dev)  ((uint8_t)((dev) - emSensorAtkMs901MDevNum0))

typedef struct {
    stSensorDeviceTdf      stBase;
    emSensorDevNumTdf      emSensorDevNum;
    fix32_t                fCurrentYaw;
    fix32_t                fLastYaw;
    fix32_t                fAccumulatedYaw;
    fix32_t                fTargetYaw;
    int32_t                lTurnCount;
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

    if (atk_ms901m_read_attitude((emAtkMs901mDevNumTdf)GYRO_SENSOR_TO_LOCAL(pstGyro->emSensorDevNum), &stAttitude) != ATK_MS901M_EOK) {
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
    return pstGyro->fCurrentYaw;
}

static fix32_t fGyroSensorGetAccumulatedValue(void *pstSensor)
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
    fGyroSensorGetAccumulatedValue,
    vGyroSensorReset,
    vGyroSensorSetTarget,
    fGyroSensorGetTarget,
};

static stGyroSensorDeviceTdf g_astGyroSensorDevices[GYRO_SENSOR_LOCAL_MAX];

void vGyroSensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit)
{
    (void)pstInit;
    uint8_t ucLocalIdx = GYRO_SENSOR_TO_LOCAL(emSensorDevNum);
    if (ucLocalIdx >= GYRO_SENSOR_LOCAL_MAX) {
        return;
    }

    stGyroSensorDeviceTdf *pstGyro = &g_astGyroSensorDevices[ucLocalIdx];
    memset(pstGyro, 0, sizeof(stGyroSensorDeviceTdf));

    pstGyro->stBase.emType = emSensorTypeAtkMs901MGyro;
    pstGyro->stBase.pstVTable = &g_stGyroSensorVTable;
    pstGyro->stBase.ucEnable = 1;
    pstGyro->stBase.fWeight = FIX32_ONE;
    pstGyro->stBase.emPidDevNum     = emNoPid;
    pstGyro->stBase.usPidPeriodMs   = 0;
    pstGyro->stBase.ulPidLastTickMs = 0;
    pstGyro->emSensorDevNum = emSensorDevNum;

    vSensorRegisterDevice(emSensorDevNum, &pstGyro->stBase);
}

#endif /* SENSOR_IS_ENABLE */

#endif
