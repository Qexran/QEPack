/**
  * @file       hmi_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/06/02
  * @brief      HMI 串口屏模块实现（陶晶驰 TJC 协议）
  *
  * 基于 uart_device 的非帧模式 + 轮询模式，自行实现 TJC 响应解析状态机。
  */

#include "hmi_device.h"

#if HMI_IS_ENABLE

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ==================== 内部状态机枚举 ==================== */

/* HMI 响应解析状态 */
typedef enum {
    emHmiParseState_WaitType   = 0,    /* 等待类型字节 */
    emHmiParseState_WaitData   = 1,    /* 等待数据字节 */
    emHmiParseState_WaitStrLen = 2,    /* 等待字符串长度字节（0x70 专用） */
    emHmiParseState_WaitStr    = 3,    /* 等待字符串数据（0x70 专用） */
    emHmiParseState_WaitFooter = 4,    /* 等待 0xFF×3 终止符 */
} emHmiParseStateTdf;

/* ==================== 全局变量 ==================== */

static stHmiDeviceParamTdf g_astHmiDeviceParam[HMI_DEV_NUM];
static char g_acHmiCmdBuf[HMI_CMD_BUF_MAX_LEN];

/* ==================== 内部函数 ==================== */

/**
 * @brief  解析一个接收到的字节（状态机）
 */
static void vHmiParseByte(emHmiDevNumTdf emDevNum, uint8_t ucByte)
{
    stHmiDeviceParamTdf *pstDev = &g_astHmiDeviceParam[emDevNum];
    stHmiRunningParamTdf *pstRun = &pstDev->stRunningParam;

    static emHmiParseStateTdf s_emParseState[HMI_DEV_NUM] = {0};
    static uint8_t s_aucDataBuf[HMI_DEV_NUM][HMI_RX_BUF_MAX_LEN];
    static uint8_t s_ucDataIdx[HMI_DEV_NUM] = {0};
    static uint8_t s_ucRcvCount[HMI_DEV_NUM] = {0};   /* 实际接收计数（用于判断是否收够） */
    static uint8_t s_ucExpectedLen[HMI_DEV_NUM] = {0};
    static uint8_t s_ucFooterCount[HMI_DEV_NUM] = {0};
    static uint8_t s_ucStrLen[HMI_DEV_NUM] = {0};
    static emHmiRespTypeTdf s_emRespType[HMI_DEV_NUM] = {0};

    switch (s_emParseState[emDevNum]) {

    case emHmiParseState_WaitType:
        if (ucByte == 0x65 || ucByte == 0x66 || ucByte == 0x70 ||
            ucByte == 0x71 || ucByte == 0x88 || ucByte == 0x24) {
            s_emRespType[emDevNum] = (emHmiRespTypeTdf)ucByte;
            s_ucDataIdx[emDevNum] = 0;
            s_ucRcvCount[emDevNum] = 0;
            s_ucFooterCount[emDevNum] = 0;

            switch (ucByte) {
            case 0x65: /* 触摸事件: 3字节数据 */
                s_ucExpectedLen[emDevNum] = 3;
                s_emParseState[emDevNum] = emHmiParseState_WaitData;
                break;
            case 0x66: /* 页面ID: 1字节数据 */
                s_ucExpectedLen[emDevNum] = 1;
                s_emParseState[emDevNum] = emHmiParseState_WaitData;
                break;
            case 0x70: /* 字符串: 先读1字节长度 */
                s_emParseState[emDevNum] = emHmiParseState_WaitStrLen;
                break;
            case 0x71: /* 数值: 4字节数据 */
                s_ucExpectedLen[emDevNum] = 4;
                s_emParseState[emDevNum] = emHmiParseState_WaitData;
                break;
            case 0x88: /* 启动成功: 无数据 */
            case 0x24: /* 缓冲区溢出: 无数据 */
                s_ucExpectedLen[emDevNum] = 0;
                s_emParseState[emDevNum] = emHmiParseState_WaitFooter;
                break;
            default:
                break;
            }
        }
        /* 非法字节：保持 WaitType 状态，丢弃 */
        break;

    case emHmiParseState_WaitData:
        if (s_ucDataIdx[emDevNum] < HMI_RX_BUF_MAX_LEN) {
            s_aucDataBuf[emDevNum][s_ucDataIdx[emDevNum]++] = ucByte;
        }
        s_ucRcvCount[emDevNum]++;
        if (s_ucRcvCount[emDevNum] >= s_ucExpectedLen[emDevNum]) {
            s_emParseState[emDevNum] = emHmiParseState_WaitFooter;
            s_ucFooterCount[emDevNum] = 0;
        }
        break;

    case emHmiParseState_WaitStrLen:
        s_ucStrLen[emDevNum] = ucByte;
        s_ucExpectedLen[emDevNum] = ucByte;
        s_ucDataIdx[emDevNum] = 0;
        s_ucRcvCount[emDevNum] = 0;
        if (ucByte == 0) {
            /* 空字符串，直接等 footer */
            s_emParseState[emDevNum] = emHmiParseState_WaitFooter;
            s_ucFooterCount[emDevNum] = 0;
        } else {
            s_emParseState[emDevNum] = emHmiParseState_WaitStr;
        }
        break;

    case emHmiParseState_WaitStr:
        if (s_ucDataIdx[emDevNum] < HMI_RX_BUF_MAX_LEN) {
            s_aucDataBuf[emDevNum][s_ucDataIdx[emDevNum]++] = ucByte;
        }
        s_ucRcvCount[emDevNum]++;
        if (s_ucRcvCount[emDevNum] >= s_ucExpectedLen[emDevNum]) {
            s_emParseState[emDevNum] = emHmiParseState_WaitFooter;
            s_ucFooterCount[emDevNum] = 0;
        }
        break;

    case emHmiParseState_WaitFooter:
        if (ucByte == 0xFF) {
            s_ucFooterCount[emDevNum]++;
            if (s_ucFooterCount[emDevNum] >= 3) {
                /* 收到完整响应，组装结果并回调 */
                stHmiResponseTdf stResp;
                stResp.emType = s_emRespType[emDevNum];

                switch (s_emRespType[emDevNum]) {
                case emHmiRespTouch:
                    stResp.unData.stTouch.ucPageId      = s_aucDataBuf[emDevNum][0];
                    stResp.unData.stTouch.ucComponentId = s_aucDataBuf[emDevNum][1];
                    stResp.unData.stTouch.ucEvent       = s_aucDataBuf[emDevNum][2];
                    break;
                case emHmiRespPageId:
                    stResp.unData.ucPageId = s_aucDataBuf[emDevNum][0];
                    pstRun->ucCurrentPageId = s_aucDataBuf[emDevNum][0];
                    break;
                case emHmiRespString: {
                    /* 截断到缓冲区大小，防止溢出 */
                    uint8_t ucCopyLen = s_ucStrLen[emDevNum];
                    if (ucCopyLen > HMI_RX_BUF_MAX_LEN - 1) {
                        ucCopyLen = HMI_RX_BUF_MAX_LEN - 1;
                    }
                    stResp.unData.stString.ucComponentId = 0; /* 无组件ID字段 */
                    stResp.unData.stString.ucLength = ucCopyLen;
                    memcpy(stResp.unData.stString.acStr, s_aucDataBuf[emDevNum], ucCopyLen);
                    stResp.unData.stString.acStr[ucCopyLen] = '\0';
                    break;
                }
                case emHmiRespNumber:
                    stResp.unData.stNumber.ucComponentId = 0; /* 无组件ID字段 */
                    memcpy(&stResp.unData.stNumber.lValue, s_aucDataBuf[emDevNum], 4);
                    break;
                case emHmiRespStartup:
                    pstRun->ucReady = 1;
                    break;
                case emHmiRespOverflow:
                    break;
                default:
                    break;
                }

                pstRun->ulRespCount++;

                /* 调用用户回调 */
                if (pstDev->stStaticParam.vCallback != NULL) {
                    pstDev->stStaticParam.vCallback(emDevNum, &stResp);
                }

                /* 重置状态机 */
                s_emParseState[emDevNum] = emHmiParseState_WaitType;
            }
        } else {
            /* 非 0xFF 字节：终止符匹配失败，重置状态机 */
            s_emParseState[emDevNum] = emHmiParseState_WaitType;
        }
        break;
    }
}

/**
 * @brief  发送格式化命令（内部实现，不追加终止符）
 */
static void vHmiSendCmdRaw(emHmiDevNumTdf emDevNum, const char *pcCmd)
{
    emUartDevNumTdf emUart = g_astHmiDeviceParam[emDevNum].stStaticParam.emUartDevNum;
    vUartPrintf(emUart, "%s", pcCmd);
}

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief  初始化 HMI 设备
 */
void vHmiDeviceInit(const stHmiStaticParamTdf *pstInit, emHmiDevNumTdf emDevNum)
{
    if (emDevNum >= HMI_DEV_NUM || pstInit == NULL) return;

    memset(&g_astHmiDeviceParam[emDevNum], 0, sizeof(stHmiDeviceParamTdf));
    g_astHmiDeviceParam[emDevNum].stStaticParam = *pstInit;
}

/**
 * @brief  周期执行函数（处理接收数据）— 在主循环中调用
 */
void vHmiDevicePeriodExecute(emHmiDevNumTdf emDevNum)
{
    if (emDevNum >= HMI_DEV_NUM) return;

    emUartDevNumTdf emUart = g_astHmiDeviceParam[emDevNum].stStaticParam.emUartDevNum;

    while (ucUartRxAvailable(emUart)) {
        uint8_t ucByte = ucUartReceiveByte(emUart);
        vHmiParseByte(emDevNum, ucByte);
    }
}

/**
 * @brief  发送原始命令（自动追加 0xFF×3 终止符）
 */
void vHmiSendCmd(emHmiDevNumTdf emDevNum, const char *pcCmd)
{
    if (emDevNum >= HMI_DEV_NUM || pcCmd == NULL) return;

    vHmiSendCmdRaw(emDevNum, pcCmd);
    vHmiSendEndBytes(emDevNum);
}

/**
 * @brief  发送格式化命令（printf 风格，自动追加 0xFF×3）
 */
void vHmiSendCmdf(emHmiDevNumTdf emDevNum, const char *pcFormat, ...)
{
    if (emDevNum >= HMI_DEV_NUM || pcFormat == NULL) return;

    va_list args;
    va_start(args, pcFormat);
    vsnprintf(g_acHmiCmdBuf, HMI_CMD_BUF_MAX_LEN, pcFormat, args);
    va_end(args);

    vHmiSendCmdRaw(emDevNum, g_acHmiCmdBuf);
    vHmiSendEndBytes(emDevNum);
}

/**
 * @brief  发送 0xFF 终止符（3字节）
 */
void vHmiSendEndBytes(emHmiDevNumTdf emDevNum)
{
    if (emDevNum >= HMI_DEV_NUM) return;

    emUartDevNumTdf emUart = g_astHmiDeviceParam[emDevNum].stStaticParam.emUartDevNum;
    vUartSendByte(emUart, 0xFF);
    vUartSendByte(emUart, 0xFF);
    vUartSendByte(emUart, 0xFF);
}

/* ==================== 高层 UI 操作 API ==================== */

/**
 * @brief  切换页面
 */
void vHmiSetPage(emHmiDevNumTdf emDevNum, const char *pcPageName)
{
    vHmiSendCmdf(emDevNum, "page %s", pcPageName);
}

/**
 * @brief  设置文本值
 */
void vHmiSetText(emHmiDevNumTdf emDevNum, const char *pcComponent, const char *pcText)
{
    vHmiSendCmdf(emDevNum, "%s.txt=\"%s\"", pcComponent, pcText);
}

/**
 * @brief  设置数值
 */
void vHmiSetNumber(emHmiDevNumTdf emDevNum, const char *pcComponent, int32_t lValue)
{
    vHmiSendCmdf(emDevNum, "%s.val=%ld", pcComponent, lValue);
}

/**
 * @brief  设置可见性
 */
void vHmiSetVisible(emHmiDevNumTdf emDevNum, const char *pcComponent, uint8_t ucVisible)
{
    vHmiSendCmdf(emDevNum, "%s.vis=%d", pcComponent, ucVisible);
}

/**
 * @brief  设置图片
 */
void vHmiSetPicture(emHmiDevNumTdf emDevNum, const char *pcComponent, uint16_t usImageId)
{
    vHmiSendCmdf(emDevNum, "%s.pic=%d", pcComponent, usImageId);
}

/**
 * @brief  请求当前页面 ID
 */
void vHmiGetPageId(emHmiDevNumTdf emDevNum)
{
    vHmiSendCmd(emDevNum, "sendme");
}

/**
 * @brief  设置触摸事件回调模式 (bkcmd)
 * @param  ucMode: 0=不返回, 1=仅成功, 2=全部返回
 */
void vHmiSetBkCmd(emHmiDevNumTdf emDevNum, uint8_t ucMode)
{
    vHmiSendCmdf(emDevNum, "bkcmd=%d", ucMode);
}

/**
 * @brief  获取屏幕就绪状态
 */
uint8_t ucHmiIsReady(emHmiDevNumTdf emDevNum)
{
    if (emDevNum >= HMI_DEV_NUM) return 0;
    return g_astHmiDeviceParam[emDevNum].stRunningParam.ucReady;
}

/**
 * @brief  获取当前页面 ID
 */
uint8_t ucHmiGetCurrentPageId(emHmiDevNumTdf emDevNum)
{
    if (emDevNum >= HMI_DEV_NUM) return 0;
    return g_astHmiDeviceParam[emDevNum].stRunningParam.ucCurrentPageId;
}

#endif /* HMI_IS_ENABLE */
