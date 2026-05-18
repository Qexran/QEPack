/**
  * @file       hc05_device.c
  * @brief      HC05蓝牙模块驱动
  */
#include "hc05_device.h"

#if HC05_IS_ENABLE

stHC05StaticParamTdf astHC05StaticParam[HC05_DEV_NUM];

/**
 * @brief  HC05设备初始化   
 */
void vHC05DeviceInit(stHC05StaticParamTdf *pstInit, emHC05DevNumTdf emDevNum)
{
    if (emDevNum >= HC05_DEV_NUM || pstInit == NULL) {
        return;
    }
    memcpy(&astHC05StaticParam[emDevNum], pstInit, sizeof(stHC05StaticParamTdf));
}

/**
 * @brief  发送AT命令
 */
void vHC05SendATCmd(emHC05DevNumTdf emDevNum, char *pcCmd)
{
    if (emDevNum >= HC05_DEV_NUM || pcCmd == NULL) {
        return;
    }
    vUartSendArray(astHC05StaticParam[emDevNum].emUartDevNum, (uint8_t *)pcCmd, strlen(pcCmd));
    vUartSendArray(astHC05StaticParam[emDevNum].emUartDevNum, (uint8_t *)"\r\n", 2);
}

/**
 * @brief  设置蓝牙名称
 */
void vHC05SetName(emHC05DevNumTdf emDevNum, char *pcName)
{
    char cCmd[64];
    if (emDevNum >= HC05_DEV_NUM || pcName == NULL) return;
    snprintf(cCmd, sizeof(cCmd), "AT+NAME=%s", pcName);
    vHC05SendATCmd(emDevNum, cCmd);
}

/**
 * @brief  设置配对密码
 */
void vHC05SetPin(emHC05DevNumTdf emDevNum, char *pcPin)
{
    char cCmd[64];
    if (emDevNum >= HC05_DEV_NUM || pcPin == NULL) return;
    snprintf(cCmd, sizeof(cCmd), "AT+PSWD=%s", pcPin);
    vHC05SendATCmd(emDevNum, cCmd);
}

/**
 * @brief  设置波特率
 */
void vHC05SetBaudRate(emHC05DevNumTdf emDevNum, unsigned long ulBaudRate)
{
    char cCmd[64];
    if (emDevNum >= HC05_DEV_NUM) return;
    snprintf(cCmd, sizeof(cCmd), "AT+UART=%lu,0,0", ulBaudRate);
    vHC05SendATCmd(emDevNum, cCmd);
}

/**
 * @brief  设置角色
 */
void vHC05SetRole(emHC05DevNumTdf emDevNum, emHC05RoleTdf emRole)
{
    char cCmd[64];
    if (emDevNum >= HC05_DEV_NUM) return;
    snprintf(cCmd, sizeof(cCmd), "AT+ROLE=%d", emRole);
    vHC05SendATCmd(emDevNum, cCmd);
}

/**
 * @brief  复位模块
 */
void vHC05Reset(emHC05DevNumTdf emDevNum)
{
    if (emDevNum >= HC05_DEV_NUM) return;
    vHC05SendATCmd(emDevNum, "AT+RESET");
}

/**
 * @brief  恢复默认设置
 */
void vHC05RestoreDefault(emHC05DevNumTdf emDevNum)
{
    if (emDevNum >= HC05_DEV_NUM) return;
    vHC05SendATCmd(emDevNum, "AT+ORGL");
}

/**
 * @brief  发送数据
 */
void vHC05Transmit(emHC05DevNumTdf emDevNum, uint8_t *pucData, uint16_t usLen)
{
    if (emDevNum >= HC05_DEV_NUM || pucData == NULL || usLen == 0) {
        return;
    }
    vUartSendArray(astHC05StaticParam[emDevNum].emUartDevNum, pucData, usLen);
}

/**
 * @brief  格式化发送数据
 */
void vHC05Printf(emHC05DevNumTdf emDevNum, const char *pcFormat, ...)
{
    if (emDevNum >= HC05_DEV_NUM || pcFormat == NULL) return;

    char cBuf[128];
    va_list args;
    va_start(args, pcFormat);
    vsnprintf(cBuf, sizeof(cBuf), pcFormat, args);
    va_end(args);

    vUartSendArray(astHC05StaticParam[emDevNum].emUartDevNum, (uint8_t *)cBuf, strlen(cBuf));
}

#endif
