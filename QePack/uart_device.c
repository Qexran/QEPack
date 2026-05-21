/**
  * @file       uart_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      UART 驱动，基于 STM32 HAL 库
  *
  */

#include "uart_device.h"
#if UART_IS_ENABLE


stUartDeviceParamTdf astUartDeviceParam[UART_DEV_NUM];

/// @brief      获取 UART 设备参数
/// @param      emDevNum   ：设备号
/// @note       返回值是只读指针
const stUartDeviceParamTdf *c_pstGetUartDeviceParam(emUartDevNumTdf emDevNum)
{
    return &astUartDeviceParam[emDevNum];
}

/// @brief      初始化UART运行参数
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
void vUartDeviceRunningParamInit(stUartRunningParamTdf *pstInit, emUartDevNumTdf emDevNum)
{
    memcpy(&astUartDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stUartRunningParamTdf));
}



/// @brief      初始化UART静态参数
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
void vUartDeviceInit(stUartStaticParamTdf *pstInit, emUartDevNumTdf emDevNum)
{
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    stUartStaticParamTdf  *pstStatic = &astUartDeviceParam[emDevNum].stStaticParam;

    memcpy(pstStatic, pstInit, sizeof(stUartStaticParamTdf));

    if(pstStatic->emFrameEn == emUartFrameOff){
        pstStatic->pucFrameHead   = NULL;
        pstStatic->pucFrameTail   = NULL;
        pstStatic->ucFrameHeadLen = 0;
        pstStatic->ucFrameTailLen = 0;
    }

    // 默认初始化运行参数
    memset(pstRunning, 0, sizeof(stUartRunningParamTdf));
    pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;



    /* 启动串口接收中断 */
    #if UART_IS_USE_DMA
        //打开DMA接收（写入专用DMA缓冲区，与环形缓冲区隔离）
        HAL_UARTEx_ReceiveToIdle_DMA(
            pstInit->pstUartHandle,
            pstRunning->aucDmaRxBuf,
            UART_BUF_MAX_LEN
        );
    #else
        #if (QEPACK_PLATFORM == TI)
            /*
             * 清理 UART 初始化期间因 RX 引脚上已有数据流而产生的错误状态。
             * 场景：传感器上电即持续发送数据（如 ATK-MS901M），MCU 在
             * SYSCFG_DL_init() 中复位/配置 UART 期间，RX 引脚上已有数据，
             * UART 硬件在未完全配好时采样到不完整字节，导致帧错误/噪声错误
             * /溢出错误等，这些残留状态会阻塞后续正常接收。
             */
            {
                volatile uint32_t ulSafetyCnt;
                /* 1. 排空 RX FIFO 残留数据 */
                ulSafetyCnt = UART_BUF_MAX_LEN;
                while (ulSafetyCnt-- && !DL_UART_isRXFIFOEmpty(pstStatic->pstUartHandle->uart_inst)) {
                    DL_UART_Main_receiveData(pstStatic->pstUartHandle->uart_inst);
                }
                /* 2. 清除所有 UART 外设端挂起的中断标志（读取 IIDX 即自动清除） */
                ulSafetyCnt = 32;
                while (ulSafetyCnt--) {
                    if (DL_UART_Main_getPendingInterrupt(pstStatic->pstUartHandle->uart_inst)
                        == DL_UART_MAIN_IIDX_NO_INTERRUPT) {
                        break;
                    }
                }
            }
            NVIC_ClearPendingIRQ(pstStatic->pstUartHandle->int_irqn);
            NVIC_EnableIRQ(pstStatic->pstUartHandle->int_irqn);
        #else
            HAL_UART_Receive_IT(
                pstInit->pstUartHandle,
                &pstRunning->stUartTempBuffer.buffer[pstRunning->stUartTempBuffer.head],
                1
            );
        #endif
    #endif

    // 注册回调函数
    if(pstInit->vCallbackFcn){
        astUartDeviceParam[emDevNum].stStaticParam.vCallbackFcn = pstInit->vCallbackFcn;
    }
}

#if UART_IS_USE_DMA
/// @brief      静态辅助：发送队列入队（环形缓冲区，无阻塞，FIFO）
/// @param      emDevNum   ：设备号
/// @param      pucData    ：待存入的发送数据指针
/// @param      ulLen      ：待存入的数据长度
/// @return     0：入队成功，1：入队失败（队列满/参数非法）
static uint8_t ucUartTxQueuePush(emUartDevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen)
{
    // 1. 参数合法性检查
    if(pucData == NULL || ulLen == 0 || ulLen > UART_TX_QUEUE_MAX_LEN)
    {
        return 1;
    }

    uint32_t primask;
    primask = __get_PRIMASK();  // 保存当前中断状态
    __disable_irq();            // 禁用中断

    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    uint32_t ulFreeSpace = UART_TX_QUEUE_MAX_LEN - pstRunning->usTxQueueCount;

    // 2. 检查队列是否有足够空闲空间
    if(ulLen > ulFreeSpace)
    {
        __set_PRIMASK(primask);     // 恢复中断状态
        return 1;  // 队列满，入队失败
    }

    // 3. 环形缓冲区：逐字节存入数据（避免越界）
    for(uint32_t i = 0; i < ulLen; i++)
    {
        pstRunning->aucTxQueue[pstRunning->usTxQueueTail] = pucData[i];
        // 队列尾循环递增（达到最大值后回到0，实现环形）
        pstRunning->usTxQueueTail = (pstRunning->usTxQueueTail + 1) % UART_TX_QUEUE_MAX_LEN;
        pstRunning->usTxQueueCount++;  // 待发送数据计数递增
    }

    __set_PRIMASK(primask);     // 恢复中断状态
    return 0;  // 入队成功
}

/// @brief      静态辅助：启动下一批DMA发送（从队列中取数据，异步无阻塞）
/// @param      emDevNum   ：设备号
static void vUartStartNextTxDMA(emUartDevNumTdf emDevNum)
{
    stUartDeviceParamTdf *pstDev = &astUartDeviceParam[emDevNum];
    stUartRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    stUartStaticParamTdf *pstStatic = &pstDev->stStaticParam;

    // 1. 保护判断：发送忙 或 队列为空，直接返回
    if(pstRunning->ucTxBusy == 1 || pstRunning->usTxQueueCount == 0)
    {
        return;
    }

    // 2. 计算本次要发送的长度（不能超过最大限制）
    uint16_t usSendLen = pstRunning->usTxQueueCount;
    if(usSendLen > UART_TX_QUEUE_MAX_LEN - pstRunning->usTxQueueHead)
    {
        usSendLen = UART_TX_QUEUE_MAX_LEN - pstRunning->usTxQueueHead;
    }

    // 3. 记录本次发送的长度
    pstRunning->usTxCurrentDmaLen = usSendLen;

    // 4. 标记发送忙
    pstRunning->ucTxBusy = 1;

    // 5. 启动DMA发送
    HAL_UART_Transmit_DMA(
        pstStatic->pstUartHandle,
        &pstRunning->aucTxQueue[pstRunning->usTxQueueHead],
        usSendLen
    );
}

#endif


/// @brief      发送字节数组
/// @param      emDevNum   ：设备号
/// @param      pucData    ：待发送数组指针
/// @param      ulLen      ：待发送长度
void vUartSendArray(emUartDevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen)
{
    if((pucData == NULL) || (ulLen == 0))
    {
        return;
    }

    stUartDeviceParamTdf *pstDev = &astUartDeviceParam[emDevNum];

    #if UART_IS_USE_DMA
        if(ucUartTxQueuePush(emDevNum, pucData, ulLen) == 0)
        {
            // 入队成功，更新发送计数
            pstDev->stRunningParam.ulTxCount += ulLen;

            // 检查当前是否空闲，如果是则立即启动发送
            if(pstDev->stRunningParam.ucTxBusy == 0)
            {
                vUartStartNextTxDMA(emDevNum);
            }
        }
    #else
        #if (QEPACK_PLATFORM == TI)
            TI_UART_Transmit(
                pstDev->stStaticParam.pstUartHandle,
                pucData,
                ulLen,
                TI_MAX_DELAY
            );
        #else
            HAL_UART_Transmit(
                pstDev->stStaticParam.pstUartHandle,
                pucData,
                ulLen,
                HAL_MAX_DELAY
            );
        #endif

        pstDev->stRunningParam.ulTxCount += ulLen;
    #endif
}


/// @brief      发送单个字节
/// @param      emDevNum   ：设备号
/// @param      ucData     ：待发送字节
void vUartSendByte(emUartDevNumTdf emDevNum, uint8_t ucData)
{
    vUartSendArray(emDevNum, &ucData, 1);
}

/// @brief      接收单个字节
/// @param      emDevNum   ：设备号
/// @return     接收到的字节
/// @note       关中断保护 count-- 操作，防止与 ISR 中 count++ 产生 RMW 竞争
///             (Cortex-M0+ 无原子 RMW 指令)
uint8_t ucUartReceiveByte(emUartDevNumTdf emDevNum)
{
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;

    uint8_t data = 0;
    uint32_t ulPrimask = __get_PRIMASK();
    __disable_irq();
    if (pstRunning->stUartTempBuffer.count > 0) {
        data = pstRunning->stUartTempBuffer.buffer[pstRunning->stUartTempBuffer.tail];
        pstRunning->stUartTempBuffer.tail = (pstRunning->stUartTempBuffer.tail + 1) % UART_BUF_MAX_LEN;
        pstRunning->stUartTempBuffer.count--;
    }
    __set_PRIMASK(ulPrimask);
    return data;
}

/** 这个函数不常使用，暂不移植. */
#if (QEPACK_PLATFORM == ST)
/// @brief      接收字节数组
/// @param      emDevNum   ：设备号
/// @param      pucBuf     ：接收缓存指针
/// @param      ulMaxLen   ：最大接收长度
/// @return     实际接收长度
uint32_t ulUartReceiveArray(emUartDevNumTdf emDevNum, uint8_t *pucBuf, uint32_t ulMaxLen)
{
    uint32_t ulRecvLen = 0;

    if((pucBuf == NULL) || (ulMaxLen == 0))
    {
        return 0;
    }

    ulRecvLen = HAL_UART_Receive(astUartDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                                 pucBuf,
                                 ulMaxLen,
                                 HAL_MAX_DELAY);
    astUartDeviceParam[emDevNum].stRunningParam.ulRxCount += ulRecvLen;
    astUartDeviceParam[emDevNum].stRunningParam.ucRxComplete = 1;

    return ulRecvLen;
}
#endif

/// @brief      格式化发送
/// @param      emDevNum   ：设备号
/// @param      pcFormat   ：格式化字符串
/// @note       支持%d/%s/%c/%f等基础格式化符
void vUartPrintf(emUartDevNumTdf emDevNum, const char *pcFormat, ...)
{
    va_list stVaList;
    uint32_t ulLen = 0;
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;

    if(pcFormat == NULL)
    {
        return;
    }

    // 格式化字符串到发送缓存（预留1字节给终止符，避免溢出）
    va_start(stVaList, pcFormat);
    ulLen = vsnprintf(
        (char*)pstRunning->aucTxBuf,
        UART_TX_BUF_MAX_LEN - 1,
        pcFormat,
        stVaList
    );
    va_end(stVaList);

    // 发送格式化后的数据
    if(ulLen > 0 && ulLen < UART_TX_BUF_MAX_LEN)
    {
        vUartSendArray(emDevNum, pstRunning->aucTxBuf, ulLen);
    }
}

/// @brief      发送整型数字
/// @param      emDevNum   ：设备号
/// @param      lNum       ：待发送整数
/// @param      ucBase     ：进制(2/10/16)
void vUartSendInt(emUartDevNumTdf emDevNum, int32_t lNum, uint8_t ucBase)
{
    char cBuf[34] = {0};

    switch(ucBase)
    {
        case 16:
            sprintf(cBuf, "%X", (uint32_t)lNum);
            break;

        case 2:
        {
            uint32_t ulAbsNum = (lNum < 0) ? (uint32_t)(-lNum) : (uint32_t)lNum;
            int8_t i = 0;
            if(lNum < 0) cBuf[i++] = '-';
            for(int8_t bit = 31; bit >= 0; bit--)
            {
                cBuf[i++] = (ulAbsNum & (1UL << bit)) ? '1' : '0';
            }
            cBuf[i] = '\0';
            // 去除前导零（保留至少1位）
            char *pcSrc = (cBuf[0] == '-') ? &cBuf[1] : cBuf;
            while(pcSrc[0] == '0' && pcSrc[1] != '\0') pcSrc++;
            if(pcSrc != cBuf) memmove(cBuf, pcSrc, strlen(pcSrc) + 1);
            break;
        }

        default:
            sprintf(cBuf, "%d", lNum);
            break;
    }

    vUartPrintf(emDevNum, "%s", cBuf);
}

/// @brief      发送浮点型数字
/// @param      emDevNum   ：设备号
/// @param      fNum       ：待发送浮点数
/// @param      ucDecBit   ：保留小数位数
void vUartSendFloat(emUartDevNumTdf emDevNum, float fNum, uint8_t ucDecBit)
{
    char cFormat[16] = {0};
    sprintf(cFormat, "%%.%df", ucDecBit);
    vUartPrintf(emDevNum, cFormat, fNum);
}

/// @brief      发送带帧头帧尾的数据帧
/// @param      emDevNum   ：设备号
/// @param      pucData    ：帧数据指针
/// @param      ulLen      ：帧数据长度
/// @note       帧格式：帧头 + 数据 + 帧尾
void vUartSendFrame(emUartDevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen)
{
    stUartStaticParamTdf *pstStatic = &astUartDeviceParam[emDevNum].stStaticParam;

    if(pstStatic->emFrameEn == emUartFrameOff || pucData == NULL || ulLen == 0)
    {
        return;
    }

    // 1. 发送帧头
    if(pstStatic->pucFrameHead != NULL && pstStatic->ucFrameHeadLen > 0)
    {
        vUartSendArray(emDevNum, pstStatic->pucFrameHead, pstStatic->ucFrameHeadLen);
    }

    // 2. 发送数据
    vUartSendArray(emDevNum, pucData, ulLen);

    // 3. 发送帧尾
    if(pstStatic->pucFrameTail != NULL && pstStatic->ucFrameTailLen > 0)
    {
        vUartSendArray(emDevNum, pstStatic->pucFrameTail, pstStatic->ucFrameTailLen);
    }
}

/// @brief      解析接收帧（统一版本，DMA / 非DMA 共用）
/// @param      emDevNum   ：设备号
/// @param      ucData     ：当前接收字节
static void vUartParseFrame(emUartDevNumTdf emDevNum, uint8_t ucData)
{
    stUartDeviceParamTdf *pstDev = &astUartDeviceParam[emDevNum];
    stUartStaticParamTdf *pstStatic = &pstDev->stStaticParam;
    stUartRunningParamTdf *pstRunning = &pstDev->stRunningParam;

    switch(pstRunning->emFrameParseState)
    {
        case emUartFrameParseState_WaitHead:
        {
            if(ucData == pstStatic->pucFrameHead[pstRunning->s_ucHeadMatchCount])
            {
                pstRunning->s_ucHeadMatchCount++;
                if(pstRunning->s_ucHeadMatchCount >= pstStatic->ucFrameHeadLen)
                {
                    pstRunning->s_ucHeadMatchCount = 0;
                    pstRunning->emFrameParseState = emUartFrameParseState_RecvData;
                    pstRunning->ulFrameDataCount = 0;
                }
            }
            else
            {
                pstRunning->s_ucHeadMatchCount = 0;
                // 支持帧头重叠匹配（如帧头0xAA 0xAA，避免遗漏跨字节帧头）
                if(ucData == pstStatic->pucFrameHead[0])
                {
                    pstRunning->s_ucHeadMatchCount = 1;
                }
            }
            break;
        }

        case emUartFrameParseState_RecvData:
        {
            // 缓冲区溢出保护：重置解析状态
            if(pstRunning->ulFrameDataCount >= UART_FRAME_MAX_LEN)
            {
                pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                pstRunning->s_ucHeadMatchCount = 0;
                pstRunning->s_ucTailMatchCount = 0;
                pstRunning->ulFrameDataCount = 0;
                break;
            }

            if(ucData == pstStatic->pucFrameTail[pstRunning->s_ucTailMatchCount])
            {
                pstRunning->s_ucTailMatchCount++;
                if(pstRunning->s_ucTailMatchCount >= pstStatic->ucFrameTailLen)
                {
                    pstRunning->s_ucTailMatchCount = 0;
                    pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                    pstRunning->ucRxComplete = 1;
                    break;
                }
            }
            else
            {
                // 帧尾匹配失败，将已匹配的帧尾字节写入数据缓冲区
                if(pstRunning->s_ucTailMatchCount > 0)
                {
                    for(uint8_t i=0; i<pstRunning->s_ucTailMatchCount; i++)
                    {
                        if(pstRunning->ulFrameDataCount < UART_FRAME_MAX_LEN)
                        {
                            pstRunning->aucFrameDataBuf[pstRunning->ulFrameDataCount++] = pstStatic->pucFrameTail[i];
                        }
                    }
                    pstRunning->s_ucTailMatchCount = 0;
                }

                // 存储当前数据字节
                if(pstRunning->ulFrameDataCount < UART_FRAME_MAX_LEN)
                {
                    pstRunning->aucFrameDataBuf[pstRunning->ulFrameDataCount++] = ucData;
                }

                // 支持数据中嵌套帧头的重新匹配
                if(ucData == pstStatic->pucFrameHead[0])
                {
                    pstRunning->s_ucHeadMatchCount = 1;
                }
            }
            break;
        }

        default:
        {
            pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
            pstRunning->s_ucHeadMatchCount = 0;
            pstRunning->s_ucTailMatchCount = 0;
            pstRunning->ulFrameDataCount = 0;
            break;
        }
    }
}

/// @brief      检查完成标志并派发回调（内部调用，由 vUartDevicePeriodExecute 触发）
/// @param      emDevNum   ：设备号
static void vUartDispatchCallback(emUartDevNumTdf emDevNum)
{
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    stUartStaticParamTdf *pstStatic  = &astUartDeviceParam[emDevNum].stStaticParam;

    if(pstRunning->ucRxComplete == 1)
    {
        // 调用回调函数
        if (pstStatic->vCallbackFcn) {
            pstStatic->vCallbackFcn(emDevNum, pstRunning);
        }

        // 重置接收状态
        pstRunning->ucRxComplete = 0;
        pstRunning->ulFrameDataCount = 0;
    }

}

/// @brief      UART周期执行（处理接收解析）
/// @param      emDevNum   ：设备号
/// @note       建议1ms调用一次，放在1ms定时器ISR中
void vUartDevicePeriodExecute(emUartDevNumTdf emDevNum)
{
    stUartStaticParamTdf *pstStatic = &astUartDeviceParam[emDevNum].stStaticParam;
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;

    if(pstStatic->pstUartHandle == NULL) return;

    if(pstRunning->stUartTempBuffer.count > 0)
    {
        if(pstStatic->emFrameEn == emUartFrameOn)
        {
            // 帧模式：循环处理所有可用字节
            while(pstRunning->stUartTempBuffer.count > 0)
            {
                uint8_t ucData = ucUartReceiveByte(emDevNum);
                vUartParseFrame(emDevNum, ucData);
                // 帧完成立即回调并重置状态机，避免后续字节被误消费
                vUartDispatchCallback(emDevNum);
            }
        }
        else if(pstStatic->vCallbackFcn)
        {
            // 非帧+回调模式：批量拷贝到 aucFrameDataBuf，统一回调读取接口
            pstRunning->ulFrameDataCount = 0;
            while(pstRunning->stUartTempBuffer.count > 0 && pstRunning->ulFrameDataCount < UART_FRAME_MAX_LEN)
            {
                pstRunning->aucFrameDataBuf[pstRunning->ulFrameDataCount++] = ucUartReceiveByte(emDevNum);
            }
            if(pstRunning->ulFrameDataCount > 0)
            {
                pstRunning->ucRxComplete = 1;
                vUartDispatchCallback(emDevNum);
            }
        }
        // 非帧+轮询模式：数据留在环形缓冲区，用户通过 ucUartRxAvailable + ucUartReceiveByte 读取
    }
}

/// @brief      检查接收缓冲区是否有可用数据
/// @param      emDevNum ：设备号
/// @return     1: 有数据, 0: 无数据
uint8_t ucUartRxAvailable(emUartDevNumTdf emDevNum)
{
    const stUartDeviceParamTdf *pstDev = c_pstGetUartDeviceParam(emDevNum);
    return pstDev->stRunningParam.stUartTempBuffer.count > 0;
}

/// @brief      设置 UART 帧回调函数（运行时替换）
/// @param      emDevNum  ：设备号
/// @param      vCallback ：新的回调函数指针，传入 NULL 可清除
void vUartSetCallback(emUartDevNumTdf emDevNum, vUartFrameCallback vCallback)
{
    astUartDeviceParam[emDevNum].stStaticParam.vCallbackFcn = vCallback;
}

/// @brief 更新环形接收缓冲区（由 UART 接收中断调用）
/// @param emDevNum ：设备号
void vUartUpdateBuffer(emUartDevNumTdf emDevNum){
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    stUartStaticParamTdf *pstStatic = &astUartDeviceParam[emDevNum].stStaticParam;

#if (QEPACK_PLATFORM == TI)
    // 一次性排空 RX FIFO，避免在阻塞式 TX 期间 FIFO 溢出导致错误中断抢占 RX 中断
    while (!DL_UART_isRXFIFOEmpty(pstStatic->pstUartHandle->uart_inst)) {
        uint8_t ucData = DL_UART_Main_receiveData(pstStatic->pstUartHandle->uart_inst);

        if (pstRunning->stUartTempBuffer.count < UART_BUF_MAX_LEN) {
            pstRunning->stUartTempBuffer.buffer[pstRunning->stUartTempBuffer.head] = ucData;
            pstRunning->stUartTempBuffer.head = (pstRunning->stUartTempBuffer.head + 1) % UART_BUF_MAX_LEN;
            pstRunning->stUartTempBuffer.count++;
            pstRunning->ulRxCount++;
        }
        // 若环形缓冲区已满，数据已被读取并丢弃（但 FIFO 被清空，避免溢出）
    }
#else
    if (pstRunning->stUartTempBuffer.count < UART_BUF_MAX_LEN) {
        pstRunning->stUartTempBuffer.head = (pstRunning->stUartTempBuffer.head + 1) % UART_BUF_MAX_LEN;
        pstRunning->stUartTempBuffer.count++;
        pstRunning->ulRxCount++;
    }
    HAL_UART_Receive_IT(pstStatic->pstUartHandle, &pstRunning->stUartTempBuffer.buffer[pstRunning->stUartTempBuffer.head], 1);
#endif
}

#if (QEPACK_PLATFORM == TI)
    void vUartRxCallBackHandler(emUartDevNumTdf emDevNum){
        vUartUpdateBuffer(emDevNum);

    }
#else
    emUartDevNumTdf vUartRxCallBackHandler(UART_HandleTypeDef *huart){
        uint8_t i = emUartDevNum0;
        for(;i < UART_DEV_NUM; i++){
            if(huart == astUartDeviceParam[i].stStaticParam.pstUartHandle){
                vUartUpdateBuffer((emUartDevNumTdf)i);
                break;
            }
        }
        return (emUartDevNumTdf)i;
    }
#endif

#if UART_IS_USE_DMA
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = emUartDevNum0;
    for(; i < UART_DEV_NUM; i++)
    {
        if(huart == astUartDeviceParam[i].stStaticParam.pstUartHandle)
        {
            stUartRunningParamTdf *pstRunning = &astUartDeviceParam[i].stRunningParam;

            pstRunning->usTxQueueHead = (pstRunning->usTxQueueHead + pstRunning->usTxCurrentDmaLen) % UART_TX_QUEUE_MAX_LEN;
            pstRunning->usTxQueueCount -= pstRunning->usTxCurrentDmaLen;
            pstRunning->usTxCurrentDmaLen = 0;
            pstRunning->ucTxBusy = 0;

            vUartStartNextTxDMA((emUartDevNumTdf)i);

            break;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t i = emUartDevNum0;
    stUartDeviceParamTdf *pstDev = NULL;
    stUartRunningParamTdf *pstRunning = NULL;
    stUartStaticParamTdf *pstStatic = NULL;

    for(; i < UART_DEV_NUM; i++)
    {
        pstDev = &astUartDeviceParam[i];
        pstStatic = &pstDev->stStaticParam;
        if(huart == pstStatic->pstUartHandle)
        {
            pstRunning = &pstDev->stRunningParam;
            break;
        }
    }

    if(i >= UART_DEV_NUM || pstRunning == NULL || pstStatic == NULL)
    {
        return;
    }

    if(Size > 0 && Size <= UART_BUF_MAX_LEN)
    {
        if(pstStatic->emFrameEn == emUartFrameOn)
        {
            // 帧模式：从DMA缓冲区逐字节送入帧解析器
            for(uint16_t idx = 0; idx < Size; idx++)
            {
                vUartParseFrame((emUartDevNumTdf)i, pstRunning->aucDmaRxBuf[idx]);
                // 帧完成立即回调，避免DMA重启后新数据覆盖 aucFrameDataBuf
                vUartDispatchCallback((emUartDevNumTdf)i);
            }
        }
        else
        {
            // 非帧模式：从DMA缓冲区拷贝到 aucFrameDataBuf（统一读取接口）
            uint16_t usCopyLen = (Size < UART_FRAME_MAX_LEN) ? Size : UART_FRAME_MAX_LEN;
            memcpy(pstRunning->aucFrameDataBuf, pstRunning->aucDmaRxBuf, usCopyLen);
            pstRunning->ulFrameDataCount = usCopyLen;
            pstRunning->ulRxCount += Size;
            pstRunning->ucRxComplete = 1;
            vUartDispatchCallback((emUartDevNumTdf)i);
        }
    }

    // 回调处理完毕后再开启DMA接收，防止新数据覆盖正在读取的缓冲区
    HAL_UARTEx_ReceiveToIdle_DMA(
        pstStatic->pstUartHandle,
        pstRunning->aucDmaRxBuf,
        UART_BUF_MAX_LEN
    );
}

#else
    #if (QEPACK_PLATFORM == ST)
        void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
        {
            emUartDevNumTdf emDevNum = vUartRxCallBackHandler(huart);
        }
    #endif

#endif
#endif
