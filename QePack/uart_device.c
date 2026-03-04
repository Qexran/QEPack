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

/// @brief          RingBuffer初始化
/// @param          pstRingBuf ：RingBuffer指针
void vRingBufferInit(RingBuffer *pstRingBuf)
{
    if (pstRingBuf == NULL) {
        return;
    }
    pstRingBuf->head = 0;
    pstRingBuf->tail = 0;
    pstRingBuf->count = 0;
}

/// @brief          RingBuffer写入数据
/// @param          pstRingBuf ：RingBuffer指针
/// @param          pucData    ：待写入数据指针
/// @param          usLen      ：待写入数据长度
/// @return         0：写入成功，1：缓冲区满
uint8_t ucRingBufferWrite(RingBuffer *pstRingBuf, uint8_t *pucData, uint16_t usLen)
{
    if (pstRingBuf == NULL || pucData == NULL || usLen == 0) {
        return 1;
    }
    
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();
    
    uint16_t usFreeSpace = UART_BUF_MAX_LEN - pstRingBuf->count;
    if (usLen > usFreeSpace) {
        __set_PRIMASK(primask);
        return 1;
    }
    
    uint16_t usWriteLen1 = UART_BUF_MAX_LEN - pstRingBuf->head;
    if (usWriteLen1 > usLen) {
        usWriteLen1 = usLen;
    }
    
    memcpy(&pstRingBuf->buffer[pstRingBuf->head], pucData, usWriteLen1);
    
    uint16_t usWriteLen2 = usLen - usWriteLen1;
    if (usWriteLen2 > 0) {
        memcpy(pstRingBuf->buffer, &pucData[usWriteLen1], usWriteLen2);
    }
    
    pstRingBuf->head = (pstRingBuf->head + usLen) % UART_BUF_MAX_LEN;
    pstRingBuf->count += usLen;
    
    __set_PRIMASK(primask);
    return 0;
}

/// @brief          RingBuffer读取数据
/// @param          pstRingBuf ：RingBuffer指针
/// @param          pucData    ：读取数据存放位置
/// @param          usLen      ：欲读取数据长度
/// @return         实际读取的数据长度
uint16_t usRingBufferRead(RingBuffer *pstRingBuf, uint8_t *pucData, uint16_t usLen)
{
    if (pstRingBuf == NULL || pucData == NULL || usLen == 0) {
        return 0;
    }
    
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();
    
    uint16_t usReadLen = pstRingBuf->count;
    if (usLen < usReadLen) {
        usReadLen = usLen;
    }
    
    uint16_t usReadLen1 = UART_BUF_MAX_LEN - pstRingBuf->tail;
    if (usReadLen1 > usReadLen) {
        usReadLen1 = usReadLen;
    }
    
    memcpy(pucData, &pstRingBuf->buffer[pstRingBuf->tail], usReadLen1);
    
    uint16_t usReadLen2 = usReadLen - usReadLen1;
    if (usReadLen2 > 0) {
        memcpy(&pucData[usReadLen1], pstRingBuf->buffer, usReadLen2);
    }
    
    pstRingBuf->tail = (pstRingBuf->tail + usReadLen) % UART_BUF_MAX_LEN;
    pstRingBuf->count -= usReadLen;
    
    __set_PRIMASK(primask);
    return usReadLen;
}

/// @brief          RingBuffer获取可用数据长度
/// @param          pstRingBuf ：RingBuffer指针
/// @return         可用数据长度
uint16_t usRingBufferGetCount(RingBuffer *pstRingBuf)
{
    if (pstRingBuf == NULL) {
        return 0;
    }
    return pstRingBuf->count;
}

/// @brief          RingBuffer清空
/// @param          pstRingBuf ：RingBuffer指针
void vRingBufferFlush(RingBuffer *pstRingBuf)
{
    if (pstRingBuf == NULL) {
        return;
    }
    
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();
    
    pstRingBuf->head = 0;
    pstRingBuf->tail = 0;
    pstRingBuf->count = 0;
    
    __set_PRIMASK(primask);
}

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
    
    // 默认初始化运行参数
    memset(pstRunning, 0, sizeof(stUartRunningParamTdf));
    pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;

	
	/* 初始化缓冲区 */
	vRingBufferInit(&pstRunning->stUartTempBuffer);
	
	
	/* 启动串口接收中断 */
	#if UART_IS_USE_DMA
		//打开DMA接收
		/*
			对于 HAL_UART_Receive_DMA()
				1.开启 IDLE 中断：
					IDLE 触发时进入USART1_IRQHandler()
					需手动清 IDLE 标志位 + 手动处理数据；
					收满字节时进入 HAL_UART_RxCpltCallback()。
				2.不开启 IDLE 中断：
					仅收满字节时进入 HAL_UART_RxCpltCallback()。
			对于 HAL_UARTEx_ReceiveToIdle_DMA()
				IDLE 触发或收满字节时，均自动进入 HAL_UARTEx_RxEventCallback()；
				无需手动清 IDLE 标志位、无需手动停止 DMA，HAL 库内部自动完成。
		*/
		HAL_UARTEx_ReceiveToIdle_DMA(
			pstInit->pstUartHandle,
			pstRunning->stUartTempBuffer.buffer, 
			UART_BUF_MAX_LEN
		);
		//使能IDLE中断（仅在使用HAL_UART_Receive_DMA接收才需要）
//		__HAL_UART_ENABLE_IT(
//			pstInit->pstUartHandle,
//			UART_IT_IDLE
//		);
	#else
		HAL_UART_Receive_IT(
			pstInit->pstUartHandle, 
			&pstRunning->stUartTempBuffer.buffer[pstRunning->stUartTempBuffer.head],
			1
		);
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
        __set_PRIMASK(primask);
        return 1;  // 队列满，入队失败
    }
    
    // 3. 环形缓冲区：使用memcpy批量拷贝数据
    uint16_t usWriteLen1 = UART_TX_QUEUE_MAX_LEN - pstRunning->usTxQueueTail;
    if (usWriteLen1 > ulLen) {
        usWriteLen1 = ulLen;
    }
    
    memcpy(&pstRunning->aucTxQueue[pstRunning->usTxQueueTail], pucData, usWriteLen1);
    
    uint16_t usWriteLen2 = ulLen - usWriteLen1;
    if (usWriteLen2 > 0) {
        memcpy(pstRunning->aucTxQueue, &pucData[usWriteLen1], usWriteLen2);
    }
    
    pstRunning->usTxQueueTail = (pstRunning->usTxQueueTail + ulLen) % UART_TX_QUEUE_MAX_LEN;
    pstRunning->usTxQueueCount += ulLen;
    
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


/// @brief      发送字节数组（修复后：入队缓存，异步发送）
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
            // 如果不是，等待当前发送完成后的回调中自动续发
            if(pstDev->stRunningParam.ucTxBusy == 0)
            {
                vUartStartNextTxDMA(emDevNum);
            }
        }
    #else
        // 非DMA模式，保留原有逻辑不变
        HAL_UART_Transmit(
            pstDev->stStaticParam.pstUartHandle, 
            pucData, 
            ulLen, 
            HAL_MAX_DELAY
        );
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
uint8_t ucUartReceiveByte(emUartDevNumTdf emDevNum)
{
	stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;

    uint8_t data = 0;
    usRingBufferRead(&pstRunning->stUartTempBuffer, &data, 1);
    return data;
}

/// @brief      接收字节数组
/// @param      emDevNum   ：设备号
/// @param      pucBuf     ：接收缓存指针
/// @param      ulMaxLen   ：最大接收长度
/// @return     实际接收长度
uint32_t ulUartReceiveArray(emUartDevNumTdf emDevNum, uint8_t *pucBuf, uint32_t ulMaxLen)
{
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    
    if((pucBuf == NULL) || (ulMaxLen == 0))
    {
        return 0;
    }
    
    uint32_t ulRecvLen = usRingBufferRead(&pstRunning->stUartTempBuffer, pucBuf, (uint16_t)ulMaxLen);
    pstRunning->ulRxCount += ulRecvLen;
    if (ulRecvLen > 0) {
        pstRunning->ucRxComplete = 1;
    }
    
    return ulRecvLen;
}

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
        UART_TX_BUF_MAX_LEN - 1,  // 优化：预留1字节，避免缓冲区溢出
        pcFormat, 
        stVaList
    );
    va_end(stVaList);
    
    // 发送格式化后的数据（复用vUartSendArray，间接入队）
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
    char cBuf[32] = {0};
    uint32_t ulAbsNum;
    int8_t i;
    uint8_t ucStartFlag = 0; // 跳过前置0标志位
    
    switch(ucBase)
    {
        case 10:
            // 10进制：直接用sprintf格式化
            sprintf(cBuf, "%d", lNum);
            break;
            
        case 16:
            // 16进制：格式化输出大写（和itoa默认风格一致）
            sprintf(cBuf, "%X", (uint32_t)lNum);
            break;
            
        case 2:
            // 二进制：sprintf无原生支持，手动转换
            if(lNum < 0)
            {
                ulAbsNum = (uint32_t)(-lNum);
                cBuf[0] = '-'; // 负数添加负号
                i = 1;
            }
            else
            {
                ulAbsNum = (uint32_t)lNum;
                i = 0;
            }
            
            // 从最高位到最低位逐位转换
            for(int8_t bit = 31; bit >= 0; bit--)
            {
                uint32_t ulMask = 1UL << bit;
                if((ulAbsNum & ulMask) != 0)
                {
                    cBuf[i++] = '1';
                    ucStartFlag = 1;
                }
                else if(ucStartFlag || (bit == 0))
                {
                    cBuf[i++] = '0';
                    ucStartFlag = 1;
                }
            }
            break;
            
        default:
            // 非法进制，默认按10进制处理
            sprintf(cBuf, "%d", lNum);
            break;
    }
    
    // 发送转换后的字符串
    vUartPrintf(emDevNum, "%s", cBuf);
}

/// @brief      发送浮点型数字
/// @param      emDevNum   ：设备号
/// @param      fNum       ：待发送浮点数
/// @param      ucDecBit   ：保留小数位数
void vUartSendFloat(emUartDevNumTdf emDevNum, float fNum, uint8_t ucDecBit)
{
    char cFormat[16] = {0};
    sprintf(cFormat, "%%.%df", ucDecBit); // 构造格式化符(如%.2f)
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
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    
    if(pstStatic->emFrameEn == 0 || pucData == NULL || ulLen == 0)
    {
        return;
    }
    
    // 计算总长度
    uint32_t ulTotalLen = 0;
    if(pstStatic->pucFrameHead != NULL && pstStatic->ucFrameHeadLen > 0)
    {
        ulTotalLen += pstStatic->ucFrameHeadLen;
    }
    ulTotalLen += ulLen;
    if(pstStatic->pucFrameTail != NULL && pstStatic->ucFrameTailLen > 0)
    {
        ulTotalLen += pstStatic->ucFrameTailLen;
    }
    
    // 检查缓冲区是否足够
    if(ulTotalLen >= UART_TX_BUF_MAX_LEN)
    {
        // 缓冲区不够，分段发送
        if(pstStatic->pucFrameHead != NULL && pstStatic->ucFrameHeadLen > 0)
        {
            vUartSendArray(emDevNum, pstStatic->pucFrameHead, pstStatic->ucFrameHeadLen);
        }
        vUartSendArray(emDevNum, pucData, ulLen);
        if(pstStatic->pucFrameTail != NULL && pstStatic->ucFrameTailLen > 0)
        {
            vUartSendArray(emDevNum, pstStatic->pucFrameTail, pstStatic->ucFrameTailLen);
        }
        return;
    }
    
    // 合并帧头、数据、帧尾到缓冲区，一次性发送
    uint32_t ulOffset = 0;
    
    if(pstStatic->pucFrameHead != NULL && pstStatic->ucFrameHeadLen > 0)
    {
        memcpy(&pstRunning->aucTxBuf[ulOffset], pstStatic->pucFrameHead, pstStatic->ucFrameHeadLen);
        ulOffset += pstStatic->ucFrameHeadLen;
    }
    
    memcpy(&pstRunning->aucTxBuf[ulOffset], pucData, ulLen);
    ulOffset += ulLen;
    
    if(pstStatic->pucFrameTail != NULL && pstStatic->ucFrameTailLen > 0)
    {
        memcpy(&pstRunning->aucTxBuf[ulOffset], pstStatic->pucFrameTail, pstStatic->ucFrameTailLen);
    }
    
    // 一次性发送
    vUartSendArray(emDevNum, pstRunning->aucTxBuf, ulTotalLen);
}

#if UART_IS_USE_DMA
/// @brief      DMA模式专用：批量解析接收帧（处理批量数据，适配DMA）
/// @param      emDevNum   ：设备号
/// @param      pucBatchBuf：DMA批量数据缓冲区指针
/// @param      usSize     ：DMA接收的有效数据长度
static void vUartParseFrame_DMA(emUartDevNumTdf emDevNum, uint8_t *pucBatchBuf, uint16_t usSize)
{
    if(pucBatchBuf == NULL || usSize == 0)
    {
        return;
    }
    
    stUartDeviceParamTdf *pstDev = &astUartDeviceParam[emDevNum];
    stUartStaticParamTdf *pstStatic = &pstDev->stStaticParam;
    stUartRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    
    // 1. 检查帧功能是否使能，帧头/帧尾是否有效
    if(pstStatic->emFrameEn == 0 || pstStatic->pucFrameHead == NULL || pstStatic->ucFrameHeadLen == 0)
    {
        return;
    }
    
    // 2. 批量遍历DMA接收的有效数据
    for(uint16_t idx = 0; idx < usSize; idx++)
    {
        uint8_t ucData = pucBatchBuf[idx]; // 当前遍历的字节
        
        switch(pstRunning->emFrameParseState)
        {
            case emUartFrameParseState_WaitHead:
            {
                // 逐字节匹配帧头（维持跨批次的匹配状态）
                if(ucData == pstStatic->pucFrameHead[pstRunning->s_ucHeadMatchCount])
                {
                    pstRunning->s_ucHeadMatchCount++;
                    // 帧头匹配完成，切换到接收数据状态
                    if(pstRunning->s_ucHeadMatchCount >= pstStatic->ucFrameHeadLen)
                    {
                        pstRunning->s_ucHeadMatchCount = 0;
                        pstRunning->emFrameParseState = emUartFrameParseState_RecvData;
                        pstRunning->ulFrameDataCount = 0; // 重置帧数据计数
                    }
                }
                else
                {
                    // 匹配失败，重置帧头匹配计数（支持跨批次重新匹配）
                    pstRunning->s_ucHeadMatchCount = 0;
                    // 优化：支持帧头重叠匹配（比如帧头0xAA0xBB，避免遗漏跨字节帧头）
                    if(ucData == pstStatic->pucFrameHead[0])
                    {
                        pstRunning->s_ucHeadMatchCount = 1;
                    }
                }
                break;
            }
            
            case emUartFrameParseState_RecvData:
            {
                // 先判断缓冲区是否溢出（避免越界）
                if(pstRunning->ulFrameDataCount >= UART_FRAME_MAX_LEN)
                {
                    // 缓冲区溢出，重置解析状态，避免数据覆盖
                    pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                    pstRunning->s_ucHeadMatchCount = 0;
                    pstRunning->s_ucTailMatchCount = 0;
                    pstRunning->ulFrameDataCount = 0;
                    memset(pstRunning->aucFrameDataBuf, 0, UART_FRAME_MAX_LEN);
                    break;
                }
                
                // 匹配帧尾（维持跨批次的匹配状态）
                if(ucData == pstStatic->pucFrameTail[pstRunning->s_ucTailMatchCount])
                {
                    pstRunning->s_ucTailMatchCount++;
                    // 帧尾匹配完成，解析结束
                    if(pstRunning->s_ucTailMatchCount >= pstStatic->ucFrameTailLen)
                    {
                        pstRunning->s_ucTailMatchCount = 0;
                        pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                        pstRunning->ucRxComplete = 1; // 标记帧接收完成
                        break;
                    }
                }
                else
                {
                    // 帧尾匹配失败，处理已匹配的帧尾字节（写入缓冲区）
                    if(pstRunning->s_ucTailMatchCount > 0)
                    {
                        for(uint8_t i=0; i<pstRunning->s_ucTailMatchCount; i++)
                        {
                            if(pstRunning->ulFrameDataCount < UART_FRAME_MAX_LEN)
                            {
                                pstRunning->aucFrameDataBuf[pstRunning->ulFrameDataCount++] = pstStatic->pucFrameTail[i];
                            }
                            else
                            {
                                // 溢出保护：直接重置解析状态
                                pstRunning->s_ucTailMatchCount = 0;
                                pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                                break;
                            }
                        }
                        pstRunning->s_ucTailMatchCount = 0;
                    }
                    
                    // 存储当前数据字节（核心：确保写入aucFrameDataBuf）
                    if(pstRunning->ulFrameDataCount < UART_FRAME_MAX_LEN)
                    {
                        pstRunning->aucFrameDataBuf[pstRunning->ulFrameDataCount++] = ucData;
                    }
                    else
                    {
                        // 溢出保护：重置解析状态
                        pstRunning->s_ucTailMatchCount = 0;
                        pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                        break;
                    }
                    
                    // 优化：支持数据中嵌套帧头的重新匹配（解决DMA批量数据的帧头重叠问题）
                    if(ucData == pstStatic->pucFrameHead[0])
                    {
                        pstRunning->s_ucHeadMatchCount = 1;
                    }
                }
                break;
            }
            
            default:
            {
                // 异常状态，重置所有解析参数
                pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                pstRunning->s_ucHeadMatchCount = 0;
                pstRunning->s_ucTailMatchCount = 0;
                pstRunning->ulFrameDataCount = 0;
                break;
            }
        }
    }
}

#endif

/// @brief      解析接收帧（内部调用）
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
            // 匹配帧头（逐字节比对）
            if(ucData == pstStatic->pucFrameHead[pstRunning->s_ucHeadMatchCount])
            {
                pstRunning->s_ucHeadMatchCount++;
                if(pstRunning->s_ucHeadMatchCount >= pstStatic->ucFrameHeadLen)
                {
                    pstRunning->s_ucHeadMatchCount = 0;
                    pstRunning->emFrameParseState = emUartFrameParseState_RecvData;
                    pstRunning->ulFrameDataCount = 0; // 重置帧数据计数
                }
            }
            else
            {
                pstRunning->s_ucHeadMatchCount = 0; // 匹配失败，重置计数
            }
            break;
        }
        
        case emUartFrameParseState_RecvData:
        {
            // 接收帧数据，直到匹配帧尾
            if(ucData == pstStatic->pucFrameTail[pstRunning->s_ucTailMatchCount])
            {
                pstRunning->s_ucTailMatchCount++;
                if(pstRunning->s_ucTailMatchCount >= pstStatic->ucFrameTailLen)
                {
                    pstRunning->s_ucTailMatchCount = 0;
                    pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
                    pstRunning->ucRxComplete = 1; // 帧接收完成
                    break;
                }
            }
            else
            {
				/*
					这里需要补充一下，当接收的过程中如果意外捕获到帧头，那么就重新开始接收第一个字节
				*/
                // 帧尾匹配失败，将当前字节计入数据，重置帧尾匹配计数
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
                
                // 存储数据字节
                if(pstRunning->ulFrameDataCount < UART_FRAME_MAX_LEN)
                {
                    pstRunning->aucFrameDataBuf[pstRunning->ulFrameDataCount++] = ucData;
                }
            }
            break;
        }
        
        default:
        {
            pstRunning->emFrameParseState = emUartFrameParseState_WaitHead;
			pstRunning->s_ucHeadMatchCount = 0;
            pstRunning->s_ucHeadMatchCount = 0;
            break;
        }
    }
}

/// @brief      接收带帧头帧尾的数据帧
/// @param      emDevNum   ：设备号
/// @param      pucFrameData ：帧数据缓存指针
/// @return     帧数据长度
void vUartReceiveFrame(emUartDevNumTdf emDevNum)
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
        memset(pstRunning->aucFrameDataBuf, 0, UART_FRAME_MAX_LEN);
    }
    
}

/// @brief      UART周期执行（处理接收解析）
/// @param      emDevNum   ：设备号
/// @note       建议1ms调用一次，或放在UART接收中断中
void vUartDevicePeriodExecute(emUartDevNumTdf emDevNum)
{
	vUartReceiveFrame(emDevNum);
	
    stUartStaticParamTdf *pstStatic = &astUartDeviceParam[emDevNum].stStaticParam;
    stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
    
    // 使用本地缓冲区批量读取，提高效率
    uint8_t ucLocalBuf[64];
    uint16_t usReadCount;
    
    // 批量读取FIFO中的数据
    while((usReadCount = usRingBufferRead(&pstRunning->stUartTempBuffer, ucLocalBuf, sizeof(ucLocalBuf))) > 0)
    {
        for(uint16_t i = 0; i < usReadCount; i++)
        {
            uint8_t ucData = ucLocalBuf[i];
            
            // 启用帧功能时解析帧，否则直接存入接收缓存
            if(pstStatic->emFrameEn == 1)
            {
                vUartParseFrame(emDevNum, ucData);
            }
            else
            {
                // 先判断缓存是否满，再存储，最后自增计数
                if(pstRunning->ulRxCount < UART_BUF_MAX_LEN)
                {
                    pstRunning->aucRxBuf[pstRunning->ulRxCount] = ucData; // 存入当前索引
                    pstRunning->ulRxCount++; // 计数后增
                    pstRunning->ucRxComplete = 1; // 标记有数据接收
                }
            }
        }
    }
}

/// @brief 检查 UART 接收缓冲区是否有可用数据
/// @param emDevNum ：设备号
/// @return 1: 有数据, 0: 无数据
uint8_t ucUartRxAvailable(emUartDevNumTdf emDevNum)
{
    const stUartDeviceParamTdf *pstDev = c_pstGetUartDeviceParam(emDevNum);
    return pstDev->stRunningParam.stUartTempBuffer.count > 0;
}

void vUartUpdateBuffer(emUartDevNumTdf emDevNum){
	stUartRunningParamTdf *pstRunning = &astUartDeviceParam[emDevNum].stRunningParam;
	stUartStaticParamTdf *pstStatic = &astUartDeviceParam[emDevNum].stStaticParam;
	
	// 更新环形缓冲区指针
	if (pstRunning->stUartTempBuffer.count < UART_BUF_MAX_LEN) {
		pstRunning->stUartTempBuffer.head = (pstRunning->stUartTempBuffer.head + 1) % UART_BUF_MAX_LEN;
		pstRunning->stUartTempBuffer.count++;
	}
        
    // 继续接收下一个字节
    HAL_UART_Receive_IT(pstStatic->pstUartHandle, &pstRunning->stUartTempBuffer.buffer[pstRunning->stUartTempBuffer.head], 1);
}

emUartDevNumTdf vUartRxCallBackHandler(UART_HandleTypeDef *huart){
	/* 查阅可用的UART句柄并更新 */
	uint8_t i = emUartDevNum0;
	for(;i < UART_DEV_NUM; i++){
		if(huart == astUartDeviceParam[i].stStaticParam.pstUartHandle){
			vUartUpdateBuffer((emUartDevNumTdf)i);
			/* 优化: 匹配到后立即返回 */
			break;
		}
	}
	return (emUartDevNumTdf)i;
}


#if UART_IS_USE_DMA
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = emUartDevNum0;
    for(; i < UART_DEV_NUM; i++)
    {
        if(huart == astUartDeviceParam[i].stStaticParam.pstUartHandle)
        {
            stUartRunningParamTdf *pstRunning = &astUartDeviceParam[i].stRunningParam;
            
            // 1. 更新队列头，移除已发送的数据
            pstRunning->usTxQueueHead = (pstRunning->usTxQueueHead + pstRunning->usTxCurrentDmaLen) % UART_TX_QUEUE_MAX_LEN;
            
            // 2. 更新队列计数，减去已发送的数据长度
            pstRunning->usTxQueueCount -= pstRunning->usTxCurrentDmaLen;
            
            // 3. 重置当前DMA发送长度
            pstRunning->usTxCurrentDmaLen = 0;
            
            // 4. 重置发送忙标记
            pstRunning->ucTxBusy = 0;
            
            // 5. 异步启动下一批DMA发送（若队列中有新数据，自动续发）
            vUartStartNextTxDMA((emUartDevNumTdf)i);
            
            break;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Size参数：返回本次实际接收的字节数（核心有效数据长度）
    uint8_t i = emUartDevNum0;
    stUartDeviceParamTdf *pstDev = NULL;
    stUartRunningParamTdf *pstRunning = NULL;
    stUartStaticParamTdf *pstStatic = NULL;

    // 匹配对应的UART设备
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

    // 未匹配到有效设备，直接返回
    if(i >= UART_DEV_NUM || pstRunning == NULL || pstStatic == NULL)
    {
        return;
    }

    // 核心步骤1：处理本次DMA接收的Size个有效数据（避免被后续数据覆盖）
    if(Size > 0 && Size <= UART_BUF_MAX_LEN)
    {
        // 使用RingBuffer写入数据
        ucRingBufferWrite(&pstRunning->stUartTempBuffer, pstRunning->stUartTempBuffer.buffer, Size);
    }

    // 核心步骤2：重新开启DMA接收，等待下一帧数据（必须保留，保证连续接收）
    HAL_UARTEx_ReceiveToIdle_DMA(
        pstStatic->pstUartHandle,
        pstRunning->stUartTempBuffer.buffer,
        UART_BUF_MAX_LEN
    );
}

#else

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    emUartDevNumTdf emDevNum = vUartRxCallBackHandler(huart);
}

#endif
#endif
