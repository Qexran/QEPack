/**
  * @file       uart_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/22
  * @brief      UART 驱动，基于 STM32 HAL 库
  *
  */
#include "project_config.h"
#if UART_IS_ENABLE

#ifndef _UART_DEVICE_H_
#define _UART_DEVICE_H_

#include "usart.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

/// @brief          UART设备号枚举
typedef enum
{
    emUartDevNum0        = 0,              	// UART0
	emUartDevNum1,              			// UART1
	emUartDevNum2,              			// UART2
    emUartDevNum3,			              	// UART3
	emUartDevNum4,              			// UART4
	emUartDevNum5,              			// UART5
} emUartDevNumTdf;


/// @brief          串口接收缓冲区结构
typedef struct {
	__attribute__((aligned(4)))
    uint8_t buffer[UART_BUF_MAX_LEN];
    volatile uint16_t head; 
    volatile uint16_t tail;
    volatile uint16_t count;
} RingBuffer;

/// @brief          RingBuffer初始化
/// @param          pstRingBuf ：RingBuffer指针
void vRingBufferInit(RingBuffer *pstRingBuf);

/// @brief          RingBuffer写入数据
/// @param          pstRingBuf ：RingBuffer指针
/// @param          pucData    ：待写入数据指针
/// @param          usLen      ：待写入数据长度
/// @return         0：写入成功，1：缓冲区满
uint8_t ucRingBufferWrite(RingBuffer *pstRingBuf, uint8_t *pucData, uint16_t usLen);

/// @brief          RingBuffer读取数据
/// @param          pstRingBuf ：RingBuffer指针
/// @param          pucData    ：读取数据存放位置
/// @param          usLen      ：欲读取数据长度
/// @return         实际读取的数据长度
uint16_t usRingBufferRead(RingBuffer *pstRingBuf, uint8_t *pucData, uint16_t usLen);

/// @brief          RingBuffer获取可用数据长度
/// @param          pstRingBuf ：RingBuffer指针
/// @return         可用数据长度
uint16_t usRingBufferGetCount(RingBuffer *pstRingBuf);

/// @brief          RingBuffer清空
/// @param          pstRingBuf ：RingBuffer指针
void vRingBufferFlush(RingBuffer *pstRingBuf);

/// @brief          UART帧使能枚举
typedef enum
{
    emUartFrameOff		 = 0,  				// 关闭
	emUartFrameOn,  						// 开启
} emUartFrameEnTdf;


/// @brief          UART帧解析状态枚举
typedef enum
{
    emUartFrameParseState_WaitHead    = 0,  // 等待帧头
    emUartFrameParseState_RecvData    = 1,  // 接收帧数据
    emUartFrameParseState_WaitTail    = 2,  // 等待帧尾
} emUartFrameParseStateTdf;

/// @brief          UART运行参数定义
/// @note           运行时动态变化的参数
typedef struct
{
    uint8_t               			aucRxBuf[UART_BUF_MAX_LEN];    			// 接收缓存
    uint8_t               			aucTxBuf[UART_TX_BUF_MAX_LEN];    		// 发送缓存
    uint32_t              			ulRxCount;                     			// 接收字节计数
    uint32_t              			ulTxCount;                     			// 发送字节计数
    uint8_t               			ucRxComplete;                  			// 接收完成标志(0:未完成 1:完成)
    emUartFrameParseStateTdf 		emFrameParseState;          			// 帧解析状态
    uint8_t               			aucFrameDataBuf[UART_FRAME_MAX_LEN];  	// 帧数据缓存
    uint32_t              			ulFrameDataCount;              			// 帧数据计数
	RingBuffer 						stUartTempBuffer;						// 临时缓冲区
	uint8_t 						s_ucHeadMatchCount;						// 帧头匹配计数器
	uint8_t 						s_ucTailMatchCount;						// 帧尾匹配计数器
	
	__attribute__((aligned(4))) 
	uint8_t  aucTxQueue[UART_TX_QUEUE_MAX_LEN];  							// 环形发送队列缓冲区（缓存待发送数据）
    uint16_t usTxQueueHead;                      							// 队列头（取数据的索引，发送完成后更新）
    uint16_t usTxQueueTail;                      							// 队列尾（存数据的索引，入队时更新）
    uint16_t usTxQueueCount;                     							// 队列中待发送的数据长度（避免频繁计算头尾差）
    uint8_t  ucTxBusy;                           							// 发送忙标记（0：空闲，1：忙，避免重复启动DMA）
	uint16_t usTxCurrentDmaLen;      										// 当前DMA发送的长度
} stUartRunningParamTdf;

/// @brief          串口接收帧回调函数实现
typedef void (*vUartFrameCallback)(emUartDevNumTdf emDevNum, stUartRunningParamTdf* pstRunningParamTdf);

/// @brief          UART静态参数定义
/// @note           不可运行时修改的参数（硬件相关、帧格式基础配置）
typedef struct
{
    UART_HandleTypeDef   *pstUartHandle;    // UART句柄指针
    uint8_t              *pucFrameHead;     // 帧头数组指针
    uint8_t               ucFrameHeadLen;   // 帧头长度
    uint8_t              *pucFrameTail;     // 帧尾数组指针
    uint8_t               ucFrameTailLen;   // 帧尾长度
    emUartFrameEnTdf      emFrameEn;        // 帧功能使能(0:关闭 1:开启)
    uint32_t              ulBaudRate;       // 波特率（备份）
	vUartFrameCallback 	  vCallbackFcn;		// 回调函数
} stUartStaticParamTdf;


/// @brief          UART设备参数定义
typedef struct
{
    stUartStaticParamTdf     stStaticParam;  // 静态参数
    stUartRunningParamTdf    stRunningParam; // 运行参数
} stUartDeviceParamTdf;


/* 获取UART设备参数 */
const stUartDeviceParamTdf *c_pstGetUartDeviceParam(emUartDevNumTdf emDevNum);

/* 初始化相关 */
void vUartDeviceInit(stUartStaticParamTdf *pstInit, emUartDevNumTdf emDevNum);
void vUartDeviceRunningParamInit(stUartRunningParamTdf *pstInit, emUartDevNumTdf emDevNum);

/* 基础收发接口 */
void vUartSendByte(emUartDevNumTdf emDevNum, uint8_t ucData);
void vUartSendArray(emUartDevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen);
uint8_t ucUartReceiveByte(emUartDevNumTdf emDevNum);
uint32_t ulUartReceiveArray(emUartDevNumTdf emDevNum, uint8_t *pucBuf, uint32_t ulMaxLen);

/* 格式化/数字发送接口 */
void vUartPrintf(emUartDevNumTdf emDevNum, const char *pcFormat, ...);
void vUartSendInt(emUartDevNumTdf emDevNum, int32_t lNum, uint8_t ucBase);
void vUartSendFloat(emUartDevNumTdf emDevNum, float fNum, uint8_t ucDecBit);

/* 帧收发接口 */
void vUartSendFrame(emUartDevNumTdf emDevNum, uint8_t *pucData, uint32_t ulLen);
void vUartReceiveFrame(emUartDevNumTdf emDevNum);

void vUartRegisterCallback(vUartFrameCallback vCallbackFcn);

uint8_t ucUartRxAvailable(emUartDevNumTdf emDevNum);

/* 周期执行（建议放在主循环/定时器中断） */
void vUartDevicePeriodExecute(emUartDevNumTdf emDevNum);

#endif
#endif

/*
// 1. 定义帧头帧尾
uint8_t ucFrameHead[] = {0xAA, 0x55};
uint8_t ucFrameTail[] = {0x0D, 0x0A};

// 2. 初始化UART1静态参数
stUartStaticParamTdf stUart1Static = {
    .pstUartHandle = &huart1,          // HAL初始化的UART句柄
    .pucFrameHead = ucFrameHead,
    .ucFrameHeadLen = sizeof(ucFrameHead),
    .pucFrameTail = ucFrameTail,
    .ucFrameTailLen = sizeof(ucFrameTail),
    .ucFrameEn = 1,                    // 启用帧功能
    .ulBaudRate = 115200
};
vUartDeviceInit(&stUart1Static, emUartDevNum1);

// 3. 基础发送
vUartSendByte(emUartDevNum1, 0x31);                          // 发送单个字节
vUartSendArray(emUartDevNum1, (uint8_t*)"Hello", 5);         // 发送数组

// 4. 格式化发送
vUartPrintf(emUartDevNum1, "UART Test: %s\r\n", "OK");       // 格式化字符串
vUartSendInt(emUartDevNum1, 255, 16);                        // 发送16进制整数
vUartSendFloat(emUartDevNum1, 6.789f, 2);                    // 发送保留2位小数的浮点数

// 5. 帧发送
uint8_t ucFrameData[] = {0x01, 0x02, 0x03};
vUartSendFrame(emUartDevNum1, ucFrameData, sizeof(ucFrameData));

// 6. 主循环中执行周期解析
while(1)
{
    vUartDevicePeriodExecute(emUartDevNum1); // 处理接收/帧解析
    
    // 接收帧数据
    uint8_t ucRecvFrameBuf[64] = {0};
    uint32_t ulRecvLen = ulUartReceiveFrame(emUartDevNum1, ucRecvFrameBuf);
    if(ulRecvLen > 0)
    {
        // 处理接收到的帧数据
        vUartPrintf(emUartDevNum1, "Recv Frame: ");
        vUartSendArray(emUartDevNum1, ucRecvFrameBuf, ulRecvLen);
    }
    
    HAL_Delay(10);
}
*/

