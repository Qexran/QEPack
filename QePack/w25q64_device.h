/**
  * @file       w25q64_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/03/07
  * @brief      W25Q64 Flash 驱动，基于 STM32 HAL 库
  *
  */
#include "project_config.h"
#if W25Q64_IS_ENABLE

#ifndef _W25Q64_DEVICE_H_
#define _W25Q64_DEVICE_H_

#include "string.h"
#include "gpio.h"
#include "spi.h"

/**
 * @brief          W25Q64 设备号枚举
 * @note 
 */
typedef enum
{
    emW25q64DevNum0        = 0,
    emW25q64DevNum1,
    emW25q64DevNum2,
    emW25q64DevNum3,
}
emW25q64DevNumTdf;

/**
 * @brief          指令码定义
 * @note 
 */
typedef enum
{
    emW25q64CmdWriteEnable     = 0x06,
    emW25q64CmdWriteDisable    = 0x04,
    emW25q64CmdReadStatus1     = 0x05,
    emW25q64CmdReadData        = 0x03,
    emW25q64CmdPageProgram     = 0x02,
    emW25q64CmdSectorErase     = 0x20,
    emW25q64CmdChipErase       = 0xC7,
    emW25q64CmdReadId          = 0x9F,
}
emW25q64CmdTdf;

/**
 * @brief          静态参数定义
 * @note           
 */
typedef struct
{
    SPI_HandleTypeDef   *pstSpiHandle;     // SPI 句柄指针
    GPIO_TypeDef        *pstCsGpioBase;    // CS 引脚 GPIOx
    uint16_t            usCsGpioPin;       // CS 引脚 GPIO_PIN_x
}
stW25q64StaticParamTdf;

/**
 * @brief          运行参数定义
 * @note           
 */
typedef struct
{
    uint32_t            ulManufacturerId;   // 厂商 ID
    uint32_t            ulMemoryType;       // 存储类型
    uint32_t            ulCapacityId;       // 容量 ID
    uint32_t            ulTotalCapacity;    // 总容量（字节）
}
stW25q64RunningParamTdf;

/**
 * @brief          设备参数定义
 * @note           
 */
typedef struct
{
    stW25q64StaticParamTdf     stStaticParam;  // 静态参数
    stW25q64RunningParamTdf    stRunningParam; // 运行参数
}
stW25q64DeviceParamTdf;


/* 获取 W25Q64 设备参数 */
const stW25q64DeviceParamTdf *c_pstGetW25q64DeviceParam(emW25q64DevNumTdf emDevNum);

/* 初始化相关 */
void vW25q64DeviceInit(stW25q64StaticParamTdf *pstInit, emW25q64DevNumTdf emDevNum);

/* 基础操作 */
void vW25q64WriteEnable(emW25q64DevNumTdf emDevNum);
uint8_t ucW25q64ReadStatus1(emW25q64DevNumTdf emDevNum);
void vW25q64WaitBusy(emW25q64DevNumTdf emDevNum);
uint32_t ulW25q64ReadId(emW25q64DevNumTdf emDevNum);

/* 数据读写 */
void vW25q64ReadData(emW25q64DevNumTdf emDevNum, uint32_t addr, uint8_t* buf, uint32_t len);
uint32_t ulW25q64PageProgram(emW25q64DevNumTdf emDevNum, uint32_t addr, const uint8_t* buf, uint32_t len);
void vW25q64ProgramPage(emW25q64DevNumTdf emDevNum, uint32_t page_num, const uint8_t* buf, uint32_t len);
void vW25q64ReadPage(emW25q64DevNumTdf emDevNum, uint32_t page_num, uint8_t* buf, uint32_t len);

/* 擦除操作 */
void vW25q64SectorErase(emW25q64DevNumTdf emDevNum, uint32_t addr);
void vW25q64ChipErase(emW25q64DevNumTdf emDevNum);
void vW25q64BatchPageErase(emW25q64DevNumTdf emDevNum, uint32_t start_page, uint32_t page_count);

#endif

#endif

/*
    
    // 1. 定义静态参数
    stW25q64StaticParamTdf stW25q64Static = {
        .pstSpiHandle = &hspi1,
        .pstCsGpioBase = W25Q64_CS_GPIO_Port,
        .usCsGpioPin = W25Q64_CS_Pin
    };

    // 2. 初始化设备
    vW25q64DeviceInit(&stW25q64Static, W25Q640);
    
    // 3. 读取芯片 ID
    uint32_t id = ulW25q64ReadId(W25Q640);
    
    // 4. 扇区擦除
    vW25q64SectorErase(W25Q640, 0x000000);
    
    // 5. 地址型页编程
    uint8_t page[] = "Hello QEPack!";
    uint8_t pages = ulW25q64PageProgram(W25Q640, 0x000000, page, sizeof(page));
    
    // 6. 地址型读取数据
    uint8_t read_buf[32];
    vW25q64ReadData(W25Q640, 0x000000, read_buf, 32);
    
    vUartPrintf(UART1, "%s\r\n", read_buf);
    
    // 7. 指定页编程
    uint8_t page2[] = "This is page two.";;
    uint8_t page3[] = "This is page three.";;
    vW25q64ProgramPage(W25Q640, 2, page2, sizeof(page2));
    vW25q64ProgramPage(W25Q640, 3, page3, sizeof(page3));
    
    // 8. 指定页读取数据
    uint8_t read_page2[32];
    uint8_t read_page3[32];
    vW25q64ReadPage(W25Q640, 2, read_page2, sizeof(read_page2));
    vW25q64ReadPage(W25Q640, 3, read_page3, sizeof(read_page3));
    vUartPrintf(UART1, "%s\r\n", read_page2);
    vUartPrintf(UART1, "%s\r\n", read_page3);
    
*/
