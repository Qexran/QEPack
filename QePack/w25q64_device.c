/**
  * @file       w25q64_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/03/07
  * @brief      W25Q64 Flash 驱动，基于 STM32 HAL 库
  *
  */
#include "w25q64_device.h"
#if W25Q64_IS_ENABLE

stW25q64DeviceParamTdf astW25q64DeviceParam[W25Q64_DEV_NUM];

/**
 * @brief       获取 W25Q64 设备参数
 * @param       emDevNum   ：设备号
 * @return      const stW25q64DeviceParamTdf * ：W25Q64 设备参数指针
 */
const stW25q64DeviceParamTdf *c_pstGetW25q64DeviceParam(emW25q64DevNumTdf emDevNum)
{
    return &astW25q64DeviceParam[emDevNum];
}

/**
 * @brief       片选拉低
 * @param       emDevNum   ：设备号
 */
static void vW25q64CsLow(emW25q64DevNumTdf emDevNum)
{
    HAL_GPIO_WritePin(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstCsGpioBase,
        astW25q64DeviceParam[emDevNum].stStaticParam.usCsGpioPin,
        GPIO_PIN_RESET
    );
}

/**
 * @brief       片选拉高
 * @param       emDevNum   ：设备号
 */
static void vW25q64CsHigh(emW25q64DevNumTdf emDevNum)
{
    HAL_GPIO_WritePin(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstCsGpioBase,
        astW25q64DeviceParam[emDevNum].stStaticParam.usCsGpioPin,
        GPIO_PIN_SET
    );
}

/**
 * @brief       写使能
 * @param       emDevNum   ：设备号
 */
void vW25q64WriteEnable(emW25q64DevNumTdf emDevNum)
{
    uint8_t cmd = emW25q64CmdWriteEnable;
    vW25q64CsLow(emDevNum);
    HAL_SPI_Transmit(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        &cmd,
        1,
        HAL_MAX_DELAY
    );
    vW25q64CsHigh(emDevNum);
}

/**
 * @brief       读取状态寄存器 1
 * @param       emDevNum   ：设备号
 * @return      uint8_t ：状态寄存器值
 */
uint8_t ucW25q64ReadStatus1(emW25q64DevNumTdf emDevNum)
{
    uint8_t cmd = emW25q64CmdReadStatus1;
    uint8_t status;
    vW25q64CsLow(emDevNum);
    HAL_SPI_Transmit(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        &cmd,
        1,
        HAL_MAX_DELAY
    );
    HAL_SPI_Receive(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        &status,
        1,
        HAL_MAX_DELAY
    );
    vW25q64CsHigh(emDevNum);
    return status;
}

/**
 * @brief       等待空闲（WIP=0）
 * @param       emDevNum   ：设备号
 */
void vW25q64WaitBusy(emW25q64DevNumTdf emDevNum)
{
    while (ucW25q64ReadStatus1(emDevNum) & 0x01);
}

/**
 * @brief       读取 JEDEC ID
 * @param       emDevNum   ：设备号
 * @return      uint32_t ：完整 ID（Manufacturer ID + Memory Type + Capacity ID）
 */
uint32_t ulW25q64ReadId(emW25q64DevNumTdf emDevNum)
{
    uint8_t cmd = emW25q64CmdReadId;
    uint8_t id_buf[3];
    vW25q64CsLow(emDevNum);
    HAL_SPI_Transmit(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        &cmd,
        1,
        HAL_MAX_DELAY
    );
    HAL_SPI_Receive(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        id_buf,
        3,
        HAL_MAX_DELAY
    );
    vW25q64CsHigh(emDevNum);
    
    astW25q64DeviceParam[emDevNum].stRunningParam.ulManufacturerId = id_buf[0];
    astW25q64DeviceParam[emDevNum].stRunningParam.ulMemoryType = id_buf[1];
    astW25q64DeviceParam[emDevNum].stRunningParam.ulCapacityId = id_buf[2];
    astW25q64DeviceParam[emDevNum].stRunningParam.ulTotalCapacity = 1UL << (id_buf[2] - 0x10);
    
    return (id_buf[0] << 16) | (id_buf[1] << 8) | id_buf[2];
}

/**
 * @brief       读取数据
 * @param       emDevNum   ：设备号
 * @param       addr       ：起始地址
 * @param       buf        ：数据缓冲区指针
 * @param       len        ：读取长度
 */
void vW25q64ReadData(emW25q64DevNumTdf emDevNum, uint32_t addr, uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len == 0 || addr >= W25Q64_TOTAL_SIZE) {
        return;
    }
    
    uint8_t cmd[4];
    cmd[0] = emW25q64CmdReadData;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    vW25q64CsLow(emDevNum);
    HAL_SPI_Transmit(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        cmd,
        4,
        HAL_MAX_DELAY
    );
    HAL_SPI_Receive(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        buf,
        len,
        HAL_MAX_DELAY
    );
    vW25q64CsHigh(emDevNum);
}

/**
 * @brief       读取指定页数据
 * @param       emDevNum   ：设备号
 * @param       page_num   ：页号
 * @param       buf        ：数据缓冲区指针
 * @param       len        ：读取长度
 */
void vW25q64ReadPage(emW25q64DevNumTdf emDevNum, uint32_t page_num, uint8_t* buf, uint32_t len)
{
    uint32_t addr = page_num * W25Q64_PAGE_SIZE;
    vW25q64ReadData(emDevNum, addr, buf, len);
}

/**
 * @brief       页编程
 * @param       emDevNum   ：设备号
 * @param       addr       ：起始地址
 * @param       buf        ：数据缓冲区指针
 * @param       len        ：写入长度
 * @return      uint32_t   ：实际写入的页数
 */
uint32_t ulW25q64PageProgram(emW25q64DevNumTdf emDevNum, uint32_t addr, const uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len == 0 || addr >= W25Q64_TOTAL_SIZE) {
        return 0;
    }
    
    uint32_t page_count = 0;
    uint32_t remaining_len = len;
    uint32_t current_addr = addr;
    
    while (remaining_len > 0) {
        uint32_t page_len = remaining_len > W25Q64_PAGE_SIZE ? W25Q64_PAGE_SIZE : remaining_len;
        
        vW25q64WriteEnable(emDevNum);
        
        uint8_t cmd[4];
        cmd[0] = emW25q64CmdPageProgram;
        cmd[1] = (current_addr >> 16) & 0xFF;
        cmd[2] = (current_addr >> 8) & 0xFF;
        cmd[3] = current_addr & 0xFF;
        
        vW25q64CsLow(emDevNum);
        HAL_SPI_Transmit(
            astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
            cmd,
            4,
            HAL_MAX_DELAY
        );
        HAL_SPI_Transmit(
            astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
            (uint8_t*)buf + (current_addr - addr),
            page_len,
            HAL_MAX_DELAY
        );
        vW25q64CsHigh(emDevNum);
        
        vW25q64WaitBusy(emDevNum);
        
        current_addr += page_len;
        remaining_len -= page_len;
        page_count++;
    }
    
    return page_count;
}

/**
 * @brief       指定页编程
 * @param       emDevNum   ：设备号
 * @param       page_num   ：页号
 * @param       buf        ：数据缓冲区指针
 * @param       len        ：写入长度
 */
void vW25q64ProgramPage(emW25q64DevNumTdf emDevNum, uint32_t page_num, const uint8_t* buf, uint32_t len)
{
    uint32_t addr = page_num * W25Q64_PAGE_SIZE;
    ulW25q64PageProgram(emDevNum, addr, buf, len);
}

/**
 * @brief       扇区擦除（按 4KB）
 * @param       emDevNum   ：设备号
 * @param       addr       ：扇区起始地址
 */
void vW25q64SectorErase(emW25q64DevNumTdf emDevNum, uint32_t addr)
{
    if (addr >= W25Q64_TOTAL_SIZE) {
        return;
    }
    
    vW25q64WriteEnable(emDevNum);
    
    uint8_t cmd[4];
    cmd[0] = emW25q64CmdSectorErase;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    vW25q64CsLow(emDevNum);
    HAL_SPI_Transmit(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        cmd,
        4,
        HAL_MAX_DELAY
    );
    vW25q64CsHigh(emDevNum);
    
    vW25q64WaitBusy(emDevNum);
}

/**
 * @brief       整片擦除
 * @param       emDevNum   ：设备号
 */
void vW25q64ChipErase(emW25q64DevNumTdf emDevNum)
{
    vW25q64WriteEnable(emDevNum);
    
    uint8_t cmd = emW25q64CmdChipErase;
    vW25q64CsLow(emDevNum);
    HAL_SPI_Transmit(
        astW25q64DeviceParam[emDevNum].stStaticParam.pstSpiHandle,
        &cmd,
        1,
        HAL_MAX_DELAY
    );
    vW25q64CsHigh(emDevNum);
    
    vW25q64WaitBusy(emDevNum);
}

/**
 * @brief       批量擦除页
 * @param       emDevNum   ：设备号
 * @param       start_page ：起始页号
 * @param       page_count ：擦除页数
 */
void vW25q64BatchPageErase(emW25q64DevNumTdf emDevNum, uint32_t start_page, uint32_t page_count)
{
    uint32_t start_addr = start_page * W25Q64_PAGE_SIZE;
    uint32_t end_addr = (start_page + page_count) * W25Q64_PAGE_SIZE;
    
    if (start_addr >= W25Q64_TOTAL_SIZE || end_addr > W25Q64_TOTAL_SIZE) {
        return;
    }
    
    // 按扇区擦除（W25Q64 最小擦除单位是扇区）
    uint32_t start_sector = start_addr / W25Q64_SECTOR_SIZE;
    uint32_t end_sector = (end_addr + W25Q64_SECTOR_SIZE - 1) / W25Q64_SECTOR_SIZE;
    
    for (uint32_t sector = start_sector; sector < end_sector; sector++) {
        vW25q64SectorErase(emDevNum, sector * W25Q64_SECTOR_SIZE);
    }
}

/**
 * @brief       W25Q64 设备初始化
 * @param       pstInit     ：初始化参数结构体的首地址
 * @param       emDevNum    ：设备编号
 */
void vW25q64DeviceInit(stW25q64StaticParamTdf *pstInit, emW25q64DevNumTdf emDevNum)
{
    if (pstInit == NULL || emDevNum >= W25Q64_DEV_NUM) {
        return;
    }
    
    memcpy(
        &astW25q64DeviceParam[emDevNum].stStaticParam,
        pstInit,
        sizeof(stW25q64StaticParamTdf)
    );
    
    memset(
        &astW25q64DeviceParam[emDevNum].stRunningParam,
        0,
        sizeof(stW25q64RunningParamTdf)
    );
    
    vW25q64CsHigh(emDevNum);
    ulW25q64ReadId(emDevNum);
}

#endif
