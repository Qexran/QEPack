#include "ti_platform.h"

#if (QEPACK_PLATFORM == TI)



void TI_Delay(uint32_t ms)
{
    mspm0_delay_ms(ms);
}

/**
 * @brief 禁用I2C
 * 
 * @param pstIdf I2C配置结构体指针
 * @return int 0 成功
 */
static int mspm0_i2c_disable(stI2CTdf *pstIdf)
{
    DL_I2C_reset(pstIdf->i2c_inst);
    DL_GPIO_initDigitalOutput(pstIdf->ulIOMuxScl);
    DL_GPIO_initDigitalInputFeatures(pstIdf->ulIOMuxSda,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(pstIdf->pstSclGpioPort, pstIdf->usSclPin);
    DL_GPIO_enableOutput(pstIdf->pstSclGpioPort, pstIdf->usSclPin);
    return 0;
}

/**
 * @brief 使能I2C
 * 
 * @param pstIdf I2C配置结构体指针
 * @return int 0 成功
 */
static int mspm0_i2c_enable(stI2CTdf *pstIdf)
{
    DL_I2C_reset(pstIdf->i2c_inst);
    DL_GPIO_initPeripheralInputFunctionFeatures(pstIdf->ulIOMuxSda,
        pstIdf->ulIOMuxSdaFunc, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(pstIdf->ulIOMuxScl,
        pstIdf->ulIOMuxSclFunc, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(pstIdf->ulIOMuxSda);
    DL_GPIO_enableHiZ(pstIdf->ulIOMuxScl);
    DL_I2C_enablePower(pstIdf->i2c_inst);
    pstIdf->vI2cInitFunc();
    return 0;
}

/**
 * @brief 解锁I2C SDA引脚
 * 
 * @param pstIdf I2C配置结构体指针
 */
static void i2c_sda_unlock(
    stI2CTdf *pstIdf
)
{
    uint8_t cycleCnt = 0;
    mspm0_i2c_disable(pstIdf);
    do
    {
        DL_GPIO_clearPins(pstIdf->pstSclGpioPort, pstIdf->usSclPin);
        mspm0_delay_ms(1);
        DL_GPIO_setPins(pstIdf->pstSclGpioPort, pstIdf->usSclPin);
        mspm0_delay_ms(1);

        if(DL_GPIO_readPins(pstIdf->pstSdaGpioPort, pstIdf->usSdaPin))
            break;
    }while(++cycleCnt < 100);
    mspm0_i2c_enable(pstIdf);
}

/**
 * @brief 向I2C设备写入内存
 * 
 * @param pstIdf I2C配置结构体指针
 * @param DevAddress 设备地址
 * @param MemAddress 内存地址
 * @param pData 数据指针
 * @param Size 数据大小
 * @param Timeout 超时时间
 */
void TI_I2C_Mem_Write(
    stI2CTdf *pstIdf,
    uint8_t DevAddress, uint8_t MemAddress,
    uint8_t *pData, uint16_t Size, uint32_t Timeout
)
{
    unsigned char ptr[2];
    unsigned long start, cur;
    /*
        原函数信息
        I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, 
        uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout
    */
    for(uint8_t i = 0;i < Size; i++){

        ptr[0] = MemAddress;
        ptr[1] = pData[i];

        mspm0_get_clock_ms(&start);

        // 填充数据到发送FIFO
        DL_I2C_fillControllerTXFIFO(pstIdf->i2c_inst, ptr, 2);
        // 清除中断标志
        DL_I2C_clearInterruptStatus(pstIdf->i2c_inst, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
        // 等待硬件空闲
        while (!(DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));
        DL_I2C_startControllerTransfer(pstIdf->i2c_inst, DevAddress, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

        while (!DL_I2C_getRawInterruptStatus(pstIdf->i2c_inst, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
        {
            mspm0_get_clock_ms(&cur);
            if(cur >= (start + Timeout))
            {
                i2c_sda_unlock(pstIdf);
                break;
            }
        }
    }
}

/**
 * @brief 读取GPIO引脚状态
 * 
 * @param GPIOx GPIO寄存器基地址
 * @param GPIO_Pin GPIO引脚号
 * @return uint8_t 引脚状态
 */
uint8_t TI_GPIO_ReadPin(GPIO_Regs *GPIOx, uint32_t GPIO_Pin){
    return DL_GPIO_readPins(GPIOx, GPIO_Pin) ? 0 : 1; 
}

/**
    函数原型：
    void HAL_GPIO_WritePin(
        GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
        GPIO_PinState PinState)

    void DL_GPIO_writePins(GPIO_Regs* gpio, uint32_t pins)
*/
void TI_GPIO_WritePin(GPIO_Regs *GPIOx, uint32_t GPIO_Pin, GPIO_PinState PinState){
    if(PinState == GPIO_PIN_SET){
        DL_GPIO_writePins(GPIOx, GPIO_Pin);
    } else {
        DL_GPIO_clearPins(GPIOx, GPIO_Pin);
    }
}

/**
    函数原型:
    HAL_StatusTypeDef HAL_UART_Transmit(
            UART_HandleTypeDef *huart, const uint8_t *pData, 
            uint16_t Size, uint32_t Timeout)
*/
void TI_UART_Transmit(
    stUartTdf *uart_inst, const uint8_t *pData, 
    uint16_t Size, uint32_t Timeout
){
    unsigned long start, cur;
    bool txSuccess = false; // 用于存储 Check 函数的返回结果
    
    for (uint16_t i = 0; i < Size; i++) {
        mspm0_get_clock_ms(&start);
        txSuccess = false; // 重置标志
        
        // 尝试写入
        while (!txSuccess) {
            // 尝试写入数据
            // 如果 FIFO 有空位：写入成功，返回 true
            // 如果 FIFO 已满：不写入，返回 false
            txSuccess = DL_UART_transmitDataCheck(uart_inst->uart_inst, pData[i]); 
            
            if (!txSuccess) {
                // 如果写入失败（因为忙），检查是否超时
                mspm0_get_clock_ms(&cur);

                if ((cur - start) >= Timeout) {
                    // === 超时处理 ===
                    // 可以选择返回错误码，或者 break 跳出循环
                    return; 
                }
                
                // 可选：稍微延时一点点，避免死循环占用 100% CPU (视系统需求而定)
                // __asm(" NOP "); 
            }
        }
        // 如果 txSuccess 为 true，说明数据已写入，继续循环发送下一个字节
    }
}






#endif
