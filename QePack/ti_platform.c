#include "ti_platform.h"

#if (QEPACK_PLATFORM == TI)

#include "ti_msp_dl_config.h"

volatile unsigned long tick_ms;
static uint8_t is_initialed_clock = 0;

/**
 * @brief SysTick处理函数
 * 
 */
void SysTick_Handler(void)
{
    tick_ms++;
}

void TI_Delay(unsigned long num_ms)
{
    volatile unsigned long start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
}

void mspm0_delay_ms(unsigned long num_ms)
{
    volatile unsigned long start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
}

void mspm0_get_clock_ms(unsigned long *count)
{
    if (count)
        *count = tick_ms;
}

uint8_t ucGetSysTickInitialState(){
    return is_initialed_clock;
}

void SysTick_Init(void)
{
    is_initialed_clock = 1;
    DL_SYSTICK_config(CPUCLK_FREQ / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
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
        TI_Delay(1);
        DL_GPIO_setPins(pstIdf->pstSclGpioPort, pstIdf->usSclPin);
        TI_Delay(1);

        if(DL_GPIO_readPins(pstIdf->pstSdaGpioPort, pstIdf->usSdaPin))
            break;
    }while(++cycleCnt < 100);
    mspm0_i2c_enable(pstIdf);
}

/**
 * @brief 向I2C设备写入内存
 *
 * @param pstIdf I2C配置结构体指针
 * @param DevAddress 设备地址（7位）
 * @param MemAddress 寄存器地址
 * @param pData 数据指针
 * @param Size 数据大小
 * @param Timeout 超时时间（ms）
 * @return QE_StatusTypeDef QE_OK 成功，QE_TIMEOUT 超时，QE_ERROR 总线错误
 *
 * 单次 I2C 事务发送 MemAddress + pData[0..Size-1]。
 * 支持大于 8 字节的数据块（利用时钟拉伸自动分片填充 FIFO）。
 * 已添加勘误 I2C_ERR_13 延迟及 BUSY/ERROR 状态检查。
 */
QE_StatusTypeDef TI_I2C_Mem_Write(
    stI2CTdf *pstIdf,
    uint8_t DevAddress, uint8_t MemAddress,
    uint8_t *pData, uint16_t Size, uint32_t Timeout
)
{
    unsigned long start, cur;
    DL_I2C_ClockConfig clockConfig;
    uint32_t delayCycles;
    uint8_t  totalBytes = (uint8_t)(1 + Size);
    uint8_t  bytesSent;

    /* 计算勘误 I2C_ERR_13 所需延迟（>= 3 个 I2C 功能时钟周期，转换为 CPU 周期） */
    DL_I2C_getClockConfig(pstIdf->i2c_inst, &clockConfig);
    delayCycles = 3 * (clockConfig.divideRatio + 1);
    /* I2C 功能时钟频率 → CPU 周期转换 */
    if (clockConfig.clockSel == DL_I2C_CLOCK_MFCLK) {
        delayCycles *= (CPUCLK_FREQ / 4000000);
    }
    /* BUSCLK 时 CPUCLK_FREQ / BUSCLK == 1，无需乘 */

    /* 初始填充 FIFO */
    bytesSent = DL_I2C_fillControllerTXFIFO(pstIdf->i2c_inst, &MemAddress, 1);
    if (Size > 0) {
        bytesSent += DL_I2C_fillControllerTXFIFO(pstIdf->i2c_inst, pData, Size);
    }

    while (!(DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(pstIdf->i2c_inst, DevAddress,
        DL_I2C_CONTROLLER_DIRECTION_TX, totalBytes);

    /* 勘误 I2C_ERR_13 */
    delay_cycles(delayCycles);

    /* 等待传输完成，期间持续补充 FIFO（利用时钟拉伸，支持 > 8 字节） */
    mspm0_get_clock_ms(&start);
    while (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR)
        {
            i2c_sda_unlock(pstIdf);
            return QE_ERROR;
        }
        /* FIFO 有空位时补充数据 */
        if (bytesSent < totalBytes)
        {
            uint8_t offset = bytesSent - 1;
            bytesSent += DL_I2C_fillControllerTXFIFO(pstIdf->i2c_inst,
                &pData[offset], totalBytes - bytesSent);
        }
        mspm0_get_clock_ms(&cur);
        if ((cur - start) >= Timeout)
        {
            i2c_sda_unlock(pstIdf);
            return QE_TIMEOUT;
        }
    }

    /* 传输完成后检查总线错误 */
    if (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        i2c_sda_unlock(pstIdf);
        return QE_ERROR;
    }

    while (!(DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));

    return QE_OK;
}

/**
 * @brief 从I2C设备读取内存
 *
 * @param pstIdf I2C配置结构体指针
 * @param DevAddress 设备地址（7位）
 * @param MemAddress 寄存器地址
 * @param pData 数据指针（输出）
 * @param Size 读取大小
 * @param Timeout 超时时间（ms）
 * @return QE_StatusTypeDef QE_OK 成功，QE_TIMEOUT 超时，QE_ERROR 总线错误
 *
 * 采用两次独立传输：先写寄存器地址（TX），再读数据（RX）。
 * 与 TI 官方轮询示例 i2c_controller_rw_multibyte_fifo_poll 流程一致。
 */
QE_StatusTypeDef TI_I2C_Mem_Read(
    stI2CTdf *pstIdf,
    uint8_t DevAddress, uint8_t MemAddress,
    uint8_t *pData, uint16_t Size, uint32_t Timeout
)
{
    unsigned long start, cur;
    DL_I2C_ClockConfig clockConfig;
    uint32_t delayCycles;

    /* 计算勘误 I2C_ERR_13 所需延迟（>= 3 个 I2C 功能时钟周期，转换为 CPU 周期） */
    DL_I2C_getClockConfig(pstIdf->i2c_inst, &clockConfig);
    delayCycles = 3 * (clockConfig.divideRatio + 1);
    if (clockConfig.clockSel == DL_I2C_CLOCK_MFCLK) {
        delayCycles *= (CPUCLK_FREQ / 4000000);
    }

    /* ===== 第一阶段：写寄存器地址（TX） ===== */
    DL_I2C_fillControllerTXFIFO(pstIdf->i2c_inst, &MemAddress, 1);

    while (!(DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(pstIdf->i2c_inst, DevAddress,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    delay_cycles(delayCycles);

    mspm0_get_clock_ms(&start);
    while (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        mspm0_get_clock_ms(&cur);
        if ((cur - start) >= Timeout)
        {
            i2c_sda_unlock(pstIdf);
            return QE_TIMEOUT;
        }
    }

    if (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        i2c_sda_unlock(pstIdf);
        return QE_ERROR;
    }

    while (!(DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* ===== 第二阶段：读取数据（RX） ===== */
    DL_I2C_startControllerTransfer(pstIdf->i2c_inst, DevAddress,
        DL_I2C_CONTROLLER_DIRECTION_RX, Size);

    delay_cycles(delayCycles);

    /* 逐字节读取 RX FIFO（与 TI 官方轮询示例一致：边收边读，不等 BUSY 清零） */
    for (uint16_t i = 0; i < Size; i++)
    {
        mspm0_get_clock_ms(&start);
        while (DL_I2C_isControllerRXFIFOEmpty(pstIdf->i2c_inst))
        {
            /* 传输过程中检测总线错误（NACK 等） */
            if (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR)
            {
                i2c_sda_unlock(pstIdf);
                return QE_ERROR;
            }
            mspm0_get_clock_ms(&cur);
            if ((cur - start) >= Timeout)
            {
                i2c_sda_unlock(pstIdf);
                return QE_TIMEOUT;
            }
        }
        pData[i] = DL_I2C_receiveControllerData(pstIdf->i2c_inst);
    }

    /* 等待传输完全结束 */
    mspm0_get_clock_ms(&start);
    while (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        mspm0_get_clock_ms(&cur);
        if ((cur - start) >= Timeout)
        {
            i2c_sda_unlock(pstIdf);
            return QE_TIMEOUT;
        }
    }

    if (DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        i2c_sda_unlock(pstIdf);
        return QE_ERROR;
    }

    while (!(DL_I2C_getControllerStatus(pstIdf->i2c_inst) & DL_I2C_CONTROLLER_STATUS_IDLE));

    return QE_OK;
}

/**
 * @brief 读取GPIO引脚状态
 * 
 * @param GPIOx GPIO寄存器基地址
 * @param GPIO_Pin GPIO引脚号
 * @return uint8_t 引脚状态
 */
GPIO_PinState TI_GPIO_ReadPin(GPIO_Regs *GPIOx, uint32_t GPIO_Pin){
    return DL_GPIO_readPins(GPIOx, GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; 
}

/**
    函数原型：
    void HAL_GPIO_WritePin(
        GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
        GPIO_PinState PinState)
        
    永远不推荐使用 DL_GPIO_writePins !
*/
void TI_GPIO_WritePin(GPIO_Regs *GPIOx, uint32_t GPIO_Pin, GPIO_PinState PinState){
    if(PinState == GPIO_PIN_SET){
        DL_GPIO_setPins(GPIOx, GPIO_Pin);
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
QE_StatusTypeDef TI_UART_Transmit(
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
                mspm0_get_clock_ms(&cur);

                if ((cur - start) >= Timeout) {
                    return QE_TIMEOUT; 
                }
                
            }
        }
    }

    return QE_OK;
}


/**
    函数原型:
    HAL_StatusTypeDef HAL_ADC_Start_DMA(
        ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
*/
#if ADC_DEVICE_IS_ENABLE
void TI_ADC_Start(stAdcTdf *pstAdcBase){
    // DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, (1024 * Length) >> 1);

    /*
        * Check the ADC started converting in single channel repeat mode.
    */
    if (DL_ADC12_STATUS_CONVERSION_ACTIVE == DL_ADC12_getStatus(ADC12_0_INST)) {
        DL_ADC12_stopConversion(ADC12_0_INST);
    }

    /* 使能 ADC 转换（针对单次触发模式，需要重新W使能才能响应下一次启动信号） */
    DL_ADC12_enableConversions(pstAdcBase->adc_inst); 

    /* 触发 ADC12_0 一次ADC转换 */
    DL_ADC12_startConversion(pstAdcBase->adc_inst);
    
    // DL_ADC12_enableDMA(pstAdcBase->adc_inst);
}

#endif

/**
 * @brief 获取自系统启动以来的毫秒数
 * @return uint32_t 系统启动以来的毫秒数
 */
uint32_t TI_GetTick(void)
{
    return tick_ms;
}


void vTiClearFlashDebris()
{
    // 1. 关闭全局中断，防止Flash操作被打断
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // 2. 清除Flash错误状态位（≥32MHz 时必须在每次 Flash 操作前调用）
    DL_FlashCTL_executeClearStatus(FLASHCTL);

    // 3. 解锁目标扇区
    DL_FlashCTL_unprotectSector(
        FLASHCTL,
        FLASH_STATUS_ADDR,
        DL_FLASHCTL_REGION_SELECT_MAIN
    );

    // 4. 再次清除状态，防止解锁操作残留标志
    DL_FlashCTL_executeClearStatus(FLASHCTL);

    // 5. 扇区擦除
    DL_FlashCTL_eraseMemory(
        FLASHCTL,
        FLASH_STATUS_ADDR,
        DL_FLASHCTL_COMMAND_SIZE_SECTOR
    );

    // 6. 等待 Flash 擦除完成（硬件状态轮询 + 超时保护）
    {
        volatile uint32_t ulFlashTimeout = 100000;
        while (ulFlashTimeout-- && DL_FlashCTL_getCommandStatus(FLASHCTL) == DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS);
    }

    // 7. 擦除完成后清除残留状态，确保后续 Program 操作不会误判
    DL_FlashCTL_executeClearStatus(FLASHCTL);

    // 8. 重新加锁扇区，保护Flash
    DL_FlashCTL_protectSector(
        FLASHCTL,
        FLASH_STATUS_ADDR,
        DL_FLASHCTL_REGION_SELECT_MAIN
    );

    // 9. 恢复全局中断
    __set_PRIMASK(primask);
}

/* ==================== Timer IC 抽象（超声波等模块使用） ==================== */

void TI_TIM_IC_Start(stTimerTdf *pstTim, uint32_t ulChannel)
{
    (void)pstTim;
    (void)ulChannel;
    /* MSPM0: 捕获通道在 SysConfig 初始化时已配置并使能，
       此处为空操作。若需动态启动，可在此添加寄存器配置。 */
}

void TI_TIM_IC_Stop(stTimerTdf *pstTim, uint32_t ulChannel)
{
    (void)pstTim;
    (void)ulChannel;
    /* MSPM0: 捕获通道无法单独停止，此处为空操作。
       超声波模块通过重置定时器计数值实现"重新测量"。 */
}

/* 将 STM32 风格的 TIM_CHANNEL_x(0-based) 转为 MSPM0 的 0-based 索引 */
#define TI_TIM_CH_TO_IDX(ch)  ((ch) & 0x03U)

uint32_t TI_TIM_ReadCapturedValue(stTimerTdf *pstTim, uint32_t ulChannel)
{
    uint32_t ulIdx = TI_TIM_CH_TO_IDX(ulChannel);
    /* MSPM0 COUNTERREGS 按通道对分组：CC_01[0]=CC0, CC_01[1]=CC1, CC_23[0]=CC2, CC_23[1]=CC3 */
    if (ulIdx < 2) {
        return pstTim->timer_inst->COUNTERREGS.CC_01[ulIdx];
    } else {
        return pstTim->timer_inst->COUNTERREGS.CC_23[ulIdx - 2];
    }
}

uint32_t TI_TIM_GetFlag(stTimerTdf *pstTim, uint32_t ulChannel)
{
    /* 读取原始中断状态（RIS），CCU0 在 bit 8，CCU1 在 bit 9 */
    return pstTim->timer_inst->CPU_INT.RIS & (1U << (8U + TI_TIM_CH_TO_IDX(ulChannel)));
}

void TI_TIM_ClearFlag(stTimerTdf *pstTim, uint32_t ulChannel)
{
    /* 写 ICLR 寄存器清除中断标志 */
    pstTim->timer_inst->CPU_INT.ICLR = (1U << (8U + TI_TIM_CH_TO_IDX(ulChannel)));
}

void TI_Delay_us(uint32_t us)
{
    /* 使用 SysTick VAL 寄存器做忙等延时
       SysTick 从 LOAD 递减到 0 后重装，VAL 表示当前剩余计数 */
    uint32_t ticks = (CPUCLK_FREQ / 1000000U) * us;
    uint32_t prev  = SysTick->VAL;
    uint32_t elapsed = 0;

    while (elapsed < ticks) {
        uint32_t curr = SysTick->VAL;
        /* SysTick 是递减计数器 */
        if (prev >= curr) {
            elapsed += (prev - curr);
        } else {
            /* 计数器重装（LOAD → 0 → LOAD） */
            elapsed += prev + (SysTick->LOAD - curr);
        }
        prev = curr;
    }
}

#if UART_IS_USE_DMA
/**
 * @brief  根据 UART 外设基址查找对应的 DMA RX 触发源
 */
uint32_t TI_GetUartDmaRxTrigger(UART_Regs *uart_inst)
{
    if (uart_inst == UART0) return DMA_UART0_RX_TRIG;
    if (uart_inst == UART1) return DMA_UART1_RX_TRIG;
    if (uart_inst == UART2) return DMA_UART2_RX_TRIG;
    if (uart_inst == UART3) return DMA_UART3_RX_TRIG;
    return 0;
}

/**
 * @brief  根据 UART 外设基址查找对应的 DMA TX 触发源
 */
uint32_t TI_GetUartDmaTxTrigger(UART_Regs *uart_inst)
{
    if (uart_inst == UART0) return DMA_UART0_TX_TRIG;
    if (uart_inst == UART1) return DMA_UART1_TX_TRIG;
    if (uart_inst == UART2) return DMA_UART2_TX_TRIG;
    if (uart_inst == UART3) return DMA_UART3_TX_TRIG;
    return 0;
}
#endif


#endif
