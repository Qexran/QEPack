#include "qmc5883_device.h"

#if QMC5883_IS_ENABLE

// 内部辅助函数：写入单个寄存器
static QMC_Status QMC_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    uint16_t addr = (uint16_t)(QMC5883_ADDR_7BIT << 1);
    
    if (HAL_I2C_Master_Transmit(hi2c, addr, buf, 2, 100) == HAL_OK) {
        return QMC_OK;
    }
    return QMC_ERROR;
}

// 内部辅助函数：读取连续寄存器
static QMC_Status QMC_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len) {
    uint16_t addr = (uint16_t)(QMC5883_ADDR_7BIT << 1);
    
    // 1. 发送寄存器起始地址
    if (HAL_I2C_Master_Transmit(hi2c, addr, &reg, 1, 100) != HAL_OK) {
        return QMC_ERROR;
    }
    // 2. 读取数据
    if (HAL_I2C_Master_Receive(hi2c, addr, data, len, 100) == HAL_OK) {
        return QMC_OK;
    }
    return QMC_ERROR;
}

/**
 * @brief 软件复位
 * 写入 CTRL1 (0x09) Bit 2 = 1
 */
void QMC_SoftReset(I2C_HandleTypeDef *hi2c) {
    // 先读取当前值，保留其他位，仅置位 SOFT_RST (Bit 2)
    // 但通常直接写入 0x0D (复位+连续) 或 0x80 (复位) 即可
    // 这里采用上传例程的逻辑：写入 0x09 = 0x80 (仅复位) 或 0x0D (复位+连续)
    // 为了安全，先写复位命令
    QMC_WriteReg(hi2c, QMC_REG_CTRL1, 0x80); 
    HAL_Delay(10); // 等待复位完成
}

/**
 * @brief 检查 ID
 * QMC5883L 的 ID 寄存器 (0x0D) 应返回 0xFF
 */
bool QMC_CheckID(I2C_HandleTypeDef *hi2c) {
    uint8_t id;
    if (QMC_ReadRegs(hi2c, QMC_REG_CHIP_ID, &id, 1) == QMC_OK) {
        return (id == 0xFF);
    }
    return false;
}

/**
 * @brief 初始化 QMC5883L
 * 流程：复位 -> 配置 CTRL2 -> 配置 CTRL1 (模式/量程/采样率)
 */
QMC_Status QMC_Init(I2C_HandleTypeDef *hi2c) {
    QMC_Status status;

    // 1. 可选：ID 检查 (如果失败可能是接线问题或假芯片)
    // 如果调试困难，可暂时注释掉下面这行
    /*
    if (!QMC_CheckID(hi2c)) {
        return QMC_ID_ERROR;
    }
    */

    // 2. 软复位
    // 写入 0x09 = 0x80 (Bit 7-6:0, Bit 2:1 RST, 其他 0)
    // 或者直接写入最终配置 0x9D (包含复位位和模式位)
    // 为了稳妥，分步操作：
    
    // Step A: 软复位 (写入 0x09 = 0x80)
    status = QMC_WriteReg(hi2c, QMC_REG_CTRL1, 0x80);
    if (status != QMC_OK) return QMC_ERROR;
    HAL_Delay(10); // 必须延时，等待内部复位完成

    // Step B: 配置 CTRL2 (0x0A) - 设置/重置周期
    status = QMC_WriteReg(hi2c, QMC_REG_CTRL2, QMC_DEFAULT_CTRL2);
    if (status != QMC_OK) return QMC_ERROR;

    // Step C: 配置 CTRL1 (0x09) - 模式 + ODR + 量程 + OSR
    // 0x9D = 1001 1101
    // Bit 7-6: 10 (OSR 128)
    // Bit 4:   1  (Range 8G)
    // Bit 2:   1  (Roll Pointer - 建议保持)
    // Bit 1-0: 01 (Mode Continuous)
    // Bit 3-2 (ODR): 11 (10Hz) -> 注意上面的宏定义组合
    status = QMC_WriteReg(hi2c, QMC_REG_CTRL1, QMC_DEFAULT_CTRL1);
    if (status != QMC_OK) return QMC_ERROR;

    // 3. 等待第一次测量完成
    // 10Hz ODR 下，最长需要 100ms
    HAL_Delay(50); 

    return QMC_OK;
}

/**
 * @brief 读取数据
 * 注意：QMC5883L 数据格式为 Little-Endian (LSB 在前)
 * 寄存器顺序: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
 */
QMC_Status QMC_ReadData(I2C_HandleTypeDef *hi2c, QMC_Data *data) {
    uint8_t buffer[6];

    // 从 0x00 开始读取 6 个字节
    if (QMC_ReadRegs(hi2c, QMC_REG_X_LSB, buffer, 6) != QMC_OK) {
        return QMC_ERROR;
    }

    // 状态检查 (可选)：检查 0x06 寄存器的 DRDY 位 (Bit 0)
    // 如果追求极致速度，可以先读状态寄存器，确认 DRDY=1 再读数据
    // 这里为了简化，直接读取，依靠初始化时的延时和主循环延时保证数据就绪

    // 组合数据：Little-Endian (MSB << 8 | LSB) -> 注意 buffer 索引
    // buffer[0]=X_LSB, buffer[1]=X_MSB
    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return QMC_OK;
}

#endif
