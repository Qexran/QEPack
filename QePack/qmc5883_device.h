#include "project_config.h"

#if QMC5883_IS_ENABLE

#ifndef __QMC5883_DEVICE_H
#define __QMC5883_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "stm32f4xx_hal.h" // 请根据您的芯片型号修改，如 stm32f1xx_hal.h
#include <stdint.h>
#include <stdbool.h>

/* ================= 配置参数 ================= */
// QMC5883L I2C 7位地址
#define QMC5883_ADDR_7BIT     0x0D

// 寄存器定义 (基于上传的 QMC5883.h)
#define QMC_REG_X_LSB         0x00
#define QMC_REG_X_MSB         0x01
#define QMC_REG_Y_LSB         0x02
#define QMC_REG_Y_MSB         0x03
#define QMC_REG_Z_LSB         0x04
#define QMC_REG_Z_MSB         0x05
#define QMC_REG_STATUS        0x06
#define QMC_REG_CTRL1         0x09
#define QMC_REG_CTRL2         0x0A
#define QMC_REG_RST_PERIOD    0x0B
#define QMC_REG_CHIP_ID       0x0D

// 控制寄存器1 (CTRL1) 位定义
#define QMC_MODE_STANDBY      0x00
#define QMC_MODE_CONTINUOUS   0x01
#define QMC_ODR_10HZ          0x0C
#define QMC_ODR_50HZ          0x08
#define QMC_ODR_100HZ         0x04
#define QMC_ODR_200HZ         0x00
#define QMC_RANGE_2G          0x00
#define QMC_RANGE_8G          0x10
#define QMC_OSR_64            0x00
#define QMC_OSR_128           0x80
#define QMC_OSR_256           0x40
#define QMC_OSR_512           0xC0

// 推荐配置：连续模式 + 10Hz + 8G量程 + 128过采样
// 0x01(Mode) | 0x0C(ODR) | 0x10(Range) | 0x80(OSR) = 0x9D
#define QMC_DEFAULT_CTRL1     0x9D 
#define QMC_DEFAULT_CTRL2     0x01  // 设置/重置周期

/* ================= 数据结构 ================= */
typedef enum {
    QMC_OK = 0,
    QMC_ERROR,
    QMC_ID_ERROR
} QMC_Status;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} QMC_Data;

/* ================= 函数声明 ================= */

/**
 * @brief 初始化 QMC5883L
 * @param hi2c I2C 句柄指针 (例如 &hi2c2)
 * @return 状态码
 */
QMC_Status QMC_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取三轴磁数据
 * @param hi2c I2C 句柄指针
 * @param data 数据输出指针
 * @return 状态码
 */
QMC_Status QMC_ReadData(I2C_HandleTypeDef *hi2c, QMC_Data *data);

/**
 * @brief 检查芯片 ID (应为 0xFF)
 * @param hi2c I2C 句柄指针
 * @return true: ID匹配, false: 不匹配
 */
bool QMC_CheckID(I2C_HandleTypeDef *hi2c);

/**
 * @brief 软件复位传感器
 * @param hi2c I2C 句柄指针
 */
void QMC_SoftReset(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif

#endif 

/* __QMC5883_HAL_H 实例 */
/*
    void MagInit() {
        // 初始化
        if (QMC_Init(&hi2c2) != QMC_OK) {
            vOledPrintf(OLED0, 1, 1, OLED_8X16, "Mag Init Fail!");
            vOledUpdateAll();
            // 可以选择卡死或继续
            //while(1); 
        } else {
            vOledPrintf(OLED0, 1, 1, OLED_8X16, "Mag Init OK!  ");
            vOledUpdateAll();
            HAL_Delay(500);
            vOledClear(OLED0);
        }
    }
    
    void main(){
        QMC_Data mag_data;
    
        MagInit();
        
        while(){
            // 读取数据
            if (QMC_ReadData(&hi2c2, &mag_data) == QMC_OK) {
                // 显示数据，%-6d 表示左对齐占6位，防止数字长度变化导致显示错乱
                vOledPrintf(OLED0, 1, 1, OLED_8X16, "X:%-6d Y:%-6d", mag_data.x, mag_data.y);
                vOledPrintf(OLED0, 2, 1, OLED_8X16, "Z:%-6d        ", mag_data.z);
            } else {
                vOledPrintf(OLED0, 1, 1, OLED_8X16, "Read Error!   ");
            }
            
            vOledUpdateAll();
        }
    }
*/
