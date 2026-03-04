#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"



// 可调整的参数使用宏定义
#define MPU6050_ADDRESS           0x68    /* I2C地址 */
#define MPU6050_TIMEOUT           100     /* I2C超时时间(ms) */
#define MPU6050_SAMPLE_RATE_DIV   0x04    /* 采样率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV) 原始值0x07 */
#define MPU6050_DLPF_CFG          0x06    /* 数字低通滤波器配置 无参数*/
#define MPU6050_GYRO_FS_SEL       0x00    /* 陀螺仪满量程选择(0:±250°/s, 1:±500°/s, 2:±1000°/s, 3:±2000°/s) */
#define MPU6050_GYRO_SENSITIVITY  131.0f  /* 陀螺仪灵敏度系数(根据FS_SEL选择) */
#define MPU6050_ACCEL_FS_SEL      0x00    /* 加速度计满量程选择(0:±2g, 1:±4g, 2:±8g, 3:±16g) */
#define MPU6050_ACCEL_SENSITIVITY 16384.0f /* 加速度计灵敏度系数(根据FS_SEL选择) */
#define Q_ANGLE                   0.001f  /* 角度过程噪声协方差 */
#define Q_BIAS                    0.003f  /* 偏置过程噪声协方差 */
#define R_MEASURE                 0.01f   /* 测量噪声协方差 */

// 寄存器地址定义
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

// 数据结构体
typedef struct {
    int16_t GX_raw;       /* 原始X轴角速度 */
    int16_t GY_raw;       /* 原始Y轴角速度 */
    int16_t GZ_raw;       /* 原始Z轴角速度 */
    int16_t AX_raw;       /* 原始X轴加速度 */
    int16_t AY_raw;       /* 原始Y轴加速度 */
    int16_t AZ_raw;       /* 原始Z轴加速度 */

    float GX;             /* 转换后X轴角速度(°/s) */
    float GY;             /* 转换后Y轴角速度(°/s) */
    float GZ;             /* 转换后Z轴角速度(°/s) */
    float AX;             /* 转换后X轴加速度(g) */
    float AY;             /* 转换后Y轴加速度(g) */
    float AZ;             /* 转换后Z轴加速度(g) */

    float roll;           /* 横滚角(°) */
    float pitch;          /* 俯仰角(°) */
    float yaw;            /* 偏航角(°) */

    float roll_kalman;    /* 卡尔曼滤波后的横滚角(°) */
    float pitch_kalman;   /* 卡尔曼滤波后的俯仰角(°) */
    float yaw_kalman;     /* 卡尔曼滤波后的偏航角(°) */

    float gyro_bias_x;    /* X轴陀螺仪偏置 */
    float gyro_bias_y;    /* Y轴陀螺仪偏置 */
    float gyro_bias_z;    /* Z轴陀螺仪偏置 */

    float dt;             /* 采样时间间隔(s) */
    uint32_t last_time;   /* 上次采样时间(ms) */
} MPU6050_Data_t;

// 卡尔曼滤波器结构体
typedef struct {
    float q_angle;        /* 角度过程噪声协方差 */
    float q_bias;         /* 偏置过程噪声协方差 */
    float r_measure;      /* 测量噪声协方差 */

    float angle;          /* 估计角度 */
    float bias;           /* 估计偏置 */
    float rate;           /* 无偏角速度 */

    float P[2][2];        /* 误差协方差矩阵 */
} Kalman_Filter_t;

// 四元数结构体
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion_t;

// MPU6050实例结构体
typedef struct {
    I2C_HandleTypeDef *hi2c;
    MPU6050_Data_t data;
    Kalman_Filter_t kalman_roll;
    Kalman_Filter_t kalman_pitch;
    Quaternion_t quaternion; // 添加四元数成员
} MPU6050_Instance_t;

// 函数声明
uint8_t MPU6050_Init(MPU6050_Instance_t *mpu);
uint8_t MPU6050_ReadRawData(MPU6050_Instance_t *mpu);
uint8_t MPU6050_ConvertData(MPU6050_Instance_t *mpu);
uint8_t MPU6050_CalculateAngle(MPU6050_Instance_t *mpu);
uint8_t MPU6050_ApplyKalmanFilter(MPU6050_Instance_t *mpu);
void Kalman_Filter_Init(Kalman_Filter_t *kalman_filter, float q_angle, float q_bias, float r_measure);
float Kalman_Filter_Update(Kalman_Filter_t *kalman_filter, float new_angle, float new_rate, float dt);
void MPU6050_CalculateQuaternion(MPU6050_Instance_t *mpu); //四元数计算函数

#endif 
/* main.c使用示例

int main(void)
{
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    // 模块初始化
    OLED_Init();
    Encoder_Init();
    TB6612_Init(&left_motor);
    TB6612_Init(&right_motor);
    MPU6050_Init(&mpu);

    // 直立行走控制初始化
    UprightWalking_Init(&upright_walking, &mpu, &left_motor, &right_motor, 0.0f, 50);

    // 延时等待系统稳定
    HAL_Delay(500);

    // 测试转向功能
    OLED_ShowString(1, 1, "Turning 90 deg...");
    UprightWalking_Turn(&upright_walking, 90.0f);  // 右转90度
    
    while (1)
    {
        // 读取原始数据
        if (MPU6050_ReadRawData(&mpu) != 0) {
            Serial_Printf("READ_RAW_DATA_FAILED!\r\n");
        }

        // 转换数据
        MPU6050_ConvertData(&mpu);

        // 计算角度
        MPU6050_CalculateAngle(&mpu);

        // 应用卡尔曼滤波
        MPU6050_ApplyKalmanFilter(&mpu);

        // 计算四元数
        MPU6050_CalculateQuaternion(&mpu);

        // 输出四元数
        Serial_Printf("Quaternion: %.4f, %.4f, %.4f, %.4f\r\n",
                      mpu.quaternion.w,
                      mpu.quaternion.x,
                      mpu.quaternion.y,
                      mpu.quaternion.z);

        // 直立行走控制
        UprightWalking_Control(&upright_walking);

        // 显示调试信息
        OLED_ShowString(1, 1, "Pitch:");
        OLED_ShowSignedNum(1, 7, (int16_t)(mpu.data.pitch_kalman * 100), 6);

        OLED_ShowString(2, 1, "Yaw:");
        OLED_ShowSignedNum(2, 5, (int16_t)(upright_walking.current_yaw * 100), 6);
        
        OLED_ShowString(3, 1, "Left:");
        OLED_ShowSignedNum(3, 6, upright_walking.left_out, 6);

        OLED_ShowString(4, 1, "Right:");
        OLED_ShowSignedNum(4, 7, upright_walking.right_out, 6);

        // 显示转向状态
        if (upright_walking.turn_status == TURN_IN_PROGRESS) {
            OLED_ShowString(5, 1, "Turning...");
        } else if (upright_walking.turn_status == TURN_COMPLETED) {
            OLED_ShowString(5, 1, "Turn Complete!");
        } else {
            OLED_ShowString(5, 1, "Idle        ");
        }

        HAL_Delay(UPDATE_INTERVAL);  // 延时
    }
}
*/
