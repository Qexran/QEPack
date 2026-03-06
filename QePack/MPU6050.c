#include "MPU6050.h"

#if MPU6050_ENABLE

#include "math.h"

#define PI 3.1415926535897932384626433832795f
#define M_PI 3.14159265358979323846f

/**
 * @brief          I2C通信函数
 * @note           用于写入和读取MPU6050的寄存器
 */
static uint8_t MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t reg_data) {
    uint8_t tx_data[2] = {reg_addr, reg_data};
    return HAL_I2C_Master_Transmit(hi2c, (MPU6050_ADDRESS << 1), tx_data, 2, MPU6050_TIMEOUT);
}

static uint8_t MPU6050_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *reg_data) {
    return HAL_I2C_Master_Transmit(hi2c, (MPU6050_ADDRESS << 1), &reg_addr, 1, MPU6050_TIMEOUT) ||
           HAL_I2C_Master_Receive(hi2c, (MPU6050_ADDRESS << 1) | 1, reg_data, 1, MPU6050_TIMEOUT);
}

static uint8_t MPU6050_ReadMultiReg(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *reg_data, uint8_t length) {
    return HAL_I2C_Master_Transmit(hi2c, (MPU6050_ADDRESS << 1), &reg_addr, 1, MPU6050_TIMEOUT) ||
           HAL_I2C_Master_Receive(hi2c, (MPU6050_ADDRESS << 1) | 1, reg_data, length, MPU6050_TIMEOUT);
}


/**
 * @brief          初始化MPU6050
 * @note           配置MPU6050的电源管理、采样率、数字低通滤波器、陀螺仪和加速度计满量程
 * @param          mpu ：MPU6050实例指针
 * @return         uint8_t 初始化状态，0表示成功
 */
uint8_t MPU6050_Init(MPU6050_Instance_t *mpu) {
    uint8_t device_id;
    uint8_t status = 0;

    // 读取设备ID
    status |= MPU6050_ReadReg(mpu->hi2c, MPU6050_WHO_AM_I, &device_id);

    if (device_id != 0x68) {
       // Serial_Printf("DEVICE_ID_ERROR!\r\n");
        return 1; /* 设备ID错误 */
    }

    // 唤醒设备
    status |= MPU6050_WriteReg(mpu->hi2c, MPU6050_PWR_MGMT_1, 0x00);
    // 设置采样率分频
    status |= MPU6050_WriteReg(mpu->hi2c, MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_RATE_DIV);
    // 设置数字低通滤波器
    status |= MPU6050_WriteReg(mpu->hi2c, MPU6050_CONFIG, MPU6050_DLPF_CFG);
    // 设置陀螺仪满量程
    status |= MPU6050_WriteReg(mpu->hi2c, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_SEL << 3);
    // 设置加速度计满量程
    status |= MPU6050_WriteReg(mpu->hi2c, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL << 3);
    // 初始化卡尔曼滤波器
    Kalman_Filter_Init(&mpu->kalman_roll, Q_ANGLE, Q_BIAS, R_MEASURE);
    Kalman_Filter_Init(&mpu->kalman_pitch, Q_ANGLE, Q_BIAS, R_MEASURE);

    // 初始化四元数
    mpu->quaternion.w = 1.0f;
    mpu->quaternion.x = 0.0f;
    mpu->quaternion.y = 0.0f;
    mpu->quaternion.z = 0.0f;

    // 初始化时间
    mpu->data.last_time = HAL_GetTick();

    return status;
}

/**
 * @brief          读取原始数据
 * @note           从MPU6050读取加速度、温度和陀螺仪的原始数据
 * @param          mpu ：MPU6050实例指针
 * @return         uint8_t 读取状态，0表示成功
 */
uint8_t MPU6050_ReadRawData(MPU6050_Instance_t *mpu) {
    uint8_t buffer[14];
    uint8_t status = 0;

    // 计算采样时间间隔
    uint32_t current_time = HAL_GetTick();
    mpu->data.dt = (current_time - mpu->data.last_time) / 1000.0f;
    mpu->data.last_time = current_time;

    // 读取加速度、温度和陀螺仪数据
    status = MPU6050_ReadMultiReg(mpu->hi2c, MPU6050_ACCEL_XOUT_H, buffer, 14);


    // 合并高低字节
    mpu->data.AX_raw = (buffer[0] << 8) | buffer[1];
    mpu->data.AY_raw = (buffer[2] << 8) | buffer[3];
    mpu->data.AZ_raw = (buffer[4] << 8) | buffer[5];
    mpu->data.GX_raw = (buffer[8] << 8) | buffer[9];
    mpu->data.GY_raw = (buffer[10] << 8) | buffer[11];
    mpu->data.GZ_raw = (buffer[12] << 8) | buffer[13];

    return status;
}

/**
 * @brief          转换原始数据为实际物理值
 * @note           从MPU6050读取的原始数据转换为实际的角速度和加速度值
 * @param          mpu ：MPU6050实例指针
 * @return         uint8_t 转换状态，0表示成功
 */
uint8_t MPU6050_ConvertData(MPU6050_Instance_t *mpu) {
    // 转换为实际角速度(°/s)
    mpu->data.GX = (float)mpu->data.GX_raw / MPU6050_GYRO_SENSITIVITY - mpu->data.gyro_bias_x;
    mpu->data.GY = (float)mpu->data.GY_raw / MPU6050_GYRO_SENSITIVITY - mpu->data.gyro_bias_y;
    mpu->data.GZ = (float)mpu->data.GZ_raw / MPU6050_GYRO_SENSITIVITY - mpu->data.gyro_bias_z;

    // 转换为实际加速度(g)
    mpu->data.AX = (float)mpu->data.AX_raw / MPU6050_ACCEL_SENSITIVITY;
    mpu->data.AY = (float)mpu->data.AY_raw / MPU6050_ACCEL_SENSITIVITY;
    mpu->data.AZ = (float)mpu->data.AZ_raw / MPU6050_ACCEL_SENSITIVITY;

    return 0;
}

/**
 * @brief          计算欧拉角
 * @note           使用加速度和陀螺仪数据计算 roll、pitch 和 yaw 角度
 * @param          mpu ：MPU6050实例指针
 * @return         uint8_t 计算状态，0表示成功
 */
uint8_t MPU6050_CalculateAngle(MPU6050_Instance_t *mpu) {
    // 使用加速度计算静态角度
    float accel_roll = atan2(mpu->data.AY, mpu->data.AZ) * 180.0f / M_PI;
    float accel_pitch = atan2(-mpu->data.AX, sqrt(mpu->data.AY * mpu->data.AY + mpu->data.AZ * mpu->data.AZ)) * 180.0f / M_PI;

    // 互补滤波融合角速度和加速度数据
    mpu->data.roll = 0.88f * (mpu->data.roll + mpu->data.GX * mpu->data.dt) + 0.02f * accel_roll;
    mpu->data.pitch = 0.88f * (mpu->data.pitch + mpu->data.GY * mpu->data.dt) + 0.02f * accel_pitch;
    mpu->data.yaw += mpu->data.GZ * mpu->data.dt;

    // 限制偏航角范围
    if (mpu->data.yaw > 180.0f) mpu->data.yaw -= 360.0f;
    if (mpu->data.yaw < -180.0f) mpu->data.yaw += 360.0f;

    return 0;
}

/**
 * @brief          应用卡尔曼滤波
 * @note           使用加速度和陀螺仪数据，通过卡尔曼滤波融合得到 roll、pitch 和 yaw 角度
 * @param          mpu ：MPU6050实例指针
 * @return         uint8_t 计算状态，0表示成功
 */
uint8_t MPU6050_ApplyKalmanFilter(MPU6050_Instance_t *mpu) {
    // 使用加速度计算静态角度
    float accel_roll = atan2(mpu->data.AY, mpu->data.AZ) * 180.0f / M_PI;
    float accel_pitch = atan2(-mpu->data.AX, sqrt(mpu->data.AY * mpu->data.AY + mpu->data.AZ * mpu->data.AZ)) * 180.0f / M_PI;

    // 应用卡尔曼滤波
    mpu->data.roll_kalman = Kalman_Filter_Update(&mpu->kalman_roll, accel_roll, mpu->data.GX, mpu->data.dt);
    mpu->data.pitch_kalman = Kalman_Filter_Update(&mpu->kalman_pitch, accel_pitch, mpu->data.GY, mpu->data.dt);
    mpu->data.yaw_kalman += mpu->data.GZ * mpu->data.dt;

    // 限制偏航角范围
    if (mpu->data.yaw_kalman > 180.0f) mpu->data.yaw_kalman -= 360.0f;
    if (mpu->data.yaw_kalman < -180.0f) mpu->data.yaw_kalman += 360.0f;

    return 0;
}

/**
 * @brief          初始化卡尔曼滤波器
 * @note           初始化卡尔曼滤波器的参数，包括角度、偏置和误差协方差矩阵
 * @param          kalman_filter ：卡尔曼滤波器实例指针
 * @param          q_angle ：角度的过程噪声方差
 * @param          q_bias ：偏置的过程噪声方差
 * @param          r_measure ：测量噪声方差
 */
void Kalman_Filter_Init(Kalman_Filter_t *kalman_filter, float q_angle, float q_bias, float r_measure) {
    kalman_filter->q_angle = q_angle;
    kalman_filter->q_bias = q_bias;
    kalman_filter->r_measure = r_measure;

    kalman_filter->angle = 0.0f;
    kalman_filter->bias = 0.0f;

    // 初始化误差协方差矩阵
    kalman_filter->P[0][0] = 1.0f;
    kalman_filter->P[0][1] = 0.0f;
    kalman_filter->P[1][0] = 0.0f;
    kalman_filter->P[1][1] = 1.0f;
}

/**
 * @brief          更新卡尔曼滤波器
 * @note           使用新的角度和角速度数据，更新卡尔曼滤波器的角度和偏置
 * @param          kalman_filter ：卡尔曼滤波器实例指针
 * @param          new_angle ：新的角度测量值
 * @param          new_rate ：新的角速度测量值
 * @param          dt ：时间间隔
 * @return         float 融合后的角度值
 */
float Kalman_Filter_Update(Kalman_Filter_t *kalman_filter, float new_angle, float new_rate, float dt) {
    // 预测步骤
    kalman_filter->rate = new_rate - kalman_filter->bias;
    kalman_filter->angle += dt * kalman_filter->rate;

    // 更新误差协方差矩阵
    kalman_filter->P[0][0] += dt * (dt * kalman_filter->P[1][1] - kalman_filter->P[0][1] - kalman_filter->P[1][0] + kalman_filter->q_angle);
    kalman_filter->P[0][1] -= dt * kalman_filter->P[1][1];
    kalman_filter->P[1][0] -= dt * kalman_filter->P[1][1];
    kalman_filter->P[1][1] += kalman_filter->q_bias * dt;

    // 计算卡尔曼增益
    float S = kalman_filter->P[0][0] + kalman_filter->r_measure;
    float K[2];
    K[0] = kalman_filter->P[0][0] / S;
    K[1] = kalman_filter->P[1][0] / S;

    // 测量更新
    float y = new_angle - kalman_filter->angle;
    kalman_filter->angle += K[0] * y;
    kalman_filter->bias += K[1] * y;

    // 更新误差协方差矩阵
    float P00_temp = kalman_filter->P[0][0];
    float P01_temp = kalman_filter->P[0][1];

    kalman_filter->P[0][0] -= K[0] * P00_temp;
    kalman_filter->P[0][1] -= K[0] * P01_temp;
    kalman_filter->P[1][0] -= K[1] * P00_temp;
    kalman_filter->P[1][1] -= K[1] * P01_temp;

    return kalman_filter->angle;
}

/**
 * @brief          计算四元数
 * @note           使用陀螺仪数据和当前四元数，计算新的四元数
 * @param          mpu ：MPU6050实例指针
 */
void MPU6050_CalculateQuaternion(MPU6050_Instance_t *mpu) {
    float gx = mpu->data.GX * M_PI / 180.0f; // 转换为弧度
    float gy = mpu->data.GY * M_PI / 180.0f;
    float gz = mpu->data.GZ * M_PI / 180.0f;
    float dt = mpu->data.dt;

    float qw = mpu->quaternion.w;
    float qx = mpu->quaternion.x;
    float qy = mpu->quaternion.y;
    float qz = mpu->quaternion.z;

    // 四元数更新公式
    qw += (-qx * gx - qy * gy - qz * gz) * (dt / 2);
    qx += (qw * gx + qy * gz - qz * gy) * (dt / 2);
    qy += (qw * gy - qx * gz + qz * gx) * (dt / 2);
    qz += (qw * gz + qx * gy - qy * gx) * (dt / 2);

    // 归一化四元数
    float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    mpu->quaternion.w = qw / norm;
    mpu->quaternion.x = qx / norm;
    mpu->quaternion.y = qy / norm;
    mpu->quaternion.z = qz / norm;
}

#endif
