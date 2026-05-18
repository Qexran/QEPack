/**
  * @file       ultrasonic_device.c
  * @author     Qe_xr
  * @version    V1.0.2
  * @date       2026/2/11
  * @brief      超声波测量驱动，基于 STM32 HAL 库
  * @note		思路: 通过占用定时器两个输入捕获通道(直接+间接)
  * @note		分别接收 Echo端 的上升沿和下降沿，这个定时器的计数值即为时间
  */

#include "kalman_controller.h"

/**
 * @brief  一阶卡尔曼标量方程
 * @param  Q_Input ：预测过程协方差
 * @param  R_Input ：测量过程协方差
 * @param  Z_Measure ：测量值
 * @param  x0 ：初始状态最优值
 * @param  p0 ：初始状态最优值协方差值
 * @return float ：最优估计值
 */
float Calc_Kalman(float Z_Measure, float Q_Input, float R_Input, float x0, float p0)
{
	static float X_Predict;
	static float X_Optimal;
	static float P_Predict;
	static float P_Optimal;
	static float K;
	static float Q;
	static float R;
	static uint8_t IsInit = 1;

	if(IsInit == 1)
	{
		IsInit = 0;
		X_Optimal = x0;
		P_Optimal = p0;
		Q = Q_Input;
		R = R_Input;
	}

	X_Predict = X_Optimal;
	P_Predict = P_Optimal + Q;

	/* 防止除零：R 应为正数 */
	float fDenom = P_Predict + R;
	if (fDenom < 1e-9f) return X_Optimal;

	K = P_Predict / fDenom;
	X_Optimal = X_Predict + K * (Z_Measure - X_Predict);
	P_Optimal = (1 - K) * P_Predict;
	return X_Optimal;
}

