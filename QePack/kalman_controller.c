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
	static float X_Predict;	//预测值
	static float X_Optimal;	//最优值
	static float P_Predict;	//预测值协方差矩阵
	static float P_Optimal;	//最优质协方差矩阵
	static float K;			//卡尔曼增益
	static float Q;			//预测过程协方差
	static float R;			//测量过程协方差
	static uint8_t IsInit = 1;	//初始化标志位
	if(IsInit == 1)	//初始化操作
	{
		IsInit = 0;	//清除标志位
		X_Optimal = x0;	//初始化
		P_Optimal = p0;
		Q = Q_Input;
		R = R_Input;
	}
	//预测过程
	X_Predict = X_Optimal;		//预测值更新
	P_Predict = P_Optimal + Q;	//预测值协方差矩阵更新
	
	//更新过程
	K = P_Predict / (P_Predict + R);	//更新卡尔曼滤波增益	
	X_Optimal = X_Predict + K*(Z_Measure - X_Predict);	//更新最优估计值
	P_Optimal = (1 - K) * P_Predict;		//更新最优估计值的协方差矩阵
	return X_Optimal;			//返回最优估计值
}

