/**
  * @file       kalman_controller.h
  * @author     Qe_xr
  * @version    V2.0.0
  * @date       2026/5/29
  * @brief      一阶卡尔曼滤波器（支持多实例）
  */

#include "project_config.h"

#ifndef _KALMAN_CONTROLLER_H_
#define _KALMAN_CONTROLLER_H_

typedef struct {
    float X_Predict;
    float X_Optimal;
    float P_Predict;
    float P_Optimal;
    float K;
    float Q;
    float R;
    uint8_t IsInit;
} stKalmanFilterTdf;

/**
 * @brief  初始化卡尔曼滤波器实例
 * @param  pstFilter  滤波器实例指针
 * @param  Q          预测过程协方差
 * @param  R          测量过程协方差
 * @param  x0         初始状态最优值
 * @param  p0         初始状态最优值协方差值
 */
void vKalmanInit(stKalmanFilterTdf *pstFilter, float Q, float R, float x0, float p0);

/**
 * @brief  一阶卡尔曼标量方程
 * @param  pstFilter   滤波器实例指针
 * @param  Z_Measure   测量值
 * @return float       最优估计值
 */
float fKalmanUpdate(stKalmanFilterTdf *pstFilter, float Z_Measure);

#endif
