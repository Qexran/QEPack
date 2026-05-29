/**
  * @file       kalman_controller.c
  * @author     Qe_xr
  * @version    V2.0.0
  * @date       2026/5/29
  * @brief      一阶卡尔曼滤波器（支持多实例）
  */

#include "kalman_controller.h"

void vKalmanInit(stKalmanFilterTdf *pstFilter, float Q, float R, float x0, float p0)
{
    pstFilter->X_Optimal = x0;
    pstFilter->P_Optimal = p0;
    pstFilter->Q = Q;
    pstFilter->R = R;
    pstFilter->IsInit = 0;
}

float fKalmanUpdate(stKalmanFilterTdf *pstFilter, float Z_Measure)
{
    pstFilter->X_Predict = pstFilter->X_Optimal;
    pstFilter->P_Predict = pstFilter->P_Optimal + pstFilter->Q;

    /* 防止除零：R 应为正数 */
    float fDenom = pstFilter->P_Predict + pstFilter->R;
    if (fDenom < 1e-9f) return pstFilter->X_Optimal;

    pstFilter->K = pstFilter->P_Predict / fDenom;
    pstFilter->X_Optimal = pstFilter->X_Predict + pstFilter->K * (Z_Measure - pstFilter->X_Predict);
    pstFilter->P_Optimal = (1 - pstFilter->K) * pstFilter->P_Predict;
    return pstFilter->X_Optimal;
}
