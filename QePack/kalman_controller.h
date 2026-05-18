/**
  * @file       kalman_controller.h
  * @author     Qe_xr
  * @version    V1.0.1
  * @date       2026/1/20
  * @brief      一阶卡尔曼滤波器
  */

#include "project_config.h"

#ifndef _KALMAN_CONTROLLER_H_
#define _KALMAN_CONTROLLER_H_

float Calc_Kalman(float Z_Measure, float Q_Input, float R_Input, float x0, float p0);

#endif
