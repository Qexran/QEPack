/**
  * @file       motor_device.c
  * @author     Qe_xr
  * @version    V1.0.2
  * @date       2026/2/11
  * @brief      电机控制驱动，基于 STM32 HAL 库
  */

#include "motor_device.h"
#if MOTOR_IS_ENABLE

stMotorDeviceParamTdf astMotorDeviceParam[MOTOR_DEV_NUM];

/**
 * @brief 初始化电机静态参数
 * @param pstInit 电机静态参数指针
 * @param emDevNum 电机设备号
 */
void vMotorDeviceInit(stMotorStaticParamTdf *pstInit, emMotorDevNumTdf emDevNum)
{
    if (emDevNum >= MOTOR_DEV_NUM || pstInit == NULL) {
        return;
    }
    
    memcpy(&astMotorDeviceParam[emDevNum].stStaticParam, 
           pstInit, 
           sizeof(stMotorStaticParamTdf));
    
    memset(&astMotorDeviceParam[emDevNum].stRunningParam, 
           0, 
           sizeof(stMotorRunningParamTdf));
    

    // 启动电机定时器
    if (
        HAL_TIM_PWM_Start(
            astMotorDeviceParam[emDevNum].stStaticParam.pstPWM_htim, 
            pstInit->u32PWM_Channel
        ) 
        != HAL_OK) {
        while(1);
    }

}

/**
 * @brief 通过PWM占空比来设置电机速度
 * @param emDevNum 电机设备号
 * @param speed 电机速度，单位：PWM占空比
 */
void vMotorSetSpeed_by_PWM(emMotorDevNumTdf emDevNum, int16_t speed)
{
    if (emDevNum >= MOTOR_DEV_NUM) {
        return;
    }
    
    stMotorStaticParamTdf *pstStatic = &astMotorDeviceParam[emDevNum].stStaticParam;
    
    uint16_t absSpeed = (speed < 0) ? -speed : speed;
    
    // 控制电机方向
    if (speed >= 0) {
        HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_SET);
    }

    // 修改PWM占空比
    __HAL_TIM_SET_COMPARE(pstStatic->pstPWM_htim, pstStatic->u32PWM_Channel, absSpeed); 
}

#endif
