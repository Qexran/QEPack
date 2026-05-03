/**
  * @file       motor_device.c
  * @author     Qe_xr
  * @version    V1.1.0
  * @date       2026/4/27
  * @brief      电机控制基类实现
  */

#include "motor_device.h"

#if MOTOR_IS_ENABLE

/* 全局电机设备指针数组 */
stMotorDeviceTdf *g_astMotorDevices[emMotorDevMax];

/**
 * @brief          注册电机设备
 * @param  emDevNum ：电机设备号
 * @param  pstMotor ：电机设备指针（包含基类成员）
 */
void vMotorRegisterDevice(uint8_t emDevNum, stMotorDeviceTdf *pstMotor)
{
    if (emDevNum < emMotorDevMax && pstMotor != NULL) {
        g_astMotorDevices[emDevNum] = pstMotor;
    }
}

/**
 * @brief          电机初始化
 * @param  emDevNum ：电机设备号
 */
void vMotorInit(uint8_t emDevNum)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vInit != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vInit((void*)g_astMotorDevices[emDevNum]);
        }
    }
}

/**
 * @brief          电机周期执行
 * @param  emDevNum ：电机设备号
 */
void vMotorPeriodExecute(uint8_t emDevNum)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vPeriodExecute != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vPeriodExecute((void*)g_astMotorDevices[emDevNum]);
        }
    }
}

/**
 * @brief          设置电机速度
 * @param  emDevNum ：电机设备号
 * @param  speed ：速度值
 */
void vMotorSetSpeed(uint8_t emDevNum, int16_t speed)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vSetSpeed != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vSetSpeed((void*)g_astMotorDevices[emDevNum], speed);
        }
    }
}

/**
 * @brief          停止电机
 * @param  emDevNum ：电机设备号
 */
void vMotorStop(uint8_t emDevNum)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vStop != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vStop((void*)g_astMotorDevices[emDevNum]);
        }
    }
}

/**
 * @brief          电机使能控制
 * @param  emDevNum ：电机设备号
 * @param  bEnable ：使能状态
 */
void vMotorEnable(uint8_t emDevNum, uint8_t bEnable)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vEnable != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vEnable((void*)g_astMotorDevices[emDevNum], bEnable);
        }
    }
}

/**
 * @brief          获取电机状态
 * @param  emDevNum ：电机设备号
 * @return emMotorStateTdf 电机状态
 */
emMotorStateTdf emGetMotorState(uint8_t emDevNum) {
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->emGetState != NULL) {
            return g_astMotorDevices[emDevNum]->pstVTable->emGetState((void*)g_astMotorDevices[emDevNum]);
        }
    }
    return emMotorStateNULL;
}


#endif
