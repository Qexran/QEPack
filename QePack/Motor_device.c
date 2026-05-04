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
 * @brief  注册电机设备
 * @param  emDevNum ：电机设备号
 * @param  pstMotor ：电机设备指针（包含基类成员）
 * @note   注册电机设备后，即可使用其他函数控制该电机
 */
void vMotorRegisterDevice(uint8_t emDevNum, stMotorDeviceTdf *pstMotor)
{
    if (emDevNum < emMotorDevMax && pstMotor != NULL) {
        g_astMotorDevices[emDevNum] = pstMotor;
    }
}

/**
 * @brief  电机初始化
 * @param  emDevNum ：电机设备号
 * @note   调用虚函数 vInit(void *pstInit);  
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
 * @brief  电机周期执行
 * @param  emDevNum ：电机设备号
 * @note   调用虚函数 vPeriodExecute(void *pstPeriodExecute);  
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
 * @brief  停止电机
 * @param  emDevNum ：电机设备号
 * @param  bSyncFlag ：是否同步控制
 * @note   调用虚函数 vStop(void *pstStop, uint8_t bSyncFlag);  
 */
void vMotorStop(uint8_t emDevNum, uint8_t bSyncFlag)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vStop != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vStop((void*)g_astMotorDevices[emDevNum], bSyncFlag);
        }
    }
}

/**
 * @brief  电机使能控制
 * @param  emDevNum ：电机设备号
 * @param  bEnable ：使能状态
 * @param  bSyncFlag ：是否同步控制
 * @note   调用虚函数 vEnable(void *pstEnable, uint8_t bEnable, uint8_t bSyncFlag);  
 */
void vMotorEnable(uint8_t emDevNum, uint8_t bEnable, uint8_t bSyncFlag)
{
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vEnable != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vEnable((void*)g_astMotorDevices[emDevNum], bEnable, bSyncFlag);
        }
    }
}

/**
 * @brief  获取电机状态
 * @param  emDevNum ：电机设备号
 * @return emMotorStateTdf 电机状态
 * @note   调用虚函数 emGetState(void *pstGetState, uint8_t bSyncFlag);  
 */
emMotorStateTdf emGetMotorState(uint8_t emDevNum) {
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->emGetState != NULL) {
            return g_astMotorDevices[emDevNum]->pstVTable->emGetState((void*)g_astMotorDevices[emDevNum]);
        }
    }
    return emMotorStateNULL;
}

/**
 * @brief  电机速度控制
 * @param  emDevNum ：电机设备号
 * @param  emDir ：电机方向
 * @param  usVel ：电机速度（单位：mm/s）
 * @param  ucAcc ：电机加速度（单位：mm/s^2）
 * @param  bSyncFlag ：是否同步控制
 * @note   调用虚函数 vVelControl(...);  
 */
void vMotorVelControl(
    uint8_t emDevNum, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, 
    uint8_t bSyncFlag) {
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vVelControl != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vVelControl(
                (void*)g_astMotorDevices[emDevNum], emDir, usVel, ucAcc, bSyncFlag
            );
        }
    }
}

/**
 * @brief  电机位置控制
 * @param  emDevNum ：电机设备号
 * @param  emDir ：电机方向
 * @param  usVel ：电机速度（单位：mm/s）
 * @param  ucAcc ：电机加速度（单位：mm/s^2）
 * @param  ulClk ：电机位置（单位：步）
 * @param  bAbsFlag ：是否绝对位置（0：相对位置，1：绝对位置）
 * @param  bSyncFlag ：是否同步控制
 * @note   调用虚函数 vPosControl(...);  
 */
void vMotorPosControl(
    uint8_t emDevNum, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, 
    uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag) {
    if (emDevNum < emMotorDevMax && g_astMotorDevices[emDevNum] != NULL && g_astMotorDevices[emDevNum]->pstVTable != NULL) {
        if (g_astMotorDevices[emDevNum]->pstVTable->vPosControl != NULL) {
            g_astMotorDevices[emDevNum]->pstVTable->vPosControl(
                (void*)g_astMotorDevices[emDevNum], emDir, usVel, ucAcc, ulClk, bAbsFlag, bSyncFlag
            );
        }
    }
}


#endif
