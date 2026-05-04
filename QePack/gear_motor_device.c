/**
  * @file       gear_motor_device.c
  * @author     Qe_xr
  * @version    V1.0.2
  * @date       2026/2/11
  * @brief      直流减速电机控制驱动
  */

#include "gear_motor_device.h"

#if GEAR_MOTOR_IS_ENABLE

stGearMotorDeviceParamTdf astGearMotorDeviceParam[GEAR_MOTOR_DEV_NUM];

/**
 * @brief 减速电机虚方法表
 */
static stMotorVTableTdf g_stGearMotorVTable = {
    vGearMotorInit,              /* 对应vInit(void *pstInit); */
    vGearMotorPeriodExecute,     /* 对应vPeriodExecute(void *pstMotor); */
    vGearMotorStop,              /* 对应vStop(void *pstMotor, uint8_t bSyncFlag); */
    vGearMotorEnable,            /* 对应vEnable(void *pstEnable); */
    emGetGearMotorState,         /* 对应emGetMotorState(void *pstMotor); */
    vGearMotorPosControl,        /* 对应vPosControl(...); */
    vGearMotorVelControl,        /* 对应vVelControl(...); */
};

/* 虚方法实现 ************************************* */
/**
 * @brief 初始化减速电机静态参数
 * @param pstInit 减速电机静态参数指针
 * @note   实现父类方法 vInit(void *pstInit);
 */
void vGearMotorInit(void *pstInit) {
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstInit;
    if (pstGearMotor == NULL) {
        return;
    }
    
    // 启动电机定时器
    #if (QEPACK_PLATFORM == TI)
        DL_Timer_startCounter(
            pstGearMotor->stStaticParam.stTimer->timer_inst
        );
    #else
        if (
            HAL_TIM_PWM_Start(
                pstGearMotor->stStaticParam.pstPWM_htim, 
                pstGearMotor->stStaticParam.u32PWM_Channel
            ) 
            != HAL_OK) {
            while(1);
        }
    #endif
}

/**
 * @brief 减速电机周期执行
 * @param pstMotor 减速电机设备指针
 * @note   实现父类方法 vPeriodExecute(void *pstMotor);
 */
void vGearMotorPeriodExecute(void *pstMotor) {
    (void)pstMotor; // 未使用
    // 减速电机不需要周期执行
}


/**
 * @brief 减速电机停止控制
 * @param pstMotor 减速电机设备指针
 * @param bSyncFlag 多机同步标志（减速电机不支持同步，此参数预留）
 */
void vGearMotorStop(void *pstMotor, uint8_t bSyncFlag)
{
    vGearMotorSetSpeed(pstMotor, 0);
}

/**
 * @brief 减速电机使能控制
 * @param pstMotor 减速电机设备指针
 * @param bEnable 使能状态
 * @param bSyncFlag 多机同步标志（减速电机不支持同步，此参数预留）
 */
void vGearMotorEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag)
{
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstMotor;
    stGearMotorStaticParamTdf *pstStatic = &pstGearMotor->stStaticParam;
    
    if(pstStatic->pstStbyGpioBase == NULL) {
        return;
    }

    if (!bEnable) {
        // 使待机引脚为低电平
        #if (QEPACK_PLATFORM == TI)
            TI_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_RESET);
        #else
            HAL_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_RESET);
        #endif
    } else {
        // 使待机引脚为高电平
        #if (QEPACK_PLATFORM == TI)
            TI_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_SET);
        #else
            HAL_GPIO_WritePin(pstStatic->pstStbyGpioBase,pstStatic->u32StbyPin,GPIO_PIN_SET);
        #endif
    }
}

/**
 * @brief 获取减速电机运动状态
 * @param pstMotor 减速电机设备指针
 * @return emMotorStateTdf 电机运动状态
 */
emMotorStateTdf emGetGearMotorState(void *pstMotor) {
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstMotor;

    return pstGearMotor->stBase.emMotorState;
}

/**
 * @brief 减速电机位置控制
 * @param pstMotor 减速电机设备指针
 * @param emDir 电机方向
 * @param usVel 速度(RPM)，范围0 - ARR RPM
 * @param ucAcc 加速度，范围0 - 255，注意：0是直接启动
 * @param ulClk 脉冲数，范围0- (2^32 - 1)个
 * @param bAbsFlag 是否绝对绝对位置
 * @param bSyncFlag 是否同步控制
 */
void vGearMotorPosControl(
    void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, 
    uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag) {
    (void)pstMotor; // 未使用
    // 减速电机不需要位置控制
}

/**
 * @brief 减速电机速度控制
 * @param pstMotor 减速电机设备指针
 * @param emDir 电机方向
 * @param usVel 电机速度，单位：RPM
 * @param ucAcc 加速度，范围0 - 255，注意：0是直接启动
 * @param bSyncFlag 是否同步控制
 */
void vGearMotorVelControl(
    void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, 
    uint8_t bSyncFlag) {
    (void)pstMotor; // 未使用
    // 减速电机不需要速度控制
}

/* ********************************************** */


/**
 * @brief 通过PWM占空比来设置减速电机速度
 * @param pstMotor 减速电机设备指针
 * @param speed 电机速度，单位：PWM占空比
 */
void vGearMotorSetSpeed(void *pstMotor, int16_t speed)
{
    stGearMotorDeviceParamTdf *pstGearMotor = (stGearMotorDeviceParamTdf *)pstMotor;
    if (pstGearMotor == NULL) {
        return;
    }
    
    stGearMotorStaticParamTdf *pstStatic = &pstGearMotor->stStaticParam;
    
    uint16_t absSpeed = (speed < 0) ? -speed : speed;
    
    // 控制电机方向
    #if (QEPACK_PLATFORM == TI)
        if (speed > 0) {
            TI_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_SET);
            TI_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        } else {
            TI_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            TI_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_SET);
        }

        if(speed == 0){
            TI_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            TI_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        }
    #else
        if (speed >= 0) {
            HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(pstStatic->pstDir1GpioBase, pstStatic->u32DirPin1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(pstStatic->pstDir2GpioBase, pstStatic->u32DirPin2, GPIO_PIN_SET);
        }
    #endif

    // 修改PWM占空比
    #if (QEPACK_PLATFORM == TI)
        DL_Timer_setCaptureCompareValue(
            pstStatic->stTimer->timer_inst, 
            absSpeed, 
            pstStatic->emChannel
        );
    #else
        __HAL_TIM_SET_COMPARE(pstStatic->pstPWM_htim, pstStatic->u32PWM_Channel, absSpeed); 
    #endif
}





/**
 * @brief 注册减速电机设备
 * @param emDevNum 设备号
 * @param pstInit 静态参数
 */
void vGearMotorRegister(emMotorDevNumTdf emDevNum, stGearMotorStaticParamTdf *pstInit)
{
    emMotorDevNumTdf offsetDevNum =  (emMotorDevNumTdf) (emDevNum - emGearMotorDevNum0);
    
    if (offsetDevNum < GEAR_MOTOR_DEV_NUM && pstInit != NULL) {
        
        // 初始化基类
        astGearMotorDeviceParam[offsetDevNum].stBase.emType = emMotorType_Gear;
        astGearMotorDeviceParam[offsetDevNum].stBase.pstVTable = &g_stGearMotorVTable;

        memcpy(&astGearMotorDeviceParam[offsetDevNum].stStaticParam, 
           pstInit, 
           sizeof(stGearMotorStaticParamTdf));
    
        memset(&astGearMotorDeviceParam[offsetDevNum].stRunningParam, 
            0, 
            sizeof(stGearMotorRunningParamTdf));
        
        // 注册到基类
        vMotorRegisterDevice(emDevNum, &astGearMotorDeviceParam[offsetDevNum].stBase);
    }
}

#endif
