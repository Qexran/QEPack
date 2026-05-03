/** 
  * @file       motor_system_controller.c 
  * @author     QePack
  * @version    V1.0.0
  * @date       2026/4/30
  * @brief      电机系统控制器
  */

#include "motor_system_controller.h"

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

stMotorSystemParamTdf g_stMotorSystemController;

/**
 * @brief  麦克纳姆轮速度解算接口
 * @param  vx:     横向速度 (左+右-)，单位 rpm（位置模式时为方向符号）
 * @param  vy:     前后速度 (前+后-)，单位 rpm（位置模式时为方向符号）
 * @param  w:      旋转角速度，单位 rpm（正为逆时针，负为顺时针）
 * @param  speed:  输出数组，speed[1-4]为四个轮子的最终速度
 * @return None
 * @note   纯X走：vy=0, w=0
 *         纯Y走：vx=0, w=0  
 *         原地转：vx=0, vy=0
 *         边走边转：三者都不为0
 */
static void vMecanumKinematics(int vx, int vy, int w, int *speed)
{
    speed[1] = +vx + vy - w;   // 右前轮
    speed[2] = -vx + vy + w;   // 左前轮
    speed[3] = -vx + vy - w;   // 左后轮
    speed[4] = +vx + vy + w;   // 右后轮
}

/**
 * @brief 获取电机运动状态
 * @return emMotorStateTdf 电机运动状态
 */
emMotorStateTdf emGetMotorSystemState() {
    return g_stMotorSystemController.stRunningParam.emMotorSystemState;
}

/**
 * @brief 设置电机系统运动状态
 * @param emState 电机系统运动状态
 */
static void vMotorSystemSetState(emMotorStateTdf emState) {
    g_stMotorSystemController.stRunningParam.emMotorSystemState = emState;
}

/**
 * @brief 获取电机系统控制器参数
 * @return stMotorSystemParamTdf* 电机系统控制器参数指针
 */
const stMotorSystemParamTdf* c_pstGetMotorSystemControllerParam(void)
{
    return &g_stMotorSystemController;
}


/**
 * @brief 初始化电机系统控制器
 * @param pstInit 静态参数指针
 */
void vMotorSystemInit(stMotorSystemStaticParamTdf *pstInit)
{
    stMotorSystemRunningParamTdf *pstRunning = &g_stMotorSystemController.stRunningParam;
    stMotorSystemStaticParamTdf  *pstStatic = g_stMotorSystemController.stStaticParam;

    if (pstInit != NULL) {
        pstStatic = pstInit;
    }

    memset(pstRunning, 
           0, 
           sizeof(stMotorSystemRunningParamTdf)
        );
    
}

/**
 * @brief 使能电机系统控制器
 * @param bEnable 使能状态
 */
void vMotorSystemEnable(uint8_t bEnable)
{
    stMotorSystemStaticParamTdf  *pstStatic = g_stMotorSystemController.stStaticParam;

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
        case emChassisMecanum4:
            vMotorEnable(pstStatic->emLeftBackMotorDevNum, bEnable);
            vMotorEnable(pstStatic->emRightBackMotorDevNum, bEnable);

        case emChassisDiff2:
            if(pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorEnable(pstStatic->emLeftFrontMotorDevNum, bEnable);
            if(pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorEnable(pstStatic->emRightFrontMotorDevNum, bEnable);

            break;
        default:
            break;
    }
}

/**
 * @brief 停止电机系统控制器
 */
void vMotorSystemStop(void)
{
    stMotorSystemStaticParamTdf  *pstStatic = g_stMotorSystemController.stStaticParam;

    vMotorSystemSetState(emMotorStateStop);

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
        case emChassisMecanum4:
            if(pstStatic->emLeftBackMotorDevNum != emNoMotorDevNum)
                vMotorStop(pstStatic->emLeftBackMotorDevNum);
            if(pstStatic->emRightBackMotorDevNum != emNoMotorDevNum)
                vMotorStop(pstStatic->emRightBackMotorDevNum);

        case emChassisDiff2:
            if(pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorStop(pstStatic->emLeftFrontMotorDevNum);
            if(pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorStop(pstStatic->emRightFrontMotorDevNum);
            break;
        default:
            break;
    }
}

/**
 * @brief 设置电机系统控制器速度
 * @param fSpeed 速度，单位：rpm
 */
void vMotorSystemSetSpeed(
    float fLeftFrontSpeed, float fRightFrontSpeed, 
    float fLeftBackSpeed,  float fRightBackSpeed
) {
    stMotorSystemStaticParamTdf  *pstStatic = g_stMotorSystemController.stStaticParam;

    switch (pstStatic->emChassisType) {
        case emChassisDiff4:
            if(pstStatic->emLeftBackMotorDevNum != emNoMotorDevNum)
                vMotorSetSpeed(pstStatic->emLeftBackMotorDevNum, (int16_t)fLeftBackSpeed);
            if(pstStatic->emRightBackMotorDevNum != emNoMotorDevNum)
                vMotorSetSpeed(pstStatic->emRightBackMotorDevNum, (int16_t)fRightBackSpeed);

        case emChassisDiff2:
            if(pstStatic->emLeftFrontMotorDevNum != emNoMotorDevNum)
                vMotorSetSpeed(pstStatic->emLeftFrontMotorDevNum, (int16_t)fLeftFrontSpeed);
            if(pstStatic->emRightFrontMotorDevNum != emNoMotorDevNum)
                vMotorSetSpeed(pstStatic->emRightFrontMotorDevNum, (int16_t)fRightFrontSpeed);
            
            break;
        case emChassisMecanum4:
            
            break;
        default:
            break;
    } 
}

/**
 * @brief 设置电机系统控制器位置
 * @param fTargetXCm 目标X轴位置，单位：cm
 */
void vMotorSystemSetPosition(float fTargetCm)
{

}
/**
 * @brief 设置麦轮X轴行走位置
 * @param fTargetXCm 目标X轴位置，单位：cm
 */
void vMotorSystemSetPositionX(float fTargetXCm)
{

}

/**
 * @brief 设置麦轮Y轴行走位置
 * @param fTargetYMm 目标Y轴位置，单位：m
 * @param fSpeedMmS 速度，单位：m/s
 */
void vMotorSystemSetPositionY(float fTargetYCm)
{

}

/**
 * @brief 设置电机系统控制器姿态
 * @param fTargetYawDeg 目标角度，单位：度
 * @param fOmegaRadS 角速度，单位：rad/s
 */
void vMotorSystemSetPose(float fTargetYawDeg, float fOmegaRadS)
{
    
}



/**
 * @brief 电机系统控制器周期执行
 */
void vMotorSystemPeriodExecute(void)
{

}


#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */