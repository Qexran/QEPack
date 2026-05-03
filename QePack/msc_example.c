#if 0
/** 
  * @file       msc_example.c 
  * @author     QePack Team
  * @version    V1.0.0
  * @date       2026/4/30
  * @brief      电机系统控制器使用示例
  */

#include "motor_system_controller.h"
#include "project_config.h"

#if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE

/* =============== 全局变量 =============== */
static uint32_t g_ulTickCount = 0;

/* =============== 示例1: 二轮差速小车初始化 =============== */

void vExample_Differential2_Init(void)
{
    /* 1. 配置底盘参数 */
    stChassisStaticParamTdf stChassisInit = {
        .emChassisType = emChassisDiff2,
        
        .fWheelRadiusMm = 50.0f,           /* 轮子半径 50mm */
        .fWheelTrackMm = 200.0f,          /* 轮距 200mm */
        .fWheelBaseMm = 200.0f,           /* 轴距 (麦轮用) */
        
        .fEncoderResolutionCpr = 1024.0f,  /* 编码器分辨率 1024线/圈 */
        .fGearRatio = 30.0f,              /* 减速比 30:1 */
        
        .fMaxSpeedMmS = 1000.0f,         /* 最大速度 1m/s */
        .fMaxOmegaRadS = 3.14f,          /* 最大角速度 180度/s */
        .fMaxAccelMmS2 = 2000.0f,        /* 加速度 2m/s^2 */
        .fMaxDecelMmS2 = 3000.0f,        /* 减速度 3m/s^2 */
        
        .aemMotorDevNums = { MOTOR_GEAR0, MOTOR_GEAR1, 0, 0 },
        .aemPidDevNums = { PID0, PID1, 0, 0 },
    };
    
    /* 2. 初始化电机系统控制器 */
    vMotorSystemControllerInit(&stChassisInit, MSC0);
    
    /* 3. 重置里程计 */
    vMotorSystemControllerResetOdometry(MSC0);
    
    /* 4. 使能 */
    vMotorSystemControllerEnable(MSC0, 1);
}

/* =============== 示例2: 速度模式 - 前进 =============== */

void vExample_Speed_Forward(void)
{
    /* 以 500mm/s (0.5m/s) 的速度前进 */
    vMotorSystemControllerSetSpeed(MSC0, 500.0f, 0, 0);
}

/* =============== 示例3: 速度模式 - 原地左转 =============== */

void vExample_Speed_SpinLeft(void)
{
    /* 原地左转，角速度 90度/s (π/2 rad/s) */
    vMotorSystemControllerSetSpeed(MSC0, 0, 0, 1.57f);
}

/* =============== 示例4: 速度模式 - 弧线行驶 =============== */

void vExample_Speed_Curve(void)
{
    /* 前进同时左转弯 */
    vMotorSystemControllerSetSpeed(MSC0, 400.0f, 0, 0.8f);
}

/* =============== 示例5: 位置模式 - 走到指定点 =============== */

void vExample_Position_GoToPoint(void)
{
    /* 从当前位置走到 (1000mm, 500mm)，速度 300mm/s */
    vMotorSystemControllerSetPosition(MSC0, 1000.0f, 500.0f, 300.0f);
}

/* =============== 示例6: 姿态模式 - 旋转到指定角度 =============== */

void vExample_Pose_RotateTo(void)
{
    /* 旋转到 90度 方向，角速度 1.57 rad/s */
    vMotorSystemControllerSetPose(MSC0, 90.0f, 1.57f);
}

/* =============== 示例7: 停止 =============== */

void vExample_Stop(void)
{
    vMotorSystemControllerStop(MSC0);
}

/* =============== 示例8: 主循环 - 完整运行示例 =============== */

void vExample_MainLoop(void)
{
    static uint8_t s_ucState = 0;
    static uint32_t s_ulStateTimer = 0;
    
    /* 假设在 10ms 周期中调用 */
    g_ulTickCount++;
    
    /* 1. 更新传感器数据 (实际项目中从传感器读取) */
    int32_t alEncoderData[4] = {0};
    /* 这里应该从编码器读取实际值... */
    vMotorSystemControllerUpdateEncoder(MSC0, alEncoderData);
    
    /* 更新IMU数据 */
    float fYawDeg = 0.0f;
    float fGyroZ = 0.0f;
    /* 这里应该从IMU读取实际值... */
    vMotorSystemControllerUpdateImu(MSC0, fYawDeg, 0, 0, fGyroZ);
    
    /* 2. 周期执行电机控制 */
    vMotorSystemControllerPeriodExecute(MSC0);
    
    /* 3. 状态机示例 - 自动往返 */
    switch (s_ucState) {
        case 0:
            /* 状态0: 前进2秒 */
            if (s_ulStateTimer == 0) {
                vExample_Speed_Forward();
            }
            s_ulStateTimer++;
            if (s_ulStateTimer >= 200) {  /* 200 * 10ms = 2s */
                s_ulStateTimer = 0;
                s_ucState = 1;
            }
            break;
            
        case 1:
            /* 状态1: 停止1秒 */
            if (s_ulStateTimer == 0) {
                vExample_Stop();
            }
            s_ulStateTimer++;
            if (s_ulStateTimer >= 100) {
                s_ulStateTimer = 0;
                s_ucState = 2;
            }
            break;
            
        case 2:
            /* 状态2: 左转90度 */
            if (s_ulStateTimer == 0) {
                vExample_Pose_RotateTo();
            }
            s_ulStateTimer++;
            /* 检查是否完成旋转 */
            if (s_ulStateTimer >= 150 || 
                emMotorSystemControllerGetMotionMode(MSC0) == emMotionMode_Idle) {
                s_ulStateTimer = 0;
                s_ucState = 0;
            }
            break;
    }
    
    /* 4. 查询并打印位置信息 (调试用) */
    if (g_ulTickCount % 100 == 0) {  /* 每1秒打印一次 */
        float fX = fMotorSystemControllerGetX(MSC0);
        float fY = fMotorSystemControllerGetY(MSC0);
        float fYaw = fMotorSystemControllerGetYaw(MSC0);
        
        /* 这里可以用串口打印 fX, fY, fYaw */
        /* 例如: vUartPrintf("X: %.1f Y: %.1f Yaw: %.1f\n", fX, fY, fYaw); */
    }
}

/* =============== 示例9: 里程计校准 =============== */

void vExample_OdometryCalibration(void)
{
    /* 步骤1: 让小车直线行驶一段已知距离，例如 1000mm */
    /* 步骤2: 读取里程计的距离，假设是 980mm */
    /* 步骤3: 计算校准比例: 1000 / 980 = 1.02 */
    /* 步骤4: 应用校准 */
    vMotorSystemControllerCalibWheelRadius(MSC0, 1.02f);
    
    /* 旋转校准类似 */
    /* 步骤1: 让小车原地旋转多圈，例如 10圈 = 3600度 */
    /* 步骤2: 读取里程计，假设是 3550度 */
    /* 步骤3: 计算比例: 3600 / 3550 = 1.014 */
    /* 步骤4: 应用 */
    vMotorSystemControllerCalibTrack(MSC0, 1.014f);
}

/* =============== 示例10: 麦克纳姆轮全向移动 =============== */

void vExample_Mecanum_Init(void)
{
    stChassisStaticParamTdf stChassisInit = {
        .emChassisType = emChassisType_Mecanum4,
        
        .fWheelRadiusMm = 50.0f,
        .fWheelTrackMm = 250.0f,
        .fWheelBaseMm = 250.0f,
        
        .fEncoderResolutionCpr = 1024.0f,
        .fGearRatio = 30.0f,
        
        .fMaxSpeedMmS = 800.0f,
        .fMaxOmegaRadS = 2.5f,
        .fMaxAccelMmS2 = 1500.0f,
        .fMaxDecelMmS2 = 2500.0f,
        
        .aemMotorDevNums = { MOTOR_GEAR0, MOTOR_GEAR1, MOTOR_GEAR2, MOTOR_GEAR3 },
        .aemPidDevNums = { PID0, PID1, PID0, PID1 },
    };
    
    vMotorSystemControllerInit(&stChassisInit, MSC0);
    vMotorSystemControllerResetOdometry(MSC0);
    vMotorSystemControllerEnable(MSC0, 1);
}

void vExample_Mecanum_MoveRight(void)
{
    /* 麦轮特有的横向移动 (右移) */
    vMotorSystemControllerSetSpeed(MSC0, 0, 300.0f, 0);
}

void vExample_Mecanum_Diagonal(void)
{
    /* 斜向移动 (右前方) */
    vMotorSystemControllerSetSpeed(MSC0, 400.0f, 400.0f, 0);
}

#endif /* MOTOR_SYSTEM_CONTROLLER_IS_ENABLE */

#endif
