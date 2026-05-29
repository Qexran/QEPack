/**
  * @file       it_controller.c
  * @author     Qe_xr
  * @version    V1.0.1
  * @date       2026/5/29
  * @brief      中断管理
  *
  */
#include "it_controller.h"

/**
 * @brief       设备周期更新函数（在 1ms 定时器 ISR 中调用）
 * @note        仅调用 ISR 安全的模块，不可调用 I2C 等阻塞操作
 */
void vDevicePeriodExecute(){
    /* KEY */
    #if KEY_IS_ENABLE
        #ifdef KEY0 
            vKeyDevicePeriodExecute(KEY0); 
        #endif
        #ifdef KEY1 
            vKeyDevicePeriodExecute(KEY1); 
        #endif
        #ifdef KEY2 
            vKeyDevicePeriodExecute(KEY2); 
        #endif
        #ifdef KEY3 
            vKeyDevicePeriodExecute(KEY3); 
        #endif
        #ifdef KEY4 
            vKeyDevicePeriodExecute(KEY4); 
        #endif
        #ifdef KEY5 
            vKeyDevicePeriodExecute(KEY5); 
        #endif
        #ifdef KEY6 
            vKeyDevicePeriodExecute(KEY6); 
        #endif
        #ifdef KEY7 
            vKeyDevicePeriodExecute(KEY7); 
        #endif
    #endif
    /* LED */
    #if LED_IS_ENABLE
        #ifdef LED0 
            vLedDevicePeriodExecute(LED0); 
        #endif
        #ifdef LED1 
            vLedDevicePeriodExecute(LED1); 
        #endif
        #ifdef LED2 
            vLedDevicePeriodExecute(LED2); 
        #endif
        #ifdef LED3 
            vLedDevicePeriodExecute(LED3); 
        #endif
        #ifdef LED4 
            vLedDevicePeriodExecute(LED4); 
        #endif
        #ifdef LED5 
            vLedDevicePeriodExecute(LED5); 
        #endif
        #ifdef LED6 
            vLedDevicePeriodExecute(LED6); 
        #endif
        #ifdef LED7 
            vLedDevicePeriodExecute(LED7); 
        #endif
    #endif
    /* SERVO */
    #if SERVO_IS_ENABLE
        #ifdef SERVO0 
            vServoDevicePeriodExecute(SERVO0); 
        #endif
        #ifdef SERVO1 
            vServoDevicePeriodExecute(SERVO1); 
        #endif
        #ifdef SERVO2 
            vServoDevicePeriodExecute(SERVO2); 
        #endif
        #ifdef SERVO3 
            vServoDevicePeriodExecute(SERVO3); 
        #endif
        #ifdef SERVO4 
            vServoDevicePeriodExecute(SERVO4); 
        #endif
        #ifdef SERVO5 
            vServoDevicePeriodExecute(SERVO5); 
        #endif
        #ifdef SERVO6 
            vServoDevicePeriodExecute(SERVO6); 
        #endif
        #ifdef SERVO7 
            vServoDevicePeriodExecute(SERVO7); 
        #endif
    #endif
    /* UART */
    #if UART_IS_ENABLE
        #ifdef UART_DEVICE_0 
            vUartDevicePeriodExecute(UART_DEVICE_0); 
        #endif
        #ifdef UART_DEVICE_1 
            vUartDevicePeriodExecute(UART_DEVICE_1); 
        #endif
        #ifdef UART_DEVICE_2 
            vUartDevicePeriodExecute(UART_DEVICE_2); 
        #endif
        #ifdef UART_DEVICE_3 
            vUartDevicePeriodExecute(UART_DEVICE_3); 
        #endif
        #ifdef UART_DEVICE_4 
            vUartDevicePeriodExecute(UART_DEVICE_4); 
        #endif
        #ifdef UART_DEVICE_5 
            vUartDevicePeriodExecute(UART_DEVICE_5); 
        #endif
        #ifdef UART_DEVICE_6 
            vUartDevicePeriodExecute(UART_DEVICE_6); 
        #endif
        #ifdef UART_DEVICE_7 
            vUartDevicePeriodExecute(UART_DEVICE_7); 
        #endif
    #endif
    /* ULTRASONIC */
    #if ULTRASONIC_IS_ENABLE
        #ifdef ULTR0 
//            vUltrasonicDevicePeriodExecute(ULTR0); 
        #endif
        #ifdef ULTR1
            vUltrasonicDevicePeriodExecute(ULTR1); 
        #endif
    #endif
    /* STEP */
    #if STEP_M_ENABLE
        #ifdef STEP0 
            vStepPeriodExecute(STEP0); 
        #endif
        #ifdef STEP1 
            vStepPeriodExecute(STEP1); 
        #endif
        #ifdef STEP2 
            vStepPeriodExecute(STEP2); 
        #endif
        #ifdef STEP3 
            vStepPeriodExecute(STEP3); 
        #endif
    #endif

    #if MOTOR_IS_ENABLE
        #ifdef MOTOR_GEAR0 
            vMotorPeriodExecute(MOTOR_GEAR0); 
        #endif
        #ifdef MOTOR_GEAR1 
            vMotorPeriodExecute(MOTOR_GEAR1); 
        #endif
        #ifdef MOTOR_GEAR2 
            vMotorPeriodExecute(MOTOR_GEAR2); 
        #endif
        #ifdef MOTOR_GEAR3 
            vMotorPeriodExecute(MOTOR_GEAR3); 
        #endif
        #ifdef MOTOR_GEAR4 
            vMotorPeriodExecute(MOTOR_GEAR4); 
        #endif
        #ifdef MOTOR_GEAR5 
            vMotorPeriodExecute(MOTOR_GEAR5); 
        #endif
        #ifdef MOTOR_GEAR6 
            vMotorPeriodExecute(MOTOR_GEAR6); 
        #endif
        #ifdef MOTOR_GEAR7 
            vMotorPeriodExecute(MOTOR_GEAR7); 
        #endif
        #ifdef MOTOR_GEAR8 
            vMotorPeriodExecute(MOTOR_GEAR8);
        #endif
        #ifdef MOTOR_GEAR9 
            vMotorPeriodExecute(MOTOR_GEAR9); 
        #endif


        /* EMM电机 */
        #ifdef MOTOR_EMM0 
            vMotorPeriodExecute(MOTOR_EMM0); 
        #endif
        #ifdef MOTOR_EMM1 
            vMotorPeriodExecute(MOTOR_EMM1); 
        #endif
        #ifdef MOTOR_EMM2 
            vMotorPeriodExecute(MOTOR_EMM2); 
        #endif
        #ifdef MOTOR_EMM3 
            vMotorPeriodExecute(MOTOR_EMM3); 
        #endif
        #ifdef MOTOR_EMM4 
            vMotorPeriodExecute(MOTOR_EMM4); 
        #endif
        #ifdef MOTOR_EMM5 
            vMotorPeriodExecute(MOTOR_EMM5); 
        #endif
        #ifdef MOTOR_EMM6 
            vMotorPeriodExecute(MOTOR_EMM6); 
        #endif
        #ifdef MOTOR_EMM7 
            vMotorPeriodExecute(MOTOR_EMM7); 
        #endif
        #ifdef MOTOR_EMM8 
            vMotorPeriodExecute(MOTOR_EMM8); 
        #endif
        #ifdef MOTOR_EMM9 
            vMotorPeriodExecute(MOTOR_EMM9); 
        #endif


        /* BLDC电机 */
        #ifdef MOTOR_BLDC0 
            vMotorPeriodExecute(MOTOR_BLDC0); 
        #endif
        #ifdef MOTOR_BLDC1 
            vMotorPeriodExecute(MOTOR_BLDC1); 
        #endif
        #ifdef MOTOR_BLDC2 
            vMotorPeriodExecute(MOTOR_BLDC2); 
        #endif
        #ifdef MOTOR_BLDC3 
            vMotorPeriodExecute(MOTOR_BLDC3); 
        #endif
        #ifdef MOTOR_BLDC4 
            vMotorPeriodExecute(MOTOR_BLDC4); 
        #endif
        #ifdef MOTOR_BLDC5 
            vMotorPeriodExecute(MOTOR_BLDC5); 
        #endif
        #ifdef MOTOR_BLDC6 
            vMotorPeriodExecute(MOTOR_BLDC6); 
        #endif
        #ifdef MOTOR_BLDC7 
            vMotorPeriodExecute(MOTOR_BLDC7); 
        #endif
        #ifdef MOTOR_BLDC8 
            vMotorPeriodExecute(MOTOR_BLDC8); 
        #endif
        #ifdef MOTOR_BLDC9 
            vMotorPeriodExecute(MOTOR_BLDC9); 
        #endif
    #endif

    #if JOYSTICK_IS_ENABLE
        #ifdef JOYSTICK0
            /* 读取摇杆数据 */
            vJoystickPeriodExecute(JOYSTICK0);
        #endif
    #endif

}

/**
 * @brief       主循环周期更新函数（在 main while(1) 中调用）
 * @note        用于需要阻塞 I2C 等操作的模块（传感器、电机系统控制器）
 */
void vDevicePeriodExecuteMainLoop(void)
{
    #if HWT101_IS_ENABLE
        #ifdef HWT101_0
            vSensorPeriodExecute(HWT101_0);
        #endif
        #ifdef HWT101_1
            vSensorPeriodExecute(HWT101_1);
        #endif
    #endif

    #if MOTOR_SYSTEM_CONTROLLER_IS_ENABLE
        vMotorSystemPeriodExecute();
    #endif
}




