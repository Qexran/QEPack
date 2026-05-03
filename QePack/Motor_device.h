/**
  * @file       motor_device.h
  * @author     Qe_xr
  * @version    V1.1.0
  * @date       2026/4/27
  * @brief      电机控制基类，基于 QePack 编码风格
  * 
  */
  
#ifndef __MOTOR_DEVICE_H
#define __MOTOR_DEVICE_H

#include "project_config.h"

#if MOTOR_IS_ENABLE

/**
 * @brief          电机设备类型枚举
 * @note 
 */
typedef enum
{
    emMotorType_Gear = 0,    /* 直流减速电机 */
    emMotorType_Emm,         /* Emm步进电机 */
} emMotorTypeTdf;

typedef enum { /* 电机运动状态 */
    emMotorStateNULL = 0,        /* 无 */
    emMotorStateIdle,            /* 空闲 */
    emMotorStateRunning,         /* 运行 */
    emMotorStateStop,            /* 停止 */
} emMotorStateTdf;

/**
 * @brief          统一电机设备号枚举
 * @note           所有电机类型共用同一套设备号
 */
typedef enum
{
    // 减速电机设备号
    emGearMotorDevNum0      = 0,
    emGearMotorDevNum1      = 1,
    emGearMotorDevNum2      = 2,
    emGearMotorDevNum3      = 3,
    emGearMotorDevNum4      = 4,
    emGearMotorDevNum5      = 5,
    emGearMotorDevNum6      = 6,
    emGearMotorDevNum7      = 7,
    emGearMotorDevNum8      = 8,
    emGearMotorDevNum9      = 9,
    emGearMotorDevMax       = 11,

    // Emm步进电机设备号
    emEmmMotorDevNum0       = 20,
    emEmmMotorDevNum1       = 21,
    emEmmMotorDevNum2       = 22,
    emEmmMotorDevNum3       = 23,
    emEmmMotorDevNum4       = 24,
    emEmmMotorDevNum5       = 25,
    emEmmMotorDevNum6       = 26,
    emEmmMotorDevNum7       = 27,
    emEmmMotorDevNum8       = 28,
    emEmmMotorDevNum9       = 29,
    emEmmMotorDevMax        = 30,
    
    // Bldc电机设备号   
    emBldcMotorDevNum0      = 40,
    emBldcMotorDevNum1      = 41,
    emBldcMotorDevNum2      = 42,
    emBldcMotorDevNum3      = 43,
    emBldcMotorDevNum4      = 44,
    emBldcMotorDevNum5      = 45,
    emBldcMotorDevNum6      = 46,
    emBldcMotorDevNum7      = 47,
    emBldcMotorDevNum8      = 48,
    emBldcMotorDevNum9      = 49,
    emBldcMotorDevMax       = 50,
    
    emMotorDevMax           = 51,

    emNoMotorDevNum         = 0xFF,    /* 无电机设备号 */
} emMotorDevNumTdf;

/**
 * @brief          电机方向枚举
 * @note 
 */
typedef enum
{
    emMotorDir_Forward  = 0,    /* 正转 */
    emMotorDir_Backward = 1,    /* 反转 */
} emMotorDirTdf;

/**
 * @brief          电机虚方法表结构体
 * @note           子类需要实现这些方法
 */
typedef struct stMotorVTableTdf
{
    void (*vInit)(void *pstInit);                                   /* 初始化 */
    void (*vPeriodExecute)(void *pstMotor);                         /* 周期执行 */
    void (*vSetSpeed)(void *pstMotor, int16_t speed);               /* 设置速度 */
    void (*vStop)(void *pstMotor);                                  /* 停止 */
    void (*vEnable)(void *pstMotor, uint8_t bEnable);               /* 使能控制 */
    emMotorStateTdf (*emGetState)(void *pstMotor);                  /* 获取电机状态 */
} stMotorVTableTdf;


/**
 * @brief          电机基类结构体
 * @note           所有电机类型的基类
 */
typedef struct stMotorDeviceTdf
{
    emMotorTypeTdf     emType;              /* 电机类型 */
    stMotorVTableTdf   *pstVTable;          /* 虚方法表 */
    emMotorStateTdf    emMotorState;        /* 电机运动状态 */
} stMotorDeviceTdf;

/* 全局电机设备数组 */
extern stMotorDeviceTdf* g_astMotorDevices[emMotorDevMax];

/**
 * @brief          注册电机设备
 * @param  emDevNum ：电机设备号
 * @param  pstMotor ：电机设备指针（包含基类成员）
 */
void vMotorRegisterDevice(uint8_t emDevNum, stMotorDeviceTdf *pstMotor);

/**
 * @brief          电机初始化
 * @param  emDevNum ：电机设备号
 */
void vMotorInit(uint8_t emDevNum);

/**
 * @brief          电机周期执行
 * @param  emDevNum ：电机设备号
 */
void vMotorPeriodExecute(uint8_t emDevNum);

/**
 * @brief          设置电机速度
 * @param  emDevNum ：电机设备号
 * @param  speed ：速度值
 */
void vMotorSetSpeed(uint8_t emDevNum, int16_t speed);

/**
 * @brief          停止电机
 * @param  emDevNum ：电机设备号
 */
void vMotorStop(uint8_t emDevNum);

/**
 * @brief          电机使能控制
 * @param  emDevNum ：电机设备号
 * @param  bEnable ：使能状态
 */
void vMotorEnable(uint8_t emDevNum, uint8_t bEnable);

emMotorStateTdf emGetMotorState(uint8_t emDevNum);
#endif

#endif
