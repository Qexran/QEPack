/**
  * @file       counter_controller.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/5/8
  * @brief      计数器控制器，基于QePack编码风格
  * 
  */
  
#include "project_config.h"

#if COUNTER_IS_ENABLE

#ifndef _COUNTER_CONTROLLER_H_
#define _COUNTER_CONTROLLER_H_

/**
 * @brief          计数器设备号枚举
 * @note           支持多个计数器实例
 */
typedef enum
{
    emCounterDevNum0      = 0,
    emCounterDevNum1      = 1,
    emCounterDevNum2      = 2,
    emCounterDevNum3      = 3,
    emCounterDevMax       = 4,
    emNoCounterDevNum     = 0xFF,
} emCounterDevNumTdf;

/**
 * @brief          计数器状态枚举
 * @note           用于查询计数器状态
 */
typedef enum
{
    emCounterStateIdle    = 0,    /* 空闲状态 */
    emCounterStateRunning = 1,    /* 计数中 */
    emCounterStateReached = 2,    /* 已达到目标次数 */
} emCounterStateTdf;

/**
 * @brief          计数器运行参数结构体
 * @note           运行时状态参数
 */
typedef struct
{
    uint32_t           ulCurrentCount;   /* 当前计数值 */
    uint32_t           ulTargetCount;    /* 目标计数值 */
    emCounterStateTdf  emState;          /* 当前状态 */
} stCounterRunningParamTdf;

/* 全局计数器设备数组 */
extern stCounterRunningParamTdf g_astCounters[COUNTER_DEV_NUM];

/**
 * @brief          初始化计数器
 * @param  emDevNum : 计数器设备号
 * @param  ulTargetCount : 目标计数次数
 */
void vCounterInit(emCounterDevNumTdf emDevNum, uint32_t ulTargetCount);

/**
 * @brief          重置计数器
 * @param  emDevNum : 计数器设备号
 */
void vCounterReset(emCounterDevNumTdf emDevNum);

/**
 * @brief          增加计数
 * @param  emDevNum : 计数器设备号
 * @param  ulIncrement : 增加的计数值(默认1)
 */
void vCounterIncrement(emCounterDevNumTdf emDevNum, uint32_t ulIncrement);

/**
 * @brief          减少计数
 * @param  emDevNum : 计数器设备号
 * @param  ulDecrement : 减少的计数值(默认1)
 */
void vCounterDecrement(emCounterDevNumTdf emDevNum, uint32_t ulDecrement);

/**
 * @brief          设置计数值
 * @param  emDevNum : 计数器设备号
 * @param  ulCount : 设置的计数值
 */
void vCounterSetCount(emCounterDevNumTdf emDevNum, uint32_t ulCount);

/**
 * @brief          设置目标次数
 * @param  emDevNum : 计数器设备号
 * @param  ulTarget : 目标计数值
 */
void vCounterSetTarget(emCounterDevNumTdf emDevNum, uint32_t ulTarget);

/**
 * @brief          获取当前计数值
 * @param  emDevNum : 计数器设备号
 * @return         当前计数值
 */
uint32_t ulCounterGetCurrent(emCounterDevNumTdf emDevNum);

/**
 * @brief          获取目标计数值
 * @param  emDevNum : 计数器设备号
 * @return         目标计数值
 */
uint32_t ulCounterGetTarget(emCounterDevNumTdf emDevNum);

/**
 * @brief          获取计数器状态
 * @param  emDevNum : 计数器设备号
 * @return         计数器状态枚举
 */
emCounterStateTdf emCounterGetState(emCounterDevNumTdf emDevNum);

/**
 * @brief          检查是否达到目标次数
 * @param  emDevNum : 计数器设备号
 * @return         0=未达到,1=已达到
 */
uint8_t u8CounterIsReached(emCounterDevNumTdf emDevNum);

/**
 * @brief          周期执行
 * @param  emDevNum : 计数器设备号
 */
void vCounterPeriodExecute(emCounterDevNumTdf emDevNum);

#endif

#endif
