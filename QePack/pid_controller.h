/**
 * @file    pid_controller.h
 * @author  Qe_xr
 * @version V2.0.0
 * @date    2026/5/11
 * @brief   PID控制器模块（定点数版本）
 *
 * 本模块实现了标准的PID控制器，支持位置式和速度式两种计算模式，
 * 支持变速积分、积分分离和不完全微分功能，包括比例、积分、微分控制，
 * 以及积分限幅和输出限幅功能。内部使用 Q16.16 定点数运算。
 */
#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include "project_config.h"
#include "arithmetic.h"
#include "string.h"
#if PID_IS_ENABLE

/** @brief PID设备号枚举 */
typedef enum {
    emPidDevNum0 = 0,
    emPidDevNum1,
    emPidDevNum2,
    emPidDevNum3,
    emNoPid = 0xFF,
} emPidDevNumTdf;

/** @brief PID计算模式枚举 */
typedef enum {
    emPidModePosition = 0,
    emPidModeIncremental,
} emPidModeTdf;

/** @brief PID静态参数定义 */
typedef struct {
    fix32_t Kp;                                  /* 比例系数 */
    fix32_t Ki;                                  /* 积分系数 */
    fix32_t Kd;                                  /* 微分系数 */
    fix32_t MaxIntegral;                         /* 最大积分值 */
    fix32_t MaxOutput;                           /* 最大输出值 */
    emPidModeTdf Mode;                           /* PID计算模式 */
    uint8_t EnableIntegralSeparation;            /* 是否启用积分分离 */
    fix32_t IntegralSeparationThreshold;         /* 积分分离阈值 */
    uint8_t EnableVariableIntegral;              /* 是否启用变速积分 */
    fix32_t VariableIntegralBeta;                /* 变速积分系数 */
    uint8_t EnableDerivativeOnFeedback;          /* 是否启用微分反馈 */
    uint8_t EnableIncompleteDerivative;          /* 是否启用不完全微分 */
    fix32_t IncompleteDerivativeAlpha;           /* 不完全微分系数 */
} stPidStaticParamTdf;

/** @brief PID运行参数定义 */
typedef struct {
    fix32_t Error;                   /* 当前误差 */
    fix32_t LastError;               /* 上一次误差 */
    fix32_t LastLastError;           /* 上一次上一次误差 */
    fix32_t Integral;                /* 当前积分值 */
    fix32_t Output;                  /* 当前输出值 */
    fix32_t LastFeedback;            /* 上一次反馈值 */
    fix32_t LastDerivative;          /* 上一次微分值 */
    fix32_t Target;                  /* 当前目标值 */
    uint8_t Enable;                  /* PID 使能开关，1=使能，0=禁用 */
} stPidRunningParamTdf;

/** @brief PID设备参数总结构体 */
typedef struct {
    stPidStaticParamTdf stStaticParam;
    stPidRunningParamTdf stRunningParam;
} stPidDeviceParamTdf;

const stPidDeviceParamTdf *c_pstGetPidDeviceParam(emPidDevNumTdf emDevNum);
void vPidDeviceInit(stPidStaticParamTdf *pstInit, emPidDevNumTdf emDevNum);
void vPidReset(emPidDevNumTdf emDevNum);
void vPidSetMode(emPidDevNumTdf emDevNum, emPidModeTdf emMode, uint8_t bReset);
void vPidCalc(emPidDevNumTdf emDevNum, fix32_t fReference, fix32_t fFeedback);
QE_StatusTypeDef ePidGetOutput(emPidDevNumTdf emDevNum, fix32_t *pfOutput);
void vPidSetParam(emPidDevNumTdf emDevNum, fix32_t fKp, fix32_t fKi, fix32_t fKd);
void vPidSetTarget(emPidDevNumTdf emDevNum, fix32_t fTarget);
void vPidSetEnable(emPidDevNumTdf emDevNum, uint8_t bEnable);
fix32_t fPidGetTarget(emPidDevNumTdf emDevNum);

#endif

#endif

/*

必填项（零值会导致 PID 不工作或行为异常）

  ┌───────────┬───────────────────────────────────────────────┐
  │   字段    │                   零值后果                    │
  ├───────────┼───────────────────────────────────────────────┤
  │ Kp        │ 比例项为 0，PID 基本无输出（除非 Ki/Kd 很大） │
  ├───────────┼───────────────────────────────────────────────┤
  │ Mode      │ 0 = emPidModePosition，这个倒是恰好能用       │
  └───────────┴───────────────────────────────────────────────┘

  MaxOutput 是最容易踩的坑 — 不填的话 PID 算出来的输出全部被限幅为 0，看起来像"PID 不工作"。

  可选项（零值 = 不启用该功能，安全）

  ┌─────────────────────────────┬────────────────────────────────┐
  │            字段             │            零值含义            │
  ├─────────────────────────────┼────────────────────────────────┤
  │ Ki                          │ 无积分，纯 PD 控制，完全合法   │
  ├─────────────────────────────┼────────────────────────────────┤
  │ Kd                          │ 无微分，纯 PI 控制，完全合法   │
  ├─────────────────────────────┼────────────────────────────────┤
  │ MaxIntegral                 │ 积分限幅为 0，但 Ki=0 时无影响 │
  ├─────────────────────────────┼────────────────────────────────┤
  │ EnableIntegralSeparation    │ 0 = 不启用积分分离             │
  ├─────────────────────────────┼────────────────────────────────┤
  │ IntegralSeparationThreshold │ 积分分离阈值，上面关了就没用   │
  ├─────────────────────────────┼────────────────────────────────┤
  │ EnableVariableIntegral      │ 0 = 不启用变速积分             │
  ├─────────────────────────────┼────────────────────────────────┤
  │ VariableIntegralBeta        │ 变速积分系数，上面关了就没用   │
  ├─────────────────────────────┼────────────────────────────────┤
  │ EnableDerivativeOnFeedback  │ 0 = 微分作用于误差（标准模式） │
  ├─────────────────────────────┼────────────────────────────────┤
  │ EnableIncompleteDerivative  │ 0 = 不启用不完全微分           │
  ├─────────────────────────────┼────────────────────────────────┤
  │ IncompleteDerivativeAlpha   │ 不完全微分系数，上面关了就没用 │
  └─────────────────────────────┴────────────────────────────────┘

  最小可用配置

  stPidStaticParamTdf stPidInit = {0};
  stPidInit.Kp        = fix32_from_float(1.0f);   // 必填
  stPidInit.MaxOutput  = fix32_from_float(1000.0f); // 必填

  
    使用示例1：位置式PID（变速积分 + 积分分离 + 不完全微分）

    stPidStaticParamTdf stPidStaticInit = {0};
    stPidStaticInit.Kp = fix32_from_float(10.0f);
    stPidStaticInit.Ki = fix32_from_float(1.0f);
    stPidStaticInit.Kd = fix32_from_float(5.0f);
    stPidStaticInit.MaxIntegral = fix32_from_float(800.0f);
    stPidStaticInit.MaxOutput = fix32_from_float(1000.0f);
    stPidStaticInit.Mode = emPidModePosition;
    stPidStaticInit.EnableIntegralSeparation = 1;
    stPidStaticInit.IntegralSeparationThreshold = fix32_from_float(100.0f);
    stPidStaticInit.EnableVariableIntegral = 1;
    stPidStaticInit.VariableIntegralBeta = fix32_from_float(0.01f);
    stPidStaticInit.EnableDerivativeOnFeedback = 1;   // 微分先行，避免目标值阶跃时微分冲击
    stPidStaticInit.EnableIncompleteDerivative = 1;
    stPidStaticInit.IncompleteDerivativeAlpha = fix32_from_float(0.1f);

    vPidDeviceInit(&stPidStaticInit, emPidDevNum0);

    while (1) {
        fix32_t fFeedback = fix32_from_float(...);  // 获取反馈值
        fix32_t fTarget = fix32_from_float(...);    // 获取目标值
        vPidCalc(emPidDevNum0, fTarget, fFeedback);
        fix32_t fOutput;
        if (ePidGetOutput(emPidDevNum0, &fOutput) == QE_OK) {
            // 使用 fOutput 驱动执行器
        }
        delay(10);
    }

    使用示例2：增量式PID（适合电机调速）

    stPidStaticParamTdf stPidInit = {0};
    stPidInit.Kp = fix32_from_float(2.0f);
    stPidInit.Ki = fix32_from_float(0.5f);
    stPidInit.Kd = fix32_from_float(0.1f);
    stPidInit.MaxOutput = fix32_from_float(999.0f);
    stPidInit.Mode = emPidModeIncremental;
    // 增量式模式下变速积分自动禁用，仅积分分离生效
    stPidInit.EnableIntegralSeparation = 1;
    stPidInit.IntegralSeparationThreshold = fix32_from_float(50.0f);

    vPidDeviceInit(&stPidInit, emPidDevNum1);

    使用示例3：运行时切换模式

    vPidSetMode(emPidDevNum0, emPidModeIncremental, 1);  // 切换到增量式并重置状态
    vPidSetMode(emPidDevNum0, emPidModePosition, 0);     // 切换回位置式，保留积分累积
*/
