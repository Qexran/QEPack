/** 
 * @file    step_machine.h
 * @author  Qe_xr
 * @version V2.0.0
 * @date    2026/3/2
 * @brief   轻量化步骤机 - 无冗余结构体/不定长传参/嵌入式内存优化
 * 特性：printf式可变参数、无结构体定义、自动步骤切换、多实例、超时检测、无堆内存
 * 传参格式：设备号+总步骤数+是否循环+结束步骤ID+[步骤参数...]+0(结束标志)
 * 单步骤参数：StepID(ushort)+回调(pfStepExecCb)+超时(ulong)+规则数(uchar)+[结果,下一步...](ushort*N)
 */
#include "project_config.h"
#include "stdarg.h"
#include "string.h"

#if STEP_M_ENABLE
#ifndef _STEP_MACHINE_H_
#define _STEP_MACHINE_H_

#if (QEPACK_PLATFORM == TI)
    #include "ti_platform.h"
#endif


/** @brief 步骤机设备号*/
typedef enum {
    emStepDevNum0 = 0,
    emStepDevNum1,
    emStepDevNum2,
    emStepDevNum3,
} emSmStepDevTdf;

/** @brief 步骤执行结果(规则)*/
typedef enum {
    emStepRetOk = 0,
    emStepRetError,
    emStepRetWait,
    emStepRetMax
} emStepRetTdf;

/** @brief 步骤机整体运行状态*/
typedef enum {
    emStepMStateIdle = 0,
    emStepMStateRunning,
    emStepMStateEnd,
    emStepMStateError
} emStepMStateTdf;

/** @brief 步骤回调函数(业务逻辑，返回执行结果)*/
typedef emStepRetTdf (*pfStepExecCb)(emSmStepDevTdf emDevNum);

/** @brief 跳转规则缓存(轻量化，无冗余)*/
typedef struct {
    emStepRetTdf ret;        // 执行结果
    uint16_t nextStep;         // 下一步ID
} stStepRuleTdf;

/** @brief 单个步骤缓存(轻量化，仅必要参数)*/
typedef struct {
    uint16_t stepId;           // 步骤ID
    pfStepExecCb pfExec;     // 执行回调
    uint32_t timeoutMs;        // 超时时间
    uint8_t ruleNum;           // 跳转规则数
    stStepRuleTdf tRules[STEP_M_MAX_RULE_NUM]; // 规则缓存(定长数组，无堆)
} stStepItemTdf;

/** @brief 步骤机设备参数(静态配置+运行状态，全轻量化)*/
typedef struct {
    // 静态配置(解析可变参数后缓存，无冗余)
    uint8_t stepTotal;         // 总步骤数
    uint8_t isCycle;           // 是否循环执行
    uint16_t endStep;          // 结束步骤ID
    stStepItemTdf tSteps[STEP_M_MAX_STEP_NUM]; // 步骤缓存(定长)
    // 运行状态(实时更新，极简)
    emStepMStateTdf mState;  // 整体状态
    uint16_t curStep;          // 当前步骤ID
    uint32_t stepStartTick;    // 当前步骤启动时间
} stStepDevParamTdf;

// 对外接口(极简，核心：可变参数初始化接口)
void vStepInit(emSmStepDevTdf emDevNum, uint8_t stepTotal, uint8_t isCycle, int endStep, ...);
void vStepReset(emSmStepDevTdf emDevNum);
void vStepStart(emSmStepDevTdf emDevNum);
void vStepStop(emSmStepDevTdf emDevNum);
emStepMStateTdf emStepGetState(emSmStepDevTdf emDevNum);
uint16_t u16StepGetCurStep(emSmStepDevTdf emDevNum);
void vStepProcess(emSmStepDevTdf emDevNum); // 核心处理函数(主循环调用)

#endif
#endif

/*
    main.c 示例
    #include "step_machine.h"

    // 步骤1：上电检测
    emStepRetTdf pfStep1_PowerCheck(emSmStepDevTdf emDevNum)
    {
        // vOledPrintf(OLED0, 1, 1, OLED_8X16, "Power Check");
        return emStepRetOk;
    }

    // 步骤2：参数初始化
    emStepRetTdf pfStep2_ParamInit(emSmStepDevTdf emDevNum)
    {
        //vOledPrintf(OLED0, 1, 1, OLED_8X16, "Param Init");
        return emStepRetOk;
        // 若初始化失败，返回Error
        // return emStepRetError;
    }

    // 步骤3：电机启动
    emStepRetTdf pfStep3_MotorStart(emSmStepDevTdf emDevNum)
    {
        // vOledPrintf(OLED0, 1, 1, OLED_8X16, "Motor Start");
        return emStepRetOk;
    }

    // 错误步骤：故障处理
    emStepRetTdf pfStep_Error(emSmStepDevTdf emDevNum)
    {
        // vOledPrintf(OLED0, 1, 1, OLED_8X16, "Error");
        return emStepRetOk;
    }


    int main(void)
    {
        // 1. 系统初始化(HAL库、GPIO、外设等)
        // ...

        // 2. 可变参数初始化步骤机(设备0，4步，单次执行，结束步骤0xFFFF)
        // 传参格式：设备号+总步骤数+是否循环+结束步骤ID+[步骤1参数]+[步骤2参数]+[步骤3参数]+[错误步骤参数]+0
        // 单步骤参数：StepID+回调+超时+规则数+[结果,下一步...]+...
        vStepInit(emSmStepDev0, 4, 0, 0xFFFF,
            // 步骤1：ID=1，回调=pfStep1，超时5000ms，3条规则→Ok→2/Wait→1/Error→0xFF
            1, pfStep1_PowerCheck, 5000UL, 3, emStepRetOk,2, emStepRetWait,1, emStepRetError, 0xFF,
            // 步骤2：ID=2，回调=pfStep2，超时2000ms，2条规则→Ok→3/Error→0xFF
            2, pfStep2_ParamInit, 2000UL, 2, emStepRetOk,3, emStepRetError, 0xFF,
            // 步骤3：ID=3，回调=pfStep3，超时3000ms，2条规则→Ok→0xFFFF/Error→0xFF
            3, pfStep3_MotorStart, 3000UL, 2, emStepRetOk,0xFFFF, emStepRetError, 0xFF,
            // 错误步骤：ID=0xFF，回调=pfStep_Error，超时0，1条规则→Ok→0xFFFF
            0xFF, pfStep_Error, 0UL, 1, emStepRetOk, 0xFFFF,    
            0 // 可变参数结束标志(必须加)
        );

        // 3. 启动步骤机
        vStepStart(STEP0);

        // 主循环(定时调用，如10ms)
        while (1) {
            vStepProcess(STEP0); // 核心处理

            // 可选：获取状态做业务处理
            emStepMStateTdf mState = emStepGetState(STEP0);
            if (mState == emStepMStateEnd) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // 点亮成功灯
            } else if (mState == emStepMStateError) {
                // 故障额外处理
            }

            HAL_Delay(10);
        }
    }
*/
