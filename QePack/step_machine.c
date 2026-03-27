/** 
 * @file    step_machine.c
 * @brief   轻量化步骤机实现
 */
#include "step_machine.h"

#if (QEPACK_PLATFORM == TI)
    #define HAL_GetTick() TI_GetTick()
#endif

#if STEP_M_ENABLE
// 全局设备参数数组
static stStepDevParamTdf astStepDev[STEP_M_NUM] = {0};

/**
 * @brief 静态函数:根据步骤ID查找步骤缓存
 */
static stStepItemTdf *pstStepFind(emSmStepDevTdf emDevNum, uint16_t stepId)
{
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    for (uint8_t i = 0; i < pstDev->stepTotal; i++) {
        if (pstDev->tSteps[i].stepId == stepId) {
            return &pstDev->tSteps[i];
        }
    }
    return NULL;
}

/**
 * @brief 静态函数:根据执行结果查找下一步ID
 */
static uint16_t u16StepFindNext(stStepItemTdf *pstItem, emStepRetTdf ret)
{
    for (uint8_t i = 0; i < pstItem->ruleNum; i++) {
        if (pstItem->tRules[i].ret == ret) {
            return pstItem->tRules[i].nextStep;
        }
    }
    return 0xFFFF; // 无效步骤
}

/**
 * @brief 静态函数:步骤超时检测
 */
static uint8_t u8StepCheckTimeout(stStepItemTdf *pstItem, uint32_t startTick)
{
    if (pstItem->timeoutMs == 0) return 0;
    return (HAL_GetTick() - startTick >= pstItem->timeoutMs) ? 1 : 0;
}

/**
 * @brief 核心接口:可变参数初始化步骤机
 * @param emDevNum 设备号
 * @param stepTotal 总步骤数（≤STEP_M_MAX_STEP_NUM）
 * @param isCycle 是否循环（0=单次,1=循环）
 * @param endStep 结束步骤ID（0xFFFF为通用结束）
 * @param ... 不定长参数:[步骤参数...]+0
 * @param 单步骤参数:stepId(ushort)+pfExec(pfStepExecCb)+timeoutMs(ulong)+ruleNum(uchar)+[ret,nextStep...](ushort*N)+...
 */
void vStepInit(emSmStepDevTdf emDevNum, uint8_t stepTotal, uint8_t isCycle, int endStep, ...)
{
    if (emDevNum >= STEP_M_NUM || stepTotal > STEP_M_MAX_STEP_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    va_list ap;
    va_start(ap, endStep); // 初始化可变参数列表

    // 1. 清空设备参数,初始化基础配置
    memset(pstDev, 0, sizeof(stStepDevParamTdf));
    pstDev->stepTotal = stepTotal;
    pstDev->isCycle = isCycle;
    pstDev->endStep = endStep;
    pstDev->mState = emStepMStateIdle;

    // 2. 解析可变参数,逐个加载步骤
    for (uint8_t i = 0; i < stepTotal; i++) {
        stStepItemTdf *pstStep = &pstDev->tSteps[i];
        // 解析步骤基础参数:StepID → 回调 → 超时时间 → 规则数
        pstStep->stepId = (uint16_t)va_arg(ap, int);
        pstStep->pfExec = (pfStepExecCb)va_arg(ap, pfStepExecCb);
        pstStep->timeoutMs = (uint32_t)va_arg(ap, long);
        pstStep->ruleNum = (uint8_t)va_arg(ap, int);
        if (pstStep->ruleNum > STEP_M_MAX_RULE_NUM) pstStep->ruleNum = STEP_M_MAX_RULE_NUM;

        // 解析跳转规则:结果1,下一步1, 结果2,下一步2...
        for (uint8_t j = 0; j < pstStep->ruleNum; j++) {
            pstStep->tRules[j].ret = (emStepRetTdf)va_arg(ap, int);
            pstStep->tRules[j].nextStep = (uint16_t)va_arg(ap, int);
        }

        // 初始化默认当前步骤
        if (i == 0) pstDev->curStep = pstStep->stepId;
    }

    va_end(ap); // 结束可变参数解析
}

/**
 * @brief 重置步骤机
 */
void vStepReset(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    pstDev->mState = emStepMStateIdle;
    pstDev->curStep = pstDev->tSteps[0].stepId; // 回到第一个步骤
    pstDev->stepStartTick = 0;
}

/**
 * @brief 启动步骤机流程
 */
void vStepStart(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    if (pstDev->mState == emStepMStateIdle || pstDev->mState == emStepMStateEnd) {
        pstDev->mState = emStepMStateRunning;
        pstDev->stepStartTick = HAL_GetTick();
    }
}

/**
 * @brief 停止步骤机流程
 */
void vStepStop(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return;
    astStepDev[emDevNum].mState = emStepMStateIdle;
}

/**
 * @brief 获取步骤机整体状态
 */
emStepMStateTdf emStepGetState(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return emStepMStateError;
    return astStepDev[emDevNum].mState;
}

/**
 * @brief 获取当前步骤ID
 */
uint16_t u16StepGetCurStep(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return 0xFFFF;
    return astStepDev[emDevNum].curStep;
}

/**
 * @brief 核心处理函数
 * @note  单次调用单次处理,自动完成「步骤执行→结果判断→步骤切换」
 */
void vStepProcess(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    if (pstDev->mState != emStepMStateRunning) return;

    // 1. 查找当前步骤缓存
    stStepItemTdf *pstCurStep = pstStepFind(emDevNum, pstDev->curStep);
    if (pstCurStep == NULL || pstCurStep->pfExec == NULL) {
        pstDev->mState = emStepMStateError;
        return;
    }

    // 2. 超时检测
    if (u8StepCheckTimeout(pstCurStep, pstDev->stepStartTick)) {
        pstDev->mState = emStepMStateError;
        return;
    }

    // 3. 执行步骤回调
    emStepRetTdf stepRet = pstCurStep->pfExec(emDevNum);
    if (stepRet >= emStepRetMax) stepRet = emStepRetError;

    // 4. 自动步骤切换
    switch (stepRet) {
        case emStepRetWait:
            // 等待:继续当前步骤,重置超时
            pstDev->stepStartTick = HAL_GetTick();
            break;
        case emStepRetOk:
        case emStepRetError: {
            // 查找下一步ID
            uint16_t nextStep = u16StepFindNext(pstCurStep, stepRet);
            if (nextStep == 0xFFFF) {
                pstDev->mState = emStepMStateError;
                break;
            }
            // 判断是否到达结束步骤
            if (nextStep == pstDev->endStep) {
                pstDev->mState = emStepMStateEnd;
                // 循环执行:重置为初始步骤
                if (pstDev->isCycle) {
                    pstDev->curStep = pstDev->tSteps[0].stepId;
                    pstDev->stepStartTick = HAL_GetTick();
                    pstDev->mState = emStepMStateRunning;
                }
            } else {
                // 自动切换下一步,重置超时
                pstDev->curStep = nextStep;
                pstDev->stepStartTick = HAL_GetTick();
            }
            break;
        }
        default:
            pstDev->mState = emStepMStateError;
            break;
    }
}

#endif
