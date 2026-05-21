/**
 * @file    step_machine.c
 * @brief   轻量化步骤机实现
 */
#include "step_machine.h"

#if STEP_M_ENABLE
// 全局设备参数数组
static stStepDevParamTdf astStepDev[STEP_M_NUM] = {0};

/**
 * @brief 静态函数:根据步骤ID查找索引
 * @return 找到返回索引，否则返回 0xFF
 */
static uint8_t u8StepFindIndex(emSmStepDevTdf emDevNum, uint16_t stepId)
{
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    for (uint8_t i = 0; i < pstDev->stepTotal; i++) {
        if (pstDev->tSteps[i].stepId == stepId) {
            return i;
        }
    }
    return 0xFF;
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
    return (QE_GET_TICK() - startTick >= pstItem->timeoutMs) ? 1 : 0;
}

/**
 * @brief 静态函数:执行步骤切换
 * @note  在关中断下执行，防止 ISR 中 vStepPeriodExecute 读到不完整的 curStep/curStepIndex/stepStartTick
 */
static void vStepSwitch(emSmStepDevTdf emDevNum, uint16_t nextStep)
{
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    uint32_t ulPrimask = __get_PRIMASK();
    __disable_irq();
    pstDev->curStep = nextStep;
    pstDev->curStepIndex = u8StepFindIndex(emDevNum, nextStep);
    pstDev->stepStartTick = QE_GET_TICK();
    __set_PRIMASK(ulPrimask);
}

/**
 * @brief 静态函数:处理错误跳转（避免代码重复）
 * @return 1=已跳转 0=没有错误步骤直接返回Error状态
 */
static uint8_t u8StepHandleError(emSmStepDevTdf emDevNum, stStepItemTdf *pstCurStep)
{
    uint16_t nextStep = u16StepFindNext(pstCurStep, emStepRetError);
    if (nextStep != 0xFFFF && u8StepFindIndex(emDevNum, nextStep) != 0xFF) {
        vStepSwitch(emDevNum, nextStep);
        return 1;
    }
    if (u8StepFindIndex(emDevNum, 0xFF) != 0xFF) {
        vStepSwitch(emDevNum, 0xFF);
        return 1;
    }
    return 0;
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
void vStepInit(emSmStepDevTdf emDevNum, uint8_t stepTotal, uint8_t isCycle, unsigned int endStep, ...)
{
    if (emDevNum >= STEP_M_NUM || stepTotal > STEP_M_MAX_STEP_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    va_list ap;
    va_start(ap, endStep); // 初始化可变参数列表

    // 1. 清空设备参数,初始化基础配置
    memset(pstDev, 0, sizeof(stStepDevParamTdf));
    pstDev->stepTotal = stepTotal;
    pstDev->isCycle = isCycle;
    pstDev->endStep = (uint16_t)endStep;
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

        // 初始化默认当前步骤和缓存索引
        if (i == 0) {
            pstDev->curStep = pstStep->stepId;
            pstDev->curStepIndex = 0;
        }
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
    pstDev->curStepIndex = 0; // 第一个步骤的索引总是0
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
        pstDev->stepStartTick = QE_GET_TICK();
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
 * @brief 周期执行函数(1ms定时器调用)
 * @note  负责超时检测，与vStepProcess分离，确保计时独立准确
 */
void vStepPeriodExecute(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    if (pstDev->mState != emStepMStateRunning) return;

    // 使用缓存的索引，避免每次遍历查找
    if (pstDev->curStepIndex >= pstDev->stepTotal) {
        pstDev->mState = emStepMStateError;
        return;
    }
    stStepItemTdf *pstCurStep = &pstDev->tSteps[pstDev->curStepIndex];

    if (u8StepCheckTimeout(pstCurStep, pstDev->stepStartTick)) {
        if (!u8StepHandleError(emDevNum, pstCurStep)) {
            pstDev->mState = emStepMStateError;
        }
    }
}

/**
 * @brief 核心处理函数(主循环调用)
 * @note  负责步骤执行，与vStepPeriodExecute分离
 */
void vStepProcess(emSmStepDevTdf emDevNum)
{
    if (emDevNum >= STEP_M_NUM) return;
    stStepDevParamTdf *pstDev = &astStepDev[emDevNum];
    if (pstDev->mState != emStepMStateRunning) return;

    // 使用缓存的索引，避免每次遍历查找
    if (pstDev->curStepIndex >= pstDev->stepTotal) {
        pstDev->mState = emStepMStateError;
        return;
    }
    stStepItemTdf *pstCurStep = &pstDev->tSteps[pstDev->curStepIndex];

    if (pstCurStep->pfExec == NULL) {
        pstDev->mState = emStepMStateError;
        return;
    }

    emStepRetTdf stepRet = pstCurStep->pfExec(emDevNum);
    if (stepRet >= emStepRetMax) stepRet = emStepRetError;

    switch (stepRet) {
        case emStepRetWait:
            break;
        case emStepRetOk: {
            uint16_t nextStep = u16StepFindNext(pstCurStep, stepRet);
            if (nextStep == 0xFFFF) {
                pstDev->mState = emStepMStateError;
                break;
            }
            if (nextStep == pstDev->endStep) {
                pstDev->mState = emStepMStateEnd;
                if (pstDev->isCycle) {
                    vStepSwitch(emDevNum, pstDev->tSteps[0].stepId);
                    pstDev->mState = emStepMStateRunning;
                }
            } else {
                vStepSwitch(emDevNum, nextStep);
            }
            break;
        }
        case emStepRetError: {
            if (!u8StepHandleError(emDevNum, pstCurStep)) {
                pstDev->mState = emStepMStateError;
            }
            break;
        }
        default:
            pstDev->mState = emStepMStateError;
            break;
    }
}

#endif
