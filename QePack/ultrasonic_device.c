/**
  * @file       ultrasonic_device.c
  * @author     Qe_xr
  * @version    V1.0.2
  * @date       2026/2/11
  * @brief      超声波测量驱动，基于 STM32 HAL 库
  * @note		思路: 通过占用定时器两个输入捕获通道(直接+间接)
  * @note		分别接收 Echo端 的上升沿和下降沿，这个定时器的计数值即为时间
  */

#include "ultrasonic_device.h"
#if ULTRASONIC_IS_ENABLE

#if (QEPACK_PLATFORM == ST)
    #include "delay.h"
#else
    #include "ti_msp_dl_config.h"
#endif


// 超声波设备参数数组
stUltrasonicDeviceParamTdf astUltrasonicDeviceParam[ULTRASONIC_DEV_NUM];

/**
 * @brief      获取超声波设备参数（只读）
 * @param      emDevNum   ：设备号
 * @return     const stUltrasonicDeviceParamTdf * ：指向设备参数的指针
 */
const stUltrasonicDeviceParamTdf *c_pstGetUltrasonicDeviceParam(emUltrasonicDevNumTdf emDevNum)
{
    if (emDevNum >= ULTRASONIC_DEV_NUM) return NULL;
    return &astUltrasonicDeviceParam[emDevNum];
}

/**
 * @brief      初始化超声波运行参数
 * @param      pstInit    ：初始化参数指针
 * @param      emDevNum   ：设备号
 */
void vUltrasonicDeviceRunningParamInit(stUltrasonicRunningParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum)
{
    memcpy(&astUltrasonicDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stUltrasonicRunningParamTdf));
}

/**
 * @brief      初始化超声波静态参数（硬件配置）
 * @param      pstInit    ：初始化参数指针
 * @param      emDevNum   ：设备号
 */
void vUltrasonicDeviceInit(stUltrasonicStaticParamTdf *pstInit, emUltrasonicDevNumTdf emDevNum)
{
    if (emDevNum >= ULTRASONIC_DEV_NUM || pstInit == NULL) return;

    // 拷贝静态参数（硬件配置）
    memcpy(&astUltrasonicDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stUltrasonicStaticParamTdf));
    
    // 默认运行参数初始化
    stUltrasonicRunningParamTdf stDefaultRunning = {
        .emCurrentStatus = emUltrasonicStatus_Idle,
        .ucIsSuccess = 0,
        .ulCCR1 = 0,
        .ulCCR2 = 0,
        .ulTimeoutMs = 200,   // 默认超时200ms
        .fDistance = 0.0f,
        .ulExpireTime = 0,
		.fTemperature = ULTRASONIC_DEFAULT_ENV_TEMP
    };
    vUltrasonicDeviceRunningParamInit(&stDefaultRunning, emDevNum);
}

/**
 * @brief      启动超声波单次测量
 * @param      emDevNum   ：设备号
 */
void vUltrasonicStartMeasure(emUltrasonicDevNumTdf emDevNum)
{
    if (emDevNum >= ULTRASONIC_DEV_NUM) return;
    stUltrasonicStaticParamTdf *pstStatic = &astUltrasonicDeviceParam[emDevNum].stStaticParam;
    stUltrasonicRunningParamTdf *pstRunning = &astUltrasonicDeviceParam[emDevNum].stRunningParam;
	
	// 状态检查
	if (pstRunning->emCurrentStatus == emUltrasonicStatus_Measuring) {
		return;
	}
    
    // 1. 置为测量中状态
    pstRunning->emCurrentStatus = emUltrasonicStatus_Measuring;
    pstRunning->ucIsSuccess = 0;
    pstRunning->fDistance = 0.0f;
    
    // 2. 重置定时器状态
#if (QEPACK_PLATFORM == ST)
    __HAL_TIM_SET_COUNTER(pstStatic->pstTimHandle, 0);
    __HAL_TIM_CLEAR_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC2);
    // 3. 启动输入捕获
    HAL_TIM_IC_Start(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
    HAL_TIM_IC_Start(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
    // 4. 发送触发信号（10us高电平）
    HAL_GPIO_WritePin(pstStatic->pstTrigGpioBase, pstStatic->usTrigGpioPin, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(pstStatic->pstTrigGpioBase, pstStatic->usTrigGpioPin, GPIO_PIN_RESET);
#else
    DL_Timer_setTimerCount(pstStatic->pstTimHandle->timer_inst, 0);
    TI_TIM_ClearFlag(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
    TI_TIM_ClearFlag(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
    TI_TIM_IC_Start(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
    TI_TIM_IC_Start(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
    TI_GPIO_WritePin(pstStatic->pstTrigGpioBase, pstStatic->usTrigGpioPin, GPIO_PIN_SET);
    TI_Delay_us(10);
    TI_GPIO_WritePin(pstStatic->pstTrigGpioBase, pstStatic->usTrigGpioPin, GPIO_PIN_RESET);
#endif
    
    // 5. 设置超时时间戳
    pstRunning->ulExpireTime = QE_GET_TICK() + pstRunning->ulTimeoutMs;
}

/**
 * @brief      计算测量距离（内部函数）
 * @param      emDevNum   ：设备号
 */
static void vUltrasonicCalcDistance(emUltrasonicDevNumTdf emDevNum)
{
	stUltrasonicStaticParamTdf *pstStatic = &astUltrasonicDeviceParam[emDevNum].stStaticParam;
    stUltrasonicRunningParamTdf *pstRunning = &astUltrasonicDeviceParam[emDevNum].stRunningParam;
    
    // 脉宽 = (CCR2 - CCR1) * 定时器计数周期
    float fPulseWidth = (pstRunning->ulCCR2 - pstRunning->ulCCR1) * pstStatic->fTimerPeriod;
    // 距离 = 声速(340m/s) * 脉宽 / 2（往返路程）
    // 旧方案:pstRunning->fDistance = 340.0f * fPulseWidth * 0.5f;
	// 新方案: 声速 = 声速基准值 + 温度系数 (声速随温度变化率, m/s/°C) * 默认环境温度 (用于无温度传感器时的默认值, °C)
	float fSpeedOfSound = 	ULTRASONIC_SOUND_SPEED_BASE 
							+ ULTRASONIC_SOUND_SPEED_TEMP_COEF * pstRunning->fTemperature;
	pstRunning->fDistance = fSpeedOfSound * fPulseWidth * 0.5f;
}

/**
 * @brief      超声波周期执行
 * @param      emDevNum   ：设备号
 */
void vUltrasonicDevicePeriodExecute(emUltrasonicDevNumTdf emDevNum)
{
    stUltrasonicStaticParamTdf *pstStatic = &astUltrasonicDeviceParam[emDevNum].stStaticParam;
    stUltrasonicRunningParamTdf *pstRunning = &astUltrasonicDeviceParam[emDevNum].stRunningParam;
    
    // 仅处理「测量中」状态
    if (pstRunning->emCurrentStatus != emUltrasonicStatus_Measuring)
    {
        return;
    }
    
    // 1. 先检查捕获标志（可能在上次轮询间隔中已完成）
#if (QEPACK_PLATFORM == ST)
    uint32_t ulCC1Flag = __HAL_TIM_GET_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC1);
    uint32_t ulCC2Flag = __HAL_TIM_GET_FLAG(pstStatic->pstTimHandle, TIM_FLAG_CC2);
#else
    uint32_t ulCC1Flag = TI_TIM_GetFlag(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
    uint32_t ulCC2Flag = TI_TIM_GetFlag(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
#endif
    if (ulCC1Flag && ulCC2Flag)
    {
        // 读取捕获值
#if (QEPACK_PLATFORM == ST)
        pstRunning->ulCCR1 = HAL_TIM_ReadCapturedValue(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        pstRunning->ulCCR2 = HAL_TIM_ReadCapturedValue(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
        // 停止输入捕获
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
#else
        pstRunning->ulCCR1 = TI_TIM_ReadCapturedValue(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        pstRunning->ulCCR2 = TI_TIM_ReadCapturedValue(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
        TI_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        TI_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
#endif

        // 计算距离
        vUltrasonicCalcDistance(emDevNum);

        // 更新状态
        pstRunning->emCurrentStatus = emUltrasonicStatus_Completed;
        pstRunning->ucIsSuccess = 1;
        return;
    }

    // 2. 再检查是否超时（捕获未完成且超时，才算失败）
    if (QE_GET_TICK() >= pstRunning->ulExpireTime)
    {
        // 停止输入捕获
#if (QEPACK_PLATFORM == ST)
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        HAL_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
#else
        TI_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel1);
        TI_TIM_IC_Stop(pstStatic->pstTimHandle, pstStatic->ulICChannel2);
#endif

        // 更新状态
        pstRunning->emCurrentStatus = emUltrasonicStatus_Timeout;
        pstRunning->ucIsSuccess = 0;
    }
}

/**
 * @brief      获取超声波测量距离
 * @param      emDevNum   ：设备号
 * @return     测量距离（单位：米）
 */
float fUltrasonicGetDistance(emUltrasonicDevNumTdf emDevNum)
{
    if (emDevNum >= ULTRASONIC_DEV_NUM) return 0.0f;
    return astUltrasonicDeviceParam[emDevNum].stRunningParam.fDistance;
}

uint8_t ucUltrasonicIsMeasureSuccess(emUltrasonicDevNumTdf emDevNum)
{
    if (emDevNum >= ULTRASONIC_DEV_NUM) return 0;
    return astUltrasonicDeviceParam[emDevNum].stRunningParam.ucIsSuccess;
}

/* ==================== 传感器基类适配 ==================== */

#if SENSOR_IS_ENABLE

#include "sensor_device.h"

#define ULTRASONIC_SENSOR_LOCAL_MAX  4
#define ULTRASONIC_SENSOR_TO_LOCAL(dev)  ((uint8_t)((dev) - emSensorUltrasonicDevNum0))

typedef struct {
    stSensorDeviceTdf      stBase;
    emSensorDevNumTdf      emSensorDevNum;
    fix32_t                fTargetValue;
} stUltrasonicSensorWrapperTdf;

static void vUltrasonicSensorInit(void *pstSensor)
{
    (void)pstSensor;
}

static void vUltrasonicSensorPeriodExecute(void *pstSensor)
{
    stUltrasonicSensorWrapperTdf *pstWrapper = (stUltrasonicSensorWrapperTdf *)pstSensor;
    emUltrasonicDevNumTdf emLocalDev = (emUltrasonicDevNumTdf)ULTRASONIC_SENSOR_TO_LOCAL(pstWrapper->emSensorDevNum);
    if (emLocalDev >= ULTRASONIC_DEV_NUM) return;

    /* 仅空闲时自动启动测量，Completed/Timeout 状态保留结果供 fGetValue 读取 */
    emUltrasonicStatusTdf emStatus = astUltrasonicDeviceParam[emLocalDev].stRunningParam.emCurrentStatus;
    if (emStatus == emUltrasonicStatus_Idle) {
        vUltrasonicStartMeasure(emLocalDev);
    }

    /* 推进状态机 */
    vUltrasonicDevicePeriodExecute(emLocalDev);
}

static fix32_t fUltrasonicSensorGetValue(void *pstSensor)
{
    stUltrasonicSensorWrapperTdf *pstWrapper = (stUltrasonicSensorWrapperTdf *)pstSensor;
    emUltrasonicDevNumTdf emLocalDev = (emUltrasonicDevNumTdf)ULTRASONIC_SENSOR_TO_LOCAL(pstWrapper->emSensorDevNum);
    if (emLocalDev >= ULTRASONIC_DEV_NUM) return FIX32_ZERO;
    return fix32_from_float(astUltrasonicDeviceParam[emLocalDev].stRunningParam.fDistance);
}

static fix32_t fUltrasonicSensorGetAccumulatedValue(void *pstSensor)
{
    /* 距离无累加概念，与 fGetValue 相同 */
    return fUltrasonicSensorGetValue(pstSensor);
}

static void vUltrasonicSensorReset(void *pstSensor)
{
    stUltrasonicSensorWrapperTdf *pstWrapper = (stUltrasonicSensorWrapperTdf *)pstSensor;
    emUltrasonicDevNumTdf emLocalDev = (emUltrasonicDevNumTdf)ULTRASONIC_SENSOR_TO_LOCAL(pstWrapper->emSensorDevNum);
    if (emLocalDev >= ULTRASONIC_DEV_NUM) return;

    astUltrasonicDeviceParam[emLocalDev].stRunningParam.emCurrentStatus = emUltrasonicStatus_Idle;
    astUltrasonicDeviceParam[emLocalDev].stRunningParam.ucIsSuccess = 0;
    astUltrasonicDeviceParam[emLocalDev].stRunningParam.fDistance = 0.0f;
    pstWrapper->fTargetValue = FIX32_ZERO;
}

static void vUltrasonicSensorSetTarget(void *pstSensor, fix32_t fTarget)
{
    stUltrasonicSensorWrapperTdf *pstWrapper = (stUltrasonicSensorWrapperTdf *)pstSensor;
    pstWrapper->fTargetValue = fTarget;
}

static fix32_t fUltrasonicSensorGetTarget(void *pstSensor)
{
    stUltrasonicSensorWrapperTdf *pstWrapper = (stUltrasonicSensorWrapperTdf *)pstSensor;
    return pstWrapper->fTargetValue;
}

static stSensorVTableTdf g_stUltrasonicSensorVTable = {
    vUltrasonicSensorInit,
    vUltrasonicSensorPeriodExecute,
    fUltrasonicSensorGetValue,
    fUltrasonicSensorGetAccumulatedValue,
    vUltrasonicSensorReset,
    vUltrasonicSensorSetTarget,
    fUltrasonicSensorGetTarget,
};

static stUltrasonicSensorWrapperTdf g_astUltrasonicSensorDevices[ULTRASONIC_SENSOR_LOCAL_MAX];

void vUltrasonicSensorRegister(emSensorDevNumTdf emSensorDevNum, void *pstInit)
{
    (void)pstInit;
    uint8_t ucLocalIdx = ULTRASONIC_SENSOR_TO_LOCAL(emSensorDevNum);
    if (ucLocalIdx >= ULTRASONIC_SENSOR_LOCAL_MAX) return;

    stUltrasonicSensorWrapperTdf *pstWrapper = &g_astUltrasonicSensorDevices[ucLocalIdx];
    memset(pstWrapper, 0, sizeof(stUltrasonicSensorWrapperTdf));

    pstWrapper->stBase.emType          = emSensorTypeUltrasonic;
    pstWrapper->stBase.pstVTable       = &g_stUltrasonicSensorVTable;
    pstWrapper->stBase.ucEnable        = 1;
    pstWrapper->stBase.fWeight         = FIX32_ONE;
    pstWrapper->stBase.emPidDevNum     = emNoPid;
    pstWrapper->stBase.usPidPeriodMs   = 0;
    pstWrapper->stBase.ulPidLastTickMs = 0;
    pstWrapper->emSensorDevNum         = emSensorDevNum;

    vSensorRegisterDevice(emSensorDevNum, &pstWrapper->stBase);
}

#endif /* SENSOR_IS_ENABLE */

#endif
