/**
  * @file       joystick_device.c
  * @author     QePack
  * @version    V2.0.0
  * @date       2026/05/28
  * @brief      PS2 双轴摇杆驱动实现（定点数版本）
  *             读取两路 ADC 值，线性映射到用户指定的输出范围
  */
#include "joystick_device.h"
#if JOYSTICK_IS_ENABLE

static stJoystickDeviceParamTdf astJoystickDeviceParam[JOYSTICK_DEV_NUM];

const stJoystickDeviceParamTdf *c_pstGetJoystickDeviceParam(emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM) return NULL;
    return &astJoystickDeviceParam[emDevNum];
}

void vJoystickDeviceInit(stJoystickStaticParamTdf *pstInit, emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM || pstInit == NULL) return;

    memcpy(&astJoystickDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stJoystickStaticParamTdf));
    memset(&astJoystickDeviceParam[emDevNum].stRunningParam, 0, sizeof(stJoystickRunningParamTdf));

    /* 填充未设置的默认值 */
    stJoystickStaticParamTdf *pstStatic = &astJoystickDeviceParam[emDevNum].stStaticParam;
    if (pstStatic->fOutputMin >= pstStatic->fOutputMax) {
        pstStatic->fOutputMin = JOYSTICK_DEFAULT_OUTPUT_MIN;
        pstStatic->fOutputMax = JOYSTICK_DEFAULT_OUTPUT_MAX;
    }
}

void vJoystickPeriodExecute(emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM) return;

    stJoystickStaticParamTdf  *pstStatic  = &astJoystickDeviceParam[emDevNum].stStaticParam;
    stJoystickRunningParamTdf *pstRunning = &astJoystickDeviceParam[emDevNum].stRunningParam;

#if ADC_IS_USE_DMA
    /* DMA 模式：从 DMA 缓存中直接读取 X/Y 通道值 */
    if (emAdcGetDataState(pstStatic->emAdcDevNumX) == UPDATED) {
        const stAdcDeviceParamTdf *pstAdcParam = c_pstGetAdcDeviceParam(pstStatic->emAdcDevNumX);
        if (pstAdcParam != NULL) {
            uint16_t *pusBuf = pstAdcParam->stStaticParam.pulDmaBuffer;
            if (pusBuf != NULL) {
                pstRunning->usXAdcRaw = pusBuf[pstStatic->ucXAdcChannel];
                pstRunning->usYAdcRaw = pusBuf[pstStatic->ucYAdcChannel];
            }
        }
        vAdcStart(pstStatic->emAdcDevNumX);
    }
#else
    /* 非 DMA 模式：逐通道启动转换并轮询等待完成 */
    #if (QEPACK_PLATFORM == TI)
        vAdcStart(pstStatic->emAdcDevNumX, DL_ADC12_MEM_IDX_0);
        TI_ADC_PollForConversion(pstStatic->emAdcDevNumX, ADC_CONVERSION_TIMEOUT_MS);
        pstRunning->usXAdcRaw = usADCGetValue(pstStatic->emAdcDevNumX);

        vAdcStart(pstStatic->emAdcDevNumY, DL_ADC12_MEM_IDX_0);
        TI_ADC_PollForConversion(pstStatic->emAdcDevNumY, ADC_CONVERSION_TIMEOUT_MS);
        pstRunning->usYAdcRaw = usADCGetValue(pstStatic->emAdcDevNumY);
    #else
        vAdcStart(pstStatic->emAdcDevNumX, ADC_CHANNEL_0);
        {
            ADC_HandleTypeDef *hadc = stGetAdcHandle(pstStatic->emAdcDevNumX);
            if (hadc != NULL) {
                HAL_ADC_PollForConversion(hadc, ADC_CONVERSION_TIMEOUT_MS);
            }
        }
        pstRunning->usXAdcRaw = usADCGetValue(pstStatic->emAdcDevNumX);

        vAdcStart(pstStatic->emAdcDevNumY, ADC_CHANNEL_1);
        {
            ADC_HandleTypeDef *hadc = stGetAdcHandle(pstStatic->emAdcDevNumY);
            if (hadc != NULL) {
                HAL_ADC_PollForConversion(hadc, ADC_CONVERSION_TIMEOUT_MS);
            }
        }
        pstRunning->usYAdcRaw = usADCGetValue(pstStatic->emAdcDevNumY);
    #endif
#endif

    /* 定点数线性映射：[0, ADC_RESOLUTION] -> [fOutputMin, fOutputMax] */
    fix32_t fAdcMax = FIX32_FROM_INT(ADC_RESOLUTION);
    pstRunning->fXValue = fix32_map(FIX32_FROM_INT(pstRunning->usXAdcRaw),
                                    0, fAdcMax,
                                    pstStatic->fOutputMin, pstStatic->fOutputMax);
    pstRunning->fYValue = fix32_map(FIX32_FROM_INT(pstRunning->usYAdcRaw),
                                    0, fAdcMax,
                                    pstStatic->fOutputMin, pstStatic->fOutputMax);

    /* 反向处理 */
    if (pstStatic->emInvertX) {
        pstRunning->fXValue = pstStatic->fOutputMax - pstRunning->fXValue + pstStatic->fOutputMin;
    }
    if (pstStatic->emInvertY) {
        pstRunning->fYValue = pstStatic->fOutputMax - pstRunning->fYValue + pstStatic->fOutputMin;
    }
}

fix32_t fJoystickGetX(emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM) return 0;
    return astJoystickDeviceParam[emDevNum].stRunningParam.fXValue;
}

fix32_t fJoystickGetY(emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM) return 0;
    return astJoystickDeviceParam[emDevNum].stRunningParam.fYValue;
}

uint16_t usJoystickGetRawX(emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM) return 0;
    return astJoystickDeviceParam[emDevNum].stRunningParam.usXAdcRaw;
}

uint16_t usJoystickGetRawY(emJoystickDevNumTdf emDevNum)
{
    if (emDevNum >= JOYSTICK_DEV_NUM) return 0;
    return astJoystickDeviceParam[emDevNum].stRunningParam.usYAdcRaw;
}

#endif
