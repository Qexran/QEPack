/**
  * @file       emm_v5_example.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm_V5模块使用示例
  *
  */
#include "emm_v5_device.h"

#if EMM_V5_IS_ENABLE

/// @brief  Emm_V5数据接收回调函数
/// @param  ucAddr ：电机地址
/// @param  pucData ：接收到的数据指针
/// @param  usLen ：接收到的数据长度
void vEmmV5RxCallback(uint8_t ucAddr, uint8_t *pucData, uint16_t usLen)
{
    // 示例：处理读取实时位置的返回数据
    if (ucAddr == 1 && pucData[1] == 0x36 && usLen == 8)
    {
        uint32_t ulPos;
        float fMotorCurPos;
        
        // 拼接成uint32_t类型
        ulPos = (uint32_t)(
                  ((uint32_t)pucData[3] << 24) |
                  ((uint32_t)pucData[4] << 16) |
                  ((uint32_t)pucData[5] << 8)  |
                  ((uint32_t)pucData[6] << 0)
                );
        
        // 转换成角度
        fMotorCurPos = (float)ulPos * 360.0f / 65536.0f;
        
        // 符号
        if (pucData[2])
        {
            fMotorCurPos = -fMotorCurPos;
        }
        
        // 在这里处理位置数据...
    }
}

/// @brief  Emm_V5使用示例
void vEmmV5Example(void)
{
    // 1. 初始化UART（假设已经在其他地方初始化了UART1）
    // ...
    
    // 2. 初始化Emm_V5静态参数
    stEmmV5StaticParamTdf stEmmV5Static = {
        .emUartDevNum = emUartDevNum1,  // 使用UART1
        .vCallbackFcn = vEmmV5RxCallback  // 设置回调函数
    };
    
    // 3. 初始化Emm_V5
    vEmmV5DeviceInit(&stEmmV5Static);
    
    // 4. 上电延时2秒等待Emm_V5.0闭环初始化完毕
    HAL_Delay(2000);
    
    // 5. 位置模式：方向CW，速度1000RPM，加速度0（不使用加减速直接启动），脉冲数3200（16细分下发送3200个脉冲电机转一圈），相对运动
    vEmmV5PosControl(1, emEmmV5Dir_CW, 1000, 0, 3200, 0, 0);
    
    // 6. 延时2秒，等待运动完成
    HAL_Delay(2000);
    
    // 7. 读取电机实时位置
    vEmmV5ReadSysParams(1, emEmmV5SysParam_CPos);
    
    // 8. 在主循环中调用vEmmV5DevicePeriodExecute()处理接收数据
    while (1)
    {
        vEmmV5DevicePeriodExecute();
        HAL_Delay(10);
    }
}

#endif
