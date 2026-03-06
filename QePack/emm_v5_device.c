/**
  * @file       emm_v5_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm_V5步进闭环电机驱动，基于 STM32 HAL 库
  *
  */
#include "emm_v5_device.h"

#if EMM_V5_IS_ENABLE

static stEmmV5DeviceParamTdf g_stEmmV5Device;

/**
 * @brief  UART数据接收回调函数
 * @param  emDevNum ：UART设备号
 * @param  pstRunningParam ：UART运行参数指针
 */
static void vEmmV5UartCallback(emUartDevNumTdf emDevNum, stUartRunningParamTdf* pstRunningParam)
{
    if (emDevNum == g_stEmmV5Device.stStaticParam.emUartDevNum)
    {
        uint16_t usLen = usRingBufferGetCount(&pstRunningParam->stUartTempBuffer);
        if (usLen > 0 && usLen <= sizeof(g_stEmmV5Device.stRunningParam.aucRxFrameBuf))
        {
            g_stEmmV5Device.stRunningParam.usRxFrameLen = usRingBufferRead(
                &pstRunningParam->stUartTempBuffer,
                g_stEmmV5Device.stRunningParam.aucRxFrameBuf,
                usLen
            );
            g_stEmmV5Device.stRunningParam.ucRxFrameComplete = 1;
        }
    }
}

/**
 * @brief  Emm_V5初始化
 * @param  pstInit ：静态参数指针
 */
void vEmmV5DeviceInit(stEmmV5StaticParamTdf *pstInit)
{
    g_stEmmV5Device.stStaticParam = *pstInit;
    g_stEmmV5Device.stRunningParam.ucAddr = 0;
    g_stEmmV5Device.stRunningParam.usRxFrameLen = 0;
    g_stEmmV5Device.stRunningParam.ucRxFrameComplete = 0;
    memset(g_stEmmV5Device.stRunningParam.aucRxFrameBuf, 0, sizeof(g_stEmmV5Device.stRunningParam.aucRxFrameBuf));
    
    vUartRegisterCallback(vEmmV5UartCallback);
}

/**
 * @brief  Emm_V5周期执行
 */
void vEmmV5DevicePeriodExecute(void)
{
    if (g_stEmmV5Device.stRunningParam.ucRxFrameComplete)
    {
        if (g_stEmmV5Device.stStaticParam.vCallbackFcn != NULL)
        {
            g_stEmmV5Device.stStaticParam.vCallbackFcn(
                g_stEmmV5Device.stRunningParam.aucRxFrameBuf[0],
                g_stEmmV5Device.stRunningParam.aucRxFrameBuf,
                g_stEmmV5Device.stRunningParam.usRxFrameLen
            );
        }
        g_stEmmV5Device.stRunningParam.ucRxFrameComplete = 0;
    }
}

/**
 * @brief  将当前位置清零
 * @param  ucAddr ：电机地址
 */
void vEmmV5ResetCurPosToZero(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x0A;
    aucCmd[2] = 0x6D;
    aucCmd[3] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 4);
}

/**
 * @brief  解除堵转保护
 * @param  ucAddr ：电机地址
 */
void vEmmV5ResetClogPro(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x0E;
    aucCmd[2] = 0x52;
    aucCmd[3] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 4);
}

/**
 * @brief  读取系统参数
 * @param  ucAddr ：电机地址
 * @param  emSysParam ：系统参数类型
 */
void vEmmV5ReadSysParams(uint8_t ucAddr, emEmmV5SysParamTdf emSysParam)
{
    uint8_t aucCmd[16] = {0};
    uint8_t ucIndex = 0;
    
    aucCmd[ucIndex++] = ucAddr;
    
    switch(emSysParam)
    {
        case emEmmV5SysParam_Ver   : aucCmd[ucIndex++] = 0x1F; break;
        case emEmmV5SysParam_RL    : aucCmd[ucIndex++] = 0x20; break;
        case emEmmV5SysParam_PID   : aucCmd[ucIndex++] = 0x21; break;
        case emEmmV5SysParam_VBus  : aucCmd[ucIndex++] = 0x24; break;
        case emEmmV5SysParam_Cpha  : aucCmd[ucIndex++] = 0x27; break;
        case emEmmV5SysParam_Encl  : aucCmd[ucIndex++] = 0x31; break;
        case emEmmV5SysParam_TPos  : aucCmd[ucIndex++] = 0x33; break;
        case emEmmV5SysParam_Vel   : aucCmd[ucIndex++] = 0x35; break;
        case emEmmV5SysParam_CPos  : aucCmd[ucIndex++] = 0x36; break;
        case emEmmV5SysParam_PErr  : aucCmd[ucIndex++] = 0x37; break;
        case emEmmV5SysParam_Flag  : aucCmd[ucIndex++] = 0x3A; break;
        case emEmmV5SysParam_Org   : aucCmd[ucIndex++] = 0x3B; break;
        case emEmmV5SysParam_Conf  : aucCmd[ucIndex++] = 0x42; aucCmd[ucIndex++] = 0x6C; break;
        case emEmmV5SysParam_State : aucCmd[ucIndex++] = 0x43; aucCmd[ucIndex++] = 0x7A; break;
        default: break;
    }
    
    aucCmd[ucIndex++] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, ucIndex);
}

/**
 * @brief  修改开环/闭环控制模式
 * @param  ucAddr ：电机地址
 * @param  bSave ：存储标志
 * @param  emCtrlMode ：控制模式
 */
void vEmmV5ModifyCtrlMode(uint8_t ucAddr, uint8_t bSave, emEmmV5CtrlModeTdf emCtrlMode)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x46;
    aucCmd[2] = 0x69;
    aucCmd[3] = bSave;
    aucCmd[4] = emCtrlMode;
    aucCmd[5] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 6);
}

/**
 * @brief  电机使能控制
 * @param  ucAddr ：电机地址
 * @param  bEnable ：使能状态，true为使能电机，false为关闭电机
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5EnControl(uint8_t ucAddr, uint8_t bEnable, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xF3;
    aucCmd[2] = 0xAB;
    aucCmd[3] = bEnable;
    aucCmd[4] = bSyncFlag;
    aucCmd[5] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 6);
}

/**
 * @brief  速度模式控制
 * @param  ucAddr ：电机地址
 * @param  emDir ：方向
 * @param  usVel ：速度，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5VelControl(uint8_t ucAddr, emEmmV5DirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xF6;
    aucCmd[2] = emDir;
    aucCmd[3] = (uint8_t)(usVel >> 8);
    aucCmd[4] = (uint8_t)(usVel >> 0);
    aucCmd[5] = ucAcc;
    aucCmd[6] = bSyncFlag;
    aucCmd[7] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 8);
}

/**
 * @brief  位置模式控制
 * @param  ucAddr ：电机地址
 * @param  emDir ：方向
 * @param  usVel ：速度(RPM)，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  ulClk ：脉冲数，范围0- (2^32 - 1)个
 * @param  bAbsFlag ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5PosControl(uint8_t ucAddr, emEmmV5DirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag)
{
    uint8_t aucCmd[32] = {0};
    
    aucCmd[0]  = ucAddr;
    aucCmd[1]  = 0xFD;
    aucCmd[2]  = emDir;
    aucCmd[3]  = (uint8_t)(usVel >> 8);
    aucCmd[4]  = (uint8_t)(usVel >> 0);
    aucCmd[5]  = ucAcc;
    aucCmd[6]  = (uint8_t)(ulClk >> 24);
    aucCmd[7]  = (uint8_t)(ulClk >> 16);
    aucCmd[8]  = (uint8_t)(ulClk >> 8);
    aucCmd[9]  = (uint8_t)(ulClk >> 0);
    aucCmd[10] = bAbsFlag;
    aucCmd[11] = bSyncFlag;
    aucCmd[12] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 13);
}

/**
 * @brief  让电机立即停止运动
 * @param  ucAddr ：电机地址
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5StopNow(uint8_t ucAddr, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xFE;
    aucCmd[2] = 0x98;
    aucCmd[3] = bSyncFlag;
    aucCmd[4] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 5);
}

/**
 * @brief  触发多机同步开始运动
 * @param  ucAddr ：电机地址
 */
void vEmmV5SynchronousMotion(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xFF;
    aucCmd[2] = 0x66;
    aucCmd[3] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 4);
}

/**
 * @brief  设置单圈回零的零点位置
 * @param  ucAddr ：电机地址
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 */
void vEmmV5OriginSetO(uint8_t ucAddr, uint8_t bSave)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x93;
    aucCmd[2] = 0x88;
    aucCmd[3] = bSave;
    aucCmd[4] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 5);
}

/**
 * @brief  修改回零参数
 * @param  ucAddr ：电机地址
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 * @param  emOrgMode ：回零模式
 * @param  emDir ：回零方向
 * @param  usOrgVel ：回零速度，单位：RPM（转/分钟）
 * @param  ulOrgTm ：回零超时时间，单位：毫秒
 * @param  usSlVel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param  usSlMa ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param  usSlMs ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param  bPotFlag ：上电自动触发回零，false为不使能，true为使能
 */
void vEmmV5OriginModifyParams(uint8_t ucAddr, uint8_t bSave, emEmmV5OrgModeTdf emOrgMode, emEmmV5DirTdf emDir, 
                               uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                               uint16_t usSlMs, uint8_t bPotFlag)
{
    uint8_t aucCmd[32] = {0};
    
    aucCmd[0]  = ucAddr;
    aucCmd[1]  = 0x4C;
    aucCmd[2]  = 0xAE;
    aucCmd[3]  = bSave;
    aucCmd[4]  = emOrgMode;
    aucCmd[5]  = emDir;
    aucCmd[6]  = (uint8_t)(usOrgVel >> 8);
    aucCmd[7]  = (uint8_t)(usOrgVel >> 0);
    aucCmd[8]  = (uint8_t)(ulOrgTm >> 24);
    aucCmd[9]  = (uint8_t)(ulOrgTm >> 16);
    aucCmd[10] = (uint8_t)(ulOrgTm >> 8);
    aucCmd[11] = (uint8_t)(ulOrgTm >> 0);
    aucCmd[12] = (uint8_t)(usSlVel >> 8);
    aucCmd[13] = (uint8_t)(usSlVel >> 0);
    aucCmd[14] = (uint8_t)(usSlMa >> 8);
    aucCmd[15] = (uint8_t)(usSlMa >> 0);
    aucCmd[16] = (uint8_t)(usSlMs >> 8);
    aucCmd[17] = (uint8_t)(usSlMs >> 0);
    aucCmd[18] = bPotFlag;
    aucCmd[19] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 20);
}

/**
 * @brief  触发回零
 * @param  ucAddr ：电机地址
 * @param  emOrgMode ：回零模式
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5OriginTriggerReturn(uint8_t ucAddr, emEmmV5OrgModeTdf emOrgMode, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x9A;
    aucCmd[2] = emOrgMode;
    aucCmd[3] = bSyncFlag;
    aucCmd[4] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 5);
}

/**
 * @brief  强制中断并退出回零
 * @param  ucAddr ：电机地址
 */
void vEmmV5OriginInterrupt(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x9C;
    aucCmd[2] = 0x48;
    aucCmd[3] = 0x6B;
    
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, aucCmd, 4);
}

#endif
