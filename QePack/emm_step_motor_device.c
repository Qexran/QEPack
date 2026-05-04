/**
  * @file       emm_step_motor_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm步进闭环电机驱动，基于 STM32 HAL 库
  *
  */
#include "emm_step_motor_device.h"

#if EMM_MOTOR_IS_ENABLE

stEmmMotorDeviceParamTdf astEmmMotorDeviceParam[EMM_MOTOR_DEV_NUM];

#define EMM_MOTOR_CMD_END_MARK 0x6B


/**
 * @brief  通用命令发送辅助函数
 * @param  pstEmmMotor ：EmmMotor设备指针
 * @param  pucCmdData ：命令数据指针
 * @param  ucDataLen ：命令数据长度（不含地址字节和结束标志）
 */
static void vEmmMotorSendCmd(
    stEmmMotorDeviceParamTdf *pstEmmMotor,
    const uint8_t *pucCmdData, uint8_t ucDataLen
) {
    uint8_t aucCmd[32];
    uint8_t ucIndex = 0;
    
    aucCmd[ucIndex++] = pstEmmMotor->stStaticParam.ucAddr;
    for (uint8_t i = 0; i < ucDataLen; i++) {
        aucCmd[ucIndex++] = pucCmdData[i];
    }
    aucCmd[ucIndex++] = EMM_MOTOR_CMD_END_MARK;
    
    vUartSendArray(pstEmmMotor->stStaticParam.emUartDevNum, aucCmd, ucIndex);
}

/**
 * @brief EmmMotor虚方法表
 */
static stMotorVTableTdf g_stEmmMotorVTable = {
    vEmmMotorInit,              /* 对应vInit(void *pstInit); */
    vEmmMotorPeriodExecute,     /* 对应vPeriodExecute(void *pstPeriodExecute); */
    vEmmMotorStop,              /* 对应vStop(void *pstMotor, uint8_t bSyncFlag); */
    vEmmMotorEnable,            /* 对应vEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag); */
    emGetEmmMotorState,         /* 对应emGetEmmMotorState(void *pstMotor); */
    vEmmMotorPosControl,        /* 对应vPosControl(...); */
    vEmmMotorVelControl,        /* 对应vVelControl(...); */
};

/* 虚方法实现 ************************************* */

/**
 * @brief  EmmMotor初始化
 * @param  pstInit ：EmmMotor设备指针
 * @note   实现父类方法 vInit(void *pstInit);
 */
void vEmmMotorInit(void *pstInit)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstInit;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    // pstEmmMotor->stRunningParam.ucAddr = 0;
    pstEmmMotor->stRunningParam.usRxFrameLen = 0;
    pstEmmMotor->stRunningParam.ucRxFrameComplete = 0;
    memset(pstEmmMotor->stRunningParam.aucRxFrameBuf, 0, sizeof(pstEmmMotor->stRunningParam.aucRxFrameBuf));
}

/**
 * @brief  EmmMotor周期执行
 * @param  pstMotor ：EmmMotor设备指针
 * @note   实现父类方法 vPeriodExecute(void *pstPeriodExecute);
 */
void vEmmMotorPeriodExecute(void *pstMotor)
{
    // stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    // if (pstEmmMotor == NULL) {
    //     return;
    // }
    
    // if (pstEmmMotor->stRunningParam.ucRxFrameComplete)
    // {
    //     if (pstEmmMotor->stStaticParam.vCallbackFcn != NULL)
    //     {
    //         pstEmmMotor->stStaticParam.vCallbackFcn(
    //             pstEmmMotor->stRunningParam.aucRxFrameBuf[0],
    //             pstEmmMotor->stRunningParam.aucRxFrameBuf,
    //             pstEmmMotor->stRunningParam.usRxFrameLen
    //         );
    //     }
    //     pstEmmMotor->stRunningParam.ucRxFrameComplete = 0;
    // }
}

/**
 * @brief  EmmMotor使能控制
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bEnable ：使能状态
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 * @note   实现父类方法 vEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag);
 */
void vEmmMotorEnable(void *pstMotor, uint8_t bEnable, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0xF3, 0xAB, bEnable, bSyncFlag};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}


/**
 * @brief 获取EmmMotor电机运动状态
 * @param pstMotor EmmMotor电机设备指针
 * @return emMotorStateTdf 电机运动状态
 * @note   实现父类方法 emGetEmmMotorState(void *pstMotor);
 */
emMotorStateTdf emGetEmmMotorState(void *pstMotor) {
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return emMotorStateNULL;
    }
    return pstEmmMotor->stBase.emMotorState;
}

/**
 * @brief  位置模式控制
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emDir ：方向
 * @param  usVel ：速度(RPM)，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  ulClk ：脉冲数，范围0- (2^32 - 1)个
 * @param  bAbsFlag ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 * @note   实现父类方法 vPosControl(...);
 */
void vEmmMotorPosControl(
    void *pstMotor, emMotorDirTdf emDir, 
    uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag, 
    uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {
        0xFD,
        emDir,
        (uint8_t)(usVel >> 8),
        (uint8_t)(usVel >> 0),
        ucAcc,
        (uint8_t)(ulClk >> 24),
        (uint8_t)(ulClk >> 16),
        (uint8_t)(ulClk >> 8),
        (uint8_t)(ulClk >> 0),
        bAbsFlag,
        bSyncFlag
    };
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}


/**
 * @brief  速度模式控制
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emDir ：方向
 * @param  usVel ：速度，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 * @note   实现父类方法 vVelControl(...);
 */
void vEmmMotorVelControl(void *pstMotor, emMotorDirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {
        0xF6,
        emDir,
        (uint8_t)(usVel >> 8),
        (uint8_t)(usVel >> 0),
        ucAcc,
        bSyncFlag
    };
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/******************************************************/


/**
 * @brief  设置EmmMotor速度
 * @param  pstMotor ：EmmMotor设备指针
 * @param  speed ：速度值（RPM）
 */
void vEmmMotorSetSpeed(void *pstMotor, int16_t speed)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    emMotorDirTdf emDir = (speed >= 0) ? emMotorDir_Forward : emMotorDir_Backward;
    uint16_t usVel = (speed < 0) ? -speed : speed;
    
    // 调用速度控制函数
    vEmmMotorVelControl(pstMotor, emDir, usVel, 10, 0);
}

/**
 * @brief  停止EmmMotor
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmMotorStop(void *pstMotor, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0xFE, 0x98, bSyncFlag};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}



/**
 * @brief  注册EmmMotor设备
 * @param  emDevNum ：设备号
 * @param  pstInit ：静态参数
 */
void vEmmMotorRegister(emMotorDevNumTdf emDevNum, stEmmMotorStaticParamTdf *pstInit)
{   
    emMotorDevNumTdf offsetDevNum = emDevNum - emEmmMotorDevNum0;
    
    if (offsetDevNum < EMM_MOTOR_DEV_NUM && pstInit != NULL) {

        // 初始化基类
        astEmmMotorDeviceParam[offsetDevNum].stBase.emType = emMotorType_Emm;
        astEmmMotorDeviceParam[offsetDevNum].stBase.pstVTable = &g_stEmmMotorVTable;

        memcpy(&astEmmMotorDeviceParam[offsetDevNum].stStaticParam, 
           pstInit, 
           sizeof(stEmmMotorStaticParamTdf));
    
        memset(&astEmmMotorDeviceParam[offsetDevNum].stRunningParam, 
            0, 
            sizeof(stEmmMotorRunningParamTdf));
        
        // 注册到基类
        vMotorRegisterDevice(emDevNum, &astEmmMotorDeviceParam[offsetDevNum].stBase);

    }
}

/**
 * @brief  将当前位置清零
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorResetCurPosToZero(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x0A, 0x6D};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/**
 * @brief  解除堵转保护
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorResetClogPro(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x0E, 0x52};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/**
 * @brief  读取系统参数
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emSysParam ：系统参数类型
 */
void vEmmMotorReadSysParams(void *pstMotor, emEmmMotorSysParamTdf emSysParam)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    // 参数码表：单字节参数和双字节参数
    static const uint8_t aucParamCode[][2] = {
        [emEmmMotorSysParam_Ver]   = {0x1F, 0x00},
        [emEmmMotorSysParam_RL]    = {0x20, 0x00},
        [emEmmMotorSysParam_PID]   = {0x21, 0x00},
        [emEmmMotorSysParam_VBus]  = {0x24, 0x00},
        [emEmmMotorSysParam_Cpha]  = {0x27, 0x00},
        [emEmmMotorSysParam_Encl]  = {0x31, 0x00},
        [emEmmMotorSysParam_TPos]  = {0x33, 0x00},
        [emEmmMotorSysParam_Vel]   = {0x35, 0x00},
        [emEmmMotorSysParam_CPos]  = {0x36, 0x00},
        [emEmmMotorSysParam_PErr]  = {0x37, 0x00},
        [emEmmMotorSysParam_Flag]  = {0x3A, 0x00},
        [emEmmMotorSysParam_Org]   = {0x3B, 0x00},
        [emEmmMotorSysParam_Conf]  = {0x42, 0x6C},
        [emEmmMotorSysParam_State] = {0x43, 0x7A},
    };
    
    // 检查参数范围
    if (emSysParam >= sizeof(aucParamCode) / sizeof(aucParamCode[0])) {
        return;
    }
    
    uint8_t aucCmd[5];
    uint8_t ucIndex = 0;
    
    aucCmd[ucIndex++] = pstEmmMotor->stStaticParam.ucAddr;
    aucCmd[ucIndex++] = aucParamCode[emSysParam][0];
    if (aucParamCode[emSysParam][1] != 0) {
        aucCmd[ucIndex++] = aucParamCode[emSysParam][1];
    }
    aucCmd[ucIndex++] = EMM_MOTOR_CMD_END_MARK;
    
    vUartSendArray(pstEmmMotor->stStaticParam.emUartDevNum, aucCmd, ucIndex);
}

/**
 * @brief  修改开环/闭环控制模式
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSave ：存储标志
 * @param  emCtrlMode ：控制模式
 */
void vEmmMotorModifyCtrlMode(void *pstMotor, uint8_t bSave, emEmmMotorCtrlModeTdf emCtrlMode)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x46, 0x69, bSave, emCtrlMode};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}





/**
 * @brief  触发多机同步开始运动
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorSynchronousMotion(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0xFF, 0x66};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/**
 * @brief  设置单圈回零的零点位置
 * @param  pstMotor ：EmmMotor设备指针
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 */
void vEmmMotorOriginSetO(void *pstMotor, uint8_t bSave)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x93, 0x88, bSave};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/**
 * @brief  修改回零参数
 * @param  pstMotor ：EmmMotor设备指针
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
void vEmmMotorOriginModifyParams(void *pstMotor, uint8_t bSave, emEmmMotorOrgModeTdf emOrgMode, emMotorDirTdf emDir, 
                               uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                               uint16_t usSlMs, uint8_t bPotFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {
        0x92,
        0x88,
        bSave,
        emOrgMode,
        emDir,
        (uint8_t)(usOrgVel >> 8),
        (uint8_t)(usOrgVel >> 0),
        (uint8_t)(ulOrgTm >> 24),
        (uint8_t)(ulOrgTm >> 16),
        (uint8_t)(ulOrgTm >> 8),
        (uint8_t)(ulOrgTm >> 0),
        (uint8_t)(usSlVel >> 8),
        (uint8_t)(usSlVel >> 0),
        (uint8_t)(usSlMa >> 8),
        (uint8_t)(usSlMa >> 0),
        (uint8_t)(usSlMs >> 8),
        (uint8_t)(usSlMs >> 0),
        bPotFlag
    };
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/**
 * @brief  触发回零
 * @param  pstMotor ：EmmMotor设备指针
 * @param  emOrgMode ：回零模式
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmMotorOriginTriggerReturn(void *pstMotor, emEmmMotorOrgModeTdf emOrgMode, uint8_t bSyncFlag)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x90, 0x88, emOrgMode, bSyncFlag};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

/**
 * @brief  强制中断并退出回零
 * @param  pstMotor ：EmmMotor设备指针
 */
void vEmmMotorOriginInterrupt(void *pstMotor)
{
    stEmmMotorDeviceParamTdf *pstEmmMotor = (stEmmMotorDeviceParamTdf *)pstMotor;
    if (pstEmmMotor == NULL) {
        return;
    }
    
    const uint8_t aucCmdData[] = {0x94, 0x88};
    vEmmMotorSendCmd(pstEmmMotor, aucCmdData, sizeof(aucCmdData));
}

#endif
