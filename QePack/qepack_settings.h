#ifndef __QEPACK_SETTINGS__
#define __QEPACK_SETTINGS__


#define QEPACK_PLATFORM     ST                    /* 可选平台: ST, TI */
#define IS_DEBUG_MODE       1                     // 开启调试模式

#if (QEPACK_PLATFORM == ST) // 设备头文件
    #include					"stm32f1xx_hal.h"			    
    #include					"stm32f1xx_hal_def.h"
#else
    #include                    "ti_msp_dl_config.h"
    #include                    <ti/driverlib/dl_flashctl.h>
#endif

/**
 * @brief 全局状态枚举
 */
typedef enum
{
  QE_OK       = 0x00U,
  QE_ERROR    = 0x01U,
  QE_BUSY     = 0x02U,
  QE_TIMEOUT  = 0x03U,
  QE_IDLE     = 0X04U
} QE_StatusTypeDef;

#endif
