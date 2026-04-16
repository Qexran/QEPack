#ifndef __TI_HARDFAULT_HELPER_H__
#define __TI_HARDFAULT_HELPER_H__

/**
    快速定位导致 HardFault_Handler 异常：
    1. 进入debug模式
    2. Run->Trace->Enable Core Trace
    3. 在Trace窗口中，当进入软断点时，即可记录程序运行的所有步骤
    4. 推荐：在工程的 properties->Build->Arm Compiler 中，将编译优化改为0
*/

#include "project_config.h"

#endif
