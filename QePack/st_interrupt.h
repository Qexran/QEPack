#ifndef _ST_INTERRUPT_H_
#define _ST_INTERRUPT_H_
#include "project_config.h"

#if (QEPACK_PLATFORM == ST)

#include "it_controller.h

#ifdef HAL_TIM_MODULE_ENABLED
    #include "tim.h"
    void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif

#endif

#endif
