#ifndef _TI_CLOCK_H_
#define _TI_CLOCK_H_

#include "ti_msp_dl_config.h"

#if (QEPACK_PLATFORM == TI)

extern volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
void SysTick_Init(void);
uint8_t ucGetSysTickInitialState();

#endif

#endif
