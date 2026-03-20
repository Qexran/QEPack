#include "ti_clock.h"

#if (QEPACK_PLATFORM == TI)

volatile unsigned long tick_ms;
volatile uint32_t start_time;
static uint8_t is_initialed_clock = 0;

int mspm0_delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

uint8_t ucGetSysTickInitialState(){
    return is_initialed_clock;
}

void SysTick_Init(void)
{
    is_initialed_clock = 1;
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

#endif
