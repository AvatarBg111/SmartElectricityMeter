#include "stm32f4xx.h"

////////////////////////////////////////////////////////////////////////////////

#include "systick_timer.h"

////////////////////////////////////////////////////////////////////////////////

volatile uint32_t ticks_1ms; // This variable has to be incremented periodically in other timer function (1ms)

////////////////////////////////////////////////////////////////////////////////

// return the systicks as milliseconds
inline uint32_t GetSysTickTimer_ms(void)
{
    return ticks_1ms;
}

////////////////////////////////////////////////////////////////////////////////