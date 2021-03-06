#ifndef __SYSTICK_TIMER_H
#define __SYSTICK_TIMER_H


////////////////////////////////////////////////////////////////////////////////

extern volatile uint32_t ticks_1ms; // This variable has to be incremented periodically in other timer function (1ms)

////////////////////////////////////////////////////////////////////////////////

uint32_t GetSysTickTimer_ms(void);


#endif // __SYSTICK_TIMER_H