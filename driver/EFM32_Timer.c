#include "timer.h"
#include "Typedefs.h"
#include <stdint.h>
#include "udelay.h"
extern volatile uint32_t Ticks;

void PIC18FXXJ_Timer_Init(void)
{
	setupTimer0();
}

uint32 PIC18FXXJ_Timer_Update(void)
{
	return Ticks;
}
