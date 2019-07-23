#include <stdint.h>
#include <stdlib.h>
#ifndef INLCUDE_TIMER_H_
#define INLCUDE_TIMER_H_

void Timer_init(void);
void setupTimer0(void);
void setupTimer1(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif /* INLCUDE_TIMER_H_ */
