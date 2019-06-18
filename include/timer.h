/*
 * timer.h
 *
 *  Created on: 2017Äê7ÔÂ4ÈÕ
 *      Author: Administrator
 */
#include <stdint.h>
#include <stdlib.h>
#ifndef INLCUDE_TIMER_H_
#define INLCUDE_TIMER_H_

void setupTimer0(void);
void setupTimer1(void);
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif /* INLCUDE_TIMER_H_ */
