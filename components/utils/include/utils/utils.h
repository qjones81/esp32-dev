/*
 * utils.h
 *
 *  Created on: Feb 12, 2017
 *      Author: qjones
 */

#ifndef UTILS_H_
#define UTILS_H_

#define NOP() asm volatile ("nop")

// Min/Max TODO: Make INLINE in Utils
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern uint32_t IRAM_ATTR micros();
extern uint32_t IRAM_ATTR millis();

extern void delay(uint32_t ms);

extern void IRAM_ATTR delay_us(uint32_t us);

extern int get_ms(unsigned long *count);

extern int delay_ms(unsigned long num_ms);

extern long map(long x, long in_min, long in_max, long out_min, long out_max);

extern float map_f(float x, float in_min, float in_max, float out_min, float out_max);
#endif /* UTILS_H_ */
