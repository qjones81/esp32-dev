/*
 * utils.h
 *
 *  Created on: Feb 12, 2017
 *      Author: qjones
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>

#define NOP() asm volatile ("nop")

// Min/Max TODO: Make INLINE in Utils
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

#ifndef DEGS_TO_RADS
#define DEGS_TO_RADS 0.01745329252f
#endif

#ifndef RADS_TO_DEGS
#define RADS_TO_DEGS 57.2957795131f
#endif

#ifndef RADS_TO_GRAD
#define RADS_TO_GRAD 57.2957795f
#endif

#ifndef GRAD_TO_RADS
#define GRAD_TO_RADS 0.01745329251994329576923690768489f
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

extern bool equal_f(float a, float b, float epsilon);
#endif /* UTILS_H_ */
