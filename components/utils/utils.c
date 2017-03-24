/*
 * utils.c
 *
 *  Created on: Feb 12, 2017
 *      Author: qjones
 */


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_attr.h>

#include "utils.h"
extern uint32_t IRAM_ATTR micros()
{
    uint32_t ccount;
    __asm__ __volatile__ ( "rsr     %0, ccount" : "=a" (ccount) );
    return ccount / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
}

extern uint32_t IRAM_ATTR millis()
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

extern void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

extern void IRAM_ATTR delay_us(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us) % ((0xFFFFFFFF / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ) + 1);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

extern int get_ms(unsigned long *count)
{
	*count = millis();
	return 0;
}

extern int delay_ms(unsigned long num_ms)
{
	delay(num_ms);
	return 0;
}

extern long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
extern float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
