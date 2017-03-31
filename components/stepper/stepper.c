
/*
 * stepper.c
 *
 *  Created on: Feb 14, 2017
 *      Author: qjones
 *
 * Copyright (c) <2017> <Quincy Jones - qjones@c4solutions.net/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "stepper.h"
#include "utils/utils.h"

#define TIMER_DIVIDER   16               /*!< Hardware timer clock divider */
#define TIMER_SCALE_SEC    (TIMER_BASE_CLK / TIMER_DIVIDER)  // Timer scale to uS.  Based off APB_CLK_FREQ timer.  80mhz
#define TIMER_SCALE_US    (TIMER_SCALE_SEC / 1000000)  // Timer scale to uS.  Based off APB_CLK_FREQ timer.  80mhz

//#define TIMER_FINE_ADJ   (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
//#define LOOP_INTERVAL   (100)   /*!< test interval for timer 0 */
#define MAX_MOTORS 2 // Max number of motors allowed to be controlled.

#define MAX_ACCEL 8
stepper_motor_device_t *motor_devices[MAX_MOTORS] = { NULL };

uint8_t num_devices = 0;
static const char *tag = "stepper";
void stepper_control_timer_init_0();
void stepper_control_timer_init_1();

void task_stepper_control(void *ignore)
{
    // Init Timers
    if (motor_devices[STEPPER_MOTOR_1] != NULL) stepper_control_timer_init_0();
    if (motor_devices[STEPPER_MOTOR_2] != NULL) stepper_control_timer_init_1();

    //stepper_control_start();
    while(1) {
        delay_ms(1000);
    }
    vTaskDelete(NULL);
}

void stepper_control_start()
{
    // Start Control Task
    xTaskCreatePinnedToCore(&task_stepper_control, "stepper_control", 2048, NULL, 7, NULL, 1);
    //xTaskCreate(&task_stepper_control, "stepper_control", 2048, NULL, 7, NULL);

    // Create Semaphore
   // vSemaphoreCreateBinary(timer_tick);
    //timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    //stepper_control_timer_loop_init();
    //xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    //stepper_control_timer_loop_init();

    // Setup Timer
    //                gpio_set_level(GPIO_NUM_16, 1);
    //                vTaskDelay(100 / portTICK_PERIOD_MS / 1000);
    //
    //                gpio_set_level(GPIO_NUM_16, 0);
    //                vTaskDelay(1000 / portTICK_PERIOD_MS / 1000);
}

void IRAM_ATTR control_isr(void *para)
{
    int timer_idx = (int) para;

    // Reload timer value
    TIMERG0.hw_timer[timer_idx].update = 1;

    TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

    if(timer_idx == 0) {
        TIMERG0.int_clr_timers.t0 = 1;
    }
    else {
        TIMERG0.int_clr_timers.t1 = 1;
    }
//    uint32_t intr_status = TIMERG0.int_st_timers.val;
//    if(state == 0) {
//        gpio_set_level(GPIO_NUM_16, 1);
//        state = 1;
//    }
//    else {
//        gpio_set_level(GPIO_NUM_16, 0);
//        state = 0;
//    }
    //ESP_LOGI(tag, "Int: %d", timer_idx);

   gpio_set_level(motor_devices[timer_idx]->cfg.step_pin, 1);
    delay_us(10);
    gpio_set_level(motor_devices[timer_idx]->cfg.step_pin, 0);

/*
    for(int i = 0; i < num_devices; i++)
    {
        motor_devices[i]->ticks++; // Update Ticks
      // ESP_LOGI(tag, "TICKS: %d / %d!\n",motor_devices[i]->ticks,motor_devices[i]->tick_target);
        switch(motor_devices[i]->state) {
            case OFF:
                if(motor_devices[i]->ticks >= 10) { //motor_devices[i]->tick_target) {
                    motor_devices[i]->ticks = 0;
                    motor_devices[i]->state = STEP_START;
                } // GOTO next state
                else { // STAY current state
                    break;
                }
            case STEP_START:
                gpio_set_level(motor_devices[i]->cfg.step_pin, 1);

                if(motor_devices[i]->ticks >= 1) { //motor_devices[i]->cfg.step_hold_delay) {
                    motor_devices[i]->ticks = 2;
                    motor_devices[i]->state = STEP_END;
                }

                break;
            case STEP_END:
                gpio_set_level(motor_devices[i]->cfg.step_pin, 0);
                // Update State
                if(motor_devices[i]->dir == 0) {
                    motor_devices[i]->current_step++;
                } else {
                    motor_devices[i]->current_step--;
                }

                motor_devices[i]->state = OFF;
                break;
            default:
                // Invalid state
                ESP_LOGE(tag, "INVALID STEPPER STATE!\n");
        }
    }*/

}

void stepper_control_timer_init_0()
{
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    //Configure timer
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    //Stop timer counter
    timer_pause(TIMER_GROUP_0, TIMER_0);
    //Load counter value
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    //Set alarm value
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 2.0 * TIMER_SCALE_SEC);
    //Enable timer interrupt
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    //Set ISR handler
    timer_isr_register(TIMER_GROUP_0, TIMER_0, control_isr,
            (void*) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    //Start timer counter
    timer_start(TIMER_GROUP_0, TIMER_0);

    ESP_LOGI(tag, "Init 0.");
}

void stepper_control_timer_init_1()
{
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    //Configure timer
    timer_init(TIMER_GROUP_0, TIMER_1, &config);
    //Stop timer counter
    timer_pause(TIMER_GROUP_0, TIMER_1);
    //Load counter value
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0x00000000ULL);
    //Set alarm value
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 2.0 * TIMER_SCALE_SEC);
    //Enable timer interrupt
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    //Set ISR handler
    timer_isr_register(TIMER_GROUP_0, TIMER_1, control_isr, (void*) TIMER_1,
            ESP_INTR_FLAG_IRAM, NULL);
    //Start timer counter
    timer_start(TIMER_GROUP_0, TIMER_1);

    ESP_LOGI(tag, "Init 1.");

}

esp_err_t stepper_control_add_device(stepper_motor_type_t motor_t, stepper_motor_device_config_t dev_config)
{
    // Allocate Memory
    stepper_motor_device_t *dev = malloc(sizeof(stepper_motor_device_t));

    if(dev == NULL) {
        ESP_LOGE(tag, "Unable to allocate memory for stepper motor device.\n");
        return ESP_ERR_NO_MEM;
    }
   // stepper_motor_device_t dev;
    memset(dev, 0, sizeof(stepper_motor_device_t));

    // Setup Defaults
    dev->cfg = dev_config;

    // Setup GPIO
    gpio_pad_select_gpio(dev->cfg.step_pin);
    gpio_set_direction(dev->cfg.step_pin, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(dev->cfg.dir_pin);
    gpio_set_direction(dev->cfg.dir_pin, GPIO_MODE_OUTPUT);

    motor_devices[motor_t] = dev;

    ESP_LOGI(tag, "Successfully added stepper on step %d and dir %d.\n", dev_config.step_pin, dev_config.dir_pin);
    // return error/success code
    return ESP_OK;
}
void stepper_control_set_speed(stepper_motor_type_t motor_t, int32_t steps_sec)
{


	motor_devices[motor_t]->steps_second = steps_sec;
    //Set alarm value
    if(motor_t == STEPPER_MOTOR_1) {

        if(steps_sec == 0)
        {
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (1.0 / 0.0001) * TIMER_SCALE_SEC);
        }
        else {

            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (1.0 / abs(motor_devices[motor_t]->steps_second)) * TIMER_SCALE_SEC);
        }
    }
    else {
        if(steps_sec == 0)
                {
                    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, (1.0 / 0.0001) * TIMER_SCALE_SEC);
                }
                else
                    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, (1.0 / abs(motor_devices[motor_t]->steps_second)) * TIMER_SCALE_SEC);
    }
    // 1 = clockwise, 0 = counter clockwise
    if(steps_sec <= 0)
        gpio_set_level(motor_devices[motor_t]->cfg.dir_pin, motor_devices[motor_t]->cfg.dir_reverse ? 0 : 1);
    else
        gpio_set_level(motor_devices[motor_t]->cfg.dir_pin, motor_devices[motor_t]->cfg.dir_reverse ? 1 : 0);
}
//
//TickType_t get_stepper_control_ticks_per_second()
//{
//    return TIMER_SCALE_SEC / LOOP_INTERVAL;
//}
