
/*
 * servo.c
 *
 *  Created on: Feb 21, 2017
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
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "utils/utils.h"
#include "servo.h"


void servo_control_init()
{
    ledc_timer_config_t timer_conf;
    timer_conf.bit_num    = LEDC_TIMER_15_BIT;
    timer_conf.freq_hz    = 50;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num  = LEDC_TIMER_0;
    ledc_timer_config(&timer_conf);
}

esp_err_t servo_control_add_device(servo_motor_type_t motor_t, servo_motor_device_config_t dev_config)
{
    // TODO: Switch based on motor type.  1 or 2.
    ledc_channel_config_t ledc_conf;
    ledc_conf.channel    = LEDC_CHANNEL_0;
    ledc_conf.duty       = (1 << 15) * (1500.0 / 20000);
    ledc_conf.gpio_num   = dev_config.attach_pin;
    ledc_conf.intr_type  = LEDC_INTR_DISABLE;
    ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf.timer_sel  = LEDC_TIMER_0;
    ledc_channel_config(&ledc_conf);

    return ESP_OK;
}

void servo_control_position(servo_motor_type_t motor_t, float angle)
{
    // TODO: Map duty to angle from 0 to 180
   // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
   // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}
