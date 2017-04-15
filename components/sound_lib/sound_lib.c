
/*
 * sound_lib.c
 *
 *  Created on: April 13, 2017
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_attr.h>
#include <esp_log.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "driver/ledc.h"
#include "utils/utils.h"

#include "sound_lib.h"

static const char *tag = "sound_lib";

// Event Groups
EventGroupHandle_t sound_lib_events;
const int STOP_BIT = BIT0;

// Sound Queue
static QueueHandle_t sound_lib_queue;

// Active device
sound_device_t *active_device = NULL;

void sound_lib_player_task()
{
	sound_event_t current_sound;
	bool in_play = false;
	while(1)
	{
		EventBits_t uxBits = xEventGroupWaitBits(sound_lib_events, STOP_BIT, pdTRUE, pdFALSE, (TickType_t) 0);

		//EventBits_t uxBits = xEventGroupGetBits(sound_lib_events);
		if (in_play && (uxBits & STOP_BIT)) {
			in_play = false;
			xQueueReset(sound_lib_queue); // Reset queue

			ESP_LOGE(tag, "SOUND PLAY ABORTED!!");
			vTaskDelay(200 / portTICK_PERIOD_MS);
			continue;
		}

		// Look for new request in queue
		if (xQueueReceive(sound_lib_queue, &(current_sound), (TickType_t ) 0)) {
			if(active_device == NULL) {
				ESP_LOGE(tag, "Attempt sound play with active sound sevice NULL!");
				vTaskDelay(100/portTICK_PERIOD_MS);
				continue;
			}

			// Look for timeout
			ledc_timer_config_t timer_conf;
			timer_conf.speed_mode = active_device->speed;
			timer_conf.bit_num = LEDC_TIMER_10_BIT;
			timer_conf.timer_num = LEDC_TIMER_0;
			timer_conf.freq_hz = current_sound.freq;
			ledc_timer_config(&timer_conf);

			ledc_channel_config_t ledc_conf;
			ledc_conf.gpio_num = active_device->output_pin;
			ledc_conf.speed_mode = active_device->speed;
			ledc_conf.channel = active_device->channel;
			ledc_conf.intr_type = LEDC_INTR_DISABLE;
			ledc_conf.timer_sel = LEDC_TIMER_0;
			ledc_conf.duty = 0x0; // 50%=0x3FFF, 100%=0x7FFF for 15 Bit
								  // 50%=0x01FF, 100%=0x03FF for 10 Bit
			ledc_channel_config(&ledc_conf);

			in_play = true;

			//ESP_LOGI(tag, "Got Sound Queue: %d, %d\n", current_sound.freq, active_device->output_pin);
		}

		if(in_play == true) {

			//ESP_LOGI(tag, "In Play.\n");
			// start
			ledc_set_duty(active_device->speed, active_device->channel, 256); // 12% duty - play here for your speaker or buzzer
			ledc_update_duty(active_device->speed, active_device->channel);

			// Wait for duration
			vTaskDelay(current_sound.duration / portTICK_PERIOD_MS);

			// stop
			ledc_set_duty(active_device->speed, active_device->channel, 0);
			ledc_update_duty(active_device->speed, active_device->channel);

			// Are we repeating?
			if(current_sound.repeat_timeout > 0) {
				vTaskDelay(current_sound.repeat_timeout / portTICK_PERIOD_MS);
			}
			else {
				in_play = false;
			}
		}

		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

void sound_lib_init()
{
	// Create queue for receiving
	sound_lib_queue = xQueueCreate(1, sizeof(sound_event_t)); // Only one item

	// Event groups
	sound_lib_events = xEventGroupCreate();
	if (sound_lib_events == NULL) {
		ESP_LOGE(tag, "Failed to create sound_lib_event groups.\n");
	}

	// TODO: Should be moved into the start function.  KILL and RESTART from those functions.  This will work for now
	xTaskCreate(&sound_lib_player_task, "sound_lib_player", 2048, NULL, 2, NULL);
}
void sound_lib_start()
{
	//ESP_LOGI(tag, "Starting Sound.\n");

	// Clear Events
	xEventGroupClearBits(sound_lib_events, STOP_BIT);
}

void sound_lib_stop()
{
	//ESP_LOGI(tag, "Starting Stop.\n");

	// Set Events
	xEventGroupSetBits(sound_lib_events, STOP_BIT);

	// Clear active device
	active_device = NULL;
}
sound_device_t *sound_lib_create_device(gpio_num_t gpio_pin)
{
	sound_device_t *sound_dev = (sound_device_t *) malloc(sizeof(sound_device_t));

	// TODO: Keep up with used channels, memory manage etc here.  Probably wait until C++ for that.
	sound_dev->channel = LEDC_CHANNEL_0;
	sound_dev->volume = 128;
	sound_dev->speed = LEDC_HIGH_SPEED_MODE;
	sound_dev->output_pin = gpio_pin;
	return sound_dev;

}
void sound_lib_play_tone(sound_device_t *device, uint32_t freq, uint32_t duration, uint32_t repeat_time)
{
	//return;
	EventBits_t uxBits = xEventGroupGetBits(sound_lib_events);
	if ((uxBits & STOP_BIT)) { // In Stop
		return;
	}
	active_device = device;

	//ESP_LOGI(tag, "Playing Sound.\n");
	sound_event_t sound_event;
	sound_event.duration = duration;
	sound_event.freq = freq;
	sound_event.repeat_timeout = repeat_time;

	// Send sound
	xQueueSendToFront(sound_lib_queue, &sound_event, (TickType_t ) 100 / portTICK_PERIOD_MS );

}
