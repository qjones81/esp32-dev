
/*
 * ar6115e.c
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
#include "driver/gpio.h"
#include "utils/utils.h"
#include "sockets/socket_server.h"

#include "ar6115e.h"

#define ESP_INTR_FLAG_DEFAULT 0

// For ISR parameter passing
typedef struct {
        gpio_num_t gpio;
        uint8_t channel;
} gpio_event_t;

static const char *tag = "ar6115e";
static QueueHandle_t q1;
static uint8_t gpio_channel_map[CHANNEL_MAX];
volatile uint16_t pulse_width[CHANNEL_MAX] = { 0 };
volatile uint16_t pulse_start_time[CHANNEL_MAX] = { 0 };

// Cache value for polling
volatile uint16_t channel_value[CHANNEL_MAX] = { 0 };

rc_event_cb_t rc_handler = NULL;

// Socket for Debug
static socket_device_t *sock = NULL;

static void isr_handler(void *arg) {

    // TODO: Rollover Handling
    gpio_event_t *event = (gpio_event_t *)arg;
    if(gpio_get_level(event->gpio) == 1) { // RISING
        pulse_start_time[event->channel] = micros();
    }
    else { // FALLING
        pulse_width[event->channel] = micros() - pulse_start_time[event->channel];
        pulse_event_t pulse_event;
        pulse_event.channel = event->channel;
        pulse_event.gpio = event->gpio;
        pulse_event.pulse_width = pulse_width[event->channel];
       // ESP_LOGI(tag, "Hello: %d\n", pulse_event.pulse_width);
        xQueueSendToBackFromISR(q1, &pulse_event, NULL);
    }
}

void ar6115e_reader_task(void *ignore)
{
    ESP_LOGD(tag, ">> ar6115e_reader_task");
    while (1) {
        pulse_event_t pulse_event;
        // ESP_LOGD(tag, "Waiting on interrupt queue");
        xQueueReceive(q1, &pulse_event, portMAX_DELAY);

        // Cache for polling
        // TODO: Mutex here?
        channel_value[pulse_event.channel] = pulse_event.pulse_width;
        //  ESP_LOGD(tag, "Woke from interrupt queue wait: %d/%d", rc, pulse_event.pulse_width);

        if (rc_handler)
            rc_handler(pulse_event); // Send to handler if registered
    }
    vTaskDelete(NULL);
}
#if defined(CONFIG_AR6115_USE_DEBUG_SERVICE)
void task_ar6115e_socket_debug(void *ignore)
{
    ESP_LOGD(tag, ">> task_ar6115e_socket_debug");
    while (1) {
        char str[80];
        // TODO: Mutex Here or does volatile work here
        sprintf(str, "%d:%d:%d:%d:%d:%d\n",  channel_value[0],
                channel_value[1],
                channel_value[2],
                channel_value[3],
                channel_value[4],
                channel_value[5]);

        ESP_LOGI(tag, "%s",str);
        socket_server_send_data(sock, (uint8_t *)str, strlen(str) + 1);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
#endif

void ar6115e_init(rc_event_cb_t handler)
{
    rc_handler = handler;

    // Create queue for receiving
    q1 = xQueueCreate(10, sizeof(pulse_event_t));

    // Zero out channels
    //memset(channel_value, 0, sizeof(uint16_t) * CHANNEL_MAX));
    for(int i = 0; i < CHANNEL_MAX; i++) // TODO: Better way here
    {
        channel_value[i] = 0;
    }

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
}
void ar6115e_start()
{
    xTaskCreate(&ar6115e_reader_task, "ar6115_reader_task", 2048, NULL, 3, NULL);

#if defined(CONFIG_AR6115_USE_DEBUG_SERVICE)
        sock = (socket_device_t *) malloc(sizeof(socket_device_t));
        sock->port = CONFIG_AR6115_DEBUG_PORT;
        socket_server_init(sock);
        socket_server_start(sock);

        xTaskCreate(&task_ar6115e_socket_debug, "ar6115_socket_task", 2048, NULL, 3, NULL);
#endif
}
void ar6115e_add_channel(channel_type_t channel_t, gpio_num_t input_pin)
{
    // Setup Config
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = ((1 << input_pin));
    gpioConfig.mode         = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpioConfig.intr_type    = GPIO_INTR_ANYEDGE;
    gpio_config(&gpioConfig);

    // Cache channel map.  May not be needed.
    gpio_channel_map[channel_t] = input_pin;

    // TODO: Clean up mem here
    gpio_event_t *gpio_event = malloc(sizeof(gpio_event_t));
    gpio_event->channel = channel_t;
    gpio_event->gpio = input_pin;
    gpio_isr_handler_add(input_pin, isr_handler, (void *)gpio_event);
}

uint16_t ar6115e_get_channel_value(channel_type_t channel_t)
{
    return channel_value[channel_t];
}


