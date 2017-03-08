
/*
 * ar6115e.h
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

#ifndef AR6115E_H_
#define AR6115E_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Channel Type Enum
 */
typedef enum  {
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    GEAR,
    AUX1,
    CHANNEL_MAX,
} channel_type_t;

/**
 * @brief Pulse Event Struct Enum
 */
typedef struct {
        gpio_num_t gpio;
        uint8_t channel;
        uint16_t pulse_width;
} pulse_event_t;

typedef void (*rc_event_cb_t)(pulse_event_t event);

/**
 * @brief Initialize the AR6115e
 * @param handler: Callback function to handle pulse input events.  Allowed to pass NULL and can still poll if needed.
 */
void ar6115e_init(rc_event_cb_t handler);

/**
 * @brief Add channel for pwm input
 * @param channel_t: Channel ID.  Valid values from channel_type_t(THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, AUX1)
 * @param input_pin: Input pin for PWM
 */
void ar6115e_add_channel(channel_type_t channel_t, gpio_num_t input_pin);

/**
 * @brief Get current pulse width on channel
 * @param channel_t: Channel ID.  Valid values from channel_type_t(THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, AUX1)
 */
uint16_t ar6115e_get_channel_value(channel_type_t channel_t);

#ifdef __cplusplus
}
#endif

#endif /* AR6115E_H_ */
