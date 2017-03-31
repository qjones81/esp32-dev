
/*
 * stepper.h
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

#ifndef STEPPER_H_
#define STEPPER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Current Stepping State Enum
 */
typedef enum  {
    STEPPER_MOTOR_1,
    STEPPER_MOTOR_2,
    STEPPER_MOTOR_MAX,
} stepper_motor_type_t;

/**
 * @brief Current Stepping State Enum
 */
typedef enum  {
    OFF,
    STEP_START,
    STEP_END,
} stepper_pulse_state_t;

/**
 * @brief Struct with stepper motor device configuration
 */
typedef struct {
    uint8_t step_pin;           // Step Pin
    uint8_t dir_pin;            // Dir Pin
    uint8_t step_hold_delay;    // Delay to hold step pin high after step (in microseconds)
    bool dir_reverse;
} stepper_motor_device_config_t;

/**
 * @brief Struct with stepper motor device status/control
 */
typedef struct {
    stepper_motor_device_config_t cfg;      // configuration parameters
    bool stopped;                           // stop/enable flag
    bool dir;                               // step direction
    int32_t current_step;                   // Current Stepper Position
    int32_t step_count;
    TickType_t ticks;                         // Current Ticks
    TickType_t tick_target;                   // Tick Target
    uint32_t steps_second;                  // Target speed in steps per second
    stepper_pulse_state_t state;            // State
} stepper_motor_device_t;

typedef struct stepper_motor_device_t* stepper_motor_device_handle_t;

//void stepper_control_init(uint8_t step_pin, uint8_t dir_pin);

/**
 * @brief Add stepper motor device to controller
 * @param motor_t: ID for stepper motor to controller.  Valid values (MOTOR_1, MOTOR_2)
 * @param dev_config: Configuration data for stepper motor
 */
esp_err_t stepper_control_add_device(stepper_motor_type_t motor_t, stepper_motor_device_config_t dev_config);

/**
 * @brief Startup stepper motor control timers.
 */
void stepper_control_start();

/**
 * @brief Stop stepper motor control timers.
 */
void stepper_control_stop();

/**
 * @brief Add stepper motor device to controller
 * @param motor_t: ID for stepper motor to controller.  Valid values (MOTOR_1, MOTOR_2)
 * @param steps_sec: Configuration data for stepper motor
 */
void stepper_control_set_speed(stepper_motor_type_t motor_t, int32_t steps_sec);

/**
 * @brief Get stepper motor device position from controller
 * @param motor_t: ID for stepper motor to controller.  Valid values (MOTOR_1, MOTOR_2)
 * @return: Current motor positionr
 */
int32_t stepper_control_get_position(stepper_motor_type_t motor_t);

//void stepper_control_step(int num_steps);
//void stepper_control_timer_init();

//TickType_t get_stepper_control_ticks_per_second();

#ifdef __cplusplus
}
#endif


#endif /* STEPPER_H_ */
