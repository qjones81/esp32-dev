
/*
 * servo.h
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

// Orange = Signal, Brown = -, Red = +
#ifndef SERVO_H_
#define SERVO_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Servo Indexing
 */
typedef enum  {
    SERVO_MOTOR_1,
    SERVO_MOTOR_2,
    SERVO_MOTOR_MAX,
} servo_motor_type_t;


/**
 * @brief Struct with servo motor device configuration
 */
typedef struct {
    uint8_t attach_pin;
    uint16_t min_pulse;
    uint16_t max_pulse;
    float    position_deg;
} servo_motor_device_config_t;


/**
 * @brief Initialize the Servo Controller
 */
void servo_control_init();

/**
 * @brief Add servo motor device to controller
 * @param motor_t: ID for stepper motor to controller.  Valid values (SERVO_MOTOR_1, SERVO_MOTOR_2)
 * @param dev_config: Configuration data for servo motor
 */
esp_err_t servo_control_add_device(servo_motor_type_t motor_t, servo_motor_device_config_t dev_config);

/**
 * @brief Set servo motor position
 * @param motor_t: ID for stepper motor to controller.  Valid values (SERVO_MOTOR_1, SERVO_MOTOR_2)
 * @param angle: Angle position (0 to 180deg)
 */
void servo_control_set_position(servo_motor_type_t motor_t, float angle);


#ifdef __cplusplus
}
#endif

#endif /* SERVO_H_ */
