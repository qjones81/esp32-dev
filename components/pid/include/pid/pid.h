
/*
 * pid.h
 *
 *  Created on: April 1, 2017
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

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Struct with pid device status/control
 */
typedef struct {
	uint32_t last_time;
	float k_p;
	float k_i;
	float k_d;
	float set_point;
	float set_point_prev;
	float input;
	float input_prev;
	float error_prev;
	float output_min;
	float output_max;
	float integral_error;
	float integral_windup_max;
	bool clamp_output;

} pid_device_control_t;


/**
 * @brief Compute PID
 * @param pid_device_control_t: Current PID state
 */
float pid_compute(pid_device_control_t *controller);

float pid_compute_angle(pid_device_control_t *controller);
#ifdef __cplusplus
}
#endif


#endif /* PID_H_ */
