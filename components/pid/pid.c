
/*
 * pid.c
 *
 *  Created on: April 2, 2017
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

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "pid.h"
#include "utils/utils.h"

static const char *tag = "pid";

float pid_compute(pid_device_control_t *controller)
{
	float output;

	// Update time deltas
	float now = millis();
	float dt = (now - controller->last_time);

	// Error Term
	float error = controller->set_point - controller->input;

	// Compute proportional term
	float p_term = controller->k_p * error;

	// Compute integral term
	controller->integral_error += error * dt; // constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
	controller->integral_error = constrain(controller->integral_error, -controller->integral_windup_max, controller->integral_windup_max);
	float i_term = (controller->k_i * controller->integral_error);

	// Compute derivative term
	float d_error = (error - controller->error_prev) / dt;
	float d_term = controller->k_d * d_error;

	// Compute output
	output = p_term + i_term + d_term;

	// Clamp output
	if(controller->clamp_output) {
		output = constrain(output, controller->output_min, controller->output_max);
	}

	// Update state
	controller->error_prev = error;
	controller->last_time = now;

	// Return
	return (output);
}

float pid_compute_angle(pid_device_control_t *controller)
{
	float output;

	// Update time deltas
	float now = millis();
	float dt = (now - controller->last_time);

	// Error Term
	float error = controller->set_point - controller->input;
	error = atan2(sin(error),cos(error)); // Keep between -PI and PI

	// Compute proportional term
	float p_term = controller->k_p * error;

	// Compute integral term
	controller->integral_error += error * dt; // constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
	controller->integral_error = constrain(controller->integral_error, -controller->integral_windup_max, controller->integral_windup_max);
	float i_term = (controller->k_i * controller->integral_error);

	// Compute derivative term
	float d_error = (error - controller->error_prev) / dt;
	float d_term = controller->k_d * d_error;

	// Compute output
	output = p_term + i_term + d_term;

	// Clamp output
	if(controller->clamp_output) {
		output = constrain(output, controller->output_min, controller->output_max);
	}

	// Update state
	controller->error_prev = error;
	controller->last_time = now;

	// Return
	return (output);
}
