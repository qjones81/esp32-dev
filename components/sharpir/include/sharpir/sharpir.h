
/*
 * sharpir.h
 *
 *  Created on: Mar 22, 2017
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

#ifndef SHARP_IR_H_
#define SHARP_IR_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Channel Type Enum
 */
typedef enum  {
	CM_20_150,
	CM_10_80,
	CM_100_500,
	CM_4_30,
} sharp_ir_model_t;

/**
 * @brief Struct with stepper motor device configuration
 */
typedef struct {
	sharp_ir_model_t type; // Model Type
	adc2_channel_t sensor_pin;           // Sensor Input Pin
    uint8_t num_samples; // How many samples to average
} sharp_ir_device_t;


/**
 * @brief Initialize the Sharp IR
 * @param device: Target Sharp IR device
 */
void sharp_ir_init(sharp_ir_device_t *device);

/**
 * @brief Get current sensor distance reading
 * @param device: Target Sharp IR device
 */
float sharp_ir_get_distance_cm(sharp_ir_device_t *device);

/**
 * @brief Get current sensor distance reading
 * @param device: Target Sharp IR device
 */
int sharp_ir_get_distance_raw(sharp_ir_device_t *device);

/**
 * @brief Get current sensor distance reading
 * @param device: Target Sharp IR device
 */
float sharp_ir_get_distance_volt(sharp_ir_device_t *device);

#ifdef __cplusplus
}
#endif

#endif /* SHARP_IR_H_ */
