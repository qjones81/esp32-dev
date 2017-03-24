
/*
 * sharpir.c
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
//#include "driver/adc.h"
#include "adc/adc.h"
#include "utils/utils.h"

#include "sharpir.h"

static const char *tag = "sharp_ir";

static float _previous_distance = 0.0;
void sharp_ir_init(sharp_ir_device_t *device)
{
	adc_2_config_width(ADC_WIDTH_12Bit);//config adc2 width
	adc_2_config_channel_atten(device->sensor_pin, ADC_ATTEN_11db);//config channel attenuation
}


int sharp_ir_get_distance_raw(sharp_ir_device_t *device)
{
	int raw_val = 0;
	for (int i = 0; i < device->num_samples; i++) {
		raw_val += adc_2_get_voltage(device->sensor_pin); //get the  val of channel
	}
	return raw_val / device->num_samples;
}

float sharp_ir_get_distance_volt(sharp_ir_device_t *device)
{
	int raw_val = sharp_ir_get_distance_raw(device);
	return 0.00084948603 * raw_val + 0.1019;
}

float sharp_ir_get_distance_cm(sharp_ir_device_t *device)
{
	//int raw_val = sharp_ir_get_distance_raw(device);
	float volts = sharp_ir_get_distance_volt(device);
	float distanceCM = 0;
	if (device->type == CM_10_80) {

		distanceCM = 27.728 * pow(volts, -1.2045);

		//float distanceCM_median = 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0,
		//				-1.2045);
	}
	return distanceCM;
//	    } else if (device.type == CM_20_150){
//
//	        // Previous formula used by  Dr. Marcal Casas-Cartagena
//	        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);
//
//	          distanceCM = 60.374 * pow(map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000)/1000.0, -1.16);
//
//
//	    } else if (device.type == CM_4_30){
//
//	        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
//
//	          distanceCM = 12.08 * pow(map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000)/1000.0, -1.058);
//
//
//	    }
}

float sharp_ir_get_distance_cm_tol(sharp_ir_device_t *device)
{
	float _p = 0;
	float _sum = 0;

	float _tol = .93f;
	for (int i = 0; i<5; i++) {

		int foo = sharp_ir_get_distance_cm(device);

		if (foo >= (_tol*_previous_distance)) {

			_previous_distance = foo;
			_sum = _sum + foo;
			_p++;

		}


	}


	float accurateDistance = _sum / _p;

	return accurateDistance;

}
