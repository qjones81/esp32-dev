/*
 * mpu_9250.h
 *
 *  Created on: Feb 13, 2017
 *      Author: qjones
 */

#ifndef MPU_9250_H_
#define MPU_9250_H_

// Optimally, these defines would be passed as compiler options, but Arduino
// doesn't give us a great way to do that.
#define MPU9250
#define AK8963_SECONDARY
#define COMPASS_ENABLED

// Include the Invensense MPU9250 driver and DMP keys:
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU9250_RegisterMap.h"

typedef int inv_error_t;
#define INV_SUCCESS 0
#define INV_ERROR 0x20

enum t_axisOrder {
	X_AXIS, // 0
	Y_AXIS, // 1
	Z_AXIS  // 2
};

// Define's passed to update(), to request a specific sensor (or multiple):
#define UPDATE_ACCEL   (1<<1)
#define UPDATE_GYRO    (1<<2)
#define UPDATE_COMPASS (1<<3)
#define UPDATE_TEMP    (1<<4)

#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW  1
#define INT_LATCHED     1
#define INT_50US_PULSE  0

#define MAX_DMP_SAMPLE_RATE 200 // Maximum sample rate for the DMP FIFO (200Hz)
#define FIFO_BUFFER_SIZE 512 // Max FIFO buffer size

#define ORIENT_PORTRAIT          0
#define ORIENT_LANDSCAPE         1
#define ORIENT_REVERSE_PORTRAIT  2
#define ORIENT_REVERSE_LANDSCAPE 3

int mpu_9250_init();
inv_error_t mpu_9250_begin();
float mpu_9250_get_gyro_sens();
float mpu_9250_get_accel_sens();

int mpu_9250_read_temp_data();

inv_error_t mpu_9250_dmp_begin(unsigned short features, unsigned short fifoRate);
inv_error_t mpu_9250_dmp_enable_features(unsigned short mask);
inv_error_t mpu_9250_dmp_set_fifo_rate(unsigned short rate);
inv_error_t mpu_9250_dmp_update_fifo(void);
unsigned short mpu_9250_fifo_available(void);

void mpu_9250_compute_euler_angles(bool degrees);

int mpu_9250_update_accel();
int mpu_9250_update_gyro();
int mpu_9250_update_compass();
float mpu_9250_compute_heading();

bool mpu_9250_data_available();

#endif /* MPU_9250_H_ */
