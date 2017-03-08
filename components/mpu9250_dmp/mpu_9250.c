/*
 * mpu_9250.c
 *
 *  Created on: Feb 13, 2017
 *      Author: qjones
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <math.h>
#include "mpu_9250.h"
#include "i2c/i2c.h"
#include "utils/utils.h"
static char tag[] = "mpu_9250";

const signed char defaultOrientation[9] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};


int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;
long qw, qx, qy, qz;
long temperature;
uint32_t _time;
float pitch, roll, yaw;
float heading;

float _mSense = 6.665f; // Constant - 4915 / 32760
float _aSense = 0.0f;   // Updated after accel FSR is set
float _gSense = 0.0f;   // Updated after gyro FSR is set

float PI = 3.14159265358979323846f;
// Debug
int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate

float sum = 0;
uint32_t sumCount = 0;

float deltat = 0.0f;                             // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval

float qToFloat(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

float mpu_9250_calc_quat(long axis)
{
	return qToFloat(axis, 30);
}


// Read Task
void task_imu_reader_task(void *ignore)
{
	ESP_LOGD(tag, ">>task_imu_reader_task");
	while(1) {

		// If intPin goes high, all data registers have new data
		if ( mpu_9250_fifo_available() ) {
			// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
			if ( mpu_9250_dmp_update_fifo() == INV_SUCCESS) {

				// mpu_9250_compute_euler_angles can be used -- after updating the
				// quaternion values -- to estimate roll, pitch, and yaw
				mpu_9250_compute_euler_angles(1); // degrees
				// After calling mpu_9250_dmp_update_fifo() the ax, gx, mx, etc. values
							  // are all updated.
							  // Quaternion values are, by default, stored in Q30 long
							  // format. calcQuat turns them into a float between -1 and 1

				//printIMUData();
				sumCount++;
			}
		}

		Now = micros();
		deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		sum += deltat;


		delt_t = millis() - count;
		if (delt_t > 500) { // update LCD once per half-second independent of read rate

			ESP_LOGI(tag, "average rate = %f\n\r", (float) sumCount/sum);

			float q0 = mpu_9250_calc_quat(qw);
			float q1 = mpu_9250_calc_quat(qx);
			float q2 = mpu_9250_calc_quat(qy);
			float q3 = mpu_9250_calc_quat(qz);

			int16_t tempCount = mpu_9250_read_temp_data();  // Read the adc values
			float temperature = (tempCount / 340.0f) + 21.0f; // Temperature in degrees Centigrade
//			ESP_LOGI(tag, "ax = %f", (float)ax/1000.0f);
//			ESP_LOGI(tag, " ay = %f", (float)ay/1000.0f);
//			ESP_LOGI(tag, " az = %f mg\n\r", (float)az/1000.0f);
//
//			ESP_LOGI(tag, "gx = %d", gx);
//			ESP_LOGI(tag, " gy = %d", gy);
//			ESP_LOGI(tag, " gz = %d  deg/s\n\r", gz);
//
//			ESP_LOGI(tag, "mx = %d", mx);
//			ESP_LOGI(tag, " my = %d", my);
//			ESP_LOGI(tag, " mz = %d  mG\n\r", mz);

			ESP_LOGI(tag, "Q: %f %f %f %f\n", q0, q1, q2, q3);
			ESP_LOGI(tag, "R/P/Y: %f %f %f\n", roll, pitch, yaw);
			ESP_LOGI(tag, "Temp: %f\n", temperature);
			ESP_LOGI(tag, "Time: %d\n", _time);

			count = millis();
			sum = 0;
			sumCount = 0;
		}

	}

	vTaskDelete(NULL);
}




inv_error_t mpu_9250_begin(void)
{
	ESP_LOGI(tag, "MPU9250 Initializing...\n");

	inv_error_t result;
	struct int_param_s int_param;

	// TODO: Start i2c here.  Check for bool if already started?
	//Wire.begin();

	result = mpu_init(&int_param);

	if (result)
		return result;

	mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	_gSense = mpu_9250_get_gyro_sens();
	_aSense = mpu_9250_get_accel_sens();

	ESP_LOGI(tag, "MPU9250 is online.\n");
	return result;
}

float mpu_9250_get_gyro_sens()
{
	float sens;
	if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}

float mpu_9250_get_accel_sens()
{
	unsigned short sens;
	if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}
int mpu_9250_read_temp_data()
{
	long data;
	mpu_get_temperature(&data, NULL);
	return (int)data;
}

inv_error_t mpu_9250_dmp_begin(unsigned short features, unsigned short fifoRate)
{
	unsigned short feat = features;
	unsigned short rate = fifoRate;

	if (dmp_load_motion_driver_firmware() != INV_SUCCESS)
		return INV_ERROR;

	// 3-axis and 6-axis LP quat are mutually exclusive.
	// If both are selected, default to 3-axis
	if (feat & DMP_FEATURE_LP_QUAT)
	{
		feat &= ~(DMP_FEATURE_6X_LP_QUAT);
		dmp_enable_lp_quat(1);
	}
	else if (feat & DMP_FEATURE_6X_LP_QUAT)
		dmp_enable_6x_lp_quat(1);

	if (feat & DMP_FEATURE_GYRO_CAL)
		dmp_enable_gyro_cal(1);

	if (mpu_9250_dmp_enable_features(feat) != INV_SUCCESS)
		return INV_ERROR;

	// TODO: Constrain Util
	//rate = constrain(rate, 1, 200);

	if (mpu_9250_dmp_set_fifo_rate(rate) != INV_SUCCESS)
		return INV_ERROR;

	return mpu_set_dmp_state(1);
}
inv_error_t mpu_9250_dmp_enable_features(unsigned short mask)
{
	unsigned short enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP;
	return dmp_enable_feature(enMask);
}

// Fifo Features
inv_error_t mpu_9250_dmp_set_fifo_rate(unsigned short rate)
{
	if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
	return dmp_set_fifo_rate(rate);
}

unsigned short mpu_9250_fifo_available(void)
{
	unsigned char fifoH, fifoL;

	fifoH = i2c_read_byte(0x68, MPU9250_FIFO_COUNTH);
	fifoL = i2c_read_byte(0x68, MPU9250_FIFO_COUNTL);
	/*if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
	{
		ESP_LOGI(tag, "FIFO H 0.\n");
		return 0;
	}

	if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
	{
		ESP_LOGI(tag, "FIFO L 0.\n");
		return 0;
	}*/
//	ESP_LOGI(tag, "FIFO:%d and %d\n", fifoH, fifoL);
	return (fifoH << 8 ) | fifoL;
}


inv_error_t mpu_9250_dmp_update_fifo(void)
{
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;

	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }

	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		qw = quat[0];
		qx = quat[1];
		qy = quat[2];
		qz = quat[3];
	}

	_time = timestamp;

	return INV_SUCCESS;
}

// Quaternion
void mpu_9250_compute_euler_angles(bool degrees)
{
    float dqw = qToFloat(qw, 30);
    float dqx = qToFloat(qx, 30);
    float dqy = qToFloat(qy, 30);
    float dqz = qToFloat(qz, 30);

    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);

	if (degrees)
	{
		pitch *= (180.0 / PI);
		roll *= (180.0 / PI);
		yaw *= (180.0 / PI);
		if (pitch < 0) pitch = 360.0 + pitch;
		if (roll < 0) roll = 360.0 + roll;
		if (yaw < 0) yaw = 360.0 + yaw;
	}
}


