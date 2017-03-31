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
#include "sockets/socket_server.h"

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

volatile float temperature;
uint32_t _time;

volatile float pitch, roll, yaw;
volatile float heading;
volatile float phi;

float _mSense = 6.665f; // Constant - 4915 / 32760
float _aSense = 0.0f;   // Updated after accel FSR is set
float _gSense = 0.0f;   // Updated after gyro FSR is set

float PI = 3.14159265358979323846f;

// Socket for Debug
static socket_device_t *sock = NULL;

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
void mpu_imu_reader_task(void *ignore)
{
	ESP_LOGD(tag, ">>mpu_imu_reader_task started.");
	while (1) {

		// If intPin goes high, all data registers have new data
		if (mpu_9250_fifo_available()) {
			// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
			if (mpu_9250_dmp_update_fifo() == INV_SUCCESS) {
				mpu_9250_compute_euler_angles(1); // degrees
			}
			//mpu_reset_fifo(); // always reset fifo
		}

		// Check non-dmp data
		if (mpu_9250_data_available()) {
			// Update Compass
			mpu_9250_update_compass();
			//mpu_9250_update_accel();
			//mpu_9250_update_gyro();

			// Update Temperature
			//int16_t tempCount = mpu_9250_read_temp_data(); // Read the adc values
			//temperature = (tempCount / 340.0f) + 21.0f; // Temperature in degrees Centigrade

			// Compute Heading
			mpu_9250_compute_heading();
		}
		vTaskDelay(2 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

#if defined(CONFIG_MPU9250_USE_DEBUG_SERVICE)
void task_mpu9250_socket_debug(void *ignore)
{
    ESP_LOGD(tag, ">> task_mpu9250_socket_debug");
    while (1) {
        char str[80];
        // TODO: Mutex Here or does volatile work here?
        //float a_sens_inv = 1.0 / _aSense;
        sprintf(str, "%f:%f\n",  heading, temperature);

        //ESP_LOGI(tag, "%s",str);
        socket_server_send_data(sock, (uint8_t *)str, strlen(str) + 1);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
#endif

inv_error_t mpu_9250_begin(void)
{
	ESP_LOGI(tag, "MPU9250 Initializing...\n");

	inv_error_t result;
	struct int_param_s int_param;

	// TODO: Start i2c here.  Check for bool if already started?

	result = mpu_init(&int_param);

	if (result)
		return result;

	mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	_gSense = mpu_9250_get_gyro_sens()/32768.0;
	_aSense = mpu_9250_get_accel_sens()/32768.0;

	ESP_LOGI(tag, "MPU9250 is online.\n");

    ESP_LOGI(tag, "Initializing DMP mode.\n");
    inv_error_t error = mpu_9250_dmp_begin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
            DMP_FEATURE_GYRO_CAL, // Use gyro calibration
            CONFIG_MPU9250_DMP_RATE); // Set DMP FIFO rate to 200 Hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
    // TOOD: Check for error
    if (error == INV_ERROR) {
        ESP_LOGE(tag, "ERROR:  Initializing MPU9250 DMP: %d\n", error);
    } else {
        ESP_LOGI(tag, "MPU9250 DMP Initialized.\n");
    }

	// SUCCESS:  Now start required task/services
	xTaskCreate(&mpu_imu_reader_task, "mpu9250_task", 2048, NULL, 3, NULL);

	#if defined(CONFIG_MPU9250_USE_DEBUG_SERVICE)
	        sock = (socket_device_t *) malloc(sizeof(socket_device_t));
	        sock->port = CONFIG_MPU9250_DEBUG_PORT;
	        socket_server_init(sock);
	        socket_server_start(sock);

	        xTaskCreate(&task_mpu9250_socket_debug, "mpu9250_socket_task", 2048, NULL, 3, NULL);
	#endif

	        mpu_reset_fifo();
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

	fifoH = i2c_read_byte(CONFIG_MPU9250_DEFAULT_ADDRESS, MPU9250_FIFO_COUNTH);
	fifoL = i2c_read_byte(CONFIG_MPU9250_DEFAULT_ADDRESS, MPU9250_FIFO_COUNTL);
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

    //ESP_LOGI(tag, "Quat: %0.2f, %0.2f, %0.2f, %0.2f", dqw, dqx, dqy, dqz);

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

    phi = (atan2(2 * (dqy * dqz + dqw * dqx), dqw * dqw - dqx * dqx - dqy * dqy + dqz * dqz));

	if (degrees)
	{
		pitch *= (180.0 / PI);
		roll *= (180.0 / PI);
		yaw *= (180.0 / PI);
		phi *= (180.0 / PI);
		if (pitch < 0) pitch = 360.0 + pitch;
		if (roll < 0) roll = 360.0 + roll;
		if (yaw < 0)
			yaw = 360.0 + yaw;
	}

}
int mpu_9250_update_accel(void)
{
    short data[3];
    unsigned long time;
    if (mpu_get_accel_reg(data, &time))
    {
        return INV_ERROR;
    }
    ax = data[X_AXIS];
    ay = data[Y_AXIS];
    az = data[Z_AXIS];
    return INV_SUCCESS;
}

int mpu_9250_update_gyro(void)
{
    short data[3];
    unsigned long time;
    if (mpu_get_gyro_reg(data, &time))
    {
        return INV_ERROR;
    }
    gx = data[X_AXIS];
    gy = data[Y_AXIS];
    gz = data[Z_AXIS];
    return INV_SUCCESS;
}
int mpu_9250_update_compass()
{
    short data[3];
    unsigned long time;
    if(mpu_get_compass_reg(data, &time))
    {
        return INV_ERROR;
    }
    mx = data[X_AXIS];
    my = data[Y_AXIS];
    mz = data[Z_AXIS];
    return INV_SUCCESS;
}


void mpu_9250_compute_heading(void)
{
    if (my == 0)
        heading = (mx < 0) ? 180.0 : 0;
    else
        heading = atan2(mx, my);

    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);
    else if (heading < 0) heading += 2 * PI;

    heading*= 180.0 / PI;

}

bool mpu_9250_data_available()
{
    uint8_t intStatusReg;
    if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS) {
        return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
    }
    return 0;
}

float mpu_9250_get_pitch()
{
	return pitch;
}
float mpu_9250_get_roll()
{
	return roll;
}
float mpu_9250_get_yaw()
{
	return yaw;
}
float mpu_9250_get_heading()
{
	return heading;
}

// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
float mpu_9250_get_phi() {

  //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
  //return Phi angle (robot orientation) from quaternion DMP output
  return phi;
}
