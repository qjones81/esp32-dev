
/*
 * qrobot.c
 *
 *  Created on: Mar 14, 2017
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

#include <lwip/sockets.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "i2c/i2c.h"
#include "spi/spi.h"
#include "mpu9250_dmp/mpu_9250.h"
#include "amis_30543/amis_30543.h"
#include "adns3080/adns3080.h"
#include "stepper/stepper.h"
#include "ar6115e/ar6115e.h"
#include "utils/utils.h"
#include "wifi/wifi.h"
#include "sockets/socket_server.h"

static const char *tag = "qrobot";

#define KP_BALANCE 0.19f
#define KD_BALANCE 28.0f

#define KP_RAISEUP 0.16f
#define KD_RAISEUP 44.0f

#define KP_SPEED 0.08f
#define KI_SPEED 0.1f

#define MAX_ACCEL 7

// save/load to flash
float Kp_position = 1.0f;
float Ki_position = 0.0f;
float Kd_position = 0.0f;

float Kp_heading = 1.0f;
float Ki_heading = 0.0f;
float Kd_heading = 0.0f;

float Kp_speed = KP_SPEED;
float Ki_speed = KI_SPEED;
float Kd_speed = 0.0f;

float Kp_balance = KP_BALANCE;
float Ki_balance = 0.0f;
float Kd_balance = KD_BALANCE;

float Kp_line_following = 1.0f;
float Ki_line_following = 0.0f;
float Kd_line_following = 0.0f;
//end save

float current_heading_target = 0.0f;
float current_heading = 0.0f;
float prev_heading = 0.0f;
float current_heading_error = 0.0f;
float prev_heading_error = 0.0f;

float current_speed_target = 0.0f;
float current_speed = 0.0f;
float prev_speed = 0.0f;
float current_speed_error = 0.0f;
float prev_speed_error = 0.0f;

float current_pos_x = 0.0f;
float current_pos_y = 0.0f;

float prev_pos_x = 0.0f;
float prev_pos_y = 0.0f;

float current_pos_error = 0.0f;
float prev_pos_error = 0.0f;

float target_tilt = 0.0f;
float current_tilt = 0.0f;
float prev_tilt = 0.0f;
float current_tilt_target = 0.0f;
float current_tilt_error = 0.0f;
float prev_tilt_error = 0.0f;
float max_target_tilt = 40.0f;

float max_control_output = 400;

int ITERM_MAX_ERROR = 25;   // Iterm windup constants for PI control //40
int ITERM_MAX = 8000;       // 5000

float speed_m1 = 0.0f;
float speed_m2 = 0.0f;

//save to flash
float max_linear_speed = 1.0f;
float max_linear_accel = 0.25f;

float max_angular_speed = 1.0f;
float max_angular_accel = 0.1f;

float wheel_radius_left = 4.0f;
float wheel_radius_right = 4.0f;
float wheel_base = 5.0f;

float control_output = 0.0f;
//end save

float pid_error_sum = 0.0f;
float steering_input = 0.0f;

bool robot_shutdown = true;
// Socket for Debug
//socket_device_t *sock = NULL;

// Sensors
adns_3080_device_t *adns_sensor;

// Motor Drivers
amis_30543_device_t *amis_motor_1;
amis_30543_device_t *amis_motor_2;

#if defined(CONFIG_QROBOT_USE_DEBUG_SERVICE)
void qrobot_control_debug_service(void *ignore)
{
    struct sockaddr_in client_address;
    struct sockaddr_in server_address;

    // Create Socket
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(tag, "ERROR: Unable to create socket:  socket(): %s", strerror(errno));
        goto END;
    }

    // Bind socket to port
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_address.sin_port = htons(CONFIG_QROBOT_DEBUG_PORT);
    int ret = bind(sock, (struct sockaddr *) &server_address,
            sizeof(server_address));
    if (ret < 0) {
        ESP_LOGE(tag, "ERROR: Unable to bind socket:  bind(): %s", strerror(errno));
        goto END;
    }

    // Set socket for listen
    ret = listen(sock, 5);
    if (ret < 0) {
        ESP_LOGE(tag, "ERROR: Unable to listen socket:  listen(): %s", strerror(errno));
        goto END;
    }
    ESP_LOGI(tag, "Control and debug service running on port: %d", CONFIG_QROBOT_DEBUG_PORT);
    int total = 2048; // 2 KB buffer
    //char *data = malloc(total);
    char data[2048];
    while (1) { // Listen for a new client connection.

        socklen_t client_address_length = sizeof(client_address);
        int client_sock = accept(sock, (struct sockaddr *) &client_address,
                &client_address_length);
        if (client_sock < 0) {
            ESP_LOGE(tag, "ERROR:  Unable to accept client connection:  accept(): %s", strerror(errno));
            goto END;
        }

        // We now have a new client ...
        ESP_LOGI(tag, "Got new connection from client.");

       // int total = 2 * 1024; // 2 KB buffer
        int size_used = 0;
       // char *data = malloc(total);
        bzero(data, sizeof(data));
        // Loop reading data.
        while (1) {
            ssize_t size_read = recv(client_sock, data + size_used,
                    total - size_used, 0);
            if (size_read < 0) {
                ESP_LOGE(tag, "recv: %d %s", size_read, strerror(errno));
                goto END;
            }
            if (size_read == 0) {
                break;
            }
            size_used += size_read;
        }

        // Finished reading data.
        ESP_LOGD(tag, "Data read (size: %d) was: %.*s", size_used, size_used,
                data);

        if(!strncmp(data, "Hello",5))
        {
            ESP_LOGI(tag, "Got HELLO!!!");

            int sent_bytes = send(client_sock, "GOTHELLO", 9, 0);
            if (sent_bytes == -1) {
                ESP_LOGE(tag, "ERROR:  Unable to send data:  send(): %s",
                        strerror(errno));

                //if (errno == ECONNRESET) {
                // TODO: Remove connection and don't send BLAH BLAH
                //socket_server_disconnect(device);
                //return;
                //}
            }
            ESP_LOGI(tag, "Sent Bytes: %d", sent_bytes);
        }
       // free(data);

        // Do something with request.
        close(client_sock);
    }

    END: vTaskDelete(NULL);
}
#endif



float qrobot_stability_PID_control(float dt, float input, float setpoint)
{
	static float setpoint_old = 0.0f;
	static float prev_error = 0.0f;
	static float prev_error_old = 0.0f;

	float error;
	float output;

	error = setpoint - input;


	// Kd is implemented in two parts
	//    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
	//    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
	output = Kp_balance * error + (Kd_balance * (setpoint - setpoint_old) - Kd_balance * (input - prev_error_old)) / dt;

	//ESP_LOGI(tag, "DT: %f", (Kd_balance * (input - prev_error_old)) / dt);
	prev_error_old = prev_error;
	prev_error = input;  // error for Kd is only the input component
	setpoint_old = setpoint;
	return (output);
}

// PI controller implementation (Proportional, integral). DT is in milliseconds
float qrobot_speed_PID_control(float dt, float input, float setPoint)
{
  float error;
  float output;

  error = setPoint - input;
  pid_error_sum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  pid_error_sum = constrain(pid_error_sum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp_speed * error + Ki_speed * pid_error_sum * dt * 0.001; // DT is in miliseconds...
  return (output);
}


void qrobot_controller_task(void *ignore)
{
    ESP_LOGD(tag, ">> qrobot_controller_task");
	uint32_t last_update = millis();
	vTaskDelay(5 / portTICK_PERIOD_MS);
	uint32_t now = 0;
	float motor_1_speed = 0;
	float motor_2_speed = 0;
	float dt;

	float robot_speed = 0.0f;
	float robot_speed_old = 0.0f;
	float estimated_speed_filtered = 0.0f;
	while (1) {
		if(robot_shutdown)
		{
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}
		now = millis();
		dt = (now - last_update);

		prev_tilt = current_tilt;
		current_tilt = mpu_9250_get_phi() * 1.1111111; //Degrees to GRAD

		//ESP_LOGI(tag, "Phi: %0.2f", current_tilt);
		//ESP_LOGI(tag, "Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f, Phi: %0.02f", mpu_9250_get_roll(), mpu_9250_get_pitch(), mpu_9250_get_yaw(), mpu_9250_get_phi());

		// We calculate the estimated robot speed:
		// Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
		robot_speed_old = robot_speed;
		robot_speed = (motor_1_speed + motor_2_speed) / 2; // Positive: forward

		float angular_velocity = (current_tilt - prev_tilt) * 90.0; // 90 is an empirical extracted factor to adjust for real units
		float estimated_speed = -robot_speed_old - angular_velocity; // We use robot_speed(t-1) or (t-2) to compensate the delay
		estimated_speed_filtered = estimated_speed_filtered * 0.95
				+ estimated_speed * 0.05; // low pass filter on estimated speed

				// SPEED CONTROL: This is a PI controller.
				//    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
		float throttle = (current_speed_target) * 500;
		float steering = (steering_input) * 200;
		target_tilt = qrobot_speed_PID_control(dt, estimated_speed_filtered, -throttle);
		target_tilt = constrain(target_tilt, -max_target_tilt, max_target_tilt); // limited output

		//ESP_LOGI(tag, "Throttle: %0.2f", throttle);

		//ESP_LOGI(tag, "Speed: %0.2f, %0.2f, %0.2f, %0.2f", target_tilt, robot_speed, estimated_speed_filtered, angular_velocity);

		control_output += qrobot_stability_PID_control(dt, current_tilt, target_tilt);
		control_output = constrain(control_output, -max_control_output, max_control_output);

		float motor_1_speed_old = motor_1_speed;
		float motor_2_speed_old = motor_2_speed;

		motor_1_speed = control_output - steering;
		motor_2_speed = control_output + steering;


		motor_1_speed = constrain(motor_1_speed, -max_control_output, max_control_output);
		motor_2_speed = constrain(motor_2_speed, -max_control_output, max_control_output);


		// WE LIMIT MAX ACCELERATION of the motors
		  if ((motor_1_speed - motor_1_speed_old) > MAX_ACCEL)
			  motor_1_speed -= MAX_ACCEL;
		  else if ((motor_1_speed - motor_1_speed_old) < -MAX_ACCEL)
			  motor_1_speed += MAX_ACCEL;
		  else
			  motor_1_speed = motor_1_speed;

		  if ((motor_2_speed - motor_2_speed_old) > MAX_ACCEL)
			  motor_2_speed -= MAX_ACCEL;
		  else if ((motor_2_speed - motor_2_speed_old) < -MAX_ACCEL)
			  motor_2_speed += MAX_ACCEL;
		  else
			  motor_2_speed = motor_2_speed;

		//ESP_LOGI(tag, "Output: %0.2f, %0.2f, %0.2f", control_output, motor_1_speed, motor_2_speed);
		if ((current_tilt < 76) && (current_tilt > -76)) // Is robot ready (upright?)
		{
			stepper_control_set_speed(STEPPER_MOTOR_1, motor_1_speed * 46);
			stepper_control_set_speed(STEPPER_MOTOR_2, motor_2_speed * 46);

			if ((current_tilt < 45) && (current_tilt > -45)) {
				Kp_balance = KP_BALANCE;         // CONTROL GAINS FOR RAISE UP
				Kd_balance = KD_BALANCE;
				Kp_speed = KP_SPEED;
				Ki_speed = KI_SPEED;
			} else // We are in the raise up procedure => we use special control parameters
			{
				Kp_balance = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
				Kd_balance = KD_RAISEUP;
				Kp_speed = 0;
				Ki_speed = 0;
			}
		}
		else
		{
			stepper_control_set_speed(STEPPER_MOTOR_1, 0);
			stepper_control_set_speed(STEPPER_MOTOR_2, 0);
			control_output = 0;
			pid_error_sum = 0;

				Kp_balance = 0.16;         // CONTROL GAINS FOR RAISE UP
				Kd_balance = 44;
				Kp_speed = 0;
				Ki_speed = 0;

		}

		last_update = millis();

		uint32_t loop_time = millis() - now;
		//ESP_LOGI(tag, "Time: %d", loop_time);

		vTaskDelay(max(1, (5 - loop_time)) / portTICK_PERIOD_MS);
		//vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS); // 5 ms loop time
		//delay_ms(5);
	}
    vTaskDelete(NULL);
}
void qrobot_rc_control_service(void *ignore)
{
    ESP_LOGD(tag, ">> task_qrobot_read");
    while (1) {
    }
    vTaskDelete(NULL);
}
void ar6115e_input_handler(pulse_event_t event)
{
    // Aileron Left = 2000, Right = 1000
    // Throttle Down = 1000, Up = 2000

    switch (event.channel) {
        case THROTTLE:
            current_speed_target = map_f(event.pulse_width, 1100, 1900, -1.0f, 1.0f);
            break;
        case AILERON:
            steering_input = map_f(event.pulse_width, 1900, 1100, -1.0f, 1.0f);
            break;
        default:
            break;

    }

    //ESP_LOGI(tag, "Steering: %d | Throttle: %d", (int)steering_input, (int)current_speed_target);


   // stepper_control_set_speed(STEPPER_MOTOR_1, min(10000, 10000 * (current_speed_target / 100.0) - steering_input));
    //stepper_control_set_speed(STEPPER_MOTOR_2, min(10000, 10000 * (current_speed_target / 100.0) + steering_input));

}

void qrobot_init_adns()
{
    adns_sensor = (adns_3080_device_t *) malloc(sizeof(adns_3080_device_t));

    // Check for allocation blah blah
    // 2Mhz, Mode 3
    ESP_LOGI(tag, "Adding ADNS-3080 to SPI Bus...\n");
    ESP_ERROR_CHECK(spi_add_device(-1, 2000000, 3, &adns_sensor->spi_device));
    ESP_LOGI(tag, "Successfully added ADNS-3080.\n");

    // Setup Pins
    adns_sensor->reset_pin = GPIO_NUM_25;
    adns_sensor->cs_pin = GPIO_NUM_13;
    adns_sensor->x = 0;
    adns_sensor->y = 0;

    adns_3080_init(adns_sensor);
}
void qrobot_init_ar6115e()
{
    ar6115e_init(ar6115e_input_handler);
    ar6115e_add_channel(THROTTLE, GPIO_NUM_14);
    ar6115e_add_channel(AILERON, GPIO_NUM_27);
    ar6115e_start();
}
void qrobot_init_mpu9250()
{
    // Init i2c
    i2c_init(GPIO_NUM_22, GPIO_NUM_21);

    // Make sure everything is up and ready
    delay_ms(250);

    // Begin IMU Setup
    // Call mpu_9250_begin() to verify communication and initialize
    if (mpu_9250_begin() != INV_SUCCESS) {
        ESP_LOGE(tag, "Error Initializing MPU9250 IMU.\n");
        return;
    }
}
void qrobot_init_amis30543_drivers()
{

    // TODO: Update CS pins to new board
    // Init Memory
    amis_motor_1 = (amis_30543_device_t *) malloc(sizeof(amis_30543_device_t));
    // TODO: Error check for allocation

    //Left
    // 10Mhz, Mode 0
    ESP_LOGI(tag, "Adding AMIS-30543 (1) to SPI Bus...\n");
    ESP_ERROR_CHECK(spi_add_device(GPIO_NUM_5, 1000000, 0, &amis_motor_1->spi_device)); // TODO: Pass In Values
    ESP_LOGI(tag, "Successfully added AMIS-30543 (1).\n");

    delay_ms(100);

    ESP_LOGI(tag, "Initializing AMIS-30543 (1)...\n");
    amis_30543_init(amis_motor_1); // Init
    amis_30543_reset(amis_motor_1); // Reset
    amis_30543_set_current(amis_motor_1, 1000); // Set current limit to 1000ma
    amis_30543_set_step_mode(amis_motor_1, MicroStep16); // Set microstepping to 1/16
    //amis_30543_pwm_frequency_double(amis_left, true); // 45.6 khz
    amis_30543_enable_driver(amis_motor_1, true); // Enable motor output

    // Verify Settings
    bool verified = amis_30543_verify(amis_motor_1);
    ESP_LOGI(tag, "AMIS-30543 Initialized (1) %d.\n", verified);

    // Right
    amis_motor_2 = (amis_30543_device_t *) malloc(sizeof(amis_30543_device_t));
    // TODO: Error check for allocation

    // 10Mhz, Mode 0
    ESP_LOGI(tag, "Adding AMIS-30543 (2) to SPI Bus...\n");
    ESP_ERROR_CHECK(spi_add_device(GPIO_NUM_4, 1000000, 0, &amis_motor_2->spi_device)); // TODO: Pass In Values
    ESP_LOGI(tag, "Successfully added AMIS-30543 (2).\n");

    delay_ms(100);

    ESP_LOGI(tag, "Initializing AMIS-30543 (2)...\n");
    amis_30543_init(amis_motor_2); // Init
    amis_30543_reset(amis_motor_2); // Reset
    amis_30543_set_current(amis_motor_2, 1500); // Set current limit to 1000ma
    amis_30543_set_step_mode(amis_motor_2, MicroStep16); // Set microstepping to 1/16
    //amis_30543_pwm_frequency_double(amis_right, true); // 45.6 khz
    amis_30543_enable_driver(amis_motor_2, true); // Enable motor output

    // Verify Settings
    verified = amis_30543_verify(amis_motor_2);
    ESP_LOGI(tag, "AMIS-30543 Initialized (2) %d.\n", verified);
}

void qrobot_init_stepper_motors()
{
	// Motor 1 = Right, Motor 2 = Left
    stepper_motor_device_config_t motor_1_cfg = { .step_pin = GPIO_NUM_17,
            .dir_pin = GPIO_NUM_16, .step_hold_delay = 10, .dir_reverse = true };

    stepper_motor_device_config_t motor_2_cfg = { .step_pin = GPIO_NUM_15,
            .dir_pin = GPIO_NUM_2, .step_hold_delay = 10, .dir_reverse = false };

    stepper_control_add_device(STEPPER_MOTOR_1, motor_1_cfg);
    stepper_control_add_device(STEPPER_MOTOR_2, motor_2_cfg);
    stepper_control_start();
}
void qrobot_init()
{
    //Init sensors
    qrobot_init_mpu9250(); // Init IMU

    delay_ms(500);
    ESP_LOGI(tag, "Gyro Calibration: Keep still for 10 seconds");
    delay_ms(500);

    qrobot_init_adns(); // Init ADNS

    //Init motors
   qrobot_init_stepper_motors(); // Init Steppers

   // Init Drivers
   qrobot_init_amis30543_drivers(); // Init motor drivers

   // Init Control
   qrobot_init_ar6115e(); // RC input

    //Init sockets

#if defined(CONFIG_QROBOT_USE_DEBUG_SERVICE)
    xTaskCreate(&qrobot_control_debug_service, "control_debug_service", 4096, NULL, 3, NULL);
#endif

    // Gyro Calibration delay
    delay_ms(5000);

    // Little motor vibration and servo move to indicate that robot is ready
      for (uint8_t k = 0; k < 5; k++)
      {
    	  stepper_control_set_speed(STEPPER_MOTOR_1, 5*46);
    	  stepper_control_set_speed(STEPPER_MOTOR_2, 5*46);
        //BROBOT.moveServo1(SERVO_AUX_NEUTRO + 100);
        delay_ms(200);
        stepper_control_set_speed(STEPPER_MOTOR_1, -5*46);
        stepper_control_set_speed(STEPPER_MOTOR_2, -5*46);
        //BROBOT.moveServo1(SERVO_AUX_NEUTRO - 100);
        delay_ms(200);
      }


    robot_shutdown = true;
    //POST
   // stepper_control_set_speed(STEPPER_MOTOR_1, 5000);
   // stepper_control_set_speed(STEPPER_MOTOR_2, 2000);
}
void qrobot_start()
{
	robot_shutdown = false;
   xTaskCreate(&qrobot_controller_task, "qrobot_controller", 4096, NULL, 6, NULL);
}



