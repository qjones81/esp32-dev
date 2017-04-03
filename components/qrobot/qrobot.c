
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
#include <stdarg.h>
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
#include "pid/pid.h"
#include "utils/utils.h"
#include "wifi/wifi.h"
#include "sockets/socket_server.h"

static const char *tag = "qrobot";

#define KP_BALANCE 0.19f
#define KD_BALANCE 28.0f

#define KP_RAISEUP 0.2f
#define KD_RAISEUP 44.0f

#define KP_SPEED 0.09f
#define KI_SPEED 0.1f

#define MAX_ACCEL 10

#define DEBUG_STACK_BUFFER_SIZE 80

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

float max_control_output = 500.0f;
float control_output = 0.0f;
uint16_t max_throttle = 300;
uint16_t max_steering = 250;

int ITERM_MAX_ERROR = 25;   // Iterm windup constants for PI control //40
int ITERM_MAX = 8000;       // 5000

//save to flash
float max_linear_speed = 1.0f;
float max_linear_accel = 0.25f;

float max_angular_speed = 1.0f;
float max_angular_accel = 0.1f;

float wheel_radius_1 = 0.045f; // 45 mm (Right)
float wheel_radius_2 = 0.045f; // 45 mm (Left)
float wheel_base = .151f; // 151 mm
float wheel_steps_per_m_1 = 11317.6848421; // steps per m (Right)
float wheel_steps_per_m_2 = 11317.6848421; // steps per m (Left)

uint16_t steps_per_rev = 3200;  // 1/16 microstepping
uint8_t microsteps = 16;
//end save

// robot pose
float x_pos = 0.0f;
float y_pos = 0.0f;
float theta = 0.0f;

float total_meters = 0.0f;
float current_goal_x = 0.0f;
float current_goal_y = 0.0f;
float current_goal_theta = 0.0f;

float pid_error_sum = 0.0f;
float pid_balance_error_sum = 0.0f;
float steering_input = 0.0f;

uint32_t stable_time = 0;

int steering_dead_zone = 40;
int throttle_dead_zone = 40;

bool robot_shutdown = true;
bool robot_stable = false;

bool robot_in_navigation = false;
// Socket for Debug
socket_device_t *sock = NULL;
static QueueHandle_t socket_debug_queue;

// Sensors
adns_3080_device_t *adns_sensor;

// Motor Drivers
amis_30543_device_t *amis_motor_1;
amis_30543_device_t *amis_motor_2;

// PIDS
pid_device_control_t goto_goal_pid;

typedef struct {
	char data[DEBUG_STACK_BUFFER_SIZE];
} socket_event_t;

#if defined(CONFIG_QROBOT_USE_CONTROL_SERVICE)
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
    server_address.sin_port = htons(CONFIG_QROBOT_CONTROL_PORT);
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
    char params[10][20]; // 10 Max
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

		// Check for commands
		char* token = strtok(data, " ");
		if(token)
		{
			if(!strncmp(data, "RESET",5))
			{
				current_goal_x = current_goal_y = current_goal_theta = 0;
				x_pos = y_pos = theta = 0;
				robot_in_navigation = false;
			}
			else if(!strncmp(data, "GOAL",4))
			{
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}

				if(counter == 3) {
					int sent_bytes = send(client_sock, "OK\n", 3, 0);
					current_goal_x = atof(params[0]);
					current_goal_y = atof(params[1]);
					current_goal_theta = atof(params[2]);
					robot_in_navigation = true;
				} else {
					send(client_sock, "INVALID\n", 8, 0);
				}
			}
			else if(!strncmp(data, "PID",3))
			{
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}

				if(counter == 3) {
					int sent_bytes = send(client_sock, "OK\n", 3, 0);
					goto_goal_pid.k_p = atof(params[0]);
					goto_goal_pid.k_i = atof(params[1]);
					goto_goal_pid.k_d = atof(params[2]);
				} else {
					send(client_sock, "INVALID\n", 8, 0);
				}
			}
		}

       // free(data);

        // Do something with request.
        close(client_sock);
    }

    END: vTaskDelete(NULL);
}
#endif

void qrobot_socket_debug_task(void *ignore)
{
    ESP_LOGD(tag, ">> qrobot_socket_debug_task");
    socket_event_t recvMe;
    while (1) {
    	xQueueReceive(socket_debug_queue, &recvMe, portMAX_DELAY);
        //ESP_LOGI(tag, "%s",recvMe.data);
        socket_server_send_data(sock, (uint8_t *)recvMe.data, strlen(recvMe.data) + 1);
        vTaskDelay(100/portTICK_PERIOD_MS); // Slow it down a bit...
    }
    vTaskDelete(NULL);
}

void qrobot_send_debug(char * format, ...)
{
#if defined(CONFIG_QROBOT_USE_DEBUG_SERVICE)
	char temp[DEBUG_STACK_BUFFER_SIZE];
	socket_event_t socket_event;
	va_list arg;
	va_start(arg, format);

  int len = vsnprintf(temp, sizeof(temp) - 1, format, arg);
  if (len > 0) {
	  sprintf(socket_event.data, "%s\n", temp);
	  xQueueSendToBack(socket_debug_queue, &socket_event, ( TickType_t ) 0);
  }
  va_end(arg);

#endif
}

float qrobot_stability_PID_control(float dt, float input, float setpoint)
{
	static float setpoint_old = 0.0f;
	static float prev_error = 0.0f;
	static float prev_error_old = 0.0f;

	float error;
	float output;

	error = setpoint - input;

	pid_balance_error_sum += error;

	if(abs(pid_balance_error_sum) > 30000)
	{
		stable_time = 0;
		control_output = 0;
		robot_stable = false;
		return 0;
	}

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
	float motor_1_speed_old = 0;
	float motor_2_speed_old = 0;
	float dt;

	float robot_speed = 0.0f;
	float robot_speed_old = 0.0f;
	float estimated_speed_filtered = 0.0f;
	float throttle = 0.0f;
	float steering = 0.0f;

	float angular_velocity = 0.0f;
	float estimated_speed = 0.0f;


	memset(&goto_goal_pid, 0, sizeof(pid_device_control_t));

	goto_goal_pid.k_p = 2.2f;
	goto_goal_pid.k_i = 0.0f;
	goto_goal_pid.k_d = 0.4f;
	goto_goal_pid.integral_windup_max = 1000;
	goto_goal_pid.output_min = -1.0f;
	goto_goal_pid.output_max = 1.0f;
	goto_goal_pid.clamp_output = true;

//	vTaskDelay(1000 / portTICK_PERIOD_MS);

	//ESP_LOGD(tag, "Begin Nav Test!");
//	while(1)
//	{
//		if (robot_shutdown) {
//			last_update = millis();
//			vTaskDelay(100 / portTICK_PERIOD_MS);
//			continue;
//		}
//
//
//		goto_goal_pid.set_point = target_x;
//		goto_goal_pid.input = x_pos;
//
//		float control_output = pid_compute(&goto_goal_pid);
//		float throttle = (control_output) * 100;
//		float steering = (steering_input) * max_steering;
//
//		motor_1_speed_old = motor_1_speed;
//		motor_2_speed_old = motor_2_speed;
//
//		motor_1_speed = throttle - steering;
//		motor_2_speed = throttle + steering;
//
//
//		motor_1_speed = constrain(motor_1_speed, -max_control_output,
//				max_control_output) * 46;
//		motor_2_speed = constrain(motor_2_speed, -max_control_output,
//				max_control_output) * 46;
//
//
//	//	 WE LIMIT MAX ACCELERATION of the motors
//				if ((motor_1_speed - motor_1_speed_old) > MAX_ACCEL)
//					motor_1_speed -= MAX_ACCEL;
//				else if ((motor_1_speed - motor_1_speed_old) < -MAX_ACCEL)
//					motor_1_speed += MAX_ACCEL;
//				else
//					motor_1_speed = motor_1_speed;
//
//				if ((motor_2_speed - motor_2_speed_old) > MAX_ACCEL)
//					motor_2_speed -= MAX_ACCEL;
//				else if ((motor_2_speed - motor_2_speed_old) < -MAX_ACCEL)
//					motor_2_speed += MAX_ACCEL;
//				else
//					motor_2_speed = motor_2_speed;
//
//
//		stepper_control_set_speed(STEPPER_MOTOR_1, motor_1_speed);
//		stepper_control_set_speed(STEPPER_MOTOR_2, motor_2_speed);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//	}

	while (1) {
		if(robot_shutdown)
		{
			last_update = millis();
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}
		else if(!robot_stable && (!equal_f(current_speed_target, 0, 0.01) || !equal_f(steering_input, 0, 0.01)))
		{
			last_update = millis();
			robot_stable = false;
			stable_time = 0;
			control_output = 0;
			motor_1_speed = motor_2_speed = motor_1_speed_old = motor_2_speed_old =0;
			pid_error_sum = 0;
			pid_balance_error_sum = 0;
			//ESP_LOGI(tag, "HERE: %f", current_speed_target);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}

		// Update Navigation. TODO: Move to task?
		float x = current_goal_x - x_pos;
		float y = current_goal_y - y_pos;

		float target_distance = sqrt((x*x)+(y*y));

		if(target_distance <= 0.1)
		{
			//robot_in_navigation = false;
		}
		now = millis();
		dt = (now - last_update);

		prev_tilt = current_tilt;
		current_tilt = mpu_9250_get_phi() * 1.1111111; //Degrees to GRAD

		// We calculate the estimated robot speed:
		// Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
		robot_speed_old = robot_speed;
		robot_speed = (motor_1_speed + motor_2_speed) / 2; // Positive: forward

		angular_velocity = (current_tilt - prev_tilt) * 90.0; // 90 is an empirical extracted factor to adjust for real units
		estimated_speed = -robot_speed_old - angular_velocity; // We use robot_speed(t-1) or (t-2) to compensate the delay
		estimated_speed_filtered = estimated_speed_filtered * 0.95
				+ estimated_speed * 0.05; // low pass filter on estimated speed

				// SPEED CONTROL: This is a PI controller.
				//    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed


		// Check if we have arrived
		float speed_output =  0;
		if(robot_in_navigation)
		{
			goto_goal_pid.set_point = current_goal_x;
			goto_goal_pid.input = x_pos;

			speed_output = pid_compute(&goto_goal_pid);
		} else {
			speed_output = current_speed_target;
		}



		throttle = (speed_output) * max_throttle;
		steering = (steering_input) * max_steering;

		target_tilt = qrobot_speed_PID_control(dt, estimated_speed_filtered, -throttle);
		target_tilt = constrain(target_tilt, -max_target_tilt, max_target_tilt); // limited output

		control_output += qrobot_stability_PID_control(dt, current_tilt, target_tilt);
		control_output = constrain(control_output, -max_control_output, max_control_output);

		motor_1_speed_old = motor_1_speed;
		motor_2_speed_old = motor_2_speed;

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

		if ((current_tilt < 76) && (current_tilt > -76)) // Is robot ready (upright?)
		{
			if(robot_stable == false && stable_time > 100)
			{
				// ESP_LOGI(tag, "Robot Is Stable...\n");
				robot_stable = true;
			}
			else
			{
				stable_time += dt;
			}

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
			pid_balance_error_sum = 0;
			stable_time = 0;
			robot_stable = false;

			Kp_balance = KP_BALANCE;         // CONTROL GAINS FOR RAISE UP
			Kd_balance = KD_BALANCE;
			Kp_speed = 0;
			Ki_speed = 0;
		}

		last_update = millis();

		uint32_t loop_time = millis() - now;
		//ESP_LOGI(tag, "Time: %d", loop_time);

		vTaskDelay(max(1, (5 - loop_time)) / portTICK_PERIOD_MS);
		//vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS); // 5 ms loop time
	}
    vTaskDelete(NULL);
}

void qrobot_odometry_task(void *ignore)
{
	// Right Wheel = Motor 1
	// Left Wheel = Motor 2

    ESP_LOGD(tag, ">> qrobot_odometry_task");
    int32_t wheel_1_step_prev = 0;
    int32_t wheel_2_step_prev = 0;
    int32_t wheel_1_step = 0;
    int32_t wheel_2_step = 0;
    float wheel_1_distance = 0;
    float wheel_2_distance = 0;
    float delta_m = 0;
    int loop_count = 0;
   // uint32_t last_update = millis();

	while (1) {

		wheel_1_step_prev = wheel_1_step;
		wheel_2_step_prev = wheel_2_step;
		wheel_1_step = stepper_control_get_position(STEPPER_MOTOR_1);
		wheel_2_step = stepper_control_get_position(STEPPER_MOTOR_2);

		//ESP_LOGI(tag, "X: %d, Y: %d", wheel_1_step, wheel_2_step);
		wheel_1_distance = (wheel_1_step - wheel_1_step_prev)
				/ wheel_steps_per_m_1; // meters
		wheel_2_distance = (wheel_2_step - wheel_2_step_prev)
				/ wheel_steps_per_m_2; // meters

		delta_m = (wheel_1_distance + wheel_2_distance) * 0.5f;

		total_meters += delta_m; // Update total odometry

		// Update Theta
		theta += (wheel_2_distance - wheel_1_distance) / wheel_base; // CW Positive

		// Clip to +/- 360 Degrees
		theta -= (float)((int)(theta/TWO_PI))*TWO_PI;

		// Keep to +/- 180 Degrees
		if (theta < -M_PI) {
			theta += TWO_PI;
		} else {
			if (theta > M_PI)
				theta -= TWO_PI;
		}

		// Update Position
		x_pos += (delta_m * (cos(theta)));
		y_pos += (delta_m * (sin(theta)));

    	if(loop_count >= 100)
    	{
			//ESP_LOGI(tag, "Pose: X: %f, Y: %f, Theta: %f", x_pos, y_pos, theta);
    		qrobot_send_debug("Pose: X: %f, Y: %f, Theta: %f\n",  x_pos, y_pos, theta);
			loop_count = 0;
    	}
    	loop_count++;
    	vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
void ar6115e_input_handler(pulse_event_t event)
{
    // Aileron Left = 2000, Right = 1000
    // Throttle Down = 1000, Up = 2000

	switch (event.channel) {
		case THROTTLE:
			if (abs(event.pulse_width - 1500) < throttle_dead_zone) { // Handle Dead Zones...
				current_speed_target = 0;
			} else if (event.pulse_width > 1500) {
				current_speed_target = map_f(event.pulse_width, 1500 + throttle_dead_zone, 1900, 0.0f, 1.0f);
			} else {
				current_speed_target = map_f(event.pulse_width, 1100, 1500 - throttle_dead_zone, -1.0f, 0.0f);
			}
			break;
		case AILERON:
			if (abs(event.pulse_width - 1500) < steering_dead_zone) { // Handle Dead Zones...
				steering_input = 0;
			} else if (event.pulse_width > 1500) {
				steering_input = map_f(event.pulse_width, 1900, 1500 + steering_dead_zone, -1.0f, 0.0f);
			}
			else {
				steering_input = map_f(event.pulse_width, 1500 - steering_dead_zone, 1100, 0.0f, 1.0f);
			}
			//ESP_LOGI(tag, "Steering: %d / %f", event.pulse_width, steering_input);
			break;
		default:
			break;
    }
//	float throttle = (current_speed_target) * max_throttle;
//	float steering = (steering_input) * max_steering;
//
//	float motor_1_speed = throttle - steering;
//	float motor_2_speed = throttle + steering;
//
//	motor_1_speed = constrain(motor_1_speed, -max_control_output,
//			max_control_output);
//	motor_2_speed = constrain(motor_2_speed, -max_control_output,
//			max_control_output);
//
//	stepper_control_set_speed(STEPPER_MOTOR_1, motor_1_speed * 46);
//	stepper_control_set_speed(STEPPER_MOTOR_2, motor_2_speed * 46);

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
	ESP_LOGI(tag, "Initializing AR6115e RC Receiver.\n");
    ar6115e_init(ar6115e_input_handler);
    ar6115e_add_channel(THROTTLE, GPIO_NUM_14);
    ar6115e_add_channel(AILERON, GPIO_NUM_27);
    ar6115e_start();
    ESP_LOGI(tag, "AR6115e started.\n");
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

    // Right
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

    // Left
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
	ESP_LOGI(tag, "QRobot by Quincy Jones v1.0.  USE AT YOUR OWN RISK!\n\n");

	// Init SPI Bus
	ESP_LOGI(tag, "Initializing SPI Bus...\n");
	ESP_ERROR_CHECK(spi_init(VSPI_MOSI, VSPI_MISO, VSPI_CLK)); // TODO: Pass In Values
	ESP_LOGI(tag, "SPI Bus Initialized.\n");

	ESP_LOGI(tag, "Initializing Sensors...");
	//Init sensors
	qrobot_init_mpu9250(); // Init IMU

	uint32_t gyro_init_begin = millis();
	delay_ms(500);
	ESP_LOGI(tag, "Begin gyro calibration: Keep robot still for 10 seconds...\n");
	delay_ms(500);

	//qrobot_init_adns(); // Init ADNS

	//Init motors
	qrobot_init_stepper_motors(); // Init Steppers

	// Init Drivers
	qrobot_init_amis30543_drivers(); // Init motor drivers

	// Init Control
	qrobot_init_ar6115e(); // RC input

	// Gyro Calibration delay
	delay_ms(max(1, 8000 - (millis() - gyro_init_begin)));

	// Pulse motors for ready indication
	for (uint8_t k = 0; k < 5; k++) {
		stepper_control_set_speed(STEPPER_MOTOR_1, 5 * 46);
		stepper_control_set_speed(STEPPER_MOTOR_2, 5 * 46);
		//BROBOT.moveServo1(SERVO_AUX_NEUTRO + 100);
		delay_ms(200);
		stepper_control_set_speed(STEPPER_MOTOR_1, -5 * 46);
		stepper_control_set_speed(STEPPER_MOTOR_2, -5 * 46);

		//BROBOT.moveServo1(SERVO_AUX_NEUTRO - 100);
		delay_ms(200);
	}
	stepper_control_set_speed(STEPPER_MOTOR_1, 0);
	stepper_control_set_speed(STEPPER_MOTOR_2, 0);

	//Init sockets
	#if defined(CONFIG_QROBOT_USE_CONTROL_SERVICE)
		xTaskCreate(&qrobot_control_debug_service, "control_debug_service", 8192, NULL, 3, NULL);
	#endif

	#if defined(CONFIG_QROBOT_USE_DEBUG_SERVICE)
		sock = (socket_device_t *) malloc(sizeof(socket_device_t));
		sock->port = CONFIG_QROBOT_DEBUG_PORT;
		socket_server_init(sock);
		socket_server_start(sock);
		socket_debug_queue = xQueueCreate(25, sizeof(socket_event_t));
		xTaskCreate(&qrobot_socket_debug_task, "qrobot_debug_socket", 2048, NULL, 3, NULL);
	#endif
	delay_ms(500);

	// Setup Robot Parameters
	steps_per_rev = microsteps * 200;
	wheel_steps_per_m_1 = steps_per_rev / (M_PI * wheel_radius_1 * 2); // steps per m (Right)
	wheel_steps_per_m_2 = steps_per_rev / (M_PI * wheel_radius_2 * 2); // steps per m (Left)

	// Reset Odometers...
	stepper_control_reset_steps(STEPPER_MOTOR_1);
	stepper_control_reset_steps(STEPPER_MOTOR_2);

	robot_shutdown = true;
	robot_stable = false;
	robot_in_navigation = false;

}
void qrobot_start()
{
   robot_shutdown = false;
   robot_stable = false;

   // Start main controller task
   xTaskCreate(&qrobot_controller_task, "qrobot_controller", 4096, NULL, 6, NULL);

   // Start odometry task
   xTaskCreate(&qrobot_odometry_task, "qrobot_odometry", 2048, NULL, 5, NULL);
}

void qrobot_stop()
{
	robot_stable = false;
	robot_shutdown = true;
}



