
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
#include "freertos/event_groups.h"
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
#include "utils/vector.h"
#include "wifi/wifi.h"
//#include "adc/adc.h"
#include "driver/adc.h"
#include "sharpir/sharpir.h"
#include "image_processing/image_processing.h"
#include "sockets/socket_server.h"

static const char *tag = "qrobot";

#define DEBUG_STACK_BUFFER_SIZE 80

// save/load to flash
float Kp_position = 2.2f;
float Ki_position = 0.0f;
float Kd_position = 0.4f;

float Kp_heading = 0.5f;
float Ki_heading = 0.0f;
float Kd_heading = 0.0f;

float Kp_speed = 0.09f;
float Ki_speed = 0.1f;
float Kd_speed = 0.0f;

float Kp_balance = 0.19f;
float Ki_balance = 0.0f;
float Kd_balance = 28.0f;

float Kp_upright = 0.19f;
float Ki_upright = 0.0f;
float Kd_upright = 28.0f;

float Kp_raise = 0.2f;
float Ki_raise = 0.0f;
float Kd_raise = 44.0f;

float Kp_line_following = 0.05f;
float Ki_line_following = 0.0f;
float Kd_line_following = 0.0f;
//end save

float current_heading = 0.0f;
float prev_heading = 0.0f;
float current_heading_error = 0.0f;
float prev_heading_error = 0.0f;
float imu_heading_bias = 0.0f;

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
float goto_goal_gain = 1.0f;

uint16_t max_throttle = 100; //200
uint16_t max_steering = 50; // 50
uint16_t steering_dead_zone = 40;
uint16_t throttle_dead_zone = 40;

int ITERM_MAX_ERROR = 25;   // Iterm windup constants for PI control //40
int ITERM_MAX = 8000;       // 5000

//save to flash
float max_linear_speed = 1.0f;
float max_linear_accel = 0.25f;
float max_motor_accel = 50000.f;
float max_motor_deccel = 50000.f;
float max_angular_speed = 1.0f;
float max_angular_accel = 0.1f;
float max_throttle_response = 5500.0f; // TODO: Should ultimately be based off the accelerations.  Once these are implemented...

float wheel_diameter_1 = 0.090f; // 90 mm (Right)
float wheel_diameter_2 = 0.090f; // 90 mm (Left)
float wheel_base = .151f; // 151 mm
float wheel_steps_per_m_1 = 11317.6848421; // steps per m (Right)
float wheel_steps_per_m_2 = 11317.6848421; // steps per m (Left)

float angle_offset = -2.0f; // Offset for hardware issues

uint16_t motor_current = 1000; // Stepper motor current in mA
uint16_t steps_per_rev = 3200;  // 1/16 microstepping
uint8_t microsteps = 16;
//end save

// robot pose
float x_pos = 0.0f;
float y_pos = 0.0f;
float theta = 0.0f;

float total_meters = 0.0f;

float pid_error_sum = 0.0f;
float pid_balance_error_sum = 0.0f;

uint32_t stable_time = 0;

bool robot_shutdown = true;
bool robot_stable = false;

bool robot_in_navigation = false;

// Socket for Debug
socket_device_t *sock = NULL;
static QueueHandle_t socket_debug_queue;

// Navigation Waypoint Queue
static QueueHandle_t navigation_waypoint_queue;

// Sensors
adns_3080_device_t *adns_sensor;

// Motor Drivers
amis_30543_device_t *amis_motor_1;
amis_30543_device_t *amis_motor_2;

// PIDS
pid_device_control_t position_pid;
pid_device_control_t heading_pid;
pid_device_control_t line_following_pid;
// Event Groups
EventGroupHandle_t qrobot_event_group;
const int ABORT_BIT = BIT0;

typedef struct {
	char data[DEBUG_STACK_BUFFER_SIZE];
} socket_event_t;

typedef struct {
	float x;
	float y;
	float theta;
	bool is_last;
	QueueHandle_t notify_queue;
} waypoint_t;

typedef struct {
	bool enable; // Enable Flag
	float v; // Linear Velocity
	float w; // Angular Velocity
} controller_output_t;


typedef enum  {
    OBSTACLE_AVOIDANCE_CONTROLLER,
    GOTO_GOAL_CONTROLLER,
    LINE_FOLLOWER_CONTROLLER,
    RC_CONTROLLER,
    CRUISE_CONTROLLER,
    CONTROLLER_TYPE_MAX,
} controller_type_t;

controller_output_t controller_output_map[CONTROLLER_TYPE_MAX];

extern void qrobot_send_debug(char *format, ...);
extern void qrobot_down_and_back_task(void *ignore);
extern void qrobot_navigation_task(void *ignore);

controller_output_t qrobot_get_active_controller_output()
{
	for(int i = 0; i < CONTROLLER_TYPE_MAX; i++)
	{
		if(controller_output_map[i].enable) {
			return controller_output_map[i];
		}
	}
	controller_output_t default_output;
	default_output.v = 0;
	default_output.w = 0;
	return default_output; // default zero output
}

void qrobot_navigation_task(void *ignore)
{
    ESP_LOGD(tag, ">> qrobot_navigation_task");

    uint32_t settle_time = 0;
    uint32_t last_time = 0;

    bool in_navigation = false;
	waypoint_t goal_waypoint;
	while (1) {
		EventBits_t uxBits = xEventGroupGetBits(qrobot_event_group);

		if (in_navigation && (uxBits & ABORT_BIT)) {
			in_navigation = false;
			xQueueReset(navigation_waypoint_queue); // Reset queue
			controller_output_map[GOTO_GOAL_CONTROLLER].enable = false;
			controller_output_map[GOTO_GOAL_CONTROLLER].v = 0;
			controller_output_map[GOTO_GOAL_CONTROLLER].w = 0;
			ESP_LOGE(tag, "NAVIGATION ABORTED!!");
			vTaskDelay(200 / portTICK_PERIOD_MS);
			continue;
		}

		if (navigation_waypoint_queue != NULL) {
			if (xQueueReceive(navigation_waypoint_queue, &(goal_waypoint),
					(TickType_t ) 0)) {
				float theta_d = atan2((goal_waypoint.y - y_pos), (goal_waypoint.x - x_pos));
				ESP_LOGI(tag, "Got new waypoint: X = %f, Y = %f, Theta_d: %f", goal_waypoint.x, goal_waypoint.y, theta_d);


				qrobot_send_debug("Got new waypoint: X = %f, Y = %f, Theta: %f", goal_waypoint.x, goal_waypoint.y, theta_d);
				controller_output_map[GOTO_GOAL_CONTROLLER].enable = true;
				//controller_output_map[GOTO_GOAL_CONTROLLER].v = goto_goal_gain;
				in_navigation = true;
			}
		}

		if (in_navigation) {
			float x = goal_waypoint.x - x_pos;
			float y = goal_waypoint.y - y_pos;
			float target_distance = sqrt((x * x) + (y * y));
			float theta_d = atan2((goal_waypoint.y - y_pos), (goal_waypoint.x - x_pos));
			if (target_distance <= 0.1) {
				settle_time += (millis() - last_time);
				if (settle_time >= 100) {
					qrobot_send_debug("Arrived: %f\n", target_distance);
					if (goal_waypoint.is_last) {
						controller_output_map[GOTO_GOAL_CONTROLLER].enable = false;
						controller_output_map[GOTO_GOAL_CONTROLLER].v = 0.0f;
						controller_output_map[GOTO_GOAL_CONTROLLER].w = 0.0f;
					}
					// Notify calling task we are there
					if(goal_waypoint.notify_queue != NULL) {
						uint8_t send;
						xQueueSendToFront(goal_waypoint.notify_queue, &send, (TickType_t ) 100 / portTICK_PERIOD_MS );
					}
					in_navigation = false;
				}
			} else {
				settle_time = 0;
			}

			position_pid.set_point = 0;
			position_pid.input = target_distance;
			position_pid.output_max = goto_goal_gain;
			position_pid.output_min = -goto_goal_gain;
			controller_output_map[GOTO_GOAL_CONTROLLER].v = -pid_compute(&position_pid);

			heading_pid.set_point = theta_d;
			heading_pid.input = theta;

			controller_output_map[GOTO_GOAL_CONTROLLER].w = -pid_compute_angle(&heading_pid);
		}
    	last_time = millis();
    	vTaskDelay(50 / portTICK_PERIOD_MS);
    }

//	while (1) {
//
//		float x = current_goal_x - x_pos;
//		float y = current_goal_y - y_pos;
//
//		float target_distance = sqrt((x*x)+(y*y));
//
//		if(target_distance <= 0.1) {
//			settle_time += (millis() - last_time);
//			if (settle_time >= 1000) {
//				qrobot_send_debug("Arrived: %f\n", target_distance);
//				navigation_speed_output = 0.0f;
//				break;
//			}
//		}
//		else {
//			settle_time = 0;
//		}
//
//		goto_goal_pid.set_point = current_goal_x;
//		goto_goal_pid.input = x_pos;
//
//		navigation_speed_output = pid_compute(&goto_goal_pid);
//		last_time = millis();
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//	}


    //ESP_LOGI(tag, "Theta %f", current_goal_theta);
//    while (1) {
//
//		float theta_delta = current_goal_theta - theta;
//
//		if (fabs(theta_delta) <= 0.01) {
//			settle_time += (millis() - last_time);
//			if (settle_time >= 1000) {
//				qrobot_send_debug("Arrived: %f\n", theta_delta);
//				navigation_steering_output = 0.0f;
//				break;
//			}
//		} else {
//			settle_time = 0;
//		}
//
//		heading_pid.set_point = current_goal_theta;
//		heading_pid.input = theta;
//
//		navigation_steering_output = pid_compute(&heading_pid);
//		//qrobot_send_debug("Output: %f: %f\n", navigation_steering_output, heading_pid.k_p);
//		last_time = millis();
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//	}
	vTaskDelete(NULL);
}

void qrobot_obstacle_avoidance_task(void *ignore) {
    ESP_LOGD(tag, ">> qrobot_obstacle_avoidance_task");

    // Init Sensors
    sharp_ir_device_t left_sensor;
    left_sensor.num_samples = 100;
    left_sensor.sensor_pin = ADC1_CHANNEL_6;
    left_sensor.type = CM_10_80;

    sharp_ir_device_t right_sensor;
    right_sensor.num_samples = 100;
    right_sensor.sensor_pin = ADC1_CHANNEL_7;
    right_sensor.type = CM_10_80;

    sharp_ir_init(&left_sensor);
    sharp_ir_init(&right_sensor);

   // float volts_l;
    float cm_l;

    //float volts_r;
    float cm_r;

    bool in_avoid = false;
    bool left_obstacle_detected;
    bool right_obstacle_detected;
   // uint32_t settle_time = 0;
    //uint32_t last_time = 0;

    waypoint_t detour_waypoint;
    detour_waypoint.x = 0;
    detour_waypoint.y = 0;
    detour_waypoint.theta = 0;
    detour_waypoint.is_last = true;
    //detour_waypoint.notify_queue = queue;

    while (1) {

        EventBits_t uxBits = xEventGroupGetBits(qrobot_event_group);
        if (uxBits & ABORT_BIT) {
            ESP_LOGE(tag, "AVOIDANCE ABORTED!!");
            controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].enable = false;
            controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].v = 0;
            controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].w = 0;
            break;
        }

        if (!in_avoid) { // Look for obstacle
           // volts_l = sharp_ir_get_distance_volt(&left_sensor);
            cm_l = sharp_ir_get_distance_cm(&left_sensor);

           // volts_r = sharp_ir_get_distance_volt(&right_sensor);
            cm_r = sharp_ir_get_distance_cm(&right_sensor);

            if (cm_l >= 70 || cm_l <= 10.0f) {
                left_obstacle_detected = false;
            } else {
                left_obstacle_detected = true;
            }

            if (cm_r >= 70 || cm_r <= 10.0f) {
                right_obstacle_detected = false;
            } else {
                right_obstacle_detected = true;
            }

            if ((left_obstacle_detected || right_obstacle_detected)
                    && (cm_r < 50.0f || cm_l < 50.0f)) {

                // TODO: Need to do some trig to convert to local coordinates and transform back
                detour_waypoint.x = x_pos + 0.5f; // 45 degrees test
                detour_waypoint.y = y_pos + 0.5f;
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].enable = true;
            } else {
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].enable = false;
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].v = 0;
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].w = 0;
            }
        }
        else {
            float x = detour_waypoint.x - x_pos;
            float y = detour_waypoint.y - y_pos;
            float target_distance = sqrt((x * x) + (y * y));
            float theta_d = atan2((detour_waypoint.y - y_pos), (detour_waypoint.x - x_pos));
            if (target_distance <= 0.1) { // Have we hit the goal?
                qrobot_send_debug("Detour Arrived: %f\n", target_distance);

                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].enable = false;
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].v = 0.0f;
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].w = 0.0f;

                in_avoid = false;

            } else {
                position_pid.set_point = 0;
                position_pid.input = target_distance;
                position_pid.output_max = goto_goal_gain * 0.5f;
                position_pid.output_min = -goto_goal_gain * 0.5f;
                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].v = -pid_compute(&position_pid);

                heading_pid.set_point = theta_d;
                heading_pid.input = theta;

                controller_output_map[OBSTACLE_AVOIDANCE_CONTROLLER].w = -pid_compute_angle(&heading_pid);
            }
        }
       // last_time = millis();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void qrobot_line_follower_task(void *ignore) {

    ESP_LOGD(tag, ">> qrobot_line_follower_task");


    // Create image
    int img_width = 30;
    int img_height = 30;
    image_t *image_frame = NULL;
    image_t *image_frame_masked = NULL;
    image_t *image_blobs = NULL;
    create_image(img_width, img_height, sizeof(uint8_t), &image_frame);
    create_image(img_width, img_height, sizeof(uint8_t), &image_frame_masked);
    create_image(img_width, img_height, sizeof(uint8_t), &image_blobs);

    // Setup masking regions
    mask_region_t image_mask_upper={
                .top=0,
                .bottom=3,
                .left=0,
                .right=30
            };
    mask_region_t image_mask_lower={
                .top=25,
                .bottom=30,
                .left=0,
                .right=30
            };

    uint32_t settle_time = 0;
    uint32_t last_time = 0;

    while (1) {
		vector image_moments;
		vector_init(&image_moments);

    	uint32_t start_read = millis(); // For profiling
        EventBits_t uxBits = xEventGroupGetBits(qrobot_event_group);
        if (uxBits & ABORT_BIT) {
            ESP_LOGE(tag, "LINE_FOLLOWER ABORTED!!");
            controller_output_map[LINE_FOLLOWER_CONTROLLER].enable = false;
            controller_output_map[LINE_FOLLOWER_CONTROLLER].v = 0;
            controller_output_map[LINE_FOLLOWER_CONTROLLER].w = 0;
            break;
        }

        // Get Image Frame
        if(!adns3080_read_frame_burst(adns_sensor, image_frame)) {
        	vTaskDelay(20 / portTICK_PERIOD_MS); // Smaller delay.  Try to read again fairly quickly
        	continue;
        }

        // Mask it to center of view top/bottom
        mask_image(image_frame, image_mask_upper, 0, image_frame_masked);
        mask_image(image_frame_masked, image_mask_lower, 0, image_frame_masked);

        //mask_image(image_frame, image_mask_lower, 0, image_frame_masked);

        // Look for connected images
        uint8_t num_blobs = image_connected_components(image_frame_masked, image_blobs);

        ESP_LOGI(tag, "Blobs Detected: %d", num_blobs);

        // Compute moments/centroids
        calculate_local_moments(image_blobs, num_blobs, &image_moments);
        for (int i = 0; i < VECTOR_TOTAL(image_moments); i++) {
        	image_moment_t *M = VECTOR_GET(image_moments, image_moment_t *, i);

			if (M->m00 > 40) {
				uint32_t cx = (uint32_t) ((float) M->m10 / M->m00);
				uint32_t cy = (uint32_t) ((float) M->m01 / M->m00);

				// Debug tracking point
				image_set_pixel(image_frame_masked, cx, cy, 88);

				// Update PIDS
				line_following_pid.set_point = img_width * 0.5;
				line_following_pid.input = cx;
				line_following_pid.output_max = 0.8f;
				line_following_pid.output_min = -0.8f;
				controller_output_map[LINE_FOLLOWER_CONTROLLER].enable = true;
				controller_output_map[LINE_FOLLOWER_CONTROLLER].v = 0.2f;
				controller_output_map[LINE_FOLLOWER_CONTROLLER].w = -pid_compute(&line_following_pid);
			} else {
				settle_time += (millis() - last_time);
				if (settle_time >= 500) {
					controller_output_map[LINE_FOLLOWER_CONTROLLER].enable = false;
					controller_output_map[LINE_FOLLOWER_CONTROLLER].v = 0.0f;
					controller_output_map[LINE_FOLLOWER_CONTROLLER].w = 0.0f;
				} else {
					controller_output_map[LINE_FOLLOWER_CONTROLLER].v = 0.1f;
				}
			}
        }

        // Now debug it
        print_image(image_frame_masked);

       // ESP_LOGI(tag, "Control signal: %.2f/%d", -controller_output_map[LINE_FOLLOWER_CONTROLLER].w, M.m00);

		ESP_LOGI(tag, "Frame end: %d", millis() - start_read);
		last_time = millis();
		vTaskDelay(1000 / portTICK_PERIOD_MS); // 10 hz
    }
    vTaskDelete(NULL);
}
void qrobot_down_and_back_task(void *ignore)
{
    ESP_LOGD(tag, ">> qrobot_down_and_back_task");

    // Make a notify queue
    QueueHandle_t queue = xQueueCreate(1, sizeof(waypoint_t));

    waypoint_t waypoint_list[3];

    waypoint_list[0].x = (6.25f * .3048); // 6-ish ft...
    waypoint_list[0].y = 0;
    waypoint_list[0].theta = 0;
    waypoint_list[0].is_last = false;
    waypoint_list[0].notify_queue = queue;

    waypoint_list[1].x = (0.5f * .3048);
    waypoint_list[1].y = -0.5f;
    waypoint_list[1].theta = 0;
    waypoint_list[1].is_last = true;
    waypoint_list[1].notify_queue = queue;

    // TODO: Add a turn?
    uint32_t start_time = 0;
    uint8_t waypoint_index = 0;
    uint8_t waypoint_count = 2;
	uint8_t recv;
	bool waypoint_completed = true; // Already at 1st.  Technically.

	// Start obstacle avoid
	TaskHandle_t xHandle = NULL;
	xTaskCreate(&qrobot_obstacle_avoidance_task, "qrobot_obstacle", 2048, NULL,5, &xHandle);

	while(1)
	{
		EventBits_t uxBits = xEventGroupGetBits(qrobot_event_group);
		if( uxBits & ABORT_BIT) {
			ESP_LOGE(tag, "DOWN AND BACK ABORTED!!");
			break;
		}

		if(start_time == 0) { // Not sure why this is necessary but just get zero the first time always...  Is there a way to force something?
			start_time = millis();
			qrobot_send_debug("Started Down and Back: %d", start_time); // And do something else like log the message apparently...  WTF
		}

		// TODO: Gotta reorder this.  Kind of weird reverse logic.
		if(waypoint_completed) {
			waypoint_completed = false;
			// Send waypoint
			xQueueSendToFront(navigation_waypoint_queue, &waypoint_list[waypoint_index], (TickType_t ) 100 / portTICK_PERIOD_MS );

			qrobot_send_debug("Sending Waypoint!");
		}

		// Wait for completion
		if (xQueueReceive(waypoint_list[waypoint_index].notify_queue, &(recv), (TickType_t ) 10 / portTICK_PERIOD_MS)) {
			qrobot_send_debug("Completed Waypoint!");
			waypoint_completed = true;
			waypoint_index++;
		}

		if(waypoint_index >= waypoint_count) {
			uint32_t finish_time = millis();
			qrobot_send_debug("Completion Time: %.2f (s)", (finish_time - start_time) * 0.001f);
			break;
		}
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}
	// Kill avoidance task
	vTaskDelete(xHandle);
	vTaskDelete(NULL);
}
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
    char data[2048];
    char params[10][20]; // 10 Max
    while (1) { // Listen for a new client connection.

        socklen_t client_address_length = sizeof(client_address);
        int client_sock = accept(sock, (struct sockaddr *) &client_address, &client_address_length);
        if (client_sock < 0) {
            ESP_LOGE(tag, "ERROR:  Unable to accept client connection:  accept(): %s", strerror(errno));
            goto END;
        }
        // We now have a new client ...
        ESP_LOGI(tag, "Got new connection from client.");

        int size_used = 0;
        bzero(data, sizeof(data));
        // Loop reading data.
        while (1) {
            ssize_t size_read = recv(client_sock, data + size_used, total - size_used, 0);
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
        ESP_LOGD(tag, "Data read (size: %d) was: %.*s", size_used, size_used, data);

		// Check for commands
		char* token = strtok(data, " ");
		if(token) {
			if(!strncmp(data, "RESET",5)) {
				x_pos = y_pos = theta = 0;
				robot_in_navigation = false;
			}
			else if(!strncmp(data, "GOAL_GAIN",9)) {
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}

				if(counter == 1) {
					send(client_sock, "OK\n", 3, 0);
					goto_goal_gain = atof(params[0]);
				} else {
					send(client_sock, "INVALID\n", 8, 0);
				}
			}
			else if(!strncmp(data, "GOAL",4)) {
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}

				if(counter == 3) {
					waypoint_t new_goal;
					new_goal.x = atof(params[0]);
					new_goal.y = atof(params[1]);
					new_goal.theta = atof(params[2]);
					new_goal.is_last = true;
					xQueueSendToFront(navigation_waypoint_queue, &new_goal, (TickType_t ) 100 / portTICK_PERIOD_MS );
					send(client_sock, "OK\n", 3, 0);
					//current_goal_x = atof(params[0]);
					//current_goal_y = atof(params[1]);
					//current_goal_theta = atof(params[2]);
					//robot_in_navigation = true;

			    	//ESP_LOGI(tag, "Theta_D = %f", theta_d * RADS_TO_DEGS);
					// Start navigation task
				//	xTaskCreate(&qrobot_navigation_task, "qrobot_navigation", 2048, NULL,4, NULL);
				} else {
					send(client_sock, "INVALID\n", 8, 0);
				}
			}
			else if(!strncmp(data, "DANDB",5)) {
				// Start task
				xTaskCreate(&qrobot_down_and_back_task, "qrobot_down_and_back", 2048, NULL,4, NULL);
			}
			else if(!strncmp(data, "LINE_FOLLOW",11)) {
				// Start task
				xTaskCreate(&qrobot_line_follower_task, "qrobot_line_follower", 4096, NULL,1, NULL);
			}
			else if(!strncmp(data, "PERF_PARAMS", 11)) {
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}

				if(counter == 4) {
					send(client_sock, "OK\n", 3, 0);
					max_throttle = (uint16_t)atof(params[0]);
					max_steering = (uint16_t)atof(params[1]);
					max_throttle_response = atof(params[2]);
					max_angular_accel = atof(params[3]);
				} else {
					send(client_sock, "INVALID\n", 8, 0);
                }
            }
            else if(!strncmp(data, "MECH_PARAMS", 11)) {
                token = strtok(NULL, " ");
                int counter = 0;
                while (token) {
                    strcpy(params[counter], token);
                    printf("token: %s\n", params[counter]);
                    token = strtok(NULL, " ");
                    counter++;
                }

                if(counter == 4) {
                    send(client_sock, "OK\n", 3, 0);
                    wheel_diameter_1 = atof(params[0]);
                    wheel_diameter_2 = atof(params[1]);
                    wheel_base = atof(params[2]);
                    angle_offset = atof(params[3]);

                    wheel_steps_per_m_1 = steps_per_rev / (M_PI * wheel_diameter_1); // steps per m (Left)
                    wheel_steps_per_m_2 = steps_per_rev / (M_PI * wheel_diameter_2); // steps per m (Right)

                } else {
                    send(client_sock, "INVALID\n", 8, 0);
                }
            }
            else if(!strncmp(data, "MOTOR_PARAMS", 12)) {
                token = strtok(NULL, " ");
                int counter = 0;
                while (token) {
                    strcpy(params[counter], token);
                    printf("token: %s\n", params[counter]);
                    token = strtok(NULL, " ");
                    counter++;
                }

                if(counter == 3) {
                    send(client_sock, "OK\n", 3, 0);
                    motor_current = (uint16_t)atoi(params[0]);
                    microsteps = (uint8_t)atoi(params[1]);
                    max_motor_accel = max_motor_deccel = atof(params[2]);
                    steps_per_rev = 200 * microsteps;
                    wheel_steps_per_m_1 = steps_per_rev / (M_PI * wheel_diameter_1); // steps per m (Left)
                    wheel_steps_per_m_2 = steps_per_rev / (M_PI * wheel_diameter_2); // steps per m (Right)

                    amis_30543_set_current(amis_motor_1, motor_current); // Set current limit
                    amis_30543_set_step_mode(amis_motor_1, microsteps); // Set microstepping to 1/16

                    amis_30543_set_current(amis_motor_2, motor_current); // Set current limit
                    amis_30543_set_step_mode(amis_motor_2, microsteps); // Set microstepping to 1/16

                } else {
                    send(client_sock, "INVALID\n", 8, 0);
                }
            }
            else if(!strncmp(data, "PID_LINE_FOLLOWER",17)) {
                token = strtok(NULL, " ");
                int counter = 0;
                while (token) {
                    strcpy(params[counter], token);
                    printf("token: %s\n", params[counter]);
                    token = strtok(NULL, " ");
                    counter++;
                }

                if(counter == 3) {
                    send(client_sock, "OK\n", 3, 0);
                    line_following_pid.k_p = atof(params[0]);
                    line_following_pid.k_i = atof(params[1]);
                    line_following_pid.k_d = atof(params[2]);
                } else {
                    send(client_sock, "INVALID\n", 8, 0);
                }
            }
            else if(!strncmp(data, "PID_RAISE",9)) {
                token = strtok(NULL, " ");
                int counter = 0;
                while (token) {
                    strcpy(params[counter], token);
                    printf("token: %s\n", params[counter]);
                    token = strtok(NULL, " ");
                    counter++;
                }

                if(counter == 3) {
                    send(client_sock, "OK\n", 3, 0);
                    Kp_raise = atof(params[0]);
                    Ki_raise = atof(params[1]);
                    Kd_raise = atof(params[2]);
                } else {
                    send(client_sock, "INVALID\n", 8, 0);
                }
            }
            else if(!strncmp(data, "PID_BALANCE",11)) {
                token = strtok(NULL, " ");
                int counter = 0;
                while (token) {
                    strcpy(params[counter], token);
                    printf("token: %s\n", params[counter]);
                    token = strtok(NULL, " ");
                    counter++;
                }

                if(counter == 3) {
                    send(client_sock, "OK\n", 3, 0);
                    Kp_balance = atof(params[0]);
                    Ki_balance = atof(params[1]);
                    Kd_balance = atof(params[2]);
                } else {
                    send(client_sock, "INVALID\n", 8, 0);
                }
            }
            else if(!strncmp(data, "PID_SPEED",9)) {
                token = strtok(NULL, " ");
                int counter = 0;
                while (token) {
                    strcpy(params[counter], token);
                    printf("token: %s\n", params[counter]);
                    token = strtok(NULL, " ");
                    counter++;
                }

                if(counter == 3) {
                    send(client_sock, "OK\n", 3, 0);
                    Kp_speed = atof(params[0]);
                    Ki_speed = atof(params[1]);
                    Kd_speed = atof(params[2]);
                } else {
                    send(client_sock, "INVALID\n", 8, 0);
                }
            }
			else if(!strncmp(data, "PID_POSITION",12)) {
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}

				if(counter == 3) {
					send(client_sock, "OK\n", 3, 0);
					position_pid.k_p = atof(params[0]);
					position_pid.k_i = atof(params[1]);
					position_pid.k_d = atof(params[2]);
				} else {
					send(client_sock, "INVALID\n", 8, 0);
				}
			}
			else if(!strncmp(data, "PID_HEADING",11)) {
				printf("HEADING PID TOKEN:\n");
				token = strtok(NULL, " ");
				int counter = 0;
				while (token) {
					strcpy(params[counter], token);
					printf("token: %s\n", params[counter]);
					token = strtok(NULL, " ");
					counter++;
				}
				if(counter == 3) {
					send(client_sock, "OK\n", 3, 0);
					heading_pid.k_p = atof(params[0]);
					heading_pid.k_i = atof(params[1]);
					heading_pid.k_d = atof(params[2]);
					printf("HEADING PID: %f\n", heading_pid.k_p);
				} else {
					send(client_sock, "INVALID\n", 8, 0);
				}
			}
			else if(!strncmp(data, "STOP",4)) {
				EventBits_t uxBits = xEventGroupGetBits(qrobot_event_group);
				if ((uxBits & ABORT_BIT)) {
					printf("GOT RESUME!\n");
					xEventGroupClearBits(qrobot_event_group, ABORT_BIT);
				}
				else {
					printf("GOT ABORT!\n");
					xEventGroupSetBits(qrobot_event_group, ABORT_BIT);
				}
				send(client_sock, "OK\n", 3, 0);
			}
		}
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

	if(abs(pid_balance_error_sum) > 20000)
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

float qrobot_limit_acceleration(float v, float v0, float max_accel, float max_deccel, float dt)
{
	const float dv_min = -max_deccel * dt;
	const float dv_max = max_accel * dt;

	const float dv = constrain(v - v0, dv_min, dv_max);

	return v0 + dv;
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
	float dt_s;

	float robot_speed = 0.0f;
	float robot_speed_old = 0.0f;
	float estimated_speed_filtered = 0.0f;
	float throttle = 0.0f;
	float throttle_prev = 0.0f;
	float steering = 0.0f;

	float angular_velocity = 0.0f;
	float estimated_speed = 0.0f;

//	while (1) {
//		throttle = qrobot_get_active_controller_output().v * max_throttle;
//		steering = -qrobot_get_active_controller_output().w * max_steering;
//
//		motor_1_speed = throttle + steering;
//		motor_2_speed = throttle - steering;
//
//		stepper_control_set_speed(STEPPER_MOTOR_1, motor_1_speed * 20);
//		stepper_control_set_speed(STEPPER_MOTOR_2, motor_2_speed * 20);
//
//		//ESP_LOGI(tag, "Set Speed: %d", (int)(motor_1_speed));
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//	}

	//vTaskDelay(1000 / portTICK_PERIOD_MS);

	while (1) {

		if(robot_shutdown) {
			last_update = millis();
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}
		else if(!robot_stable && controller_output_map[RC_CONTROLLER].enable) { // If throttle input and not stable don't allow to stand
			last_update = millis();
			robot_stable = false;
			stable_time = 0;
			control_output = 0;
			motor_1_speed = motor_2_speed = motor_1_speed_old = motor_2_speed_old = 0;
			pid_error_sum = 0;
			pid_balance_error_sum = 0;
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}

		now = millis();
		dt = (now - last_update); // in ms
		dt_s = dt * 0.001f; // in s
		prev_tilt = current_tilt;
		current_tilt = mpu_9250_get_phi() + angle_offset; //Degrees, Roll axis

		// We calculate the estimated robot speed:
		// Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
		robot_speed_old = robot_speed;
		robot_speed = (motor_1_speed + motor_2_speed) / 2; // Positive: forward

		angular_velocity = (current_tilt - prev_tilt) * 90.0; // 90 is an empirical extracted factor to adjust for real units
		estimated_speed = -robot_speed_old - angular_velocity; // We use robot_speed(t-1) or (t-2) to compensate the delay
		estimated_speed_filtered = estimated_speed_filtered * 0.95 + estimated_speed * 0.05; // low pass filter on estimated speed

		throttle_prev = throttle;
		throttle = qrobot_get_active_controller_output().v * max_throttle;
		steering = qrobot_get_active_controller_output().w * max_steering;

		// Limit Throttle Response
		throttle = qrobot_limit_acceleration(throttle, throttle_prev, max_throttle_response, max_throttle_response, dt_s);


		target_tilt = qrobot_speed_PID_control(dt_s, estimated_speed_filtered, -throttle);


		//if(loop_count++ >= 100)
		 //   	{
		////ESP_LOGI(tag, "Speed: %f and Throttle: %f and Tilt: %f/%f\n",estimated_speed_filtered, throttle, target_tilt, dt_s * (target_tilt - current_tilt));
		//loop_count = 0;
		//    	}
		//float tilt_change = (target_tilt - current_tilt) * dt_s;
		//tilt_change = constrain(tilt_change, -1, 1); //  deg/s

		//target_tilt  = current_tilt + (tilt_change / dt_s);

		target_tilt = constrain(target_tilt, -max_target_tilt, max_target_tilt); // limited output

		control_output += qrobot_stability_PID_control(dt, current_tilt, target_tilt);
		control_output = constrain(control_output, -max_control_output, max_control_output);

		motor_1_speed_old = motor_1_speed;
		motor_2_speed_old = motor_2_speed;

		motor_1_speed = control_output + steering;
		motor_2_speed = control_output - steering;

		motor_1_speed = constrain(motor_1_speed, -max_control_output, max_control_output);
		motor_2_speed = constrain(motor_2_speed, -max_control_output, max_control_output);

		motor_1_speed = qrobot_limit_acceleration(motor_1_speed, motor_1_speed_old, max_motor_accel, max_motor_deccel, dt_s);
		motor_2_speed = qrobot_limit_acceleration(motor_2_speed, motor_2_speed_old, max_motor_accel, max_motor_deccel, dt_s);

		if ((current_tilt < 76) && (current_tilt > -76)) // Is robot ready (upright?)
		{
			// Make sure motors are enabled, TODO: Better way for this to just be flagged when
			if (!amis_motor_1->enable) {
				amis_30543_enable_driver(amis_motor_1, true);
			}
			if (!amis_motor_2->enable) {
				amis_30543_enable_driver(amis_motor_2, true);
			}

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
				Kp_balance = Kp_upright;         // CONTROL GAINS FOR UPRIGHT
				Kd_balance = Kd_upright;
			} else { // We are in the raise up procedure => we use special control parameters
				Kp_balance = Kp_raise;         // CONTROL GAINS FOR RAISE UP
				Kd_balance = Kd_raise;
			}
		}
		else {
			stepper_control_set_speed(STEPPER_MOTOR_1, 0);
			stepper_control_set_speed(STEPPER_MOTOR_2, 0);
			control_output = 0;
			pid_error_sum = 0;
			pid_balance_error_sum = 0;
			stable_time = 0;
			throttle = throttle_prev = 0.0f;
			robot_stable = false;

			Kp_balance = Kp_upright;         // CONTROL GAINS FOR RAISE UP
			Kd_balance = Kd_upright;
			// Kill motor drivers.  Try to save some poweer
			// Make sure motors are disabled, TODO: Better way for this to just be flagged when
			if (amis_motor_1->enable) {
				amis_30543_enable_driver(amis_motor_1, false);
			}
			if (amis_motor_2->enable) {
				amis_30543_enable_driver(amis_motor_2, false);
			}
		}
		last_update = millis();
		uint32_t loop_time = millis() - now;
		vTaskDelay(max(1, (5 - loop_time)) / portTICK_PERIOD_MS);
		//vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS); // 5 ms loop time
	}
    vTaskDelete(NULL);
}

void qrobot_odometry_task(void *ignore)
{
	// Right Wheel = Motor 2
	// Left Wheel = Motor 1

    ESP_LOGD(tag, ">> qrobot_odometry_task");
    int32_t wheel_1_step_prev = 0;
    int32_t wheel_2_step_prev = 0;
    int32_t wheel_1_step = 0;
    int32_t wheel_2_step = 0;
    float wheel_1_distance = 0;
    float wheel_2_distance = 0;
    float delta_m = 0;
    int loop_count = 0;

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
		//theta = mpu_9250_get_yaw() - imu_heading_bias;
		theta += (wheel_2_distance - wheel_1_distance) / wheel_base; // CCW Positive

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

		//ESP_LOGI(tag, "Pitch: %f, Roll: %f, Yaw: %f", mpu_9250_get_pitch(), mpu_9250_get_roll(), mpu_9250_get_yaw());

    	if(loop_count >= 100)
    	{
			//ESP_LOGI(tag, "Pose: X: %f, Y: %f, Theta: %f/%f", x_pos, y_pos, theta, mpu_9250_get_yaw());
    		//qrobot_send_debug("Pose: X: %f, Y: %f, Theta: %f/%f\n",  x_pos, y_pos, theta, mpu_9250_get_yaw());
			loop_count = 0;
    	}
    	loop_count++;
    	vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void qrobot_rc_input_handler(pulse_event_t event)
{
    // Aileron Left = 2000, Right = 1000
    // Throttle Down = 1000, Up = 2000

	static bool throttle_enable = false;
	static bool steering_enable = false;
	switch (event.channel) {
		case THROTTLE:
			if (abs(event.pulse_width - 1500) < throttle_dead_zone) { // Handle Dead Zones...
				controller_output_map[RC_CONTROLLER].v = 0;
				throttle_enable = false;
			} else if (event.pulse_width > 1500) {
				controller_output_map[RC_CONTROLLER].v = map_f(event.pulse_width, 1500 + throttle_dead_zone, 1900, 0.0f, 1.0f);
				throttle_enable = true;
			} else {
				controller_output_map[RC_CONTROLLER].v = map_f(event.pulse_width, 1100, 1500 - throttle_dead_zone, -1.0f, 0.0f);
				throttle_enable = true;
			}
			break;
		case AILERON:
			if (abs(event.pulse_width - 1500) < steering_dead_zone) { // Handle Dead Zones...
				controller_output_map[RC_CONTROLLER].w = 0;
				steering_enable = false;
			} else if (event.pulse_width > 1500) {
				controller_output_map[RC_CONTROLLER].w = map_f(event.pulse_width, 1900, 1500 + steering_dead_zone, -1.0f, 0.0f);
				steering_enable = true;
			}
			else {
				controller_output_map[RC_CONTROLLER].w = map_f(event.pulse_width, 1500 - steering_dead_zone, 1100, 0.0f, 1.0f);
				steering_enable = true;
			}
			break;
		default:
			break;
    }
	controller_output_map[RC_CONTROLLER].enable = throttle_enable | steering_enable;
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
    ar6115e_init(qrobot_rc_input_handler);
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

    // Left
    // 10Mhz, Mode 0
    ESP_LOGI(tag, "Adding AMIS-30543 (1) to SPI Bus...\n");
    ESP_ERROR_CHECK(spi_add_device(GPIO_NUM_5, 1000000, 0, &amis_motor_1->spi_device)); // TODO: Pass In Values
    ESP_LOGI(tag, "Successfully added AMIS-30543 (1).\n");

    delay_ms(100);

    ESP_LOGI(tag, "Initializing AMIS-30543 (1)...\n");
    amis_30543_init(amis_motor_1); // Init
    amis_30543_reset(amis_motor_1); // Reset
    amis_30543_set_current(amis_motor_1, motor_current); // Set current limit
    amis_30543_set_step_mode(amis_motor_1, microsteps); // Set microstepping to 1/16
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
    amis_30543_set_current(amis_motor_2, motor_current); // Set current limit to 1000ma
    amis_30543_set_step_mode(amis_motor_2, microsteps); // Set microstepping to 1/16
    //amis_30543_pwm_frequency_double(amis_right, true); // 45.6 khz
    amis_30543_enable_driver(amis_motor_2, true); // Enable motor output

    // Verify Settings
    verified = amis_30543_verify(amis_motor_2);
    ESP_LOGI(tag, "AMIS-30543 Initialized (2) %d.\n", verified);
}

void qrobot_init_stepper_motors()
{
	// Motor 1 = Left, Motor 2 = Right
    stepper_motor_device_config_t motor_1_cfg = { .step_pin = GPIO_NUM_17,
            .dir_pin = GPIO_NUM_16, .step_hold_delay = 10, .dir_reverse = false };

    stepper_motor_device_config_t motor_2_cfg = { .step_pin = GPIO_NUM_15,
            .dir_pin = GPIO_NUM_2, .step_hold_delay = 10, .dir_reverse = true };

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

//	//Init sensors
//	qrobot_init_mpu9250(); // Init IMU
//
//	uint32_t gyro_init_begin = millis();
//	delay_ms(500);
//	ESP_LOGI(tag, "Begin gyro calibration: Keep robot still for 10 seconds...\n");
//	delay_ms(500);
//
//	// Init ADNS
	qrobot_init_adns(); // Init ADNS
//
//	//Init motors
//	qrobot_init_stepper_motors(); // Init Steppers
//
//	// Init Drivers
//	qrobot_init_amis30543_drivers(); // Init motor drivers
//
//	// Init Control
//	qrobot_init_ar6115e(); // RC input
//
//	// Gyro Calibration delay
//	delay_ms(max(1, 10000 - (millis() - gyro_init_begin)));
//
//	// Pulse motors for ready indication
//	for (uint8_t k = 0; k < 5; k++) {
//		stepper_control_set_speed(STEPPER_MOTOR_1, 5 * 46);
//		stepper_control_set_speed(STEPPER_MOTOR_2, 5 * 46);
//		//BROBOT.moveServo1(SERVO_AUX_NEUTRO + 100);
//		delay_ms(200);
//		stepper_control_set_speed(STEPPER_MOTOR_1, -5 * 46);
//		stepper_control_set_speed(STEPPER_MOTOR_2, -5 * 46);
//
//		//BROBOT.moveServo1(SERVO_AUX_NEUTRO - 100);
//		delay_ms(200);
//	}
//	stepper_control_set_speed(STEPPER_MOTOR_1, 0);
//	stepper_control_set_speed(STEPPER_MOTOR_2, 0);
//
//	//Init sockets
//	#if defined(CONFIG_QROBOT_USE_CONTROL_SERVICE)
//		xTaskCreate(&qrobot_control_debug_service, "control_debug_service", 8192, NULL, 3, NULL);
//	#endif
//
//	#if defined(CONFIG_QROBOT_USE_DEBUG_SERVICE)
//		sock = (socket_device_t *) malloc(sizeof(socket_device_t));
//		sock->port = CONFIG_QROBOT_DEBUG_PORT;
//		socket_server_init(sock);
//		socket_server_start(sock);
//		socket_debug_queue = xQueueCreate(25, sizeof(socket_event_t));
//		xTaskCreate(&qrobot_socket_debug_task, "qrobot_debug_socket", 2048, NULL, 3, NULL);
//	#endif
//	delay_ms(500);
//
	// Setup event groups
	qrobot_event_group = xEventGroupCreate();
	if (qrobot_event_group == NULL) {
		ESP_LOGE(tag, "Failed to create contorller event groups.\n");
	}
//
//	// Setup queues
//	navigation_waypoint_queue = xQueueCreate(1, sizeof(waypoint_t));
//
//	// Setup Robot Parameters
//	steps_per_rev = microsteps * 200;
//	wheel_steps_per_m_1 = steps_per_rev / (M_PI * wheel_diameter_1); // steps per m (Left)
//	wheel_steps_per_m_2 = steps_per_rev / (M_PI * wheel_diameter_2); // steps per m (Right)
//
//	// Reset Heading biases...
//	//imu_heading_bias = mpu_9250_get_yaw(); // TODO: Need to get full 9 Dof sensor fusion working.  Too much drift
//
//	// Reset Odometers...
//	stepper_control_reset_steps(STEPPER_MOTOR_1);
//	stepper_control_reset_steps(STEPPER_MOTOR_2);
//
//	// Update PIDs
//	memset(&position_pid, 0, sizeof(pid_device_control_t));
//
//	position_pid.k_p = Kp_position;
//	position_pid.k_i = Ki_position;
//	position_pid.k_d = Kd_position;
//	position_pid.integral_windup_max = 0.01f;
//	position_pid.output_min = -1.0f;
//	position_pid.output_max = 1.0f;
//	position_pid.clamp_output = true;
//
//	memset(&heading_pid, 0, sizeof(pid_device_control_t));
//
//	heading_pid.k_p = Kp_heading;
//	heading_pid.k_i = Ki_heading;
//	heading_pid.k_d = Kd_heading;
//	heading_pid.integral_windup_max = 0.01f;
//	heading_pid.output_min = -1.0f;
//	heading_pid.output_max = 1.0f;
//	heading_pid.clamp_output = true;

	memset(&line_following_pid, 0, sizeof(pid_device_control_t));

	line_following_pid.k_p = Kp_line_following;
	line_following_pid.k_i = Ki_line_following;
	line_following_pid.k_d = Kd_line_following;
	line_following_pid.integral_windup_max = 0.01f;
	line_following_pid.output_min = -1.0f;
	line_following_pid.output_max = 1.0f;
	line_following_pid.clamp_output = true;

	robot_shutdown = true;
	robot_stable = false;
	robot_in_navigation = false;
}

void qrobot_start()
{
   robot_shutdown = false;
   robot_stable = false;

   // Start main controller task pin it with the steppers
  // xTaskCreatePinnedToCore(&qrobot_controller_task, "qrobot_controller", 4096, NULL, 6, NULL, 1);

   // Start odometry task
  // xTaskCreate(&qrobot_odometry_task, "qrobot_odometry", 2048, NULL, 5, NULL);

   // Start navigation task
 //  xTaskCreate(&qrobot_navigation_task, "qrobot_navigation", 2048, NULL,4, NULL);
   xTaskCreate(&qrobot_line_follower_task, "qrobot_line_follower", 4096, NULL,4, NULL);

}

void qrobot_stop()
{
	robot_stable = false;
	robot_shutdown = true;
}



