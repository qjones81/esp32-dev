
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

// save/load to flash
float Kp_position = 1.0f;
float Ki_position = 0.0f;
float Kd_position = 0.0f;

float Kp_heading = 1.0f;
float Ki_heading = 0.0f;
float Kd_heading = 0.0f;

float Kp_speed = 1.0f;
float Ki_speed = 0.0f;
float Kd_speed = 0.0f;

float Kp_balance = 1.0f;
float Ki_balance = 0.0f;
float Kd_balance = 0.0f;

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

float current_tilt = 0.0f;
float prev_tilt = 0.0f;
float current_tilt_target = 0.0f;
float current_tilt_error = 0.0f;
float prev_tilt_error = 0.0f;

//save to flash
float max_linear_speed = 1.0f;
float max_linear_accel = 0.25f;

float max_angular_speed = 1.0f;
float max_angular_accel = 0.1f;

float wheel_radius_left = 4.0f;
float wheel_radius_right = 4.0f;
float wheel_base = 5.0f;
//end save


float steering_input = 0.0f;

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
            current_speed_target = map(event.pulse_width, 1100, 1900, -100, 100);
            break;
        case AILERON:
            steering_input = map(event.pulse_width, 1900, 1100, -5000, 5000);
            break;
        default:
            break;

    }

   // ESP_LOGI(tag, "Steering: %d | Throttle: %d", (int)steering_input, (int)current_speed_target);
    stepper_control_set_speed(STEPPER_MOTOR_1, min(10000, 10000 * (current_speed_target / 100.0) - steering_input));
    stepper_control_set_speed(STEPPER_MOTOR_2, min(10000, 10000 * (current_speed_target / 100.0) + steering_input));

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
    amis_30543_set_current(amis_motor_2, 1000); // Set current limit to 1000ma
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

    //POST
   // stepper_control_set_speed(STEPPER_MOTOR_1, 5000);
   // stepper_control_set_speed(STEPPER_MOTOR_2, 2000);
}
void qrobot_start()
{
   // xTaskCreate(&task_qrobot_read, "qrobot_task", 2048, NULL, 3, NULL);


}



