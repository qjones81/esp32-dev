
/*
 * adns_3080.c
 *
 *  Created on: Feb 28, 2017
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
#include "freertos/task.h"
#include "driver/gpio.h"
#include "spi/spi.h"
#include "esp_log.h"

#include "utils/utils.h"
#include "image_processing/image_processing.h"
#include "sockets/socket_server.h"
#include "adns3080.h"

static const char *tag = "adns-3080";


#define MAX_SENSORS 3 // Max number of optical sensors allowed to be controlled.
#define BURST_BUFFER_SIZE 1024 // Only have to technically read 900 bytes, but if data not ready it could go over.  Could be smarter by doing first read.  Seeing if we found pixel where needed and grab more data if necessary
#define PIXEL_THRESHOLD 50 // TODO: Send as user param from mobile app/socket
adns_3080_device_t *adns_devices[MAX_SENSORS] = { NULL };

// Socket for Debug
socket_device_t *frame_sock = NULL;
socket_device_t *optical_flow_sock = NULL;

// Frame Buffer
uint16_t frame_buffer_size = ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y;

void read_register_data(uint8_t address, uint8_t *rx_data, uint8_t length, adns_3080_device_t *device)
{
    // TODO: Setup callback for pre-post transmission for CS callback instead.  Might help with clock line sync issues
    uint8_t tx_data = (ADNS3080_SPI_READ | address); // Register to read
    //uint8_t rx_data; // Receive Byte
    //rx_data = 0; // Register to read

    gpio_set_level(device->cs_pin, 0); // Toggle our own CS in software so we can support the mid transaction delays
    spi_transfer(&tx_data, 0, 1, device->spi_device);   // Transfer out read reg
    delay_us(75); // Must delay atleast 50us, or 75us for motion reads.  Use workcase for now
    spi_transfer(&tx_data, rx_data, length, device->spi_device); // Transfer out read reg
    gpio_set_level(device->cs_pin, 1); // Toggle our own CS in software so we can support the mid transaction delays

  //  ESP_LOGI(tag, "Received: 0x%x.\n", rx_data);
}

uint8_t read_register(uint8_t address, adns_3080_device_t *device)
{
    // TODO: Setup callback for pre-post transmission for CS callback instead.  Might help with clock line sync issues
    uint8_t tx_data = (ADNS3080_SPI_READ | address); // Register to read
    uint8_t rx_data[2] = {0}; // Receive Byte

    spi_transfer_fast((ADNS3080_SPI_READ | address), &tx_data, rx_data, 1, 2,device->spi_device);
    return rx_data[1];
}

void write_register(uint8_t address, uint8_t data, adns_3080_device_t *device)
{

	// TODO: May need two stage write here
	uint8_t tx_data[2];
	tx_data[0] = ADNS3080_SPI_WRITE | address;
	tx_data[1] = data;

	spi_transfer_fast((ADNS3080_SPI_WRITE | address), &tx_data, 0, 2, 0, device->spi_device);
}

void adns3080_read_sensor(adns_3080_device_t *p)
{
    // Read sensor
    uint8_t buf[7];
    read_register_data(ADNS3080_MOTION_BURST, buf, 7, adns_devices[0]);
    p->motion_buf.motion = buf[0];
    // ESP_LOGI(tag, "ADNS motion resolution: %d", motion & 0x01); // Resolution

    if (p->motion_buf.motion & 0x10) { // Check if we've had an overflow
        ESP_LOGD(tag, "ADNS-3080 overflow\n");
    } else if (p->motion_buf.motion & 0x80) {
        p->motion_buf.dx = buf[1];
        p->motion_buf.dy = buf[2];
        p->motion_buf.squal = buf[3];
        p->motion_buf.shutter = (buf[4] << 8) | buf[5];
        p->motion_buf.max_pixel = buf[6];

        // Print values
        ESP_LOGI(tag, "x=%d,y=%d\tSQUAL:%d", p->motion_buf.dx,
                p->motion_buf.dy, p->motion_buf.squal);
    }
}
void adns3080_read_frame(adns_3080_device_t *p)
{
//	uint32_t start_read = millis();
// // ESP_LOGI(tag, "Frame read: %d",start_read);
//    write_register(ADNS3080_FRAME_CAPTURE, 0x83, p);
//    delay_us(1510);
//
//   //read_register_data(ADNS3080_PIXEL_BURST, frame, frame_buffer_size, adns_devices[0]);
//    bool is_first_pixel = true;
//    for (uint8_t y = 0; y < ADNS3080_PIXELS_Y; y++) {
//        for (uint8_t x = 0; x < ADNS3080_PIXELS_X; x++) {
//            uint8_t reg_value = read_register(ADNS3080_FRAME_CAPTURE, p);
//            if (is_first_pixel && !(reg_value & 0x40)) {
//                ESP_LOGE(tag, "ERROR: Failed to find first pixel.  Resetting device...");
//                adns_3080_reset(p);
//                return;
//            }
//            is_first_pixel = false;
//            uint8_t pixel = (reg_value & 0x3f) << 2; // Lower 6 bits contain data.  Clear upper 2 bits and Convert to 0-255 standard grayscale
//
//            // Threshold and Binarize Pixel and Reorder image
//            pixel = pixel < 50 ? 1 : 0;
//            frame[ADNS3080_PIXELS_X * (ADNS3080_PIXELS_X - x) + (ADNS3080_PIXELS_Y - y)] = pixel;
//        }
//    }
//
//    printf("\n--------------------------------------------------\n");
//    for (uint8_t y = 0; y < ADNS3080_PIXELS_Y; y++) {
//        for (uint8_t x = 0; x < ADNS3080_PIXELS_X; x++) {
//            printf("%d", frame[ADNS3080_PIXELS_Y * y + x]);
//            if (x != ADNS3080_PIXELS_X - 1)
//                printf(",");
//        }
//        printf("\n");
//    }
//    printf("--------------------------------------------------\n\n");
//   // ESP_LOGI(tag, "Frame end: %d", millis() - start_read);
}
bool adns3080_read_frame_burst(adns_3080_device_t *p, image_t *frame_out)
{
	uint32_t start_read = millis(); // For profiling
   // ESP_LOGI(tag, "Frame read: %d",start_read);

    gpio_set_level(p->cs_pin, 0); // Software CS select
    spi_nodma_device_select(p->spi_device, 1);
    write_register(ADNS3080_FRAME_CAPTURE, 0x83, p);  // Start Frame Capture
    uint8_t tx_data = ADNS3080_PIXEL_BURST;
    spi_transfer_fast((ADNS3080_SPI_READ | ADNS3080_PIXEL_BURST), &tx_data, 0, 1, 0,p->spi_device);

    delay_us(1510); // 10us + 3 frame periods.  Being conservative here by using worst case frame period.

    // TODO: Make this only get extra bits if information not in first burst
	uint8_t frame_in[BURST_BUFFER_SIZE]; // Only have to technically read 900 bytes, but if data not ready it could go over.  Could be smarter by doing first read.  Seeing if we found pixel where needed and grab more data if necessary
	uint16_t offset = 0;
	bool found_start = false;
	memset(&frame_in, 0, BURST_BUFFER_SIZE * sizeof(uint8_t)); // Zero out just in case.  Probably could go away.

	spi_transfer_fast(0, 0, frame_in, 0, BURST_BUFFER_SIZE, p->spi_device);

	// Find first pixel offset
	for (int i = 0; i < BURST_BUFFER_SIZE; i++) {
		if (frame_in[i] & 0x40) { // Got first pixel
			//ESP_LOGI(tag, "First pixel: %d/%d", i, (row_in[i] & 0x3f) << 2);
			offset = i;
			found_start = true;
			break;
		}
	}

	// All done.  Deselect chip and process frame
	spi_nodma_device_deselect(p->spi_device);
	gpio_set_level(p->cs_pin, 1);

	if(!found_start) // Didn't find first pixel, return false and hope for better next time
		return false;

	int i = 0;
	for (uint8_t y = 0; y < ADNS3080_PIXELS_Y; y++) {
		for (uint8_t x = 0; x < ADNS3080_PIXELS_X; x++) {
			uint8_t pixel_out = (frame_in[offset++] & 0x3f) << 2; // Lower 6 bits contain data.  Clear upper 2 bits and Convert to 0-255 standard grayscale
			pixel_out = pixel_out < PIXEL_THRESHOLD ? 1 : 0; // Threshold and Binarize Pixel and Reorder image
			frame_out->data[ADNS3080_PIXELS_X * (ADNS3080_PIXELS_X - (x + 1)) + (ADNS3080_PIXELS_Y - y - 1)] = pixel_out; // Copy into frame buffer
		}
	}

	//ESP_LOGI(tag, "Frame end ADNS: %d", millis() - start_read);
	return true;

}

// Read Task
void task_adns3080_reader_task(void *ignore)
{
    ESP_LOGD(tag, ">>task_adns3080_reader_task");

    // TODO: Pass sensor id, replace [0] blah blah
    while (1) {

#if defined(CONFIG_ADNS3080_ENABLE_FRAME_CAPTURE)
       // adns3080_read_frame_burst(adns_devices[0], frame);
       // vTaskDelay(CONFIG_ADNS3080_FRAME_CAPTURE_UPDATE_PERIOD/portTICK_PERIOD_MS);
#else
        adns3080_read_sensor(adns_devices[0]);
        adns_devices[0]->x += adns_devices[0]->motion_buf.dx;
        adns_devices[0]->y += adns_devices[0]->motion_buf.dy;
        vTaskDelay(CONFIG_ADNS3080_OPTICAL_FLOW_UPDATE_PERIOD/portTICK_PERIOD_MS);
#endif

    }

    vTaskDelete(NULL);
}


#if defined(CONFIG_ADNS3080_USE_FRAME_DEBUG_SERVICE)
void task_adns3080_socket_frame_debug(void *ignore)
{
    ESP_LOGD(tag, ">> task_adns3080_socket_frame_debug");
    while (1) {
        frame_debug_packet_t frame_packet;
        frame_packet.header.packet_type = 0;
        frame_packet.header.packet_length = 900;
        frame_packet.header.frame_width = 30;
        frame_packet.header.frame_height = 30;

        // TODO: Need to make this work for refactored tasks
        //memcpy (&frame_packet.frame_data, &frame, frame_buffer_size);
        socket_server_send_data(frame_sock, (uint8_t *)&frame_packet, sizeof(frame_debug_packet_t));

       // ESP_LOGD(tag, "Sent size: %d", sizeof(frame_debug_packet_t));
       // ESP_LOGD(tag, "Head Sent size: %d", sizeof(frame_debug_header_t));

        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
#endif

#if defined(CONFIG_ADNS3080_USE_OPTICAL_FLOW_DEBUG_SERVICE)
void task_adns3080_socket_optical_flow_debug(void *ignore)
{

    ESP_LOGD(tag, ">> task_adns3080_socket_sensor_debug");
    while (1) {
        char str[80];
        // TODO: Mutex Here or does volatile work here?
        sprintf(str, "SENSOR DATA\n");

        //ESP_LOGI(tag, "%s",str);
        socket_server_send_data(optical_flow_sock, (uint8_t *)str, strlen(str) + 1);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
#endif

// ADNS 3080 Functions
esp_err_t adns_3080_init(adns_3080_device_t *device)
{
    // Setup device pins
    gpio_pad_select_gpio(device->reset_pin);
    gpio_set_direction(device->reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(device->reset_pin, 0);

    gpio_pad_select_gpio(device->cs_pin);
    gpio_set_direction(device->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(device->cs_pin, 1);

    // Update Positions
    device->x = 0;
    device->y = 0;

    gpio_set_level(device->cs_pin, 0); // Software CS select
    // Hack to get clock going properly
    spi_nodma_device_select(device->spi_device, 1);
    read_register(ADNS3080_PRODUCT_ID, device);
    gpio_set_level(device->cs_pin, 1); // Software CS deselect

    delay_ms(10);

    // Reset device
    adns_3080_reset(device);

    gpio_set_level(device->cs_pin, 0); // Software CS select

    uint8_t id = read_register(ADNS3080_PRODUCT_ID, device);
    if (id == ADNS3080_PRODUCT_ID_VALUE) {
        ESP_LOGI(tag, "ADNS-3080 found. id = 0x%x\n", id);
    } else {
        ESP_LOGE(tag, "Could not find ADNS-3080. id: 0x%x\n", id);
        spi_nodma_device_deselect(device->spi_device);
        gpio_set_level(device->cs_pin, 1);
        return ESP_FAIL;
    }

    uint8_t config = read_register(ADNS3080_CONFIGURATION_BITS, device);
    ESP_LOGI(tag, "ADNS-3080 configuration: 0x%x\n", config);

    // 1600 counts per inch
    if (CONFIG_ADNS3080_DEFAULT_COUNTS_PER_INCH == 1600)
        write_register(ADNS3080_CONFIGURATION_BITS, (config | 0b00010000),
                device);
    else
        // Low Res 400
        write_register(ADNS3080_CONFIGURATION_BITS, (config & ~0b00010000),
                device);

    config = read_register(ADNS3080_CONFIGURATION_BITS, device);
    ESP_LOGI(tag, "ADNS-3080 configuration saved: 0x%x\n", config);

    spi_nodma_device_deselect(device->spi_device);
    gpio_set_level(device->cs_pin, 1);

    // Add it
    adns_devices[0] = device;

#if defined(CONFIG_ADNS3080_USE_FRAME_DEBUG_SERVICE)
        frame_sock = (socket_device_t *) malloc(sizeof(socket_device_t));
        frame_sock->port = CONFIG_ADNS3080_FRAME_DEBUG_PORT;
        socket_server_init(frame_sock);
        socket_server_start(frame_sock);

        xTaskCreate(&task_adns3080_socket_frame_debug, "adns3080_socket_frame_task", 4096, NULL, 7, NULL);
#endif

#if defined(CONFIG_ADNS3080_USE_OPTICAL_FLOW_DEBUG_SERVICE)
        optical_flow_sock = (socket_device_t *) malloc(sizeof(socket_device_t));
        optical_flow_sock->port = CONFIG_ADNS3080_OPTICAL_FLOW_DEBUG_PORT;
        socket_server_init(optical_flow_sock);
        socket_server_start(optical_flow_sock);

        xTaskCreate(&task_adns3080_socket_optical_flow_debug, "adns3080_socket_optical_flow_task", 2048, NULL, 7, NULL);
#endif

    return ESP_OK;
}

void adns_3080_reset(adns_3080_device_t *device)
{
    gpio_set_level(device->reset_pin, 1);
    delay_us(10);
    gpio_set_level(device->reset_pin, 0);
    delay_us(500); // Give device time to wake up
}

