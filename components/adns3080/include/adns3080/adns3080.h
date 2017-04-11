
/*
 * adns3080.h
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

#ifndef ADNS_3080_H_
#define ADNS_3080_H_


#include "image_processing/image_processing.h"

#ifdef __cplusplus
extern "C"
{
#endif


// ADNS3080 Image Configuration
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Default Register Value for ADNS3080
#define ADNS3080_PRODUCT_ID_VALUE      0x17

/**
 * @brief Data Register Maps for ADNS3080 optical sensor
 */
enum adns3080_register {
    ADNS3080_PRODUCT_ID = 0x00,
    ADNS3080_REVISION_ID = 0x01,
    ADNS3080_MOTION = 0x02,
    ADNS3080_DELTA_X = 0x03,
    ADNS3080_DELTA_Y = 0x04,
    ADNS3080_SQUAL = 0x05,
    ADNS3080_PIXEL_SUM = 0x06,
    ADNS3080_MAXIMUM_PIXEL = 0x07,
    ADNS3080_CONFIGURATION_BITS = 0x0a,
    ADNS3080_EXTENDED_CONFIG = 0x0b,
    ADNS3080_DATA_OUT_LOWER = 0x0c,
    ADNS3080_DATA_OUT_UPPER = 0x0d,
    ADNS3080_SHUTTER_LOWER = 0x0e,
    ADNS3080_SHUTTER_UPPER = 0x0f,
    ADNS3080_FRAME_PERIOD_LOWER = 0x10,
    ADNS3080_FRAME_PERIOD_UPPER = 0x11,
    ADNS3080_MOTION_CLEAR = 0x12,
    ADNS3080_FRAME_CAPTURE = 0x13,
    ADNS3080_SROM_ENABLE = 0x14,
    ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER = 0x19,
    ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER = 0x1a,
    ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER = 0x1b,
    ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER = 0x1c,
    ADNS3080_SHUTTER_MAX_BOUND_LOWER = 0x1d,
    ADNS3080_SHUTTER_MAX_BOUND_UPPER = 0x1e,
    ADNS3080_SROM_ID = 0x1f,
    ADNS3080_OBSERVATION = 0x3d,
    ADNS3080_INVERSE_PRODUCT_ID = 0x3f,
    ADNS3080_PIXEL_BURST = 0x40,
    ADNS3080_MOTION_BURST = 0x50,
    ADNS3080_SROM_LOAD = 0x60,
    ADNS3080_SPI_READ = 0x00,
    ADNS3080_SPI_WRITE = 0x80
};

typedef struct
{
    uint8_t motion;
    int8_t dx, dy;
    uint8_t squal;
    uint16_t shutter;
    uint8_t max_pixel;
} motion_struct_t;

typedef struct
{
    uint8_t reset_pin;      // Reset pin
    gpio_num_t cs_pin;      // CS pin
    int32_t x;              // X Pos
    int32_t y;              // Y Pos
    motion_struct_t motion_buf;
    spi_device_handle_t spi_device; // Device handle for spi communications

} adns_3080_device_t;


/**
 * @brief Struct to hold network packet header
 */

typedef struct {
        uint16_t packet_type;
        uint16_t packet_length;
        uint16_t frame_width;
        uint16_t frame_height;
} frame_debug_header_t;

/**
 * @brief Struct to hold network packet for image data
 */

typedef struct {
    frame_debug_header_t header;
    uint8_t frame_data[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y]; // 900 pixels/bytes
} frame_debug_packet_t;

/**
 * @brief Initializes ADNS-3080
 * @param device: Pointer to ADNS-3080 device to initialize.  Modifies data in the structure.
 */
esp_err_t adns_3080_init(adns_3080_device_t *device);

/**
 * @brief Reset ADNS-3080
 * @param device: Target ADNS-3080 device
 */
void adns_3080_reset(adns_3080_device_t *device);

/**
 * @brief Write registers values to AMIS-30543
 * @param device: Target ADNS-3080 device
 */
void adns_3080_apply(adns_3080_device_t device);

/**
 * @brief Clear motion values on ADNS-3080
 * @param device: Target ADNS-3080 device
 */
void adns_3080_clear(adns_3080_device_t device);

/**
 * @brief Write registers values to AMIS-30543
 * @param device: Target ADNS-3080 device
 * @param device: Output frame
 */
bool adns3080_read_frame_burst(adns_3080_device_t *p, image_t *frame_out);

#ifdef __cplusplus
}
#endif

#endif /* ADNS_3080_H_ */
