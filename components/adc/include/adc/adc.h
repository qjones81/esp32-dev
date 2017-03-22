
/*
 * adc.h
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

#ifndef ADC_H_
#define ADC_H_

#include "esp32-hal-adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Defaults for ESP32 SPI pinouts.  These can be remapped to other possible pins as well.  See TRM/Datasheet for options.
#define SPI_MOSI    8
#define SPI_MISO    7
#define SPI_CLK     6
#define SPI_CS      11

#define HSPI_MOSI   13
#define HSPI_MISO   12
#define HSPI_CLK    14
#define HSPI_CS     15

#define VSPI_MOSI   23
#define VSPI_MISO   19
#define VSPI_CLK    18
#define VSPI_CS     5

/**
 * @brief SPI Bus initialization routine
 * @param MOSI_pin: SPI Output pin (TX)
 * @param MISO_pin: SPI Input pin (RX)
 * @param CLK_pin: SPI Clock pin
 */
esp_err_t spi_init(uint8_t MOSI_pin, uint8_t MISO_pin, uint8_t CLK_pin);

/**
 * @brief SPI Bus device add routine
 * @param CS_pin: SPI Chip Select pin
 * @param CLK_frequency: SPI Clock frequency
 * @param SPI_Mode: SPI device transfer mode, (0, 1, 2 or 3)
 * @param spi_device_handle_t: Return reference for created SPI device
 */
esp_err_t spi_add_device(int8_t CS_pin, int CLK_frequency, uint8_t SPI_Mode, spi_device_handle_t *spi_dev);

/**
 * @brief SPI Device transfer routine
 * @param tx_data: Data buffer to send
 * @param rx_data: Data buffer to receive.  NULL if write only operation
 * @param length: Transmission length in bytes
 * @param spi_device_handle_t: Return reference for created SPI device
 */
esp_err_t spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, int length, spi_device_handle_t spi_dev);

/**
 * @brief SPI Device read transfer routine
 * @param rx_data: Data buffer to receive.
 * @param length: Transmission length in bytes
 * @param spi_device_handle_t: Return reference for created SPI device
 */
esp_err_t spi_read_transfer(uint8_t address, const uint8_t *tx_data, uint8_t *rx_data, int length, spi_device_handle_t spi_dev);

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* SPI_H_ */
