
/*
 * spi.c
 *
 *  Created on: Feb 14, 2017
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
#include <esp_log.h>

#include "soc/spi_reg.h"
#include "soc/spi_struct.h"

#include "spi.h"

static const char *TAG = "spi";

esp_err_t spi_init(uint8_t MOSI_pin, uint8_t MISO_pin, uint8_t CLK_pin)
{
	esp_err_t ret;
	spi_bus_config_t buscfg={
	        .miso_io_num=MISO_pin,
	        .mosi_io_num=MOSI_pin,
	        .sclk_io_num=CLK_pin,
	        .quadwp_io_num=-1,
	        .quadhd_io_num=-1
	    };

	ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);

	return ret;
}
esp_err_t spi_add_device(int8_t CS_pin, int CLK_frequency, uint8_t SPI_Mode, spi_device_handle_t *spi_dev)
{
	esp_err_t ret;

	// TODO: Send this as param probably...
	spi_device_interface_config_t dev_config;
	dev_config.address_bits = 0;
	dev_config.command_bits = 0;
	dev_config.dummy_bits = 0;
	dev_config.mode = SPI_Mode;
	dev_config.duty_cycle_pos = 0;
	dev_config.cs_ena_posttrans = 0;
	dev_config.cs_ena_pretrans = 0;
	dev_config.clock_speed_hz = CLK_frequency;
	dev_config.spics_io_num = CS_pin;
	dev_config.flags = 0;
	dev_config.queue_size = 5;
	dev_config.pre_cb = NULL;
	dev_config.post_cb = NULL;

	ret = spi_bus_add_device(VSPI_HOST, &dev_config, spi_dev);

	return ret;
}

esp_err_t spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, int length, spi_device_handle_t spi_dev)
{
	esp_err_t ret; // Return Error Code

	spi_transaction_t trans_desc;
	memset(&trans_desc, 0, sizeof(trans_desc));
	trans_desc.address = 0;
	trans_desc.command = 0;
	trans_desc.flags = 0;
	trans_desc.length = length * 8;
	trans_desc.rxlength = 0;
	trans_desc.tx_buffer = tx_data;
	trans_desc.rx_buffer = rx_data;

	ret = spi_device_transmit(spi_dev, &trans_desc);  //Transmit!
	return ret;
}

esp_err_t spi_transfer_test(const uint8_t *tx_data, uint8_t *rx_data, int length, int rx_length, spi_device_handle_t spi_dev)
{
	esp_err_t ret; // Return Error Code

	spi_transaction_t trans_desc;
	memset(&trans_desc, 0, sizeof(trans_desc));
	trans_desc.address = 0;
	trans_desc.command = 0;
	trans_desc.flags = 0;
	trans_desc.length = length * 8;
	trans_desc.rxlength = rx_length * 8;
	trans_desc.tx_buffer = tx_data;
	trans_desc.rx_buffer = rx_data;

	ret = spi_device_transmit(spi_dev, &trans_desc);  //Transmit!
	return ret;
}

esp_err_t spi_read_transfer(uint8_t address, const uint8_t *tx_data, uint8_t *rx_data, int length, spi_device_handle_t spi_dev)
{
    esp_err_t ret; // Return Error Code

    spi_transaction_t trans_desc;
    memset(&trans_desc, 0, sizeof(trans_desc));
    trans_desc.address = 0;
    trans_desc.command = 0;
    trans_desc.flags = 0;
    trans_desc.length = length * 8;
    trans_desc.rxlength = 0;
    trans_desc.tx_buffer = tx_data;
    trans_desc.rx_buffer = rx_data;

    ret = spi_device_transmit(spi_dev, &trans_desc);  //Transmit!
    return ret;
}
