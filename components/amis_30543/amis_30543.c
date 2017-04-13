
/*
 * amis_30543.c
 *
 *  Created on: Feb 15, 2017
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

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "spi/spi.h"
#include "amis_30543.h"

static const char *tag = "amis_30543";

void write_reg(uint8_t address, uint8_t data, spi_nodma_device_handle_t device)
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	tx_data[0] = 0x80 | (address & 0b11111);
	tx_data[1] = data;
	spi_transfer(tx_data, rx_data, 2, device);
}

uint8_t read_reg(uint8_t address, spi_nodma_device_handle_t device)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    tx_data[0] = (address & 0b11111);
    tx_data[1] = 0x00;
    spi_transfer(tx_data, rx_data, 2, device);

   // ESP_LOGI(tag, "Read back: %d", rx_data[1]);

    return rx_data[1];
}


// Write cached value of the WR register to the device.
void write_WR(amis_30543_device_t device)
{
	write_reg(WR, device.wr, device.spi_device);
}

// Write cached value of the CR0 register to the device.
void write_CR0(amis_30543_device_t device)
{
	write_reg(CR0, device.cr0, device.spi_device);
}

// Write cached value of the CR1 register to the device.
void write_CR1(amis_30543_device_t device)
{
	write_reg(CR1, device.cr1, device.spi_device);
}

// Write cached value of the CR2 register to the device.
void write_CR2(amis_30543_device_t device)
{
	write_reg(CR2, device.cr2, device.spi_device);
}

// Write cached value of the CR3 register to the device.
void write_CR3(amis_30543_device_t device)
{
	write_reg(CR3, device.cr3, device.spi_device);
}

// AMIS 30543 Functions
void amis_30543_init(amis_30543_device_t *device)
{
    device->wr = 0;
    device->cr0 = 0;
    device->cr1 = 0;
    device->cr2 = 0;
    device->cr3 = 0;
}
void amis_30543_reset(amis_30543_device_t *device)
{
    device->wr = 0;
    device->cr0 = 0;
    device->cr1 = 0;
    device->cr2 = 0;
    device->cr3 = 0;
	amis_30543_apply(*device);
}

void amis_30543_apply(amis_30543_device_t device)
{
    write_CR2(device); // Write CR2 register first to ensure motor enable bit does not become invalid

    // Write the rest
    write_WR(device);
    write_CR0(device);
    write_CR1(device);
    write_CR3(device);
}

void amis_30543_enable_driver(amis_30543_device_t *device, bool enable)
{
	if (enable) {
	    device->cr2 |= 0b10000000;
	} else {
	    device->cr2 &= ~0b10000000;
	}
	device->enable = enable;
	amis_30543_apply(*device);
}
void amis_30543_set_step_mode(amis_30543_device_t *device, uint8_t mode)
{
	// Pick 1/32 micro-step by default.
	uint8_t esm = 0b000;
	uint8_t sm = 0b000;

	// The order of these cases matches the order in Table 12 of the
	// AMIS-30543 datasheet.
	switch (mode) {
	case MicroStep32:
		sm = 0b000;
		break;
	case MicroStep16:
		sm = 0b001;
		break;
	case MicroStep8:
		sm = 0b010;
		break;
	case MicroStep4:
		sm = 0b011;
		break;
	case CompensatedHalf:
		sm = 0b100;
		break; /* a.k.a. MicroStep2 */
	case UncompensatedHalf:
		sm = 0b101;
		break;
	case UncompensatedFull:
		sm = 0b110;
		break;
	case MicroStep128:
		esm = 0b001;
		break;
	case MicroStep64:
		esm = 0b010;
		break;
	case CompensatedFullTwoPhaseOn:
		esm = 0b011;
		break; /* a.k.a. MicroStep 1 */
	case CompensatedFullOnePhaseOn:
		esm = 0b100;
		break;
	}

	device->cr0 = (device->cr0 & ~0b11100000) | (sm << 5);
	device->cr3 = (device->cr3 & ~0b111) | esm;
	write_CR0(*device);
	write_CR3(*device);
}
void amis_30543_set_current(amis_30543_device_t *device, uint16_t current_milliamps)
{
	// This comes from Table 13 of the AMIS-30543 datasheet.
	uint8_t code = 0;
	if (current_milliamps >= 3000) {
		code = 0b11001;
	} else if (current_milliamps >= 2845) {
		code = 0b11000;
	} else if (current_milliamps >= 2700) {
		code = 0b10111;
	} else if (current_milliamps >= 2440) {
		code = 0b10110;
	} else if (current_milliamps >= 2240) {
		code = 0b10101;
	} else if (current_milliamps >= 2070) {
		code = 0b10100;
	} else if (current_milliamps >= 1850) {
		code = 0b10011;
	} else if (current_milliamps >= 1695) {
		code = 0b10010;
	} else if (current_milliamps >= 1520) {
		code = 0b10001;
	} else if (current_milliamps >= 1405) {
		code = 0b10000;
	} else if (current_milliamps >= 1260) {
		code = 0b01111;
	} else if (current_milliamps >= 1150) {
		code = 0b01110;
	} else if (current_milliamps >= 1060) {
		code = 0b01101;
	} else if (current_milliamps >= 955) {
		code = 0b01100;
	} else if (current_milliamps >= 870) {
		code = 0b01011;
	} else if (current_milliamps >= 780) {
		code = 0b01010;
	} else if (current_milliamps >= 715) {
		code = 0b01001;
	} else if (current_milliamps >= 640) {
		code = 0b01000;
	} else if (current_milliamps >= 585) {
		code = 0b00111;
	} else if (current_milliamps >= 540) {
		code = 0b00110;
	} else if (current_milliamps >= 485) {
		code = 0b00101;
	} else if (current_milliamps >= 445) {
		code = 0b00100;
	} else if (current_milliamps >= 395) {
		code = 0b00011;
	} else if (current_milliamps >= 355) {
		code = 0b00010;
	} else if (current_milliamps >= 245) {
		code = 0b00001;
	}

	device->cr0 = (device->cr0 & 0b11100000) | code;
	write_CR0(*device);
}

void amis_30543_pwm_frequency_double(amis_30543_device_t *device, bool enable_double_frequency)
{
    if(enable_double_frequency) { // Clears the PWMF bit, which sets the PWM frequency to its default value (45.6 kHz).
        device->cr1 |= (1 << 3);
        write_CR1(*device);
    }
    else {  // Clears the PWMF bit, which sets the PWM frequency to its default value (22.8 kHz).
        device->cr1 &= ~(1 << 3);
        write_CR1(*device);
    }
}

bool amis_30543_verify(amis_30543_device_t *device)
{
    return (read_reg(WR, device->spi_device) == device->wr &&
                read_reg(CR0, device->spi_device) == device->cr0 &&
                read_reg(CR1, device->spi_device) == device->cr1 &&
                read_reg(CR2, device->spi_device) == device->cr2 &&
                read_reg(CR3, device->spi_device) == device->cr3);
}
