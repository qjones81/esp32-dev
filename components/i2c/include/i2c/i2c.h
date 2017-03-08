/*
 * i2c.h
 *
 *  Created on: Feb 11, 2017
 *      Author: qjones
 */
// Wrapper for esp32 i2c libraries

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

void i2c_init(uint8_t scl_pin, uint8_t sda_pin);

// Write a byte to I2C bus. Return true if slave acked.
bool i2c_write(uint8_t byte);

// Read a byte from I2C bus. Return true if slave acked.
uint8_t i2c_read();

void i2c_start(uint8_t address, i2c_rw_t read_write);
void i2c_stop(void);

int i2c_write_byte(uint8_t address, uint8_t sub_address, uint8_t data);
int i2c_write_bytes(uint8_t address, uint8_t sub_address, uint8_t count, uint8_t *src);

uint8_t i2c_read_byte(uint8_t address, uint8_t sub_address);
int i2c_read_bytes(uint8_t address, uint8_t sub_address, uint8_t count, uint8_t *dest);
#endif /* I2C_H_ */
