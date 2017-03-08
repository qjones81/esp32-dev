#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#include "i2c.h"

static bool started;
static uint8_t g_scl_pin;
static uint8_t g_sda_pin;
static i2c_cmd_handle_t cmd;
//static const char *TAG = "i2c";

void i2c_init(uint8_t scl_pin, uint8_t sda_pin)
{

	// TODO: If started, either reconfigure or bail, etc.

	// Cache for later
	g_scl_pin = scl_pin;
	g_sda_pin = sda_pin;

	i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    started = 1;
}

// Write a byte to I2C bus. Return true if slave acked.
bool i2c_write(uint8_t byte)
{
	return i2c_master_write_byte(cmd, byte, 1);
}

// Read a byte from I2C bus. Return true if slave acked.
uint8_t i2c_read()
{
	uint8_t data;
	i2c_master_read_byte(cmd, &data, 1);

	return data;
}

void i2c_start(uint8_t address, i2c_rw_t read_write)
{
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | read_write, 1 /* expect ack */);
}
void i2c_stop(void)
{
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

int i2c_write_byte(uint8_t address, uint8_t sub_address, uint8_t data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, sub_address, 1);
	i2c_master_write_byte(cmd, data, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return 0;
}

int i2c_write_bytes(uint8_t address, uint8_t sub_address, uint8_t count, uint8_t *src)
{
	if(count == 0)
		return 0;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, sub_address, 1);
	i2c_master_write(cmd, src, count, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return 0;
}

uint8_t i2c_read_byte(uint8_t address, uint8_t sub_address)
{
	uint8_t data;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, sub_address, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1 /* expect ack */);
	i2c_master_read_byte(cmd, &data, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return data;
}
int i2c_read_bytes(uint8_t address, uint8_t sub_address, uint8_t count, uint8_t *dest)
{
	//char data[14];
	if(count == 0)
		return 0;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, sub_address, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, 1 /* expect ack */);

	if (count > 1) {
		i2c_master_read(cmd, dest, count-1, 0);
	}

	i2c_master_read_byte(cmd, dest + count - 1, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return 0;
}
