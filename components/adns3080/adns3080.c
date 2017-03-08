
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "spi/spi.h"
#include "esp_log.h"

#include "utils/utils.h"
#include "adns3080.h"

static const char *TAG = "adns-3080";


#define MAX_SENSORS 3 // Max number of optical sensors allowed to be controlled.

adns_3080_device_t *adns_devices[MAX_SENSORS] = { NULL };

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

  //  ESP_LOGI(TAG, "Received: 0x%x.\n", rx_data);
}

uint8_t read_register(uint8_t address, adns_3080_device_t *device)
{
    // TODO: Setup callback for pre-post transmission for CS callback instead.  Might help with clock line sync issues
    uint8_t tx_data = (ADNS3080_SPI_READ | address); // Register to read
    uint8_t rx_data; // Receive Byte
    rx_data = 0; // Register to read

    gpio_set_level(device->cs_pin, 0); // Toggle our own CS in software so we can support the mid transaction delays
    spi_transfer(&tx_data, 0, 1, device->spi_device);   // Transfer out read reg
    delay_us(75); // Must delay atleast 50us, or 75us for motion reads.  Use workcase for now
    spi_transfer(&tx_data, &rx_data, 1, device->spi_device); // Transfer out read reg
    gpio_set_level(device->cs_pin, 1); // Toggle our own CS in software so we can support the mid transaction delays

  //  ESP_LOGI(TAG, "Received: 0x%x.\n", rx_data);
    return rx_data;
}

void write_register(uint8_t address, uint8_t data, adns_3080_device_t *device)
{

    // TODO: May need two stage write here
    uint8_t tx_data[2];
    tx_data[0] = ADNS3080_SPI_WRITE | address;
    tx_data[1] = data;
    gpio_set_level(device->cs_pin, 0); // Toggle our own CS in software so we can support the mid transaction delays
    spi_transfer(tx_data, 0, 2, device->spi_device);    // Do write transaction
    gpio_set_level(device->cs_pin, 1); // Toggle our own CS in software so we can support the mid transaction delays
}

// Read Task
void task_adns3080_reader_task(void *ignore)
{
    ESP_LOGD(TAG, ">>task_adns3080_reader_task");

    while (1) {
        // Read sensor
        uint8_t buf[4];
        read_register_data(ADNS3080_MOTION_BURST, buf, 4, adns_devices[0]);
        uint8_t motion = buf[0];
        ESP_LOGI(TAG, "ADNS motion resolution: %d", motion & 0x01); // Resolution

        if (motion & 0x10) // Check if we've had an overflow
        {
            ESP_LOGD(TAG, "ADNS-3080 overflow\n");
        }
        else if (motion & 0x80) {
            int8_t dx = buf[1];
            int8_t dy = buf[2];
            uint8_t SQUAL = buf[3];

            adns_devices[0]->x += dx;
            adns_devices[0]->y += dy;

            // Print values
            ESP_LOGI(TAG,"x=%d,y=%d\tSQUAL:%d",adns_devices[0]->x,adns_devices[0]->y,SQUAL);
        }

        vTaskDelay(10);
    }

    vTaskDelete(NULL);
}



// ADNS 3080 Functions
esp_err_t adns_3080_init(adns_3080_device_t *device)
{


    // Setup device pins
    gpio_set_direction(device->reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(device->reset_pin, 1);
    gpio_set_direction(device->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(device->cs_pin, 1);

    // Update Positions
    device->x = 0;
    device->y = 0;

    // Hack to get clock going properly
    //read_reg_1(ADNS3080_PRODUCT_ID, device->spi_device);
    read_register(ADNS3080_PRODUCT_ID, device);

    delay_ms(10);

    // Reset device
    adns_3080_reset(device);

    uint8_t id = read_register(ADNS3080_PRODUCT_ID, device);
    if (id == ADNS3080_PRODUCT_ID_VALUE) {
        ESP_LOGI(TAG, "ADNS-3080 found. id = 0x%x\n", id);
    } else {
        ESP_LOGE(TAG, "Could not find ADNS-3080. id: 0x%x\n", id);

       return ESP_FAIL;
   }

    uint8_t config = read_register(ADNS3080_CONFIGURATION_BITS, device);
    ESP_LOGI(TAG, "ADNS-3080 configuration: 0x%x\n", config);

    // 1600 counts per inch
    write_register(ADNS3080_CONFIGURATION_BITS, (config | 0b00010000), device);

        config = read_register(ADNS3080_CONFIGURATION_BITS, device);
        ESP_LOGI(TAG, "ADNS-3080 configuration saved: 0x%x\n", config);

        // Add it
        adns_devices[0] = device;
while(1)
{

        uint8_t buf[4];
                read_register_data(ADNS3080_MOTION_BURST, buf, 4, adns_devices[0]);
                uint8_t motion = buf[0];
                ESP_LOGI(TAG, "ADNS motion resolution: %d", motion & 0x01); // Resolution

                int8_t dx = buf[1];
                            int8_t dy = buf[2];
                            uint8_t SQUAL = buf[3];

                            adns_devices[0]->x += dx;
                            adns_devices[0]->y += dy;

                            // Print values
                            ESP_LOGI(TAG,"x=%d,y=%d\tSQUAL:%d",adns_devices[0]->x,adns_devices[0]->y,SQUAL);

                delay_ms(100);
}
    return ESP_OK;
}

void adns_3080_reset(adns_3080_device_t *device)
{
    gpio_set_level(device->reset_pin, 1);
    delay_us(10);
    gpio_set_level(device->reset_pin, 0);

    delay_us(500); // Give device time to wake up
}

void amis_30543_apply(adns_3080_device_t device)
{

}
