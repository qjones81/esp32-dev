
/*
 * wifi.h
 *
 *  Created on: Mar 7, 2017
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

#ifndef WIFI_H_
#define WIFI_H_

#include <lwip/sockets.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Struct with wifi device configuration
 */
typedef struct {
    char device_ip[15];           // IP Address
    char device_gw[15];           // Gateway
    char device_netmask[15];      // Netmask
    char device_ssid_name[32];    // SSID Name
    char device_ssid_password[64]; // SSID Password
} wifi_device_config_t;


void wifi_init_connect_ap(wifi_device_config_t config);

void wifi_init_start_ap(wifi_device_config_t config);


#ifdef __cplusplus
}
#endif


#endif /* WIFI_H_ */
