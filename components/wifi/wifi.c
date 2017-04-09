
/*
 * wifi.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include <esp_log.h>
#include "wifi.h"

static const char *tag = "wifi";

const int SCAN_BIT = BIT0;
const int STRT_BIT = BIT1;
const int DISC_BIT = BIT2;
const int DHCP_BIT = BIT3;

struct known_ap {
    char *ssid;
    char *passwd;
};

struct known_ap known_aps[CONFIG_WIFI_AP_COUNT] = {
#ifdef CONFIG_WIFI_AP1
      {.ssid = CONFIG_WIFI_AP1_SSID, .passwd = CONFIG_WIFI_AP1_PASSWORD}
#ifdef CONFIG_WIFI_AP2
    , {.ssid = CONFIG_WIFI_AP2_SSID, .passwd = CONFIG_WIFI_AP2_PASSWORD}
#ifdef CONFIG_WIFI_AP3
    , {.ssid = CONFIG_WIFI_AP3_SSID, .passwd = CONFIG_WIFI_AP3_PASSWORD}
#ifdef CONFIG_WIFI_AP4
    , {.ssid = CONFIG_WIFI_AP4_SSID, .passwd = CONFIG_WIFI_AP4_PASSWORD}
#ifdef CONFIG_WIFI_AP5
    , {.ssid = CONFIG_WIFI_AP5_SSID, .passwd = CONFIG_WIFI_AP5_PASSWORD}
#endif
#endif
#endif
#endif
#endif
};


// Cache IP Information
ip4_addr_t ip;
ip4_addr_t gw;
ip4_addr_t msk;


//static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
//{
//    ESP_LOGI(tag, "event_handler: 0x%x", event->event_id);
//
//    switch (event->event_id) {
//        case SYSTEM_EVENT_STA_START:
//            ESP_LOGI(tag, "Connecting...");
//            esp_wifi_connect();
//            break;
//
//        case SYSTEM_EVENT_STA_GOT_IP:
//            ip = event->event_info.got_ip.ip_info.ip;
//            gw = event->event_info.got_ip.ip_info.gw;
//            msk = event->event_info.got_ip.ip_info.netmask;
//
//            ESP_LOGI(tag, "Connected. IP: [%s]", inet_ntoa(ip));
//            break;
//
//        case SYSTEM_EVENT_STA_DISCONNECTED:
//            ESP_LOGI(tag, "Station Disconnected.  Attempting Reconnect...");
//            esp_wifi_connect();
//            break;
//
//        default:
//            break;
//    }
//    return ESP_OK;
//}

static esp_err_t
wifi_event_handler(void *ctx, system_event_t *event)
{
    EventGroupHandle_t evg = (EventGroupHandle_t)ctx;

    switch(event->event_id) {
    	// Station event handling
        case SYSTEM_EVENT_STA_START:
        	ESP_LOGI(__func__, "started");
            xEventGroupSetBits(evg, STRT_BIT);
            break;
        case SYSTEM_EVENT_SCAN_DONE:
        	ESP_LOGI(__func__, "Scan done");
            xEventGroupSetBits(evg, SCAN_BIT);
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
        	ESP_LOGI(__func__, "connected");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
        	ip = event->event_info.got_ip.ip_info.ip;
            ESP_LOGI(__func__, "Connected. IP: [%s]", inet_ntoa(ip));
            xEventGroupSetBits(evg, DHCP_BIT);
            break;
		case SYSTEM_EVENT_STA_STOP:
			ESP_LOGI(__func__, "stopped");
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
        	ESP_LOGI(__func__, "Station Disconnected.  Attempting Reconnect...");
        	esp_wifi_connect();
            xEventGroupSetBits(evg, DISC_BIT);
            break;
            // AP Event Handling
		case SYSTEM_EVENT_AP_START:
			ESP_LOGI(__func__, "Soft-AP Started: [%s]", inet_ntoa(ip));
			break;
		case SYSTEM_EVENT_AP_STOP:
			ESP_LOGI(__func__, "Soft-AP Stopped");
			break;
		case SYSTEM_EVENT_AP_STACONNECTED:
			ESP_LOGI(__func__, "Soft-AP Client connected");
			break;
		case SYSTEM_EVENT_AP_STADISCONNECTED:
			ESP_LOGI(__func__, "Soft-AP Client disconnected");
			break;
        default:
            ESP_LOGW(__func__, "Unhandled event (%d)", event->event_id);
            break;
    }
    return ESP_OK;
}

esp_err_t wifi_init_connect_ap()
{
	ESP_LOGI(tag, "wifi_init_connect_ap...");
	esp_err_t error;
	uint16_t ap_count;
	wifi_ap_record_t *list = NULL;
	int i = 0, j, f, match;
	wifi_config_t wifi_config;

	EventGroupHandle_t evg = xEventGroupCreate();
	wifi_scan_config_t scanConf = { .ssid = NULL, .bssid = NULL, .channel = 0,
			.show_hidden = true };
	tcpip_adapter_init();
	error = esp_event_loop_init(wifi_event_handler, evg);
	if (error != ESP_OK)
		return error;
	wifi_init_config_t
	cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_STA);
	xEventGroupClearBits(evg, STRT_BIT);
	esp_wifi_start();

	int state = 0;
	int done = 0;
	while (!done) {
		switch (state) {
		case 0:
			while (!(xEventGroupWaitBits(evg, STRT_BIT, pdFALSE, pdFALSE,
			portMAX_DELAY) & STRT_BIT));
			xEventGroupClearBits(evg, SCAN_BIT);
			ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, 0));
			while (!(xEventGroupWaitBits(evg, SCAN_BIT, pdFALSE, pdFALSE,
			portMAX_DELAY) & SCAN_BIT));
			ap_count = 0;
			esp_wifi_scan_get_ap_num(&ap_count);
			if (!ap_count)
				break;
			free(list);
			list = (wifi_ap_record_t *) malloc(
					sizeof(wifi_ap_record_t) * ap_count);
			ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, list));
			for (i = 0; i < ap_count; i++) {
				ESP_LOGD(__func__, "%d %s", list[i].rssi, (char * )list[i].ssid);
			}
			i = 0;
		case 1:
			state = 0;
			for (match = 0; i < ap_count && !match; i++) {
				for (j = 0; j < CONFIG_WIFI_AP_COUNT && !match; j++) {
					match = !strcasecmp((char *) list[i].ssid,
							known_aps[j].ssid);
				}
			}
			if (!match)
				break;
			state = 1;
			strncpy((char *) wifi_config.sta.ssid, (char *) list[i - 1].ssid, 32);
			strncpy((char *) wifi_config.sta.password, known_aps[j - 1].passwd,64);
			ESP_LOGI(__func__, "Setting WiFi configuration SSID %s...", (char * )wifi_config.sta.ssid);

			esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
			xEventGroupClearBits(evg, DISC_BIT | DHCP_BIT);
			esp_wifi_connect();
			ESP_LOGI(__func__, "Waiting for IP address...");
			state = 2;
		case 2:
			f = xEventGroupWaitBits(evg, DHCP_BIT | DISC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
			if (f & DISC_BIT)
				state = 1;
			done = f & DHCP_BIT;
		}
	}
	free(list);
	return ESP_OK;
}

#if defined(CONFIG_WIFI_HOST_AP_MODE)
esp_err_t wifi_init_start_ap() {
 //   nvs_flash_init();
	ESP_LOGI(tag, "Starting AP...");
	wifi_config_t ap_config = { .ap =
			{ .ssid = CONFIG_WIFI_HOST_AP_SSID,
			.ssid_len = 0,
			.password = CONFIG_WIFI_HOST_AP_PASSWORD,
			.channel = CONFIG_WIFI_HOST_AP_CHANNEL,
			.authmode = WIFI_AUTH_OPEN,
			.ssid_hidden = 0,
			.max_connection = CONFIG_WIFI_HOST_AP_MAX_CONNECTIONS,
			.beacon_interval = 100 }
	};

    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );

    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &ap_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    return ESP_OK;
}
#endif

esp_err_t wifi_network_up() {

	#if defined(CONFIG_WIFI_HOST_AP_MODE)
			// Start an AP
		   return wifi_init_start_ap();
	#elif CONFIG_WIFI_AP_COUNT != 0
			// Connect to an AP
		   return wifi_init_connect_ap();
	#else
		   return ESP_FAIL;
	#endif
}
