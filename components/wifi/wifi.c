
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

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include <esp_log.h>

#include "wifi.h"

static const char *tag = "wifi";


// The identity of the access point to which we wish to connect.
//#define AP_TARGET_SSID     "C4SN"
#define AP_TARGET_SSID     "AndroidAP"
// The password we need to supply to the access point for authorization.
#define AP_TARGET_PASSWORD "8ace66a8947f"


// Cache IP Information
ip4_addr_t ip;
ip4_addr_t gw;
ip4_addr_t msk;

// Connected?
bool connected = 0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
   ESP_LOGI(tag, "event_handler: 0x%x", event->event_id);

   switch (event->event_id)
   {
      case SYSTEM_EVENT_STA_START:
          ESP_LOGI(tag, "Connecting...");
         esp_wifi_connect();
         break;

      case SYSTEM_EVENT_STA_GOT_IP:
          ip = event->event_info.got_ip.ip_info.ip;
          gw = event->event_info.got_ip.ip_info.gw;
          msk = event->event_info.got_ip.ip_info.netmask;
          connected = 1;

          ESP_LOGI(tag, "Connected. IP: [%s]", inet_ntoa(ip));
         break;

      case SYSTEM_EVENT_STA_DISCONNECTED:
          connected = 0;
          ESP_LOGI(tag, "Station Disconnected.  Attempting Reconnect...");
         /* This is a workaround as ESP32 WiFi libs don't currently auto reassociate. */
         esp_wifi_connect();
         break;

      default:
         break;
   }
   return ESP_OK;
}

void wifi_init_connect_ap(wifi_device_config_t config)
{

    ESP_LOGI(tag, "wifi_init_connect_ap...");
    tcpip_adapter_init();

    // Static Config
    //tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA); // Don't run a DHCP client
    //tcpip_adapter_ip_info_t ipInfo;

    //inet_pton(AF_INET, config.device_ip, &ipInfo.ip);
    //inet_pton(AF_INET, config.device_gw, &ipInfo.gw);
    //inet_pton(AF_INET, config.device_netmask, &ipInfo.netmask);
    //tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);


    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t sta_config = {
          .sta = {
                .ssid = AP_TARGET_SSID,
                .password = AP_TARGET_PASSWORD,
                .bssid_set = 0
          }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(tag, "wifi_init_connect_ap finished");

}


void wifi_ap_task(void *ignore) {
 //   nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
    wifi_config_t apConfig = {
       .ap = {
          .ssid="ESP32_TESTAP",
          .ssid_len=0,
          .password="",
          .channel=0,
          .authmode=WIFI_AUTH_OPEN,
          .ssid_hidden=0,
          .max_connection=4,
          .beacon_interval=100
       }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &apConfig) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    vTaskDelete(NULL);
}
