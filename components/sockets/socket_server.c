
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

#include <lwip/sockets.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include <string.h>
#include <errno.h>


#include "socket_server.h"

static const char *tag = "socket_server";

// Listener task
void task_server_listener(void *data) {

    socket_device_t *sock = (socket_device_t *)data;
    struct sockaddr_in client_address;

    while(1) {
        socklen_t client_address_len = sizeof(client_address);
        int temp_sock = accept(sock->server_sock, (struct sockaddr *)&client_address, &client_address_len);
        if (temp_sock == -1) {
            ESP_LOGE(tag, "ERROR:  Unable to accept client connection:  accept(): %s", strerror(errno));
        }


        // TODO: Need more handling here for multiple client connections
        if (sock->client_sock != -1) { // Socket already open.  Need to close old.
            int ret = close(sock->client_sock);
            if (ret == -1) {
                ESP_LOGE(tag, "ERROR:  Unable to close client socket [task_server_listener]:  close(): %s/%d", strerror(errno),sock->client_sock);
            }
        }
        sock->client_sock = temp_sock;

        ESP_LOGD(tag, "New client connected! [%d]", socket_server_connected_count(sock));
    }
} // acceptTask

void socket_server_init(socket_device_t *device)
{
    device->client_sock = -1;

    // TODO: More intialization here
}

uint16_t socket_server_connected_count(socket_device_t *device)
{
    // TODO: Support multiple clients.  For now this will work for one.
    if(device->client_sock == -1)
        return 0;

    return 1;
}

void socket_server_disconnect(socket_device_t *device)
{
    if (device->client_sock != -1) {
        int ret = close(device->client_sock);
        if (ret == -1) {
            ESP_LOGE(tag, "ERROR: Unable to close socket:  close(): %s", strerror(errno));
        }
        device->client_sock = -1;
    }
}

void socket_server_send_data(socket_device_t *device, uint8_t *data, size_t length)
{
    if (socket_server_connected_count(device) == 0) { // No Connected clients.  Bail.
        return;
    }

    // Check for close
    char buffer[32];
    if (recv(device->client_sock, buffer, sizeof(buffer),
            MSG_PEEK | MSG_DONTWAIT) == 0) {
        ESP_LOGE(tag, "ERROR: Client connection closed");

        // TODO: Remove connection and don't send BLAH BLAH
        socket_server_disconnect(device);
        return;
    }

    int sent_bytes = send(device->client_sock, data, length, 0);
    if (sent_bytes == -1) {
        ESP_LOGE(tag, "ERROR:  Unable to send data:  send(): %s",
                strerror(errno));

        if (errno == ECONNRESET) {
            // TODO: Remove connection and don't send BLAH BLAH
            socket_server_disconnect(device);
            return;
        }
    }
    ESP_LOGI(tag, "Sent Bytes: %d", sent_bytes);
}
void socket_server_start(socket_device_t *device)
{
    // TODO: Should check for socket already open.  Either close or error out here.

    device->server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (device->server_sock == -1) {
        ESP_LOGE(tag, "ERROR: Unable to create socket:  socket(): %s", strerror(errno));
    }


    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_address.sin_port = htons(device->port);

    int ret = bind(device->server_sock, (struct sockaddr *)&server_address, sizeof(server_address));
    if (ret == -1) {
        ESP_LOGE(tag, "ERROR: Unable to bind socket:  bind(): %s", strerror(errno));
    }
    ret = listen(device->server_sock, 5);
    if (ret == -1) {
        ESP_LOGE(tag, "listen(): %s", strerror(errno));
    }

    // Set no delay options
    //int flag = 1;
    ////setsockopt(device->server_sock, IPPROTO_TCP, TCP_NODELAY, (char *)flag, sizeof(int));
    ESP_LOGD(tag, "Now listening on port %d", device->port);

    xTaskCreate(&task_server_listener, "tcp_listen_task", 2048, device, 3, NULL);
}
