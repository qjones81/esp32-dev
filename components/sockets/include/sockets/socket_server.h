
/*
 * socket_server.h
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

#ifndef SOCKET_SERVER_H_
#define SOCKET_SERVER_H_


#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Struct to hold socket configuration and state
 */

typedef struct {
    int32_t server_sock;
    int32_t client_sock; // TODO: Should be a list of clients.  For more than 1.  This will work for now
    uint16_t port;

} socket_device_t;



/**
 * @brief Initializes a socket server
 * @param device: Pointer to socket device to initialize.  Modifies data in the structure.
 */
void socket_server_init(socket_device_t *device);

/**
 * @brief Get number of currently connected clients
 * @param device: Pointer to socket device.
 * @return Number of currently connected clients
 */
uint16_t socket_server_connected_count(socket_device_t *device);

/**
 * @brief Disconnect socket server
 * @param device: Pointer to socket device.
 */
void socket_server_disconnect(socket_device_t *device);

/**
 * @brief Send data over socket
 * @param device: Pointer to socket device.
 * @param data: byte data to send
 * @param device: length of byte array to send
 */
void socket_server_send_data(socket_device_t *device, uint8_t *data, size_t length);

/**
 * @brief Start socket server
 * @param device: Pointer to socket device.
 */
void socket_server_start(socket_device_t *device);

/**
 * @brief Stop socket server
 * @param device: Pointer to socket device.
 */
void socket_server_stop(socket_device_t *device);


#ifdef __cplusplus
}
#endif


#endif /* SOCKET_SERVER_H_ */
