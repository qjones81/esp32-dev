
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
#include <esp_log.h>
#include "image_processing.h"

static const char *tag = "image_processing";

// Reference: http://aishack.in/tutorials/image-moments/
image_moment_t calculate_moments(image_t *in_image)
{
	image_moment_t ret_moments;
	if(in_image == NULL) {
		ESP_LOGE(tag, "calculate_moments: NULL Parameter passed for input image");
		return ret_moments;
	}

	memset(&ret_moments, 0, sizeof(image_moment_t));
	for(int y = 0; y < in_image->height; y++)
	{
		for (int x = 0; x < in_image->width; x++)
		{
			if(in_image->data[in_image->height * y + x] == 1)
			{
				ret_moments.m00 += 1; //sum(X,Y) of x^0*y^0
				ret_moments.m10 += x; //sum(X,Y) of x^1*y^0
				ret_moments.m01 += y; //sum(X,Y) of x^0*y^1
				ret_moments.m11 += (x*y); //sum(X,Y) of x^1*y^1
			}
		}
	}
	return ret_moments;
}

void create_image(uint32_t width, uint32_t height, uint8_t depth, image_t **out_image)
{
	image_t *image = malloc(sizeof(image_t));
	image->width = width;
	image->height = height;
	image->depth = depth;
	image->data = malloc(image->width * image->height * image->depth);
	*out_image = image;

}

void mask_image(image_t *in_image, mask_region_t image_mask, uint32_t value, image_t *out_mask)
{
	if(out_mask == NULL || in_image == NULL) {
		ESP_LOGE(tag, "NULL Parameter passed for image output mask");
		return;
	}
	else if(out_mask->depth != in_image->depth ||
			out_mask->width != in_image->width ||
			out_mask->height != in_image->height) {
		ESP_LOGE(tag, "Mask/Image dimension mismatch");
				return;
	}
	for(int y = 0; y < in_image->height; y++)
	{
		for (int x = 0; x < in_image->width; x++)
		{
			if(y >= image_mask.top && y <= image_mask.bottom && x >= image_mask.left && x <= image_mask.right) {
				out_mask->data[out_mask->height * y + x] = value;
			}
			else {
				out_mask->data[out_mask->height * y + x] = in_image->data[in_image->height * y + x];
			}
		}
	}
}
void image_set_pixel(image_t *in_image, uint32_t x,  uint32_t  y, uint8_t value)
{
	if(in_image == NULL) {
		ESP_LOGE(tag, "image_set_pixel: NULL Parameter passed for input image");
		return;
	}
	in_image->data[in_image->height * y + x] = value;
}
void print_image(image_t *in_image)
{
	if(in_image == NULL) {
		ESP_LOGE(tag, "print_image: NULL Parameter passed for input image");
		return;
	}

    printf("\n--------------------------------------------------\n");
    for (uint32_t y = 0; y < in_image->height; y++) {
        for (uint32_t x = 0; x < in_image->width; x++) {
            printf("%d", in_image->data[in_image->height * y + x]);
            if (x != in_image->width - 1)
                printf(",");
        }
        printf("\n");
    }
    printf("--------------------------------------------------\n\n");
}
