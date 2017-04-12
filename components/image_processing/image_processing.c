
/*
 * image_processing.c
 *
 *  Created on: April 10, 2017
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_attr.h>
#include <esp_log.h>
#include "utils/utils.h"

#include "image_processing.h"

static const char *tag = "image_processing";

typedef struct label_node {
	uint8_t label_id;
	struct label_node *parent;

} label_node_t;


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

void calculate_local_moments(image_t *in_image, vector *moments_out)
{
	if(in_image == NULL) {
		ESP_LOGE(tag, "calculate_local_moments: NULL Parameter passed for input image");
		return;
	}

	image_moment_t* blob_moments[32]; // Support 32 for now.  Needs to be a parameter of some sort
	memset(&blob_moments, 0, sizeof(image_moment_t *));
    for (int y = 0; y < in_image->height; y++) {
        for (int x = 0; x < in_image->width; x++) {
            uint8_t blob_id = in_image->data[in_image->height * y + x];
            if (blob_moments[blob_id] == NULL) {
                image_moment_t *moment = malloc(sizeof(image_moment_t));
                memset(moment, 0, sizeof(image_moment_t)); // Zero it out just in case
                vector_add(moments_out, moment); // Add to output vector
                blob_moments[blob_id] = moment;
                ESP_LOGI(tag, "Adding Moment: %d", blob_id);
            }
            blob_moments[blob_id]->m00 += 1; //sum(X,Y) of x^0*y^0
            blob_moments[blob_id]->m10 += x; //sum(X,Y) of x^1*y^0
            blob_moments[blob_id]->m01 += y; //sum(X,Y) of x^0*y^1
            blob_moments[blob_id]->m11 += (x * y); //sum(X,Y) of x^1*y^1
        }
    }
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

uint8_t image_connected_components(image_t *in_image, image_t *out_labeled)
{
	uint8_t num_components = 0;
	uint8_t num_labels = 1; // Start with 1 for non zero indexing purposes.  0 is used for background

	uint32_t width_labels = in_image->width + 1; // Pad it by 1 pixel
	uint32_t height_labels = in_image->height + 1; // Pad it by 1 pixel

	// Using uint8 here, so 255 max labels.
	uint8_t *label_image = malloc(width_labels * height_labels * sizeof(uint8_t));

	// Create label component array with padding for edge cases
	label_node_t labels[32]; // 32 max labels for now

	// zero it all out.
	memset(label_image, 0, width_labels * height_labels * sizeof(uint8_t));
	memset(&labels, 0, 32 * sizeof(label_node_t));

	// First Pass
	for (int y = 0, y_label = 1; y < in_image->height; y++, y_label++) { // Start at one since we are padded for labels
		for (int x = 0, x_label = 1; x < in_image->width; x++, x_label++) {
			uint8_t pixel = in_image->data[in_image->height * y + x];
			uint8_t *curr_label = &label_image[height_labels * y_label + x_label]; // offset for padding with ptr math
			uint8_t left_label = label_image[height_labels * y_label + (x_label - 1)]; // offset for padding with ptr math
			uint8_t above_label = label_image[height_labels * (y_label - 1) + x_label]; // offset for padding with ptr math

			if(pixel != 0)  { // Not a background pixel
				if(left_label == 0 && above_label == 0) {
					// Create a new label
					labels[num_labels].label_id = num_labels;
					labels[num_labels].parent = NULL; // Shouldn't be necessary but just make it explicit
					*curr_label = num_labels++;
				}
				else if (left_label == 0 && above_label != 0) {
					*curr_label = above_label;
				}
				else if(left_label != 0 && above_label == 0) {
					*curr_label = left_label;
				}
				else if(left_label != 0 && above_label != 0) {
					if(left_label == above_label) { // Same just pick one
						*curr_label = left_label;
					}
					else if(left_label < above_label) {
						*curr_label = left_label;
						labels[above_label].parent = &labels[left_label];
					}
					else {
						*curr_label = above_label;
						labels[left_label].parent = &labels[above_label];
					}
				}
			}
		}
	}

	// Second Pass for label cleanup
	for (int y = 0, y_label = 1; y < out_labeled->height; y++, y_label++) { // Start at one since we are padded for labels
		for (int x = 0, x_label = 1; x < out_labeled->width; x++, x_label++) {
			uint8_t equivalent_label = label_image[height_labels * y_label + x_label];
			label_node_t *node = &labels[equivalent_label];
			while(node != NULL) {
				equivalent_label = node->label_id;
				node = node->parent;
			}
			out_labeled->data[out_labeled->height * y + x] = equivalent_label;
			num_components = max(equivalent_label, num_components);
		}
	}
	ESP_LOGI(tag, "Found: %d components", num_components);

	// Free Memory
	free(label_image);

	// Return
	return num_components;
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
