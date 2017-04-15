
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

static uint8_t *label_image = NULL;
//static image_moment_t* blob_moments[32] = {NULL};
//static bool moment_alloc = false;
// Reference: http://aishack.in/tutorials/image-moments/
image_moment_t calculate_moments(image_t *in_image)
{
	image_moment_t ret_moments;
	if(in_image == NULL) {
		ESP_LOGE(tag, "calculate_moments: NULL Parameter passed for input image");
		return ret_moments;
	}

	memset(&ret_moments, 0, sizeof(image_moment_t));
	ret_moments.lower_bound = INT32_MAX;
	ret_moments.left_bound = INT32_MAX;
	ret_moments.upper_bound = INT32_MIN;
	ret_moments.right_bound = INT32_MIN;

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
				ret_moments.lower_bound = min(ret_moments.lower_bound, y);
				ret_moments.left_bound = min(ret_moments.left_bound, x);
				ret_moments.upper_bound = max(ret_moments.upper_bound, y);
				ret_moments.right_bound = max(ret_moments.right_bound, x);
				ret_moments.label_id = 1;
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

	static image_moment_t blob_moments[32]; // Not a huge fan of this (wastes memory a bit) but was having some allocation issues. Will fix this once switching over to C++
	//image_moment_t* blob_moments[32]; // Support 32 for now.  Needs to be a parameter of some sort
	memset(&blob_moments, 0, 32 * sizeof(image_moment_t));

    bool used_id[32] = { 0 }; // Set to zeros

	for (int y = 0; y < in_image->height; y++) {
        for (int x = 0; x < in_image->width; x++) {
            uint8_t blob_id = in_image->data[in_image->height * y + x];
            if(blob_id == 0) continue; // Background pixel

            if(used_id[blob_id] == false) {
            	blob_moments[blob_id].lower_bound = y;
            	blob_moments[blob_id].left_bound = x;
            	blob_moments[blob_id].upper_bound = y;
            	blob_moments[blob_id].right_bound = x;
            	blob_moments[blob_id].label_id = blob_id;

            	vector_add(moments_out, &blob_moments[blob_id]); // Add to output vector
            	used_id[blob_id] = true;
            }

			blob_moments[blob_id].m00 += 1; 		//sum(X,Y) of x^0*y^0
			blob_moments[blob_id].m10 += x; 		//sum(X,Y) of x^1*y^0
			blob_moments[blob_id].m01 += y; 		//sum(X,Y) of x^0*y^1
			blob_moments[blob_id].m11 += (x * y); 	//sum(X,Y) of x^1*y^1
			blob_moments[blob_id].lower_bound = min(blob_moments[blob_id].lower_bound, y);
			blob_moments[blob_id].left_bound = min(blob_moments[blob_id].left_bound, x);
			blob_moments[blob_id].upper_bound = max(blob_moments[blob_id].upper_bound, y);
			blob_moments[blob_id].right_bound = max(blob_moments[blob_id].right_bound, x);
        }
    }
}

uint32_t calculate_row_center(image_t *in_image, uint32_t blob_id, uint32_t y)
{
	if(in_image == NULL) {
		ESP_LOGE(tag, "calculate_row_center: NULL Parameter passed for input image");
		return;
	} else if(y > (in_image->width-1)) {
		ESP_LOGE(tag, "calculate_row_center: Index of out range.");
		return;
	}

	uint32_t row_accum = 0;
	uint32_t pixel_count = 0;
	for (int x = 0; x < in_image->width; x++) {
		if(blob_id != in_image->data[in_image->height * y + x]) {
			continue;
		}
		row_accum += x;
		pixel_count++;
	}
	return (uint32_t) ((float) row_accum / pixel_count);
}

void create_image(uint32_t width, uint32_t height, uint8_t depth, image_t **out_image)
{
	image_t *image = malloc(sizeof(image_t));
	image->width = width;
	image->height = height;
	image->depth = depth;
	image->data = malloc(image->width * image->height * depth);
	memset(image->data, 0, image->width * image->height * depth);
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
		ESP_LOGE(tag, "Mask/Image dimension mismatch: %d/%d and %d/%d and %d/%d", out_mask->depth, in_image->depth,out_mask->width, in_image->width,out_mask->height,in_image->height);
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

	// TODO: Offset or inset image.  For now hard card to inset
	uint32_t width_labels = in_image->width;// + 1; // Pad it by 1 pixel
	uint32_t height_labels = in_image->height;// + 1; // Pad it by 1 pixel

	// Using uint8 here, so 255 max labels.
	if(label_image == NULL) {
		label_image = malloc(width_labels * height_labels * sizeof(uint8_t));
	}

	// Create label component array with padding for edge cases
	label_node_t labels[32]; // 32 max labels for now

	// zero it all out.
	memset(label_image, 0, width_labels * height_labels * sizeof(uint8_t));
	memset(&labels, 0, 32 * sizeof(label_node_t));

	// Start at x,y = 1 for padding
	// First Pass
	for (int y = 1, y_label = 1; y < in_image->height-1; y++, y_label++) { // Start at one since we are padded for labels
		for (int x = 1, x_label = 1; x < in_image->width-1; x++, x_label++) {
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

	uint8_t max_label = 0;
	// Second Pass for label cleanup
	for (int y = 1, y_label = 1; y < out_labeled->height; y++, y_label++) { // Start at one since we are padded for labels
		for (int x = 1, x_label = 1; x < out_labeled->width; x++, x_label++) {
			uint8_t equivalent_label = label_image[height_labels * y_label + x_label];
			label_node_t *node = &labels[equivalent_label];
			while(node != NULL) {
				equivalent_label = node->label_id;
				node = node->parent;
			}
			out_labeled->data[out_labeled->height * y + x] = equivalent_label;
			max_label = max(max_label, equivalent_label);
		}
	}

	// TODO:  Needs to be way better.  Out of time for now and this works good enough.  But should be able to solve this in the main loops
	for(int i = 1; i <= max_label; i++) {
		if(labels[i].parent == NULL) {
			num_components++;
		}
	}
	//ESP_LOGI(tag, "Found: %d components", num_components);

	// Free Memory
	//free(label_image);

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
