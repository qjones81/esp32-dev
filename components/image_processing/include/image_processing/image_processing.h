
/*
 * image_processing.h
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

#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Spatial moment struct
 */
typedef struct {
    uint32_t m00; // Image Area Spatial Moment
    uint32_t m10; // Partial 1st order moment in X
    uint32_t m01; // Partial 1st order moment in Y
    uint32_t m11; // 1st order moment in X/Y
} image_moment_t;


/**
 * @brief Spatial moment struct
 */
typedef struct {
    uint32_t width;   	// Image width
    uint32_t height; 	// Image height
    uint8_t depth; 		// Image depth
    uint8_t *data; 		// Image data
} image_t;

/**
 * @brief Mask region  struct
 */
typedef struct {
    uint32_t top; 		// upper mask region cutoff
    uint32_t bottom; 	// lower mask region cutoff
    uint32_t left; 		// left mask region cutoff
    uint32_t right; 	// right mask region cutoff
} mask_region_t;


/**
 * @brief Calculate binary image moments, only up the 1st order for now.  Is a microcontroller after all...
 * @param in_image: Image to compute moments for.  Is not modified.
 * @return image_moment_t: Calculated moments
 */
image_moment_t calculate_moments(image_t *_inimage);

/**
 * @brief Create image utility helper function
 * @param width: Desired image width
 * @param height: Desired image height
 * @param depth: Desired image depth
 * @param out_image: Created image output ptr
 */
void create_image(uint32_t width, uint32_t height, uint8_t depth, image_t **out_image);

/**
 * @brief Mask image function.  This does NOT resize images.  Output image must match input image dimensions and depth
 * @param in_image: image to mask
 * @param mask_region_t: mask parameters for area of image to KEEP
 * @param value: value to mask with
 * @param out_mask: returned masked imag
 */
void mask_image(image_t *in_image, mask_region_t image_mask, uint32_t value, image_t *out_mask);

/**
 * @brief Create image utility helper function
 * @param in_image: Image to set
 * @param x: Pixel x location
 * @param y: Pixel y location
 * @param value: Desired pixel value
 */
void image_set_pixel(image_t *in_image, uint32_t x, uint32_t y, uint8_t value);

/**
 * @brief Print image to screen.  Need ascii art option
 * @param in_image: Input image.  Is not modified.
 */
void print_image(image_t *_inimage);

#ifdef __cplusplus
}
#endif


#endif /* IMAGE_PROCESSING_H_ */
