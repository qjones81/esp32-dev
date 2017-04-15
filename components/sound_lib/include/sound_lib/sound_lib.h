
/*
 * sound_lib.h
 *
 *  Created on: April 14, 2017
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

#ifndef SOUND_LIB_H_
#define SOUND_LIB_H_


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {
 C = 261,
 D = 294,
 E = 329,
 F = 349,
 G = 391,
 G_Sharp_4 = 415,
 A = 440,
 A_Sharp = 455,
 B = 466,
 C_5 = 523,
 C_Sharp = 554,
 D_5 = 587,
 D_Sharp = 622,
 E_5 = 659,
 F_5 = 698,
 F_Sharp = 740,
 G_5 = 784,
 G_Sharp = 830,
 A_5 = 880
} sound_note_t;


/**
 * @brief Sound Event struct enum
 */
typedef struct {
	uint32_t freq; 		// sound frequency
	uint32_t duration;	// sound duration
	uint32_t repeat_timeout; // sound timeout before repeat.  set to 0 if one time
} sound_event_t;


/**
 * @brief Sound device struct
 */
typedef struct {
    uint8_t volume; 		// 0-255 Volume
    gpio_num_t output_pin;  // Driver pin
    ledc_mode_t speed;			// Speed mode, LEDC_LOW_SPEED_MODE or LEDC_HIGH_SPEED_MODE
    ledc_channel_t channel; // ledc Channel
} sound_device_t;

/**
 * @brief Song struct enum
 */
typedef struct {
	uint32_t *notes; 	// notes to play
	uint32_t *times;	// duration of notes
	uint32_t note_count;		// How many notes?
} sound_song_t;

sound_device_t *sound_lib_create_device(gpio_num_t gpio_pin);

void sound_lib_init();

void sound_lib_start();

void sound_lib_stop();

void sound_lib_play_tone(sound_device_t *device, uint32_t freq, uint32_t duration, uint32_t repeat_time);

void sound_lib_play_theme(sound_device_t *device);

void sound_lib_play_march(sound_device_t *device);

#ifdef __cplusplus
}
#endif


#endif /* SOUND_LIB_H_ */
