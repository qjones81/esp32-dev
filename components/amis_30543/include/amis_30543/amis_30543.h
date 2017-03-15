
/*
 * amis_30543.h
 *
 *  Created on: Feb 15, 2017
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

#ifndef AMIS_30543_H_
#define AMIS_30543_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Struct with data register state and spi device for AMIS-30543 stepper driver
 */

typedef struct {
	uint8_t wr;     // Watchdog register
	uint8_t cr0;    // Control register 1
	uint8_t cr1;    // Control register 2
	uint8_t cr2;    // Control register 3
	uint8_t cr3;    // Control register 4
	spi_device_handle_t spi_device; // Device handle for spi communications

} amis_30543_device_t;

/**
 * @brief Microstepping modes to be passed to amis_30543_set_step_mode
 */
typedef enum {
	MicroStep128 = 128,
	MicroStep64 = 64,
	MicroStep32 = 32,
	MicroStep16 = 16,
	MicroStep8 = 8,
	MicroStep4 = 4,
	MicroStep2 = 2,
	MicroStep1 = 1,
	CompensatedHalf = MicroStep2,
	CompensatedFullTwoPhaseOn = MicroStep1,
	CompensatedFullOnePhaseOn = 200,
	UncompensatedHalf = 201,
	UncompensatedFull = 202,
} step_mode_t;

/**
 * @brief Bitmasks for the return value of amis_30543_read_non_latched_status_flags
 */
typedef enum  {
	OPENY = (1 << 2),
	OPENX = (1 << 3),
	WD = (1 << 4),
	CPFAIL = (1 << 5),
	TW = (1 << 6),
} non_latched_status_flag_t;

/**
 * @brief Bitmasks for the return value of amis_30543_read_latched_status_flags_and_clear
 */
typedef enum {
	OVCXNB = (1 << 3),
	OVCXNT = (1 << 4),
	OVCXPB = (1 << 5),
	OVCXPT = (1 << 6),
	TSD = (1 << 10),
	OVCYNB = (1 << 11),
	OVCYNT = (1 << 12),
	OVCYPB = (1 << 13),
	OVCYPT = (1 << 14),
} latched_status_flag_t;

/**
 * @brief Addresses of the control and status registers
 */
typedef enum  {
	WR = 0x0,
	CR0 = 0x1,
	CR1 = 0x2,
	CR2 = 0x3,
	CR3 = 0x9,
	SR0 = 0x4,
	SR1 = 0x5,
	SR2 = 0x6,
	SR3 = 0x7,
	SR4 = 0xA,
} reg_addr_t;

/**
 * @brief Initializes AMIS-30543
 * @param device: Pointer to AMIS-30543 device to initialize.  Modifies data in the structure.
 */
void amis_30543_init(amis_30543_device_t *device);

/**
 * @brief Reset AMIS-30543
 * @param device: Target AMIS-30543 device
 */
void amis_30543_reset(amis_30543_device_t *device);

/**
 * @brief Verify AMIS-30543 status
 * @param device: Target AMIS-30543 device
 * @return  State Valid = 1, State Invalid 0
 */
bool amis_30543_verify(amis_30543_device_t *device);

/**
 * @brief Write registers values to AMIS-30543
 * @param device: Target AMIS-30543 device
 */
void amis_30543_apply(amis_30543_device_t device);

/**
 * @brief Enable Motor Output on AMIS-30543
 * @param device: Target AMIS-30543 device
 * @param enable: Enable = 1, Disable = 0
 */
void amis_30543_enable_driver(amis_30543_device_t *device, bool enable);

/**
 * @brief Set Programmable peak current output for AMIS-30543
 * @param device: Target AMIS-30543 device
 * @param current_ma: Desired peak current for stepper motor (0 - 3000)
 */
void amis_30543_set_current(amis_30543_device_t *device, uint16_t current_ma);

/**
 * @brief Read the current microstepping position of the driven stepper motor
 * @param device: Target AMIS-30543 device
 * @return Current microstepping position of stepper motor (0 - 511)
 */
uint16_t amis_30543_read_position(amis_30543_device_t *device);

/**
 * @brief Set the DIRCTRL register value.
 * @param device:   Target AMIS-30543 device
 * @param ccw:      ccw = 1, cw = 0 [DIR = 0] or cw = 1, ccw = 0 [DIR = 1]
 */
void amis_30543_set_direction(amis_30543_device_t *device, bool ccw);

/**
 * @brief Get the cached value of the DIRCTRL register.  Does not perform SPI communication
 * @param device:   Target AMIS-30543 device
 * @return     ccw = 1, cw = 0 [DIR = 0] or cw = 1, ccw = 0 [DIR = 1]
 */
bool amis_30543_get_direction(amis_30543_device_t *device);

/**
 * @brief Configure stepping mode for stepper driver
 * @param device:   Target AMIS-30543 device
 * @param mode:   See definition of step_mode_t for valid types.  Sets 1/32 microstepping by default if invalid value passed
 */
void amis_30543_set_step_mode(amis_30543_device_t *device, uint8_t mode);

/**
 * @brief Set sleep bit for stepper driver.  SPI communication still possible.
 * @param device:   Target AMIS-30543 device
 * @param sleep_enable:  Enable = 1, Disable = 0
 */
void amis_30543_sleep(amis_30543_device_t *device, bool sleep_enable);

/**
 * @brief Set NXTP configuration bit.  Determines which way steps are triggered.
 * @param device:   Target AMIS-30543 device
 * @param rising_edge_bit:  Rising Edge = 0, Falling Edge = 0
 */
void amis_30543_step_on_rising_edge(amis_30543_device_t *device, bool rising_edge_bit);

/**
 * @brief Set PWMF configuration bit.  Determines frequency of PWM driver.  22.8kHz, or 45.6kHz
 * @param device:   Target AMIS-30543 device
 * @param enable_double_frequency:  45.6kHz = 1, 22.8kHz = 0
 */
void amis_30543_pwm_frequency_double(amis_30543_device_t *device, bool enable_double_frequency);

/**
 * @brief Set PWMJ configuration bit.  Determines if using PWM artificial jittering to help with motor control current
 * @param device:   Target AMIS-30543 device
 * @param jitter_enable:  Disable Jitter = 1, Enable Jitter = 0
 */
void amis_30543_pwm_jitter(amis_30543_device_t *device, bool jitter_enable);

/**
 * @brief Update PWM slope values.  Controls PWM rise and fall times.
 * @param device:   Target AMIS-30543 device
 * @param emc:  PWM motor control slope value (0 - 3).  Higher values lead to longer rise/fall times.
 */
void amis_30543_pwm_slope(amis_30543_device_t device, uint8_t emc);

/**
 * @brief Set SLA pin to have a gain value of 0.5 (default)
 * @param device:   Target AMIS-30543 device
 */
void amis_30543_set_sla_gain_default(amis_30543_device_t *device);

/**
 * @brief Set SLA pin to have a gain value of 0.25 (half)
 * @param device:   Target AMIS-30543 device
 */
void amis_30543_set_sla_gain_half(amis_30543_device_t *device);

/**
 * @brief Set SLAT bit to have a value of 0 (default).  This disables transparency on the SLA pin.  See DATASHEET.
 * @param device:   Target AMIS-30543 device
 * @param enable:   Enable Transparency = 1, Disable Transparency = 0
 */
void amis_30543_set_sla_transparency(amis_30543_device_t *device, bool enable);

/**
 * @brief Read non latched status flags from SR0 register.  See DATASHEET.
 * @param device:   Target AMIS-30543 device
 * @return          16-bit flag value.
 */
uint16_t amis_30543_read_non_latched_status_flags(amis_30543_device_t *device);

/**
 * @brief Read latched status flags from SR1 and SR2 registers.  See DATASHEET.
 * @param device:   Target AMIS-30543 device
 * @return          16-bit flag value.
 */
uint16_t amis_30543_read_latched_status_flags_and_clear(amis_30543_device_t *device);

/*! Reads a status register and returns the lower 7 bits (the parity bit is
 *  set to 0 in the return value). */

/**
 * @brief Reads a status register.
 * @param device:    Target AMIS-30543 device
 * @param address:   Status register address to read.  (SR0, SR1, SR2, SR3, SR4)
 * @return          Register value.  Lower 7 bits with parity bit set to 0.
 */
uint8_t amis_30543_read_status_reg(amis_30543_device_t *device, uint8_t address);


#ifdef __cplusplus
}
#endif

#endif /* AMIS_30543_H_ */
