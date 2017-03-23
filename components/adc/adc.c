
/*
 * adc.c
 *
 *  Created on: Feb 14, 2017
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

#include <esp_types.h>
#include <stdlib.h>
#include <ctype.h>
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/xtensa_api.h"

static const char *ADC_MODULE_TAG = "ADC_MODULE";

#define ADC_MODULE_CHECK(a, str, ret_val) if (!(a)) {                                \
    ESP_LOGE(ADC_MODULE_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
    return (ret_val);                                                              \
}

#define ADC_CHECK_FUNCTION_RET(fun_ret) if(fun_ret!=ESP_OK){\
    ESP_LOGE(ADC_MODULE_TAG,"%s:%d\n",__FUNCTION__,__LINE__);\
    return ESP_FAIL;\
}

gpio_num_t adc2ch2gpionum[10] = {4, 0, 2, 15, 13, 12, 14, 27, 25, 26};

portMUX_TYPE adc_spinlock = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t rtc_gpio_output_disable(gpio_num_t gpio_num)
{
    int rtc_gpio_num = rtc_gpio_desc[gpio_num].rtc_num;
    ADC_MODULE_CHECK(rtc_gpio_num != -1, "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    CLEAR_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TS_REG, (1 << (rtc_gpio_num + RTC_GPIO_ENABLE_W1TS_S)));
    SET_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TC_REG, (1 << ( rtc_gpio_num + RTC_GPIO_ENABLE_W1TC_S)));

    return ESP_OK;
}

static esp_err_t rtc_gpio_input_disable(gpio_num_t gpio_num)
{
    ADC_MODULE_CHECK(RTC_GPIO_IS_VALID_GPIO(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&adc_spinlock);
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].ie);
    portEXIT_CRITICAL(&adc_spinlock);

    return ESP_OK;
}


/*---------------------------------------------------------------
                    ADC
---------------------------------------------------------------*/
static esp_err_t adc_1_pad_get_io_num(adc1_channel_t channel, gpio_num_t *gpio_num)
{
    ADC_MODULE_CHECK(channel < ADC1_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);

    switch (channel) {
    case ADC1_CHANNEL_0:
        *gpio_num = 36;
        break;
    case ADC1_CHANNEL_1:
        *gpio_num = 37;
        break;
    case ADC1_CHANNEL_2:
        *gpio_num = 38;
        break;
    case ADC1_CHANNEL_3:
        *gpio_num = 39;
        break;
    case ADC1_CHANNEL_4:
        *gpio_num = 32;
        break;
    case ADC1_CHANNEL_5:
        *gpio_num = 33;
        break;
    case ADC1_CHANNEL_6:
        *gpio_num = 34;
        break;
    case ADC1_CHANNEL_7:
        *gpio_num = 35;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


static esp_err_t adc_2_pad_get_io_num(adc2_channel_t channel, gpio_num_t *gpio_num)
{
    ADC_MODULE_CHECK(channel < ADC2_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);

    switch (channel) {
    case ADC2_CHANNEL_0:
        *gpio_num = 4;
        break;
    case ADC2_CHANNEL_1:
        *gpio_num = 0;
        break;
    case ADC2_CHANNEL_2:
        *gpio_num = 2;
        break;
    case ADC2_CHANNEL_3:
        *gpio_num = 15;
        break;
    case ADC2_CHANNEL_4:
        *gpio_num = 13;
        break;
    case ADC2_CHANNEL_5:
        *gpio_num = 12;
        break;
    case ADC2_CHANNEL_6:
        *gpio_num = 14;
        break;
    case ADC2_CHANNEL_7:
        *gpio_num = 27;
    case ADC2_CHANNEL_8:
        *gpio_num = 25;
    case ADC2_CHANNEL_9:
        *gpio_num = 26;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}
static esp_err_t adc_1_pad_init(adc1_channel_t channel)
{
    gpio_num_t gpio_num = 0;
    ADC_CHECK_FUNCTION_RET(adc_1_pad_get_io_num(channel, &gpio_num));
    ADC_CHECK_FUNCTION_RET(rtc_gpio_init(gpio_num));
    ADC_CHECK_FUNCTION_RET(rtc_gpio_output_disable(gpio_num));
    ADC_CHECK_FUNCTION_RET(rtc_gpio_input_disable(gpio_num));
    ADC_CHECK_FUNCTION_RET(gpio_set_pull_mode(gpio_num, GPIO_FLOATING));

    return ESP_OK;
}

static esp_err_t adc_2_pad_init(adc2_channel_t channel)
{
    gpio_num_t gpio_num = 0;
    ADC_CHECK_FUNCTION_RET(adc_2_pad_get_io_num(channel, &gpio_num));
    ADC_CHECK_FUNCTION_RET(rtc_gpio_init(gpio_num));

    REG_CLR_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_TO_GPIO);
    REG_SET_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_MUX_SEL);
    REG_CLR_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_FUN_SEL);

    if (gpio_num == 25) {
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE); //stop dac1
    } else if (gpio_num == 26) {
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE); //stop dac2
    }

    ADC_CHECK_FUNCTION_RET(rtc_gpio_output_disable(gpio_num));
    ADC_CHECK_FUNCTION_RET(rtc_gpio_input_disable(gpio_num));
    ADC_CHECK_FUNCTION_RET(gpio_set_pull_mode(gpio_num, GPIO_FLOATING));

    return ESP_OK;
}

esp_err_t adc_1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten)
{
    ADC_MODULE_CHECK(channel < ADC1_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    ADC_MODULE_CHECK(atten <= ADC_ATTEN_11db, "ADC Atten Err", ESP_ERR_INVALID_ARG);
    adc_1_pad_init(channel);
    portENTER_CRITICAL(&adc_spinlock);
    SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, 3, atten, (channel * 2)); //SAR1_atten
    portEXIT_CRITICAL(&adc_spinlock);

    return ESP_OK;
}

esp_err_t adc_2_config_channel_atten(adc2_channel_t channel, adc_atten_t atten)
{
    ADC_MODULE_CHECK(channel < ADC2_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    ADC_MODULE_CHECK(atten <= ADC_ATTEN_11db, "ADC Atten Err", ESP_ERR_INVALID_ARG);
    adc_2_pad_init(channel);
    portENTER_CRITICAL(&adc_spinlock);
    SET_PERI_REG_BITS(SENS_SAR_ATTEN2_REG, 3, atten, (channel * 2)); //SAR1_atten
    portEXIT_CRITICAL(&adc_spinlock);

    return ESP_OK;
}

esp_err_t adc_1_config_width(adc_bits_width_t width_bit)
{
    portENTER_CRITICAL(&adc_spinlock);
    SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR1_BIT_WIDTH_V, width_bit, SENS_SAR1_BIT_WIDTH_S);  //SAR2_BIT_WIDTH[1:0]=0x3, SAR1_BIT_WIDTH[1:0]=0x3
    //Invert the adc value,the Output value is invert
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DATA_INV);
    //Set The adc sample width,invert adc value,must
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_BIT_V, width_bit, SENS_SAR1_SAMPLE_BIT_S); //digital sar1_bit_width[1:0]=3
    portEXIT_CRITICAL(&adc_spinlock);

    return ESP_OK;
}

esp_err_t adc_2_config_width(adc_bits_width_t width_bit)
{

    portENTER_CRITICAL(&adc_spinlock);
    //SAR2_BIT_WIDTH[1:0]=0x3, SAR1_BIT_WIDTH[1:0]=0x3
    SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR2_BIT_WIDTH_V, width_bit, SENS_SAR2_BIT_WIDTH_S);
    //Invert the adc value,the Output value is invert
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);
    //Set The adc sample width,invert adc value,must
    //digital sar2_bit_width[1:0]=3
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_BIT_V, width_bit, SENS_SAR2_SAMPLE_BIT_S);

    portEXIT_CRITICAL(&adc_spinlock);

    return ESP_OK;
}

int adc_1_get_voltage(adc1_channel_t channel)
{
    uint16_t adc_value;

    ADC_MODULE_CHECK(channel < ADC1_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&adc_spinlock);
    //Adc Controler is Rtc module,not ulp coprocessor
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 1, SENS_MEAS1_START_FORCE_S); //force pad mux and force start
    //Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 0, SENS_FORCE_XPD_SAR_S); //force XPD_SAR=0, use XPD_FSM
    //Disable Amp Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0
    //Open the ADC1 Data port Not ulp coprocessor
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 1, SENS_SAR1_EN_PAD_FORCE_S); //open the ADC1 data port
    //Select channel
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S); //pad enable
    SET_PERI_REG_BITS(SENS_SAR_MEAS_CTRL_REG, 0xfff, 0x0, SENS_AMP_RST_FB_FSM_S);  //[11:8]:short ref ground, [7:4]:short ref, [3:0]:rst fb
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0); //wait det_fsm==0
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 0, SENS_MEAS1_START_SAR_S); //start force 0
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 1, SENS_MEAS1_START_SAR_S); //start force 1
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0) {}; //read done
    adc_value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    portEXIT_CRITICAL(&adc_spinlock);

    return adc_value;
}

int adc_2_get_voltage(adc2_channel_t channel)
{
    uint16_t adc_value;

    ADC_MODULE_CHECK(channel < ADC2_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&adc_spinlock);
    //Adc Controler is Rtc module,not ulp coprocessor
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, 1, 1, SENS_MEAS2_START_FORCE_S); //force pad mux and force start
    //Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 0, SENS_FORCE_XPD_SAR_S); //force XPD_SAR=0, use XPD_FSM
    //Disable Amp Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0
    //Open the ADC2 Data port Not ulp coprocessor
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, 1, 1, SENS_SAR2_EN_PAD_FORCE_S); //open the ADC2 data port
    //Select channel
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S); //pad enable
    //[11:8]:short ref ground, [7:4]:short ref, [3:0]:rst fb
    SET_PERI_REG_BITS(SENS_SAR_MEAS_CTRL_REG, 0xfff, 0x0, SENS_AMP_RST_FB_FSM_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0); //wait det_fsm==0
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, 1, 0, SENS_MEAS2_START_SAR_S); //start force 0
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, 1, 1, SENS_MEAS2_START_SAR_S); //start force 1
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0) {}; //read done
    adc_value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    portEXIT_CRITICAL(&adc_spinlock);

    return adc_value;
}
