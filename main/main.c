#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "i2c/i2c.h"
#include "driver/adc.h"
#include "utils/utils.h"
#include "wifi/wifi.h"

#include <string.h>

#include "qrobot/qrobot.h"
//#include "sockets/socket_server.h"

// WARNING.  THIS FILE IS A MESS.  JUST A PLACEHOLDER FOR TESTING NEW FUNCTIONS/FEATURES AS I ADD THEM.  WILL WORKING CORE TEST FUNCTIONS SOON.

static char tag[] = "ROBOT_MAIN";

//void task_servoSweep(void *ignore) {
//    int bitSize         = 15;
//    int minValue        = 500;  // micro seconds (uS)
//    int maxValue        = 2500; // micro seconds (uS)
//    int sweepDuration   = 1500; // milliseconds (ms)
//    int duty            = (1<<bitSize) * minValue / 20000 ;
//   // int direction       = 1; // 1 = up, -1 = down
//    int valueChangeRate = 20; // msecs
//
//    ESP_LOGD(tag, ">> task_servo1");
//    ledc_timer_config_t timer_conf;
//    timer_conf.bit_num    = LEDC_TIMER_15_BIT;
//    timer_conf.freq_hz    = 50;
//    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
//    timer_conf.timer_num  = LEDC_TIMER_0;
//    ledc_timer_config(&timer_conf);
//
//    ledc_channel_config_t ledc_conf;
//    ledc_conf.channel    = LEDC_CHANNEL_0;
//    ledc_conf.duty       = duty;
//    ledc_conf.gpio_num   = 12;
//    ledc_conf.intr_type  = LEDC_INTR_DISABLE;
//    ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
//    ledc_conf.timer_sel  = LEDC_TIMER_0;
//    ledc_channel_config(&ledc_conf);
//
//    int changesPerSweep = sweepDuration / valueChangeRate;
//    int changeDelta = (maxValue-minValue) / changesPerSweep;
//    //int i;
//    ESP_LOGD(tag, "sweepDuration: %d seconds", sweepDuration);
//    ESP_LOGD(tag, "changesPerSweep: %d", changesPerSweep);
//    ESP_LOGD(tag, "changeDelta: %d", changeDelta);
//    ESP_LOGD(tag, "valueChangeRate: %d", valueChangeRate);
//
//    int test_duty = (32768) * 600 / 20000;
//
//
//    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, test_duty);
//                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//
//    while(1) {
//       /* for (i=700; i<2400; i+=20) {
//
//            duty = (1<<bitSize) * ((float)i / 20000);
//            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
//            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//
//            //ESP_LOGD(tag, "DUTY 1: %d", duty);
//            //ESP_LOGD(tag, "DUTY: %f", (duty / (32768.0)) * 20000);
//            vTaskDelay(30/portTICK_PERIOD_MS);
//        }
//        direction = -direction;
//        ESP_LOGD(tag, "Direction now %d", direction);*/
//
//        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (1<<bitSize) * (2400.0 / 20000));
//                   ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//
//                   vTaskDelay(5000/portTICK_PERIOD_MS);
//        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (1<<bitSize) * (600.0 / 20000));
//                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//        vTaskDelay(5000/portTICK_PERIOD_MS);
//    } // End loop forever
//
//    vTaskDelete(NULL);
//}
void app_main(void)
{
    nvs_flash_init();

    esp_err_t err = wifi_network_up();
    if(err != ESP_OK)
    {
    	ESP_LOGD(tag, "WI-FI disabled.\n");
    }

    // Init platform
    qrobot_init();

    // Start it up!
    qrobot_start();

    //xTaskCreate(&task_servoSweep, "servoTask", 2048, NULL, 5, NULL);

    /*
    ledc_timer_config_t ledc_timer = {
            //set timer counter bit number
            .bit_num = LEDC_TIMER_20_BIT,
            //set frequency of pwm
            .freq_hz = 2000,
            //timer mode,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            //timer index
            .timer_num = LEDC_TIMER_0
        };

        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel = {
            //set LEDC channel 0
            .channel = LEDC_CHANNEL_0,
            //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
            .duty = 1000,
            //GPIO number
            .gpio_num = GPIO_NUM_16,
            //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
            .intr_type = LEDC_INTR_FADE_END,
            //set LEDC mode, from ledc_mode_t
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            //set LEDC timer source, if different channel use one timer,
            //the frequency and bit_num of these channels should be the same
            .timer_sel = LEDC_TIMER_0
        };
        //set the configuration
        ledc_channel_config(&ledc_channel);

        //ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 4096);
      //  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

*/

    // ledc test
//    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
   // int i = 5000;
   // while (1) // Cannot Continue.  Loop Forever with delay a bit
   // {
   //             gpio_set_level(GPIO_NUM_16, 1);
             //   vTaskDelay(100 / portTICK_PERIOD_MS / 1000);

    //            gpio_set_level(GPIO_NUM_16, 0);
        // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1 * i++);
        //  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
   //     ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, (i));
        //i+=100;

   //     ESP_LOGI(tag, "Frequency: %d\n", i);
   //     vTaskDelay(500 / portTICK_PERIOD_MS);
   // }

}

