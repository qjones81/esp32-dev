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
#include "spi/spi.h"
#include "mpu9250_dmp/mpu_9250.h"
#include "driver/adc.h"
#include "sharpir/sharpir.h"

//#include "amis_30543/amis_30543.h"
//#include "adns3080/adns3080.h"
//#include "stepper/stepper.h"
//#include "ar6115e/ar6115e.h"
#include "utils/utils.h"
#include "wifi/wifi.h"

#include <string.h>

#include "qrobot/qrobot.h"
//#include "sockets/socket_server.h"

// WARNING.  THIS FILE IS A MESS.  JUST A PLACEHOLDER FOR TESTING NEW FUNCTIONS/FEATURES AS I ADD THEM.  WILL WORKING CORE TEST FUNCTIONS SOON.

//esp_err_t event_handler(void *ctx, system_event_t *event)
//{
//    return ESP_OK;
//}

static char tag[] = "ROBOT_MAIN";

extern void task_stepper_control(void *ignore);
extern void task_stepper_control2(void *ignore);
extern void task_adns3080_reader_task(void *ignore);
extern void task_imu_reader_task(void *ignore);

//adns_3080_device_t adns_test_2;
//
//void task_adc_test(void *ignore) {
//    ESP_LOGD(tag, ">> task_adc");
//
//         adc1_config_width(ADC_WIDTH_12Bit);//config adc2 width
//         adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_11db);//config channel0 attenuation
//
//        adc1_config_width(ADC_WIDTH_12Bit);//config adc2 width
//         adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_11db);//config channel0 attenuation
//
//
//        while(1) {
//
//             int val=adc1_get_voltage(ADC1_CHANNEL_6);//get the  val of channel0
//            int val2=adc1_get_voltage(ADC1_CHANNEL_7);//get the  val of channel0
//            ESP_LOGI(tag, "Value: %d and %d", val, val2);
//
//            vTaskDelay(1000/portTICK_PERIOD_MS);
//        } // End loop forever
//
//    vTaskDelete(NULL);
//}
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

#if defined(CONFIG_WIFI_HOST_AP_MODE)

    wifi_config_t apConfig = {
           .ap = {
              .ssid=CONFIG_WIFI_HOST_AP_SSID,
              .ssid_len=0,
              .password=CONFIG_WIFI_HOST_AP_PASSWORD,
              .channel=CONFIG_WIFI_HOST_AP_CHANNEL,
              .authmode=WIFI_AUTH_OPEN,
              .ssid_hidden=0,
              .max_connection=CONFIG_WIFI_HOST_AP_MAX_CONNECTIONS,
              .beacon_interval=100
           }
        };

        // Start an AP
        wifi_init_start_ap(apConfig);
#elif WIFI_AP_COUNT != 0

        // TODO: Loop this for configured APs...
        // Wifi Config
        wifi_config_t sta_config;
        memset(&sta_config, 0, sizeof(sta_config));
        memcpy(sta_config.sta.ssid, CONFIG_WIFI_AP1_SSID,
                strlen(CONFIG_WIFI_AP1_SSID) + 1);
        memcpy(sta_config.sta.password, CONFIG_WIFI_AP1_PASSWORD,
                strlen(CONFIG_WIFI_AP1_PASSWORD) + 1);
        sta_config.sta.bssid_set = 0;

        // Init
        //wifi_init_connect_ap(sta_config);

#endif



    // TODO: Delay and Wait for IP address
    // POLL For Connection
   // delay_ms(5000);

    qrobot_init();

    qrobot_start();

/*
    // Pin 5, 1Mhz, Mode 0
    ESP_LOGI(tag, "Adding AMIS-30543 to SPI Bus...\n");
    ESP_ERROR_CHECK(spi_add_device(VSPI_CS, 1000000, 0, &amis_test.spi_device)); // TODO: Pass In Values
    ESP_LOGI(tag, "Successfully added AMIS-30543.\n");

    delay_ms(10);

    ESP_LOGI(tag, "Initializing AMIS-30543...\n");
    amis_30543_init(&amis_test); // Init

    amis_30543_reset(&amis_test); // Reset

    amis_30543_set_current(&amis_test, 1000); // Set current limit to 1000ma

    amis_30543_set_step_mode(&amis_test, MicroStep16); // Set microstepping to 1/16

    //amis_30543_pwm_frequency_double(&amis_test, true); // 45.6 khz

    amis_30543_enable_driver(&amis_test, true); // Enable motor output
*/

   // ESP_LOGI(tag, "AMIS-30543 Initialized.\n");

       // xTaskCreate(&task_adns3080_reader_task, "adns3080_task", 2048, NULL, 3, NULL);
  //  ESP_LOGI(tag, "Clock Frequency: %d.\n", APB_CLK_FREQ);
    //stepper_motor_device_t *motor_1_device = NULL;
    //stepper_motor_device_t *motor_2_device = NULL;

//    stepper_motor_device_config_t motor_1_cfg={
//            .step_pin=GPIO_NUM_16,
//            .dir_pin=GPIO_NUM_4,
//            .step_hold_delay=10
//        };
//
//    stepper_motor_device_config_t motor_2_cfg={
//            .step_pin=GPIO_NUM_15,
//          l  .dir_pin=GPIO_NUM_2,
//            .step_hold_delay=10
//        };
//
//    stepper_control_add_device(STEPPER_MOTOR_1, motor_1_cfg);
   // stepper_control_add_device(STEPPER_MOTOR_2, motor_2_cfg);
   // stepper_control_add_device(motor_2_cfg, motor_2_device);
   // stepper_control_start();

    //TickType_t ticks_sec = get_stepper_control_ticks_per_second();

    //ESP_LOGI(tag, "Ticks per Second: %d\n", ticks_sec);

   // TickType_t ticks_step = ticks_sec / 5333;
   // ESP_LOGI(tag, "Num Ticks For 5333 Steps per Second: %d\n", ticks_step);

   // ESP_LOGI(tag, "Address: 0%p\n", (void*)&motor_1_device);
    //motor_1_device->tick_target = ticks_step;
    //(stepper_motor_device_t *)motor_1_device->tick_target = ticks_step;

    //xTaskCreatePinnedToCore(&task_stepper_control, "stepper_control", 2048, NULL, 5, NULL, 1);

    // SAVE ME FOR AR 6115 INIT
    /*
    ar6115e_init(spektrum_handler);
    ar6115e_add_channel(THROTTLE, GPIO_NUM_27);
    ar6115e_add_channel(AILERON, GPIO_NUM_14);
    ar6115e_start();*/

/*
    // SAVE FOR IMU INIT!!!
       // Init i2c
       i2c_init(22, 21);

       // Make sure everything is up and ready
       delay_ms(250);

       // Begin IMU Setup
       ESP_LOGI(tag, "MPU9250 Initializing...\n");
       // Call imu.begin() to verify communication and initialize
       if (mpu_9250_begin() != INV_SUCCESS)
       {
           ESP_LOGE(tag, "Error Initializing MPU9250 IMU.\n");
         while (1) // Cannot Continue.  Loop Forever with delay a bit
         {
               delay(5000);
         }
       }

       inv_error_t error = mpu_9250_dmp_begin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                    DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                   CONFIG_MPU9250_DMP_RATE); // Set DMP FIFO rate to 200 Hz
       // DMP_FEATURE_LP_QUAT can also be used. It uses the
       // accelerometer in low-power mode to estimate quat's.
       // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
       // TOOD: Check for error
       ESP_LOGI(tag, "MPU9250 Initialized: %d\n", error);
*/




   // xTaskCreatePinnedToCore(&task_stepper_control2, "imuReadTask2", 2048, NULL, 5, NULL, 0);

    //xTaskCreate(&task_servoSweep, "servoTask", 2048, NULL, 5, NULL);

   // xTaskCreate(&task_adc_test, "adcTask", 2048, NULL, 5, NULL);

  //  stepper_control_set_speed(2000);

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

