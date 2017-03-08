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
#include "amis_30543/amis_30543.h"
#include "adns3080/adns3080.h"
#include "stepper/stepper.h"
#include "ar6115e/ar6115e.h"
#include "utils/utils.h"
#include "wifi/wifi.h"
#include "sockets/socket_server.h"
//esp_err_t event_handler(void *ctx, system_event_t *event)
//{
//    return ESP_OK;
//}

socket_device_t *sock = NULL;

static char tag[] = "MPU9250_DMP_TEST";

extern void task_stepper_control(void *ignore);
extern void task_stepper_control2(void *ignore);
extern void task_ar6115e_read(void *ignore);
extern void task_adns3080_reader_task(void *ignore);
// TODO: Move this to another file/task. For testing
void spektrum_handler(pulse_event_t event)
{


    // TODO: Blah set speed
    int max_speed = 50000;

    // Aileron Left = 2000, Right = 1000
    // Throttle Down = 1000, Up = 2000

    long output_speed = map(event.pulse_width, 1100, 1900, -100, 100);

    ESP_LOGI(tag, "Hello: %d\n", (int32_t)output_speed);

        //ESP_LOGI(tag, "Hello: %f\n", max_speed * (output_speed / 100.0));
    stepper_control_set_speed(STEPPER_MOTOR_1, max_speed * (output_speed / 100.0));


}

void task_servoSweep(void *ignore) {


    while(1)
    {
        socket_server_send_data(sock, (uint8_t *)"HELLO", 5);
        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
//    int bitSize         = 15;
//    int minValue        = 500;  // micro seconds (uS)
//    int maxValue        = 2500; // micro seconds (uS)
//    int sweepDuration   = 1500; // milliseconds (ms)
//    int duty            = (1<<bitSize) * minValue / 20000 ;
//    int direction       = 1; // 1 = up, -1 = down
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
//    ledc_conf.gpio_num   = 15;
//    ledc_conf.intr_type  = LEDC_INTR_DISABLE;
//    ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
//    ledc_conf.timer_sel  = LEDC_TIMER_0;
//    ledc_channel_config(&ledc_conf);
//
//    int changesPerSweep = sweepDuration / valueChangeRate;
//    int changeDelta = (maxValue-minValue) / changesPerSweep;
//    int i;
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

    vTaskDelete(NULL);
}
void app_main(void)
{
   // gpio_pad_select_gpio(GPIO_NUM_4);
   // gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    nvs_flash_init();


 //   amis_30543_device_t amis_test;

  //  adns_3080_device_t adns_test;


    // Init SPI Bus
    ESP_LOGI(tag, "Initializing SPI Bus...\n");
    ESP_ERROR_CHECK(spi_init(VSPI_MOSI, VSPI_MISO, VSPI_CLK)); // TODO: Pass In Values
    ESP_LOGI(tag, "SPI Bus Initialized.\n");

    wifi_device_config_t wifi_config;

    //wifi_config.device_ssid_name = "C4SN";
    //wifi_config.device_ssid_password = "Panels2k13!";

    wifi_init_connect_ap(wifi_config);

    delay_ms(10000);

    sock = (socket_device_t *)malloc(sizeof(socket_device_t));
    sock->port = 9001;
    socket_server_init(sock);
    socket_server_start(sock);

    // Should wait on callback for when connected and have ip to start socket server


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
    //ESP_LOGI(tag, "int %d, long %d.\n", sizeof(int32_t), sizeof(long));

    // Pin 5, 2Mhz, Mode 0
     //   ESP_LOGI(tag, "Adding ADNS-3080 to SPI Bus...\n");
      //  ESP_ERROR_CHECK(spi_add_device(GPIO_NUM_26, 2000000, 3, &adns_test.spi_device)); // TODO: Pass In Values
      //  ESP_LOGI(tag, "Successfully added ADNS-3080.\n");


        // Setup Pins
      //  adns_test.reset_pin = 4;
      //  adns_test.cs_pin = GPIO_NUM_5;
      //  adns_test.x = 0;
      //  adns_test.y = 0;

     //   adns_3080_init(&adns_test);

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
//            .dir_pin=GPIO_NUM_2,
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


    //ar6115e_init(spektrum_handler);
    //ar6115e_add_channel(THROTTLE, GPIO_NUM_15);
    //ar6115e_add_channel(AILERON, GPIO_NUM_4);
    //xTaskCreatePinnedToCore(&task_ar6115e_read, "ar6115_task", 2048, NULL, 3, NULL, 0);

   // xTaskCreatePinnedToCore(&task_stepper_control2, "imuReadTask2", 2048, NULL, 5, NULL, 0);

    xTaskCreate(&task_servoSweep, "stepperTask", 2048, NULL, 5, NULL);

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

/*
    // Init i2c
    i2c_init(22, 21);

    // Make sure everything is up and ready
    delay_ms(500);


    // Begin IMU Setup


    // Call imu.begin() to verify communication and initialize
    if (mpu_9250_begin() != INV_SUCCESS)
    {
    	ESP_LOGE(tag, "Error Initializing MPU9250 IMU.\n");
      while (1) // Cannot Continue.  Loop Forever with delay a bit
      {
    	    delay(5000);
      }
    }

    ESP_LOGI(tag, "MPU9250 Initialized...\n");

    inv_error_t error = mpu_9250_dmp_begin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                200); // Set DMP FIFO rate to 200 Hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
    ESP_LOGI(tag, "MPU9250 Initialized: %d\n", error);

    // Init Sensor

 //   if(mpu_9250_init() != 0)
   // {
   // 	ESP_LOGE(tag, "Error Initializing MPU9250 IMU.\n");
   // 	while(1); // Cannot Continue.  Loop Forver
  //  }


    xTaskCreatePinnedToCore(&task_imu_reader_task, "imuReadTask", 2048, NULL, 5, NULL, 0);

    ESP_LOGI(tag, "Created IMU Polling Task");
    // Test ADC

   // printf("Hello World!\n");
//    tcpip_adapter_init();
//    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
//    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
//    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
//    wifi_config_t sta_config = {
//        .sta = {
//            .ssid = "access_point_name",
//            .password = "password",
//            .bssid_set = false
//        }
//    };
//    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
//    ESP_ERROR_CHECK( esp_wifi_start() );
//    ESP_ERROR_CHECK( esp_wifi_connect() );
//
//    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
//    int level = 0;
//    while (1) {
//        gpio_set_level(GPIO_NUM_27, level);
//        level = !level;
//        //printf("Blink\n");
  //      vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }*/


}

