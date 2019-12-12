/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"

#define BLINK_GPIO  GPIO_NUM_22
#define TRIAC_SW  GPIO_NUM_23

#include "mydef.h"
#include "esp_chatter.h"
#include "esp_log.h"


void tp_example_touch_pad_init(void);
unsigned char tp_read(void);

static const char *TAG3 = "TCP->";

void app_main()
{


    printf("Starting!\n");
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIAC_SW, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIAC_SW, 0);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

#if 0
    initialise_wifi();
    wait_for_ip();
    tcp_client_task();
#endif
#if 1
    rosserial_setup();
    while(1) {
        rosserial_publish();
        //ESP_LOGI(TAG3, "msg sent");
        //vTaskDelay(100 / portTICK_PERIOD_MS);
    }
#endif

#if 0
    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    tp_example_touch_pad_init();


    while(1){
        _status=tp_read();
        if(_status==_STATUS_VOLP){
            gpio_set_level(BLINK_GPIO, 1);
            gpio_set_level(TRIAC_SW, 1);
        }else{
            if(_status==_STATUS_VOLM){
                gpio_set_level(BLINK_GPIO, 0);
                gpio_set_level(TRIAC_SW, 0);
            }
        }
    }
#endif

    //esp_restart();
}
