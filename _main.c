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



#include "mydef.h"
#include "esp_chatter.h"
#include "esp_log.h"




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


#if 1
    rosserial_setup();
    while(1) {
        rosserial_publish();
        ESP_LOGI(TAG3, "topic sent");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
#endif


    //esp_restart();
}
