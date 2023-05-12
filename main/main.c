/* LVGL Example project
 *
 * Basic project to test LVGL on ESP32 based projects.
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "main.h"
#include "esp_gatt_common_api.h"

#include "mpu6050.h"
#include "lvgl_display.h"
#include "ble.h"

#define GATTC_TAG "GATTC_DEMO"
#define MPU6050_TAG "MPU6050"

float accel_data[3];
float gyro_data[3];

/**********************
 *   APPLICATION MAIN
 **********************/
void app_main() {

    bleInit();

    if(mpu6050_setup() != ESP_OK){
        ESP_LOGE("MPU6050", "Failed to set up device! Check your wiring");
        return;
    }   
    lvgl_displayInit();

    while(1){
        vTaskDelay(500 / portTICK_PERIOD_MS);

        uint8_t data_to_send = lvgl_displayGetData();

        uint8_t char_new_valueA[4] = {data_to_send, 0x22, 0x33, 0x44};
        uint8_t char_new_valueB[4] = {data_to_send, 0x55, 0x66, 0x77};  
        vTaskDelay(500 / portTICK_PERIOD_MS);
        data_to_send++;

        ble_set_attr_value(sizeof(char_new_valueB), char_new_valueB);
        ble_send_indicate(sizeof(char_new_valueB), char_new_valueB);

        mpu6050_read_accel(accel_data);
        mpu6050_read_gyro(gyro_data);
        ESP_LOGI(MPU6050_TAG, "Accel X: %12.5f  Y: %12.5f  Z: %12.5f", accel_data[0], accel_data[1], accel_data[2]);
        ESP_LOGI(MPU6050_TAG, "Gyro  X: %12.5f  Y: %12.5f  Z: %12.5f", gyro_data[0], gyro_data[1], gyro_data[2]);
    }
}

