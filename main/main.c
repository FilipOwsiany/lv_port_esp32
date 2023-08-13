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
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
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

#include "driver/uart.h"
#include "driver/gpio.h"
//#include "C:\Users\filov\Desktop\esp-idf\components\soc\include\hal\uart_types.h"

#define GATTC_TAG "GATTC_DEMO"
#define MPU6050_TAG "MPU6050"

float accel_data[3];
float gyro_data[3];
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
float roll, pitch, yaw;

double tau = 0.4;

/**********************
 *   APPLICATION MAIN
 **********************/

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_0)
#define RXD_PIN (GPIO_NUM_3)

#define UART UART_NUM_0

int num = 0;

void uartInit(void) 
{
    const uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void tx_task(void *arg)
{
	char* Txdata = (char*) malloc(512);

    for (int cal_int = 0; cal_int < 3000; cal_int ++){
        mpu6050_read_gyro(gyro_data);
        gyro_x_cal += gyro_data[0];
        gyro_y_cal += gyro_data[1];
        gyro_z_cal += gyro_data[2];
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    gyro_x_cal /= 3000;
    gyro_y_cal /= 3000;
    gyro_z_cal /= 3000;


    while (1) {
        esp_timer_get_time();
        mpu6050_read_accel(accel_data);
        mpu6050_read_gyro(gyro_data);

        gyro_data[0] -= gyro_x_cal;
        gyro_data[1] -= gyro_y_cal;
        gyro_data[2] -= gyro_z_cal;


        accAngleX = atan2(accel_data[1], accel_data[2]) * (180 / 3.1415);
        accAngleY = atan2(accel_data[0], accel_data[2]) * (180 / 3.1415);

        pitch = (tau)*(pitch + gyroAngleX) + (1-tau)*(accAngleX);
        roll = (tau)*(roll - gyroAngleY) + (1-tau)*(accAngleY);

        gyroAngleX = gyroAngleX + gyro_data[0] * 0.1; // deg/s * s = deg
        gyroAngleY = gyroAngleY + gyro_data[1] * 0.1;

        // yaw =  yaw + gyro_data[2] * (50 / portTICK_PERIOD_MS);
        // // Complementary filter - combine acceleromter and gyro angle values
        // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
        // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

        // gyroAngleX = roll;
        // gyroAngleY = pitch;

    	sprintf (Txdata, "%12.5f|%12.5f|%12.5f\n", pitch, accAngleX, gyroAngleX);
        uart_write_bytes(UART, Txdata, strlen(Txdata));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main() {

    bleInit();
    uartInit();

    if(mpu6050_setup() != ESP_OK){
        ESP_LOGE("MPU6050", "Failed to set up device! Check your wiring");
        return;
    }   
    lvgl_displayInit();

    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL);

    while(1){
        vTaskDelay(500 / portTICK_PERIOD_MS);

        uint8_t data_to_send = lvgl_displayGetData();

        uint8_t char_new_valueA[4] = {data_to_send, 0x22, 0x33, 0x44};
        uint8_t char_new_valueB[4] = {data_to_send, 0x55, 0x66, 0x77};  
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ble_set_attr_value(sizeof(char_new_valueB), char_new_valueB);
        ble_send_indicate(sizeof(char_new_valueB), char_new_valueB);
        lvgl_next_data_in_chart();

        // mpu6050_read_accel(accel_data);
        // mpu6050_read_gyro(gyro_data);
        // ESP_LOGI(MPU6050_TAG, "Accel X: %12.5f  Y: %12.5f  Z: %12.5f", accel_data[0], accel_data[1], accel_data[2]);
        // ESP_LOGI(MPU6050_TAG, "Gyro  X: %12.5f  Y: %12.5f  Z: %12.5f", gyro_data[0], gyro_data[1], gyro_data[2]);
    }
}

