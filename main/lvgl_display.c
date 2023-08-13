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

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"

#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    #if defined CONFIG_LV_USE_DEMO_WIDGETS
        #include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
    #elif defined CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
        #include "lv_examples/src/lv_demo_keypad_encoder/lv_demo_keypad_encoder.h"
    #elif defined CONFIG_LV_USE_DEMO_BENCHMARK
        #include "lv_examples/src/lv_demo_benchmark/lv_demo_benchmark.h"
    #elif defined CONFIG_LV_USE_DEMO_STRESS
        #include "lv_examples/src/lv_demo_stress/lv_demo_stress.h"
    #else
        #error "No demo application selected."
    #endif
#endif

/*********************
 *      DEFINES
 *********************/
#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

uint8_t data_to_send = 0x00;

static lv_obj_t * chart;
static lv_chart_series_t * ser1;

static lv_obj_t * scr1; 
static lv_obj_t * scr2; 

LV_IMG_DECLARE(AGHimage);

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void create_demo_application(void);

static void btn_event_cb(lv_obj_t * btn, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;
        data_to_send++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}

static void next_windows_btn_event_cb(lv_obj_t * btn, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        lv_scr_load(scr2);
    }
}

static void next_windows_btn1_event_cb(lv_obj_t * btn, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        lv_scr_load(scr1);
    }
}

void lvgl_displayInit(void){
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1); 
}

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
#endif

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    create_demo_application();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}
static void create_demo_application(void)
{
    //lv_demo_widgets();
    scr1 = lv_scr_act();

    lv_obj_set_style_local_bg_color(scr1, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	lv_obj_set_style_local_bg_opa(scr1, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);  

    lv_obj_t * btn = lv_btn_create(scr1, NULL);
    lv_obj_set_size(btn, 120, 70);
    lv_obj_set_style_local_bg_color(btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_MAGENTA);
    lv_obj_align(btn, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);
    lv_obj_set_event_cb(btn, btn_event_cb);

    lv_obj_t * label = lv_label_create(btn, NULL);
    lv_label_set_text(label, "Click me!");
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);


    lv_obj_t * next_windows_btn = lv_btn_create(scr1, NULL);
    lv_obj_set_size(next_windows_btn, 80, 50);
    lv_obj_set_style_local_bg_color(next_windows_btn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_MAGENTA);
    lv_obj_align(next_windows_btn, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -30);
    lv_obj_set_event_cb(next_windows_btn, next_windows_btn_event_cb);

    lv_obj_t * next_windows_label = lv_label_create(next_windows_btn, NULL);
    lv_label_set_text(next_windows_label, "NEXT");
    lv_obj_align(next_windows_label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -30);



    /*Create a chart*/
    chart = lv_chart_create(scr1, NULL);
    lv_obj_set_size(chart, 200, 130);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_range(chart, 0, 100);
    lv_chart_set_point_count(chart, 100);    
    /*Add two data series*/
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);


    /*Set the next points on 'ser1'*/
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);
    lv_chart_set_next(chart, ser1, 0);

    lv_chart_refresh(chart); /*Required after direct set*/


    scr2 = lv_obj_create(NULL, NULL);
    lv_obj_set_style_local_bg_color(scr2, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
	lv_obj_set_style_local_bg_opa(scr2, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER); 

    lv_obj_t * next_windows_btn1 = lv_btn_create(scr2, NULL);
    lv_obj_set_size(next_windows_btn1, 80, 50);
    lv_obj_set_style_local_bg_color(next_windows_btn1, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_MAGENTA);
    lv_obj_align(next_windows_btn1, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -30);
    lv_obj_set_event_cb(next_windows_btn1, next_windows_btn1_event_cb);

    lv_obj_t * next_windows_label1 = lv_label_create(next_windows_btn1, NULL);
    lv_label_set_text(next_windows_label1, "NEXT");
    lv_obj_align(next_windows_label1, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -30);

    lv_obj_t * img = lv_img_create(scr2, NULL);
    lv_img_set_src(img, &AGHimage);
    lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, -20);

}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

uint8_t lvgl_displayGetData(void){
    return data_to_send;
}

void lvgl_next_data_in_chart(void){
    lv_chart_set_next(chart, ser1, 69);
    // lv_chart_refresh(chart);
}