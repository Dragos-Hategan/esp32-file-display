#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "file_manager.h"
#include "settings.h"
#include "sd_card.h"

static char *TAG = "app_main";

#define LOG_MEM_INFO    (0)

static void main_task(void *arg)
{
    ESP_LOGI(TAG, "\n\n ********** LVGL File Display ********** \n");

    settings_starting_routine();

    esp_err_t err = init_sdspi();
    if (err != ESP_OK){
        retry_init_sdspi();
    }    

    esp_err_t fb_err = file_manager_start();
    if (fb_err != ESP_OK) {
        ESP_LOGE(TAG, "file_manager_start failed: %s (waiting for SD retry)", esp_err_to_name(fb_err));
        vTaskDelete(NULL);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreatePinnedToCore(main_task, "MyTask", 8 * 1024, NULL, 1, NULL, 0);
    size_t last_free_heap = 0;
    size_t min_free_heap = UINT_MAX;
    size_t max_free_heap = 0;
    
    while (1){
        size_t free_heap = esp_get_free_heap_size();

        if (free_heap < min_free_heap){
            min_free_heap = free_heap;
        }

        if (free_heap > max_free_heap){
            max_free_heap = free_heap;
        }

        if (free_heap != last_free_heap){
            printf("----- HEAP INFO ----- free=%u  min_free_heap_ever=%u max_free_heap_ever=%u ----- HEAP INFO ----- \n", free_heap, min_free_heap, max_free_heap);
            last_free_heap = free_heap;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
