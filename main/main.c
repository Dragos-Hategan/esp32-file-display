#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "file_manager.h"
#include "settings.h"
#include "sd_card.h"

enum {
    MAIN_TASK_STACK_WORDS       = 8 * 1024,
    MAIN_TASK_PRIORITY          = 1,
    MAIN_TASK_CORE              = 1,
    HEAP_STATS_TASK_STACK_WORDS = 4 * 1024,
    HEAP_STATS_TASK_PRIORITY    = 1,
    HEAP_STATS_TASK_CORE        = 0,
};

static const char *TAG = "app_main";
static const char *TAG_HEAP = "--- HEAP INFO---";

static const char *COLOR_RESET = "\033[0m";
static const char *COLOR_CYAN = "\033[1;36m";

static const TickType_t HEAP_STATS_INTERVAL_TICKS = pdMS_TO_TICKS(100);

/** 
 * @brief Task entry that bootstraps settings, SD card, and file manager. 
 */
static void main_task(void *arg);

/*
 * @brief Task that logs heap usage periodically. 
 */
static void heap_stats_task(void *arg);

/**
 * @brief Format a number with thousands separator as "xxx.xxx".
 *
 * @param out_number Buffer where the formatted string is written.
 * @param len        Length of @p out_number.
 * @param number     Value to format (in bytes).
 */
static void build_formatted_number(char* out_number, size_t len, size_t number);

void app_main(void)
{
    if (xTaskCreatePinnedToCore(main_task,
                                "main_task",
                                MAIN_TASK_STACK_WORDS,
                                NULL,
                                MAIN_TASK_PRIORITY,
                                NULL,
                                MAIN_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main task");
        return;
    }

#if CONFIG_APP_ENABLE_HEAP_STATS
    if (xTaskCreatePinnedToCore(heap_stats_task,
                                "heap_stats",
                                HEAP_STATS_TASK_STACK_WORDS,
                                NULL,
                                HEAP_STATS_TASK_PRIORITY,
                                NULL,
                                HEAP_STATS_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heap stats task");
    }
#endif
}

static void main_task(void *arg)
{
    ESP_LOGI(TAG, "\n\n ********** LVGL File Display ********** \n");

    settings_starting_routine();

    esp_err_t err = sd_card_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD init failed (%s). Starting retry flow.", esp_err_to_name(err));
        sd_card_retry_init();
    }

    err = file_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "file_manager_start failed: %s. Restarting...", esp_err_to_name(err));
        esp_restart();
    }

    settings_start_screensaver_timers();

    vTaskDelete(NULL);
}

static void heap_stats_task(void *arg)
{
    size_t last_free_heap = 0;
    size_t min_free_heap = SIZE_MAX;
    const TickType_t delay_ticks = (HEAP_STATS_INTERVAL_TICKS == 0) ? 1 : HEAP_STATS_INTERVAL_TICKS;
    
    while (1) {
        size_t free_heap = esp_get_free_heap_size();
        char formatted_free_heap[16];
        char formatted_min_heap[16];

        if (free_heap < min_free_heap) {
            min_free_heap = free_heap;
        }
        
        if (free_heap != last_free_heap) {
            last_free_heap = free_heap;

            build_formatted_number(formatted_free_heap, sizeof(formatted_free_heap), free_heap);
            build_formatted_number(formatted_min_heap, sizeof(formatted_min_heap), min_free_heap);

            ESP_LOGI(TAG_HEAP, "%s REAL_TIME: %sB available. Least_Heap_Ever: %sB%s.", COLOR_CYAN, formatted_free_heap, formatted_min_heap, COLOR_RESET);
        }

        vTaskDelay(delay_ticks);
    }
}

static void build_formatted_number(char* out_number, size_t len, size_t number)
{
    if (!out_number || len == 0) {
        return;
    }

    if (number >= 1000) {
        snprintf(out_number, len, "%zu.%03zu", number / 1000, number % 1000);
    } else {
        snprintf(out_number, len, "%zu", number);
    }
}
