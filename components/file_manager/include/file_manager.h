#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#include "esp_err.h"
#include "sd_card.h"

typedef struct {
    const char *root_path;
    size_t max_items;
} file_manager_config_t;

/**
 * @brief Create the LVGL file-browser screen using the default SDSPI root.
 *
 * Initializes the internal navigation context with `CONFIG_SDSPI_MOUNT_POINT`,
 * builds the LVGL widgets, and loads the screen. The SD card must already
 * be mounted via `sd_card_init()`.
 *
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if `CONFIG_SDSPI_MOUNT_POINT` is NULL
 * - Errors propagated from `fs_nav_init`
 * - ESP_ERR_TIMEOUT if the LVGL display lock cannot be acquired
 */
esp_err_t file_manager_start(void);

/**
 * @brief Reset the header clock display to default (show button, hide label).
 */
void file_manager_reset_clock_display(void);

/**
 * @brief Mark the clock as user-set and refresh the header label/button state.
 */
void file_manager_on_time_set(void);

#ifdef __cplusplus
}
#endif
