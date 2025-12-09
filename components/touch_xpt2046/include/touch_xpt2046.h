#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "esp_lcd_touch.h"
#include "esp_err.h"
#include "lvgl.h"

#define TOUCH_X_MAX         320
#define TOUCH_Y_MAX         240
#define TOUCH_CAL_NVS_NS    "touch_cal"
#define TOUCH_CAL_NVS_KEY   "affine_v1"

/**
 * @brief Initialize the SPI bus and the XPT2046 touch driver.
 *
 * This function sets up the SPI bus used by the XPT2046 touch controller,
 * creates the `esp_lcd_panel_io` handle, and initializes the XPT2046 touch driver
 * through the `esp_lcd_touch` API.  
 * It also applies orientation flags such as axis swap and mirroring according
 * to the configuration macros (`TOUCH_SWAP_XY`, `TOUCH_MIRROR_X`, `TOUCH_MIRROR_Y`).
 *
 * If the function is called multiple times, initialization will only be performed once.
 * On success, a global handle to the touch driver is stored internally and can be
 * retrieved later using `touch_get_handle()`.
 *
 * @return
 * - ESP_OK on successful initialization  
 * - Error code from underlying ESP-IDF driver functions if initialization fails  
 *   (for example: @c ESP_ERR_NO_MEM, @c ESP_ERR_INVALID_ARG, etc.)
 *
 * @note The SPI bus is initialized with automatic DMA channel selection.
 * @note The XPT2046 interrupt pin is expected to be active-low.
 * @note The recommended SPI clock for XPT2046 is typically ≤ 2.5 MHz.
 *
 */
esp_err_t init_touch(void);

/**
 * @brief Registers the XPT2046 touch driver as an LVGL input device.
 *
 * This function locks the display, registers the touch driver with LVGL
 * using `register_touch_with_lvgl()`, and unlocks the display before returning.
 *
 * @return ESP_OK   Touch input device successfully registered.
 * @return ESP_FAIL Failed to register the touch device (NULL handle).
 */
esp_err_t register_touch_to_lvgl(void);

/**
 * @brief Get a pointer to the global touch input device created by lv_indev_create().
 *
 * @return lv_indev_t Current touch input device pointer or NULL if not initialized.
 */
lv_indev_t *touch_get_indev(void);

/**
 * @brief Get the global touch handle created by the XPT2046 driver.
 *
 * @return esp_lcd_touch_handle_t Current touch handle or NULL if not initialized.
 */
esp_lcd_touch_handle_t touch_get_handle(void);

/**
 * @brief Log a touch press with calibrated coordinates.
 *
 * @param x Calibrated X coordinate
 * @param y Calibrated Y coordinate
 */
void touch_log_press(uint16_t x, uint16_t y);

#ifdef __cplusplus
}
#endif
