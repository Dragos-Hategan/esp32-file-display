#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

/**
 * @brief Load touch calibration data from NVS and indicate whether it was found.
 *
 * This function attempts to read previously saved touch calibration data from NVS.
 * It checks if valid calibration data exists and updates the provided flag accordingly.
 * If no calibration is found, the system will print a message indicating that a new
 * calibration is required.
 *
 * @param[out] calibration_found Pointer to a boolean that will be set to `true`
 * if valid calibration data was found, or `false` otherwise.
 *
 * @note This function relies on the touch calibration subsystem and should be called
 * after `init_nvs()` and touch driver initialization.
 *
 * @see touch_cal_load_nvs()
 */
void calibration_load_cal_data(bool *calibration_found);

/**
 * @brief Run touch screen calibration if needed or requested by the user.
 *
 * This helper checks whether a valid calibration was previously found and either:
 *  - runs the 5-point calibration immediately if @p calibration_found is false, or
 *  - asks the user (via a yes/no LVGL dialog) whether to run calibration again
 *    if @p calibration_found is true.
 *
 * Behavior:
 *  - If no calibration is found: calls run_5point_touch_calibration().
 *  - If calibration exists:
 *      - On "Yes": runs run_5point_touch_calibration().
 *      - On "No" : keeps the existing calibration (@ref s_cal) and clears
 *                 the active LVGL screen.
 *
 * UI objects used during the prompt are created and cleaned up internally.
 *
 * @param[in] calibration_found  True if a valid calibration was loaded from NVS,
 *                               false if no calibration data is available.
 *
 * @note User prompt behavior:
 *       - When the loader is enabled (see @ref calibration_set_show_loader),
 *         the Yes/No dialog auto-confirms "Yes" after 10 seconds.
 *       - When the loader is disabled (Settings flow), there is no countdown and
 *         the function waits for an explicit Yes/No.
 *
 * @return ESP_OK                  Calibration flow completed successfully
 *                                 (either run or skipped by user).
 * @return Other esp_err_t codes   If run_5point_touch_calibration() fails.
 */
esp_err_t calibration_run_cal(bool calibration_found);

/**
 * @brief Control whether the countdown loader is shown in the calibration dialog.
 *
 * Call with @c false when launching calibration from Settings to skip the loader,
 * and @c true for the startup flow to keep it visible.
 */
void calibration_set_show_loader(bool enable);

/**
 * @brief Apply current touch calibration to a raw (x,y) reading.
 *
 * If no valid calibration is available, the raw coordinates are clamped to the display bounds.
 * Otherwise, an affine transform is applied:
 * @code
 * x' = xA*x + xB*y + xC
 * y' = yA*x + yB*y + yC
 * @endcode
 *
 * @param raw_x Raw X from controller.
 * @param raw_y Raw Y from controller.
 * @param[out] out_point Output LVGL point (screen space).
 * @param xmax Screen width (max X, exclusive).
 * @param ymax Screen height (max Y, exclusive).
 */
void calibration_apply_cal_data(uint16_t raw_x, uint16_t raw_y, lv_point_t *out_point, int xmax, int ymax);

#ifdef __cplusplus
}
#endif
