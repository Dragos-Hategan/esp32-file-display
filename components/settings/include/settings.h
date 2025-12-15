#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "esp_err.h"
#include "lvgl.h"

/**
 * @brief Execute the full startup flow (NVS, display, settings, SNTP, touch).
 *
 * Initializes NVS, starts the display and styles, loads persisted settings, then either shows
 * the splash screen with SNTP sync on power-on resets or resumes the saved SNTP state after
 * other resets. Finally, it brings up the touch driver and optionally runs calibration.
 */
void settings_starting_routine(void);

/**
 * @brief Open the Settings UI, creating it on first call and loading it into LVGL.
 *
 * @param return_screen Screen to switch back to when closing settings (nullable).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on bad return screen
 */
esp_err_t settings_open_settings(lv_obj_t *return_screen);

/**
 * @brief Show the Set Date&Time dialog.
 *
 * Builds the date&time picker overlay on the top layer. Caller provides the
 * screen to return focus to (used for context). The dialog uses the shared
 * settings context/state.
 *
 * @param return_screen Screen that owns the caller UI (can be NULL).
 */
void settings_show_date_time_dialog(lv_obj_t *return_screen);

 /**
 * @brief Show the SNTP refresh confirmation dialog.
 *
 * Stores the caller screen and builds a confirmation message box to refresh time via SNTP.
 *
 * @param return_screen Screen that owns the caller UI (can be NULL).
 */
void settings_show_sntp_dialog(lv_obj_t *return_screen);

/**
 * @brief Register callbacks for time set/reset events.
 *
 * @param on_time_set   Called after a successful Apply in the date&time dialog.
 * @param on_time_reset Called when settings are reset (to clear clock UI).
 *
 * Callbacks run in the LVGL/task context that owns the dialog; avoid heavy work.
 */
void settings_register_time_callbacks(void (*on_time_set)(void),
                                      void (*on_time_reset)(void));

/**
 * @brief persists current system time to NVS.
 */                                     
void settings_shutdown_save_time(void);   

/**
 * @brief Persist the SD card restart flag to NVS.
 */
void settings_persist_sd_card_restart(void);

/**
 * @brief Set the SD card restart flag in RAM (non-persistent).
 *
 * @param enable True to request restart after SD events; false to clear.
 */
void settings_set_sd_card_restart(bool enable);

/**
 * @brief Check if there is any valid value in NVS for system time.
 *
 * @return true if time is valid, false otherwise.
 */
bool settings_is_time_valid(void);

/**
 * @brief Fade brightness to saved_brightness over SETTINGS_UP_FADE_MS.
 */
void settings_fade_to_saved_brightness(void);

/**
 * @brief (Re)start dim/off timers according to current settings state.
 */
void settings_start_screensaver_timers(void);

/**
 * @brief Check if a wake fade (brightness ramp-up) is in progress.
 * @return true if currently fading up from 0 toward saved_brightness.
 */
bool settings_is_wake_in_progress(void);

/**
 * @brief Get the current active brightness percentage.
 * @return Brightness percent (0..100).
 */
int settings_get_active_brightness(void);

/**
 * @brief Get whether brightness/backlight is effectively on ( >0 ).
 * @return true if brightness > 0, false otherwise.
 */
bool settings_is_brightness_changing(void);

/**
 * @brief Get the stored preference for prompting calibration at startup.
 *
 * @return true if the prompt is enabled, false otherwise.
 */
bool settings_get_calibration_prompt_enabled(void);

/**
 * @brief Persist the preference for prompting calibration at startup.
 *
 * @param enable true to enable the prompt, false to disable.
 */
void settings_set_calibration_prompt_enabled(bool enable);

/**
 * @brief Get the state of the calibration wizard.
 *
 * @return true if the calibration is being run, false otherwise.
 */
bool settings_get_running_calibration(void);

/**
 * @brief Set the calibration running guard flag.
 *
 * @param enable true when calibration is in progress, false when idle.
 */
void settings_set_running_calibration(bool enable);

/**
 * @brief Get the dark theme flag.
 * 
 * @return true if the dark theme is on, false otherwise.* 
 */
bool settings_is_theme_dark(void);

/**
 * @brief Set the dark theme flag. 
 */
void settings_set_dark_theme_flag(bool is_dark);

/**
 * @brief Get the configured Access Point SSID string.
 *
 * @return Pointer to the null-terminated SSID stored in settings (do not free or modify).
 */
char* settings_get_ap_ssid(void);

/**
 * @brief Get the configured Access Point password string.
 *
 * @return Pointer to the null-terminated password stored in settings (do not free or modify).
 */
char* settings_get_ap_pwd(void);

/**
 * @brief Stop the screensaver off timer.
 */
void screensaver_off_stop(void);

/**
 * @brief Stop the screensaver dim timer.
 */
void screensaver_dim_stop(void);

#ifdef __cplusplus
}
#endif
