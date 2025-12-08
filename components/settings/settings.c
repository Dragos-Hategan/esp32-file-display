#include "settings.h"

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "Domine_14.h"
#include "Domine_16.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "wifi.h"
#include "sntp_header.h"
#include "nvs.h"

#include "calibration_xpt2046.h"
#include "touch_xpt2046.h"
#include "styles.h"

#define SETTINGS_NVS_NS                     "settings"
#define SETTINGS_NVS_ROT_KEY                "rotation_step"
#define SETTINGS_NVS_BRIGHTNESS_KEY         "brightness_pct"
#define SETTINGS_NVS_MANUAL_RESTART_KEY     "manual_restart"
#define SETTINGS_NVS_SNTP_RESULT_KEY        "sntp_result"
#define SETTINGS_NVS_TIME_KEY               "time_epoch"
#define SETTINGS_NVS_VALID_TIME_FLAG_KEY    "is_time_valid"
#define SETTINGS_NVS_DIM_EN_KEY             "dim_en"
#define SETTINGS_NVS_DIM_TIME_KEY           "dim_time"
#define SETTINGS_NVS_DIM_LEVEL_KEY          "dim_level"
#define SETTINGS_NVS_OFF_EN_KEY             "off_en"
#define SETTINGS_NVS_OFF_TIME_KEY           "off_time"
#define SETTINGS_NVS_CALIB_PROMPT_KEY       "calib_prompt"
#define SETTINGS_NVS_THEME_KEY              "theme"

#define SETTINGS_ROTATION_STEPS          4
#define SETTINGS_MINIMUM_BRIGHTNESS      1   /**< Lowest brightness percentage to avoid black screen */

#define SETTINGS_DEFAULT_BRIGHTNESS             100
#define SETTINGS_DEFAULT_ROTATION_STEP          3
#define SETTINGS_DEFAULT_SCREEN_DIM             false
#define SETTINGS_DEFAULT_SCREEN_DIM_TIME        -1
#define SETTINGS_DEFAULT_SCREEN_DIM_LEVEL       -1
#define SETTINGS_DEFAULT_SCREEN_OFF             false
#define SETTINGS_DEFAULT_SCREEN_OFF_TIME        -1
#define SETTINGS_DEFAULT_CALI_PROMPT_ENABLE     true
#define SETTINGS_DEFAULT_DARK_THEME             true
#define SETTINGS_DEFAULT_TIME_VALID             false

#define SETTINGS_CALIBRATION_TASK_STACK  (10 * 1024)
#define SETTINGS_CALIBRATION_TASK_PRIO   (5)
#define SETTINGS_DIM_FADE_MS             500
#define SETTINGS_OFF_FADE_MS             500
#define SETTINGS_UP_FADE_MS              250

#define STR_HELPER(x)               #x
#define STR(x)                      STR_HELPER(x)

typedef struct{
    int screen_rotation_step;           /**< Current rotation step (0-3) applied to display */
    int saved_rotation_step;            /**< Last persisted rotation step */
    int brightness;                     /**< Current brightness percentage */
    int saved_brightness;               /**< Last persisted brightness percentage */
    int dt_month;
    int dt_day;
    int dt_year;
    int dt_hour;
    int dt_minute;
    int dt_second;
    bool time_valid;                    /**< True if a valid time was set/restored */
    bool screen_dim;
    int dim_time;
    int dim_level;
    bool screen_off;
    int off_time;
    bool calibration_prompt_enabled;    /**< True to ask for calibration at startup */
    bool running_calibration;
    bool dark_theme;
    bool sntp_success;
    bool manual_restart;
}settings_t;

typedef struct{
    bool active;                        /**< True while the settings screen is active */
    bool changing_brightness;           /**< True while changing values to the brightness slider */
    lv_obj_t *return_screen;            /**< Screen to return to on close */
    lv_obj_t *screen;                   /**< Root LVGL screen object */
    lv_obj_t *toolbar;                  /**< Toolbar container */
    lv_obj_t *brightness_label;         /**< Label showing current brightness percent */
    lv_obj_t *brightness_slider;        /**< Slider to pick brightness percent */
    lv_obj_t *restart_confirm_mbox;     /**< Active restart confirmation dialog (NULL when closed) */
    lv_obj_t *reset_confirm_mbox;       /**< Active reset confirmation dialog (NULL when closed) */
    lv_obj_t *theme_confirm_mbox;       /**< Active theme change confirmation dialog (NULL when closed) */
    lv_obj_t *datetime_overlay;         /**< Active date&time overlay (NULL when closed) */
    lv_obj_t *screensaver_overlay;      /**< Active screensaver overlay (NULL when closed) */
    lv_obj_t *dt_month_ta;              /**< Month input (MM) */
    lv_obj_t *dt_day_ta;                /**< Day input (DD) */
    lv_obj_t *dt_year_ta;               /**< Year input (YY) */
    lv_obj_t *dt_hour_ta;               /**< Hour input (HH) */
    lv_obj_t *dt_min_ta;                /**< Minute input (MM) */
    lv_obj_t *dt_keyboard;              /**< On-screen keyboard for date&time dialog */
    lv_obj_t *dt_dialog;                /**< Date&time dialog container */
    lv_obj_t *screensaver_dialog;       /**< Date&time dialog container */
    lv_obj_t *dt_row_time;              /**< Time row container */
    lv_obj_t *ss_dim_lbl;               /**< Screensaver dimming label */
    lv_obj_t *ss_dim_switch;            /**< Screensaver dimming on/off switch */
    lv_obj_t *ss_dim_after_lbl;         /**< Label: "Dim after" */
    lv_obj_t *ss_seconds_lbl;           /**< Label: "seconds" */
    lv_obj_t *ss_at_lbl;                /**< Label: "at" */
    lv_obj_t *ss_pct_lbl;               /**< Label: "%" */
    lv_obj_t *ss_dim_after_ta;          /**< Screensaver dim delay input (seconds) */
    lv_obj_t *ss_dim_pct_ta;            /**< Screensaver dim level input (%) */
    lv_obj_t *ss_off_lbl;               /**< Screensaver off label */
    lv_obj_t *ss_off_switch;            /**< Screensaver off on/off switch */
    lv_obj_t *ss_off_after_lbl;         /**< Label: "Turn screen off after" */
    lv_obj_t *ss_off_seconds_lbl;       /**< Label: "seconds." */
    lv_obj_t *ss_off_after_ta;          /**< Screensaver off delay input (seconds) */
    lv_obj_t *ss_keyboard;              /**< Screensaver numeric keyboard */
    settings_t settings;                /**< Information about the current session */
}settings_ctx_t;

static settings_ctx_t s_settings_ctx;
static const char *TAG = "settings";
static esp_timer_handle_t s_ss_off_timer = NULL;
static esp_timer_handle_t s_ss_dim_timer = NULL;
static esp_timer_handle_t s_fade_timer = NULL;
static int s_fade_target = 0;
static int s_fade_steps_left = 0;
static int s_fade_direction = 0;
static bool s_wake_in_progress = false;

/**
 * @brief Build the settings screen (header + scrollable settings list).
 *
 * Creates the root screen, toolbar (Back/About), and the scrollable list of settings.
 *
 * @param ctx Active settings context.
 */
static void settings_build_screen(settings_ctx_t *ctx);

/**
 * @brief Show the About overlay with setting descriptions.
 *
 * Opens a modal overlay on the top layer with descriptive labels and an OK button.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_on_about(lv_event_t *e);

/**
 * @brief Close handler for the About overlay.
 *
 * Deletes the overlay provided via event user data.
 *
 * @param e LVGL event (CLICKED) with user data = overlay obj.
 */
static void settings_on_about_close(lv_event_t *e);

/**
 * @brief Update brightness level when the slider value changes.
 *
 * Refreshes the brightness label and drives the backlight to the new level.
 *
 * @param e LVGL event (VALUE_CHANGED) with user data = settings_ctx_t*.
 */
static void settings_on_brightness_changed(lv_event_t *e);

/**
 * @brief Back button handler for the settings screen.
 *
 * Retrieves the settings context from event user data and closes the settings UI.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_on_back(lv_event_t *e);

/**
 * @brief Close the settings screen and restore the previous screen.
 *
 * Persists brightness/rotation changes to NVS when needed, marks the context inactive,
 * and loads @ref settings_ctx_t::return_screen if set.
 *
 * @param ctx Active settings context.
 */
static void settings_close(settings_ctx_t *ctx);

/**
 * @brief Show a restart confirmation overlay.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_restart(lv_event_t *e);

/**
 * @brief Handler for confirming restart from the overlay.
 *
 * Persists pending brightness/rotation changes and then triggers a restart.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_restart_confirm(lv_event_t *e);

/**
 * @brief Close the restart overlay without restarting.
 *
 * Dismisses the confirmation dialog and clears the stored pointer.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_close_restart(lv_event_t *e);

/**
 * @brief Show a reset confirmation overlay.
 *
 * Creates a confirmation dialog for resetting settings and stores it in the context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_reset(lv_event_t *e);

/**
 * @brief Confirm reset, restore defaults, and reinitialize settings.
 *
 * Resets all configurable settings to defaults, persists them, reinitializes runtime state,
 * and closes the reset dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_reset_confirm(lv_event_t *e);

/**
 * @brief Toggle theme (light/dark) and restart system.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_toggle_theme(lv_event_t *e);

/**
 * @brief Saves theme and other the rest of the custom flags to NVS and restarts.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_theme_confirm_yes(lv_event_t *e);

/**
 * @brief Closes the theme change dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_theme_confirm_no(lv_event_t *e);

/**
 * @brief Close the reset confirmation overlay without applying changes.
 *
 * Dismisses the reset dialog and clears its pointer from the context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_close_reset(lv_event_t *e);

/**
 * @brief Launch touch calibration from Settings (async).
 *
 * Cleans the current settings screen, marks the context inactive, and spawns a
 * FreeRTOS task to run the calibration flow without blocking the LVGL handler.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_run_calibration(lv_event_t *e);

/**
 * @brief Open the screensaver dialog from settings.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_screensaver(lv_event_t *e);

/**
 * @brief Build the screensaver dialog UI and attach it to the top layer.
 *
 * @param ctx Active settings context.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ctx is NULL.
 */
static esp_err_t settings_build_screensaver_dialog(settings_ctx_t *ctx);

/**
 * @brief Apply screensaver settings from the dialog (Dim/Off).
 *
 * Parses inputs, validates, persists to NVS, (re)starts timers and closes dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_apply_screensaver(lv_event_t *e);

/**
 * @brief Close the screensaver dialog without applying changes.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_close_screensaver(lv_event_t *e);

/**
 * @brief Background task to run touch calibration and restore UI state.
 *
 * Temporarily forces default rotation for calibration, runs @ref run_calibration,
 * restores the previous rotation, and reopens the settings screen.
 *
 * @param param settings_ctx_t* passed from @ref settings_run_calibration.
 */
static void settings_calibration_task(void *param);

/**
 * @brief Clear cached LVGL object pointers in the settings context.
 *
 * Call after deleting/cleaning the settings UI to avoid dangling pointers being
 * used by async callbacks (e.g., brightness updates).
 */
static void settings_clear_ui_refs(settings_ctx_t *ctx);

/**
 * @brief Initialize the Non-Volatile Storage (NVS) flash partition.
 *
 * This function initializes the NVS used for storing persistent configuration
 * and calibration data.  
 * If the NVS partition is full, corrupted, or created with an incompatible SDK version,
 * it will be erased and reinitialized automatically.
 *
 * @return
 * - ESP_OK on successful initialization  
 * - ESP_ERR_NVS_NO_FREE_PAGES if the partition had to be erased  
 * - ESP_ERR_NVS_NEW_VERSION_FOUND if a version mismatch was detected  
 * - Other error codes from @ref nvs_flash_init() if initialization fails
 *
 * @note This function should be called before performing any NVS read/write operations.
 */
static esp_err_t init_nvs(void);

/**
 * @brief Starts the BSP display subsystem and reports the initialization result.
 *
 * This function calls `bsp_display_start()` and converts its boolean return
 * value into an `esp_err_t`.  
 * 
 * @return ESP_OK      Display successfully initialized.
 * @return ESP_FAIL    Display failed to initialize.
 */
static esp_err_t bsp_display_start_result(void);

/**
 * @brief Apply the Domine 14 font as the app-wide default LVGL theme font.
 *
 * @param[in] lock_display True when calling from non-LVGL context (takes display lock);
 *                         False when already in LVGL task (no extra lock).
 */
static void apply_default_font_theme(bool lock_display);

/**
 * @brief Show connection result message after a software restart.
 *
 * Uses the persisted SNTP result to decide whether to display success/failure.
 */
static void sntp_restart_flow(void);

/**
 * @brief Show the SNTP connection message when a manual restart is requested.
 */
static void manual_restart_flow(void);

/**
 * @brief Handle SNTP startup flow depending on reset reason.
 *
 * @param reason Current reset reason (hard vs software/manual restart).
 */
static void sntp_startup(esp_reset_reason_t reason);

/**
 * @brief Apply the desired timezone to the C library clock.
 */
static void apply_timezone(void);

/**
 * @brief Apply the current rotation step to the active LVGL display.
 *
 * Maps @ref s_settings_ctx.settings.screen_rotation_step to an LVGL display rotation and sets it,
 * clamping to a valid state if needed. Logs a warning when no display exists.
 *
 * @param[in] lock_display True when calling from non-LVGL context (takes display lock);
 *                         False when already in LVGL task (no extra lock).
 */
static void apply_rotation_to_display(bool lock_display);

/**
 * @brief Load persisted rotation step from NVS into @ref s_settings_ctx.settings.
 *
 * Reads @ref SETTINGS_NVS_ROT_KEY from @ref SETTINGS_NVS_NS; keeps the
 * default if the key or namespace is missing or out of range.
 */
static void load_rotation_from_nvs(void);

/**
 * @brief Persist current rotation step to NVS.
 *
 * Writes @ref s_settings_ctx.settings.screen_rotation_step to @ref SETTINGS_NVS_ROT_KEY inside
 * @ref SETTINGS_NVS_NS, logging warnings on failure but not aborting flow.
 */
static void persist_rotation_to_nvs(void);

/**
 * @brief Load persisted brightness percent from NVS (defaults to 100 if missing).
 */
static void load_brightness_from_nvs(void);

/**
 * @brief Persist current brightness percent to NVS.
 */
static void persist_brightness_to_nvs(void);

/**
 * @brief Load screensaver dim/off settings from NVS (defaults: disabled, -1 values).
 */
static void load_screensaver_from_nvs(void);

/**
 * @brief Persist screensaver dim/off settings to NVS.
 */
static void persist_screensaver_to_nvs(void);

/**
 * @brief Load calibration prompt preference from NVS (defaults to enabled).
 */
static void load_calibration_prompt_from_nvs(void);

/**
 * @brief Persist calibration prompt preference to NVS.
 */
static void persist_calibration_prompt_to_nvs(void);

/**
 * @brief Initialize runtime settings defaults.
 *
 * Seeds defaults, loads persisted brightness/rotation, applies backlight level,
 * and updates the LVGL display rotation.
 */
static void init_settings(void);

/**
 * @brief Seed in-memory settings with compile-time defaults.
 *
 * Resets runtime flags and stored values (brightness, rotation, dim/off timers,
 * calibration prompt, theme, time-valid flag) to their default constants.
 * Call before loading persisted settings so defaults act as fallbacks.
 */
static void init_default_configs(void);

/**
 * @brief Load persisted settings from NVS and apply to runtime state.
 *
 * Reads brightness, rotation, screensaver, calibration prompt, theme, and time
 * from NVS; then applies rotation to the display. Defaults set by
 * @ref init_default_configs act as fallbacks when keys are missing/invalid.
 */
static void load_last_saved_configs(void);

/**
 * @brief Rotate the display in 90-degree increments (0 -> 90 -> 180 -> 270 -> 0).
 * 
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_rotate_screen(lv_event_t *e);

/**
 * @brief Build the date&time dialog overlay and wire its events.
 *
 * Destroys any existing dialog, constructs the overlay, text areas, action buttons,
 * and numeric keyboard, and stores pointers in the shared settings context.
 *
 * @param ctx Active settings context (must be non-NULL).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ctx is NULL.
 */
static esp_err_t settings_build_date_time_dialog(settings_ctx_t *ctx);

/**
 * @brief Settings button handler to open the date&time dialog.
 *
 * Uses @ref settings_build_date_time_dialog() to display the picker.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_set_date_time(lv_event_t *e);

/**
 * @brief Build the splash screen shown at startup.
 */
static void build_splash_screen(void);

/**
 * @brief Build the connection result screen after Wi-Fi/SNTP attempt.
 *
 * @param result ESP_OK when the time set succesfully; other codes indicate failure.
 */
static void build_connection_result_message(esp_err_t result);

/**
 * @brief Build the screen shown while connecting to Wi-Fi/SNTP.
 */
static void build_connecting_screen(void);

/**
 * @brief Display the splash screen with backlight fade timing.
 */
static void show_splash_screen(void);

/**
 * @brief Display the "connecting" message.
 */
static void show_connecting_message(void);

/**
 * @brief Display the connection result message.
 *
 * @param result ESP_OK when Wi-Fi/SNTP succeeded; other codes for failure.
 */
static void show_connection_result_message(esp_err_t result);

/**
 * @brief Capture current system time into settings and persist to NVS.
 */
static void save_time_data(void);

/**
 * @brief Apply handler for the date&time dialog.
 *
 * Parses and validates all fields, writes them into ctx->settings, and closes the dialog.
 * Shows an "Incorrect Input" message box when validation fails.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_apply_date_time(lv_event_t *e);

/**
 * @brief Close handler for the date&time dialog (Cancel or overlay tap).
 *
 * Deletes the overlay and clears dialog-related pointers in the settings context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_close_set_date_time(lv_event_t *e);

/**
 * @brief OK handler for the invalid-input message box.
 *
 * @param e LVGL event (CLICKED) with user data = msgbox obj.
 */
static void settings_invalid_ok(lv_event_t *e);

/**
 * @brief Show a simple "Incorrect Input" message box.
 */
static void settings_show_invalid_input(void);

/**
 * @brief Parse an integer from text and clamp to a [min, max] range.
 *
 * @param txt Input string.
 * @param min Minimum accepted value.
 * @param max Maximum accepted value.
 * @param out_val Parsed integer on success.
 * @return true if parse and range check succeed; false otherwise.
 */
static bool settings_parse_int_range(const char *txt, int min, int max, int *out_val);

/**
 * @brief Textarea focus/click handler to prep the keyboard and clear placeholders.
 *
 * @param e LVGL event (FOCUSED/CLICKED) with user data = settings_ctx_t*.
 */
static void settings_on_dt_textarea_focus(lv_event_t *e);

/**
 * @brief Overlay/dialog tap handler to hide the keyboard when tapping outside fields.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_on_dt_background_tap(lv_event_t *e);

/**
 * @brief Keyboard CANCEL/READY handler to hide the keyboard.
 *
 * @param e LVGL event (CANCEL/READY) with user data = settings_ctx_t*.
 */
static void settings_on_dt_keyboard_event(lv_event_t *e);

/**
 * @brief Textarea defocus handler to restore placeholders when left empty.
 *
 * @param e LVGL event (DEFOCUSED) with user data = settings_ctx_t*.
 */
static void settings_on_dt_textarea_defocus(lv_event_t *e);

/**
 * @brief Scroll the dialog so the given field (or its row) stays visible.
 *
 * @param ctx Settings context.
 * @param ta  Target textarea to bring into view.
 */
static void settings_scroll_field_into_view(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Hide the date&time keyboard and detach it from any textarea.
 *
 * @param ctx Settings context.
 */
static void settings_hide_dt_keyboard(settings_ctx_t *ctx);

/**
 * @brief Re-align the date&time dialog based on which textarea is active.
 * @param ctx Settings context.
 * @param ta  Active textarea (NULL to reset to default position).
 */
static void settings_realign_dt_dialog(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Focus/click handler for screensaver numeric fields (dim delay/percent).
 * @param e LVGL event with user data = settings_ctx_t*.
 */
static void settings_on_ss_textarea_focus(lv_event_t *e);

/**
 * @brief Toggle handler for screensaver dim switch to enable/disable related fields.
 * @param e LVGL event (VALUE_CHANGED) with user data = settings_ctx_t*.
 */
static void settings_on_dim_switch_changed(lv_event_t *e);

/**
 * @brief Apply enabled/disabled state to dimming controls based on switch state.
 * @param ctx Settings context.
 * @param enabled True to enable fields, false to disable.
 */
static void settings_update_dim_controls_enabled(settings_ctx_t *ctx, bool enabled);

/**
 * @brief Hide the screensaver keyboard and detach it from any textarea.
 * @param ctx Settings context.
 */
static void settings_hide_ss_keyboard(settings_ctx_t *ctx);

/**
 * @brief Re-align the screensaver dialog based on which textarea is active.
 * @param ctx Settings context.
 * @param ta  Active textarea (NULL to reset to default position).
 */
static void settings_realign_screensaver_dialog(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Overlay/dialog tap handler for screensaver dialog.
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void settings_on_ss_background_tap(lv_event_t *e);

/**
 * @brief Screensaver keyboard CANCEL/READY handler.
 * @param e LVGL event (CANCEL/READY) with user data = settings_ctx_t*.
 */
static void settings_on_ss_keyboard_event(lv_event_t *e);

/**
 * @brief Start dim timer; delays and target level for screensaver dim.
 * @param seconds Delay in seconds before dim.
 * @param level_pct Target dim level percent (0..100).
 */
static void screensaver_dim_start(int seconds, int level_pct);

/**
 * @brief Stop the screensaver dim timer.
 */
static void screensaver_dim_stop(void);

/**
 * @brief Start the screensaver off timer.
 * @param seconds Delay in seconds before turning screen off.
 */
static void screensaver_off_start(int seconds);

/**
 * @brief Stop the screensaver off timer.
 */
static void screensaver_off_stop(void);

/**
 * @brief Toggle handler for screensaver off switch to enable/disable related fields.
 * @param e LVGL event (VALUE_CHANGED) with user data = settings_ctx_t*.
 */
static void settings_on_off_switch_changed(lv_event_t *e);

/**
 * @brief Apply enabled/disabled state to off controls based on switch state.
 * @param ctx Settings context.
 * @param enabled True to enable, false to disable.
 */
static void settings_update_off_controls_enabled(settings_ctx_t *ctx, bool enabled);

/**
 * @brief esp_timer callback for delayed screen off.
 * @param arg Unused.
 */
static void settings_off_timer_cb(void *arg);

/**
 * @brief esp_timer callback for delayed screen dim.
 * @param arg Unused.
 */
static void settings_dim_timer_cb(void *arg);

/**
 * @brief Helper to animate brightness to a target percentage over a duration using esp_timer.
 * @param target_pct Target brightness percent.
 * @param duration_ms Fade duration in milliseconds.
 */
static void settings_fade_brightness(int target_pct, uint32_t duration_ms);

/**
 * @brief Fade step timer callback for brightness animation.
 * @param arg Unused.
 */
static void settings_fade_step_cb(void *arg);

/**
 * @brief Sync brightness slider/label to the current brightness value.
 * @param ctx Settings context.
 * @param val Brightness percent to display.
 */
static void settings_sync_brightness_ui(settings_ctx_t *ctx, int val);

/**
 * @brief Async worker to update brightness UI elements.
 * @param arg Brightness value (cast from uintptr_t).
 */
static void settings_sync_brightness_ui_async(void *arg);

/**
 * @brief Utility to check if an object is a descendant of another.
 *
 * @param obj            Candidate child object.
 * @param maybe_ancestor Candidate ancestor object.
 * @return true if obj is a descendant (or same) as maybe_ancestor; false otherwise.
 */
static bool settings_is_descendant(lv_obj_t *obj, lv_obj_t *maybe_ancestor);

/**
 * @brief Validate a date considering leap years and month lengths.
 *
 * @param year_full Full year (e.g., 2025).
 * @param month     Month 1-12.
 * @param day       Day (1..n based on month and leap year).
 * @return true if the date is valid, false otherwise.
 */
static bool settings_is_valid_date(int year_full, int month, int day);

/**
 * @brief Notify registered listeners that time was set via dialog Apply.
 */
static void settings_notify_time_set(void);

/**
 * @brief Notify registered listeners that time was reset via settings reset.
 */
static void settings_notify_time_reset(void);

/**
 * @brief Restore system time from NVS only after a software reset; clear otherwise.
 */
static void load_time_from_nvs(void);

/**
 * @brief Persist the given epoch seconds to NVS.
 *
 * @param epoch Epoch seconds to store.
 */
static void settings_persist_time_to_nvs(time_t epoch);

/**
 * @brief Erase the stored time key from NVS.
 */
static void settings_clear_time_in_nvs(void);

/**
 * @brief Remove the stored "time valid" flag from NVS.
 */
static void settings_clear_valid_time_flag_in_nvs(void);

/**
 * @brief Persist the "time valid" flag to NVS.
 *
 * @param is_valid True if system time is considered valid; false otherwise.
 */
static void settings_persist_valid_time_flag_to_nvs(bool is_valid);

/**
 * @brief Persist the last SNTP synchronization result to NVS.
 *
 * @param is_successful True if SNTP succeeded; false otherwise.
 */
static void settings_persist_sntp_result(bool is_successful);

/**
 * @brief Load the last SNTP synchronization result from NVS.
 */
static void load_sntp_result_from_nvs(void);

/**
 * @brief Persist the manual restart flag to NVS.
 *
 * @param manual_restart True if the last restart was user-initiated.
 */
static void settings_persist_manual_restart(bool manual_restart);

/**
 * @brief Load the manual restart flag from NVS.
 */
static void load_manual_restart_from_nvs(void);

/**
 * @brief Persist the theme preference to NVS.
 */
static void persist_theme_to_nvs(void);

/**
 * @brief Load the theme preference from NVS.
 */
static void load_theme_from_nvs(void);

/* Callbacks registered by other modules to react to time set/reset events. */
static void (*s_time_set_cb)(void) = NULL;
static void (*s_time_reset_cb)(void) = NULL;

static void get_sntp_time()
{
    show_connecting_message();
    lv_timer_handler();
    if (!s_settings_ctx.settings.time_valid && s_settings_ctx.settings.manual_restart){
        vTaskDelay(150);
        bsp_display_backlight_on(); 
    }
    bsp_display_stop();

    esp_err_t err = wifi_init_sta();
    if (err == ESP_OK){
        err = init_sntp();    
    }
    if (err == ESP_OK){
        settings_persist_sntp_result(true);
        save_time_data();
    }else{
        settings_persist_sntp_result(false);
    }
    settings_persist_manual_restart(false);
    esp_restart();
}

void starting_routine(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    /* ----- NSV ----- */
    ESP_LOGI(TAG, "Initializing NVS");
    ESP_ERROR_CHECK(init_nvs());

    /* ----- Display and LVGL ----- */
    ESP_LOGI(TAG, "Starting bsp for ILI9341 display");
    ESP_ERROR_CHECK(bsp_display_start_result());
    if (reason != ESP_RST_SW) {
        bsp_display_backlight_off();
    }
    styles_init_colors();

    /* ----- Configurations ----- */
    ESP_LOGI(TAG, "Loading configurations");
    init_settings();
    apply_default_font_theme(true); // Mostly used to apply the font

    /* ----- Wi-Fi & SNTP ----- */
    sntp_startup(reason);

    /* ----- XPT2046 Driver Init ----- */
    ESP_LOGI(TAG, "Initializing XPT2046 touch driver");
    ESP_ERROR_CHECK(init_touch()); 
    ESP_LOGI(TAG, "Registering touch driver to LVGL");
    ESP_ERROR_CHECK(register_touch_to_lvgl());
    ESP_LOGI(TAG, "Load touch driver calibration data");
    bool calibration_found;
    load_nvs_calibration(&calibration_found);

    /* ----- XPT2046 Calibration ----- */
    if (s_settings_ctx.settings.calibration_prompt_enabled){ // Default is true
        ESP_LOGI(TAG, "Start calibration dialog");
        calibration_set_show_loader(true);
        settings_set_running_calibration(true);
        ESP_ERROR_CHECK(run_calibration(calibration_found));
        settings_set_running_calibration(false);
    }
}

esp_err_t settings_open_settings(lv_obj_t *return_screen)
{
    if (!return_screen){
        return ESP_ERR_INVALID_ARG;
    }

    settings_ctx_t *ctx = &s_settings_ctx;
    if (!ctx->screen){
        settings_build_screen(ctx);
    }

    ctx->active = true;
    ctx->return_screen = return_screen;
    lv_screen_load(ctx->screen);

    return ESP_OK;
}

esp_err_t settings_show_date_time_dialog(lv_obj_t *return_screen)
{
    settings_ctx_t *ctx = &s_settings_ctx;
    ctx->return_screen = return_screen;
    return settings_build_date_time_dialog(ctx);
}

void settings_register_time_callbacks(void (*on_time_set)(void),
                                      void (*on_time_reset)(void))
{
    s_time_set_cb = on_time_set;
    s_time_reset_cb = on_time_reset;

    if (s_settings_ctx.settings.time_valid) {
        if (s_time_set_cb) {
            s_time_set_cb();
        }
    } else {
        if (s_time_reset_cb) {
            s_time_reset_cb();
        }
    }
}

void settings_shutdown_save_time(void)
{
    time_t now = time(NULL);
    if (now > 0) {
        settings_persist_time_to_nvs(now);
    }
}

bool settings_is_time_valid()
{
    return s_settings_ctx.settings.time_valid == true;
}

void settings_fade_to_saved_brightness(void)
{
    int target = s_settings_ctx.settings.saved_brightness;
    if (target < SETTINGS_MINIMUM_BRIGHTNESS) target = SETTINGS_MINIMUM_BRIGHTNESS;
    if (target > 100) target = 100;
    settings_fade_brightness(target, SETTINGS_UP_FADE_MS);
}

void settings_start_screensaver_timers(void)
{
    bool dim_allowed = s_settings_ctx.settings.screen_dim &&
                        (!s_settings_ctx.settings.screen_off ||
                        s_settings_ctx.settings.off_time <= 0 ||
                        s_settings_ctx.settings.dim_time <= 0 ||
                        s_settings_ctx.settings.dim_time < s_settings_ctx.settings.off_time);

    if (dim_allowed) {
        screensaver_dim_start(s_settings_ctx.settings.dim_time, s_settings_ctx.settings.dim_level);
    } else {
        screensaver_dim_stop();
    }

    if (s_settings_ctx.settings.screen_off) {
        screensaver_off_start(s_settings_ctx.settings.off_time);
    } else {
        screensaver_off_stop();
    }
}

int settings_get_active_brightness(void){
    return s_settings_ctx.settings.brightness;
}

bool settings_is_wake_in_progress(void)
{
    return s_wake_in_progress;
}

bool settings_get_brightness_state(void)
{
    return s_settings_ctx.changing_brightness; 
}

bool settings_get_calibration_prompt_enabled(void)
{
    return s_settings_ctx.settings.calibration_prompt_enabled;
}

void settings_set_calibration_prompt_enabled(bool enable)
{
    if (s_settings_ctx.settings.calibration_prompt_enabled == enable) {
        return;
    }
    s_settings_ctx.settings.calibration_prompt_enabled = enable;
    persist_calibration_prompt_to_nvs();
}

bool settings_get_running_calibration(void)
{
    return s_settings_ctx.settings.running_calibration;
}

void settings_set_running_calibration(bool enable)
{
    s_settings_ctx.settings.running_calibration = enable;
}

bool settings_get_dark_theme_flag(void)
{
    return s_settings_ctx.settings.dark_theme;
}

void settings_set_dark_theme_flag(bool is_dark)
{
    s_settings_ctx.settings.dark_theme = is_dark;
}

static void settings_build_screen(settings_ctx_t *ctx)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 2, 0);
    lv_obj_set_style_pad_gap(scr, 5, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    ctx->screen = scr;

    lv_obj_t *toolbar = lv_obj_create(scr);
    lv_obj_remove_style_all(toolbar);
    styles_set_card_color(toolbar, 0);
    lv_obj_set_size(toolbar, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(toolbar, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(toolbar, 3, 0);
    lv_obj_set_flex_align(toolbar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(toolbar, LV_OPA_COVER, 0);
    ctx->toolbar = toolbar;    

    lv_obj_t *back_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_set_style_pad_all(back_btn, 6, 0);    
    styles_set_button(back_btn);
    lv_obj_add_event_cb(back_btn, settings_on_back, LV_EVENT_CLICKED, ctx);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_center(back_lbl);

    lv_obj_t *about_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(about_btn, 6, 0);
    lv_obj_set_style_pad_all(about_btn, 6, 0);   
    styles_set_button(about_btn); 
    lv_obj_add_event_cb(about_btn, settings_on_about, LV_EVENT_CLICKED, ctx);
    lv_obj_t *about_lbl = lv_label_create(about_btn);
    lv_label_set_text(about_lbl, "About");
    lv_obj_center(about_lbl);    

    /* Scrollable settings list */
    lv_obj_t *settings_list = lv_obj_create(scr);
    lv_obj_remove_style_all(settings_list);
    lv_obj_set_width(settings_list, LV_PCT(100));
    lv_obj_set_height(settings_list, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(settings_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(settings_list,
                          LV_FLEX_ALIGN_START,   /* main axis alignment: start to avoid overlap with header */
                          LV_FLEX_ALIGN_CENTER,  /* cross axis alignment */
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_flex_grow(settings_list, 1);
    lv_obj_set_scroll_dir(settings_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(settings_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_top(settings_list, 4, 0);
    lv_obj_set_style_pad_bottom(settings_list, 4, 0);
    lv_obj_set_style_pad_left(settings_list, 8, 0);
    lv_obj_set_style_pad_right(settings_list, 8, 0);
    lv_obj_set_style_pad_row(settings_list, 4, 0);  
    styles_set_bg_color(settings_list, 0);
    lv_obj_set_style_bg_opa(settings_list, LV_OPA_TRANSP, 0);

    lv_obj_t *brightness_card = lv_button_create(settings_list);
    lv_obj_set_width(brightness_card, LV_PCT(100));
    lv_obj_set_height(brightness_card, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(brightness_card, 10, 0);
    lv_obj_set_style_pad_row(brightness_card, 6, 0);
    lv_obj_set_style_radius(brightness_card, 8, 0);
    styles_set_bg_color(brightness_card, 0);
    styles_set_border_color(brightness_card, 0);
    lv_obj_set_style_bg_opa(brightness_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(brightness_card, 1, 0);
    lv_obj_set_flex_flow(brightness_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(brightness_card,
                          LV_FLEX_ALIGN_START,   /* keep vertical stacking */
                          LV_FLEX_ALIGN_CENTER,  /* center horizontally */
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_align(brightness_card, LV_ALIGN_CENTER, 0);
    lv_obj_clear_flag(brightness_card, LV_OBJ_FLAG_CLICKABLE); /* container only */

    ctx->brightness_label = lv_label_create(brightness_card);
    lv_obj_set_width(ctx->brightness_label, LV_PCT(100));
    lv_obj_set_style_text_align(ctx->brightness_label, LV_TEXT_ALIGN_CENTER, 0);
    styles_set_text_color(ctx->brightness_label, 0);

    ctx->brightness_slider = lv_slider_create(brightness_card);
    lv_obj_set_width(ctx->brightness_slider, LV_PCT(90));
    lv_slider_set_range(ctx->brightness_slider, SETTINGS_MINIMUM_BRIGHTNESS, 100);
    lv_slider_set_value(ctx->brightness_slider, ctx->settings.brightness, LV_ANIM_OFF);
    lv_obj_add_event_cb(ctx->brightness_slider, settings_on_brightness_changed, LV_EVENT_VALUE_CHANGED, ctx);
    styles_set_slider(ctx->brightness_slider);
    lv_obj_set_style_bg_opa(ctx->brightness_slider, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ctx->brightness_slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ctx->brightness_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(ctx->brightness_slider, 1, LV_PART_KNOB);
    lv_obj_set_style_radius(ctx->brightness_slider, 6, 0);
    lv_obj_set_style_radius(ctx->brightness_slider, 6, LV_PART_INDICATOR);
    lv_obj_set_style_radius(ctx->brightness_slider, 5, LV_PART_KNOB);

    int init_val = lv_slider_get_value(ctx->brightness_slider);
    char init_txt[32];
    lv_snprintf(init_txt, sizeof(init_txt), "Brightness: %d%%", init_val);
    lv_label_set_text(ctx->brightness_label, init_txt);

    /* Row: Calibration + Screensaver */
    lv_obj_t *row_actions0 = lv_obj_create(settings_list);
    lv_obj_remove_style_all(row_actions0);
    lv_obj_set_flex_flow(row_actions0, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_actions0, LV_PCT(100));
    lv_obj_set_style_pad_gap(row_actions0, 2, 0);
    lv_obj_set_style_pad_all(row_actions0, 0, 0);
    lv_obj_set_height(row_actions0, LV_SIZE_CONTENT);    

    lv_obj_t *screen_saver_button = lv_button_create(row_actions0);
    lv_obj_set_flex_grow(screen_saver_button, 1);
    lv_obj_set_style_radius(screen_saver_button, 8, 0);
    lv_obj_set_style_pad_all(screen_saver_button, 6, 0); 
    styles_set_button(screen_saver_button);
    lv_obj_add_event_cb(screen_saver_button, settings_screensaver, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(screen_saver_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *screen_saver_lbl = lv_label_create(screen_saver_button);
    lv_label_set_text(screen_saver_lbl, "Screensaver");
    lv_obj_center(screen_saver_lbl);  

    lv_obj_t *set_date_time_button = lv_button_create(row_actions0);
    lv_obj_set_flex_grow(set_date_time_button, 1);
    lv_obj_set_style_radius(set_date_time_button, 8, 0);
    lv_obj_set_style_pad_all(set_date_time_button, 6, 0);  
    styles_set_button(set_date_time_button);  
    lv_obj_add_event_cb(set_date_time_button, settings_set_date_time, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(set_date_time_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *set_date_time_lbl = lv_label_create(set_date_time_button);
    lv_label_set_text(set_date_time_lbl, "Set Date/Time");
    lv_obj_center(set_date_time_lbl);          
    
    /* Row: Rotate + Set Date/Time */
    lv_obj_t *row_actions1 = lv_obj_create(settings_list);
    lv_obj_remove_style_all(row_actions1);
    lv_obj_set_flex_flow(row_actions1, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_actions1, LV_PCT(100));
    lv_obj_set_style_pad_gap(row_actions1, 2, 0);
    lv_obj_set_style_pad_all(row_actions1, 0, 0);
    lv_obj_set_height(row_actions1, LV_SIZE_CONTENT);

    lv_obj_t *rotate_button = lv_button_create(row_actions1);
    lv_obj_set_flex_grow(rotate_button, 1);
    lv_obj_set_style_radius(rotate_button, 8, 0);
    lv_obj_set_style_pad_all(rotate_button, 6, 0);    
    styles_set_button(rotate_button);
    lv_obj_add_event_cb(rotate_button, settings_rotate_screen, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(rotate_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *rotate_lbl = lv_label_create(rotate_button);
    lv_label_set_text(rotate_lbl, "Rotate Screen");
    lv_obj_center(rotate_lbl);   

    lv_obj_t *calibration_button = lv_button_create(row_actions1);
    lv_obj_set_flex_grow(calibration_button, 1);
    lv_obj_set_style_radius(calibration_button, 8, 0);
    lv_obj_set_style_pad_all(calibration_button, 6, 0); 
    styles_set_button(calibration_button);
    lv_obj_add_event_cb(calibration_button, settings_run_calibration, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(calibration_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *calibration_lbl = lv_label_create(calibration_button);
    lv_label_set_text(calibration_lbl, "Run Calibration");
    lv_obj_center(calibration_lbl);   

    /* Row: Theme + Wi-Fi Connection */
    lv_obj_t *row_actions2 = lv_obj_create(settings_list);
    lv_obj_remove_style_all(row_actions2);
    lv_obj_set_flex_flow(row_actions2, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_actions2, LV_PCT(100));
    lv_obj_set_style_pad_gap(row_actions2, 2, 0);
    lv_obj_set_style_pad_all(row_actions2, 0, 0);
    lv_obj_set_height(row_actions2, LV_SIZE_CONTENT);

    lv_obj_t *theme_button = lv_button_create(row_actions2);
    lv_obj_set_flex_grow(theme_button, 1);
    lv_obj_set_style_radius(theme_button, 8, 0);
    lv_obj_set_style_pad_all(theme_button, 6, 0);  
    styles_set_button(theme_button);  
    lv_obj_add_event_cb(theme_button, settings_toggle_theme, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(theme_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *theme_lbl = lv_label_create(theme_button);
    lv_label_set_text(theme_lbl, "Change Theme");
    lv_obj_center(theme_lbl);

    lv_obj_t *connection_button = lv_button_create(row_actions2);
    lv_obj_set_flex_grow(connection_button, 1);
    lv_obj_set_style_radius(connection_button, 8, 0);
    lv_obj_set_style_pad_all(connection_button, 6, 0);
    styles_set_button(connection_button);    
    lv_obj_add_event_cb(connection_button, settings_reset, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(connection_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *connection_lbl = lv_label_create(connection_button);
    lv_label_set_text(connection_lbl, "SNTP & Wi-Fi");
    lv_obj_center(connection_lbl);  

    /* Row: Restart + Reset */
    lv_obj_t *row_actions3 = lv_obj_create(settings_list);
    lv_obj_remove_style_all(row_actions3);
    lv_obj_set_flex_flow(row_actions3, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_actions3, LV_PCT(100));
    lv_obj_set_style_pad_gap(row_actions3, 2, 0);
    lv_obj_set_style_pad_all(row_actions3, 0, 0);
    lv_obj_set_height(row_actions3, LV_SIZE_CONTENT);

    lv_obj_t *restart_button = lv_button_create(row_actions3);
    lv_obj_set_flex_grow(restart_button, 1);
    lv_obj_set_style_radius(restart_button, 8, 0);
    lv_obj_set_style_pad_all(restart_button, 6, 0);  
    styles_set_button(restart_button);  
    lv_obj_add_event_cb(restart_button, settings_restart, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(restart_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *restart_lbl = lv_label_create(restart_button);
    lv_label_set_text(restart_lbl, "Restart");
    lv_obj_center(restart_lbl);

    lv_obj_t *reset_button = lv_button_create(row_actions3);
    lv_obj_set_flex_grow(reset_button, 1);
    lv_obj_set_style_radius(reset_button, 8, 0);
    lv_obj_set_style_pad_all(reset_button, 6, 0);
    styles_set_button(reset_button);    
    lv_obj_add_event_cb(reset_button, settings_reset, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(reset_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *reset_lbl = lv_label_create(reset_button);
    lv_label_set_text(reset_lbl, "Reset");
    lv_obj_center(reset_lbl);      
}

static void settings_on_about(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);

    lv_obj_t *dlg = lv_obj_create(overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 8, 0);
    styles_set_dialog(dlg);
    lv_obj_set_style_bg_opa(dlg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_set_width(dlg, LV_PCT(80));
    lv_obj_set_height(dlg, LV_PCT(90));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_center(dlg);

    lv_obj_t *list = lv_obj_create(dlg);
    lv_obj_remove_style_all(list);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_set_width(list, LV_PCT(100));
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(list, 1);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_row(list, 10, 0);

    const char *lines[] = {
        "Brightness: adjusts backlight between " STR(SETTINGS_MINIMUM_BRIGHTNESS) "\% and 100\%.",
        "Screensaver: opens the screensaver configuration for dimming and turning off the screen.",
        "Set Date/Time: opens the date&time picker to set values in this format: HH:MM MM/DD/YY.",
        "Rotate Screen: rotates the display 90 degrees each time.",
        "Run Calibration: starts the touch calibration wizard and saves the new calibration data. Also offers startup calibration toggle.",
        "Change Theme: toggles between dark and light system theme, saves other unsaved configs and restarts.",
        "SNTP & Wi-Fi: configure the SSID and password of the Wi-fi connection for a precise and automatic time at startup.",
        "Restart: reboots the device after saving configs. Note: configs are also saved by simply leaving settings.",
        "Reset: restores all configs to default - calibration, screensaver, brightness, rotation, theme, Wi-Fi connection and date&time.",
    };

    for (size_t i = 0; i < sizeof(lines)/sizeof(lines[0]); i++) {
        lv_obj_t *lbl = lv_label_create(list);
        lv_label_set_text(lbl, lines[i]);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(lbl, LV_PCT(100));
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        styles_set_text_color(lbl, 0);
    }

    lv_obj_t *close_btn = lv_button_create(dlg);
    lv_obj_set_width(close_btn, LV_PCT(55));
    lv_obj_set_style_radius(close_btn, 8, 0);
    lv_obj_set_style_pad_all(close_btn, 8, 0);
    styles_set_button(close_btn);
    lv_obj_set_style_align(close_btn, LV_ALIGN_CENTER, 0);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "Close");
    lv_obj_center(close_lbl);

    lv_obj_add_event_cb(close_btn, settings_on_about_close, LV_EVENT_CLICKED, overlay);
}

static void settings_on_about_close(lv_event_t *e)
{
    lv_obj_t *overlay = lv_event_get_user_data(e);
    if (overlay) {
        lv_obj_del(overlay);
    }
}

static void settings_on_back(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    settings_start_screensaver_timers();
    settings_close(ctx);
}

static void settings_close(settings_ctx_t *ctx)
{
    if (ctx && ctx->brightness_slider) {
        int val = lv_slider_get_value(ctx->brightness_slider);
        if (val < SETTINGS_MINIMUM_BRIGHTNESS) val = SETTINGS_MINIMUM_BRIGHTNESS;
        if (val > 100) val = 100;
        ctx->settings.brightness = val;
        s_settings_ctx.changing_brightness = false; 
        if (ctx->settings.brightness != ctx->settings.saved_brightness) {
            persist_brightness_to_nvs();
        }
    }

    if (ctx->settings.screen_rotation_step != ctx->settings.saved_rotation_step) {
        persist_rotation_to_nvs();
    }

    ctx->active = false;
    if (ctx->return_screen)
    {
        lv_screen_load(ctx->return_screen);
    }    
    lv_obj_del(ctx->screen);
    settings_clear_ui_refs(ctx);
    ctx->active = false;
    ctx->screen = NULL;
}

static esp_err_t init_nvs(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }

    return nvs_err;
}

static esp_err_t bsp_display_start_result(void)
{
    if (!bsp_display_start()){
        ESP_LOGE(TAG, "BSP failed to initialize display.");
        return ESP_FAIL;
    } 
    return ESP_OK;
}

static void apply_default_font_theme(bool lock_display)
{
    lv_display_t *disp = lv_display_get_default();
    if (!disp) {
        ESP_LOGW(TAG, "No LVGL display available; cannot set theme font");
        return;
    }
    
    if (lock_display){
        bsp_display_lock(0);
    }

    lv_theme_t *theme = lv_theme_default_init(
        disp,
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_RED),
        s_settings_ctx.settings.dark_theme,
        &Domine_14);

    if (!theme) {
        ESP_LOGW(TAG, "Failed to init LVGL default theme with Domine_14");
        if (lock_display){
            bsp_display_unlock();
        }
        return;
    }

    lv_display_set_theme(disp, theme);

    /* Ensure overlay/system layers also inherit the font (dialogs, prompts, etc.) */
    lv_obj_t *act_scr = lv_display_get_screen_active(disp);
    lv_obj_t *top_layer = lv_display_get_layer_top(disp);
    lv_obj_t *sys_layer = lv_display_get_layer_sys(disp);
    lv_obj_set_style_text_font(act_scr, &Domine_14, 0);
    lv_obj_set_style_text_font(top_layer, &Domine_14, 0);
    lv_obj_set_style_text_font(sys_layer, &Domine_14, 0);

    if (lock_display){
        bsp_display_unlock();
    }
}

static void sntp_restart_flow(void)
{
    load_sntp_result_from_nvs();            
    if (s_settings_ctx.settings.sntp_success){
        show_connection_result_message(ESP_OK);
    }else{
        show_connection_result_message(ESP_FAIL);
    }
    vTaskDelay(pdMS_TO_TICKS(150));
    bsp_display_backlight_on();            
    vTaskDelay(pdMS_TO_TICKS(1000));
}

static void manual_restart_flow(void)
{
    if(!s_settings_ctx.settings.time_valid){
        get_sntp_time();
    }else{
        vTaskDelay(pdMS_TO_TICKS(150));
        bsp_display_backlight_on(); 
    }
}

static void sntp_startup(esp_reset_reason_t reason)
{
    if (reason == ESP_RST_SW){
        load_manual_restart_from_nvs();
        if (!s_settings_ctx.settings.manual_restart){
            sntp_restart_flow();
        }else{
            manual_restart_flow();
        }
    }else{
        ESP_LOGI(TAG, "Showing splash & connection screens");
        show_splash_screen();
        get_sntp_time();
    }
    
}

static void apply_rotation_to_display(bool lock_display)
{
    lv_display_t *display = lv_display_get_default();
    if (!display) {
        ESP_LOGW(TAG, "No display available; skip applying rotation");
        return;
    }

    if (lock_display) {
        bsp_display_lock(0);
    }

    /* Map state index to rotation (0:270, 1:180, 2:90, 3:0). */
    switch (s_settings_ctx.settings.screen_rotation_step % SETTINGS_ROTATION_STEPS) {
        case 0: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_270); break;
        case 1: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_180); break;
        case 2: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);  break;
        case 3: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_0);   break;
        default:
            s_settings_ctx.settings.screen_rotation_step = SETTINGS_ROTATION_STEPS - 1;
            lv_display_set_rotation(display, LV_DISPLAY_ROTATION_0);
            break;
    }

    if (lock_display) {
        bsp_display_unlock();
    }
}

static void load_rotation_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for rotation: %s", esp_err_to_name(err));
        return;
    }

    int32_t stored = s_settings_ctx.settings.screen_rotation_step;
    err = nvs_get_i32(h, SETTINGS_NVS_ROT_KEY, &stored);
    nvs_close(h);

    if (err == ESP_OK && stored >= 0 && stored < SETTINGS_ROTATION_STEPS) {
        s_settings_ctx.settings.screen_rotation_step = (int)stored;
        s_settings_ctx.settings.saved_rotation_step = s_settings_ctx.settings.screen_rotation_step;
    }
}

static void persist_rotation_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for rotation: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(h, SETTINGS_NVS_ROT_KEY, s_settings_ctx.settings.screen_rotation_step);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save rotation to NVS: %s", esp_err_to_name(err));
    } else {
        s_settings_ctx.settings.saved_rotation_step = s_settings_ctx.settings.screen_rotation_step;
    }
}

static void load_brightness_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        s_settings_ctx.settings.brightness = SETTINGS_DEFAULT_BRIGHTNESS;
        s_settings_ctx.settings.saved_brightness = s_settings_ctx.settings.brightness;
        return;
    }

    int32_t stored = SETTINGS_DEFAULT_BRIGHTNESS;
    err = nvs_get_i32(h, SETTINGS_NVS_BRIGHTNESS_KEY, &stored);
    nvs_close(h);

    if (err == ESP_OK && stored >= SETTINGS_MINIMUM_BRIGHTNESS && stored <= 100) {
        s_settings_ctx.settings.brightness = (int)stored;
        s_settings_ctx.settings.saved_brightness = s_settings_ctx.settings.brightness;
    } else {
        s_settings_ctx.settings.brightness = SETTINGS_DEFAULT_BRIGHTNESS;
        s_settings_ctx.settings.saved_brightness = s_settings_ctx.settings.brightness;
    }
}

static void persist_brightness_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for brightness: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(h, SETTINGS_NVS_BRIGHTNESS_KEY, s_settings_ctx.settings.brightness);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save brightness to NVS: %s", esp_err_to_name(err));
    } else {
        s_settings_ctx.settings.saved_brightness = s_settings_ctx.settings.brightness;

        /* Adjust dim level to stay within saved brightness and above minimum. */
        if (s_settings_ctx.settings.dim_level >= 0) {
            int max_level = s_settings_ctx.settings.saved_brightness;
            if (max_level < SETTINGS_MINIMUM_BRIGHTNESS) {
                max_level = SETTINGS_MINIMUM_BRIGHTNESS;
            }
            int clamped = s_settings_ctx.settings.dim_level;
            if (clamped > max_level) {
                clamped = max_level;
            }
            if (clamped < SETTINGS_MINIMUM_BRIGHTNESS) {
                clamped = SETTINGS_MINIMUM_BRIGHTNESS;
            }
            if (clamped != s_settings_ctx.settings.dim_level) {
                s_settings_ctx.settings.dim_level = clamped;
                persist_screensaver_to_nvs();
            }
        }
    }
}

static void load_screensaver_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return;
    }

    int8_t dim_en = 0;
    if (nvs_get_i8(h, SETTINGS_NVS_DIM_EN_KEY, &dim_en) == ESP_OK) {
        s_settings_ctx.settings.screen_dim = dim_en ? true : false;
    }

    int32_t dim_time = -1;
    if (nvs_get_i32(h, SETTINGS_NVS_DIM_TIME_KEY, &dim_time) == ESP_OK) {
        if (dim_time >= -1) {
            s_settings_ctx.settings.dim_time = (int)dim_time;
        }
    }

    int32_t dim_level = -1;
    if (nvs_get_i32(h, SETTINGS_NVS_DIM_LEVEL_KEY, &dim_level) == ESP_OK) {
        if (dim_level >= -1 && dim_level <= 100) {
            s_settings_ctx.settings.dim_level = (int)dim_level;
        }
    }

    int8_t off_en = 0;
    if (nvs_get_i8(h, SETTINGS_NVS_OFF_EN_KEY, &off_en) == ESP_OK) {
        s_settings_ctx.settings.screen_off = off_en ? true : false;
    }

    int32_t off_time = -1;
    if (nvs_get_i32(h, SETTINGS_NVS_OFF_TIME_KEY, &off_time) == ESP_OK) {
        if (off_time >= -1) {
            s_settings_ctx.settings.off_time = (int)off_time;
        }
    }

    nvs_close(h);

    /* Clamp dim level against current saved brightness and minimum brightness. */
    if (s_settings_ctx.settings.dim_level >= 0) {
        int max_level = s_settings_ctx.settings.saved_brightness > 0 ? s_settings_ctx.settings.saved_brightness : SETTINGS_DEFAULT_BRIGHTNESS;
        int clamped = s_settings_ctx.settings.dim_level;
        if (max_level < SETTINGS_MINIMUM_BRIGHTNESS) {
            max_level = SETTINGS_MINIMUM_BRIGHTNESS;
        }
        if (clamped > max_level) {
            clamped = max_level;
        }
        if (clamped < SETTINGS_MINIMUM_BRIGHTNESS) {
            clamped = SETTINGS_MINIMUM_BRIGHTNESS;
        }
        s_settings_ctx.settings.dim_level = clamped;
    }
}

static void persist_screensaver_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for screensaver: (%s)", esp_err_to_name(err));
        return;
    }

    esp_err_t res = nvs_set_i8(h, SETTINGS_NVS_DIM_EN_KEY, s_settings_ctx.settings.screen_dim ? 1 : 0);
    res |= nvs_set_i32(h, SETTINGS_NVS_DIM_TIME_KEY, s_settings_ctx.settings.dim_time);
    res |= nvs_set_i32(h, SETTINGS_NVS_DIM_LEVEL_KEY, s_settings_ctx.settings.dim_level);
    res |= nvs_set_i8(h, SETTINGS_NVS_OFF_EN_KEY, s_settings_ctx.settings.screen_off ? 1 : 0);
    res |= nvs_set_i32(h, SETTINGS_NVS_OFF_TIME_KEY, s_settings_ctx.settings.off_time);
    if (res == ESP_OK) {
        res = nvs_commit(h);
    }
    nvs_close(h);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save screensaver settings: (%s)", esp_err_to_name(res));
    }
}

static void load_calibration_prompt_from_nvs(void)
{
    /* Default: enabled */
    s_settings_ctx.settings.calibration_prompt_enabled = true;

    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    int8_t raw = -1;
    if (nvs_get_i8(h, SETTINGS_NVS_CALIB_PROMPT_KEY, &raw) == ESP_OK) {
        s_settings_ctx.settings.calibration_prompt_enabled = (raw != 0);
    }

    nvs_close(h);
}

static void persist_calibration_prompt_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for calibration prompt: (%s)", esp_err_to_name(err));
        return;
    }

    esp_err_t res = nvs_set_i8(h, SETTINGS_NVS_CALIB_PROMPT_KEY, s_settings_ctx.settings.calibration_prompt_enabled ? 1 : 0);
    if (res == ESP_OK) {
        res = nvs_commit(h);
    }
    nvs_close(h);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration prompt preference: (%s)", esp_err_to_name(res));
    }
}

static void persist_theme_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for theme: (%s)", esp_err_to_name(err));
        return;
    }

    esp_err_t res = nvs_set_i8(h, SETTINGS_NVS_THEME_KEY, s_settings_ctx.settings.dark_theme ? 1 : 0);
    if (res == ESP_OK) {
        res = nvs_commit(h);
    }
    nvs_close(h);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save theme preference: (%s)", esp_err_to_name(res));
    }
}

static void load_theme_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for themes: (%s)", esp_err_to_name(err));
        return;
    }

    int8_t raw = -1;
    if (nvs_get_i8(h, SETTINGS_NVS_THEME_KEY, &raw) == ESP_OK) {
        s_settings_ctx.settings.dark_theme = (raw != 0);
    }

    nvs_close(h);
}

static void init_settings(void)
{
    init_default_configs();
    apply_timezone();
    load_last_saved_configs();
}

static void init_default_configs(void)
{
    s_settings_ctx.settings.calibration_prompt_enabled = SETTINGS_DEFAULT_CALI_PROMPT_ENABLE;
    s_settings_ctx.settings.screen_rotation_step = SETTINGS_DEFAULT_ROTATION_STEP;
    s_settings_ctx.settings.saved_rotation_step = SETTINGS_DEFAULT_ROTATION_STEP;
    s_settings_ctx.settings.saved_brightness = SETTINGS_DEFAULT_BRIGHTNESS;
    s_settings_ctx.settings.brightness = SETTINGS_DEFAULT_BRIGHTNESS;
    s_settings_ctx.settings.dim_level = SETTINGS_DEFAULT_SCREEN_DIM_LEVEL;
    s_settings_ctx.settings.dim_time = SETTINGS_DEFAULT_SCREEN_DIM_TIME;
    s_settings_ctx.settings.screen_dim = SETTINGS_DEFAULT_SCREEN_DIM;
    s_settings_ctx.settings.off_time = SETTINGS_DEFAULT_SCREEN_OFF_TIME;
    s_settings_ctx.settings.screen_off = SETTINGS_DEFAULT_SCREEN_OFF;
    s_settings_ctx.settings.dark_theme = SETTINGS_DEFAULT_DARK_THEME;
    s_settings_ctx.settings.time_valid = SETTINGS_DEFAULT_TIME_VALID;
    s_settings_ctx.settings.running_calibration = false;
    s_settings_ctx.settings.manual_restart = false;
    s_settings_ctx.settings.sntp_success = false;
    s_settings_ctx.changing_brightness = false; 
}

static void load_last_saved_configs()
{
    load_calibration_prompt_from_nvs();
    load_manual_restart_from_nvs();
    load_brightness_from_nvs();
    load_rotation_from_nvs();
    apply_rotation_to_display(true);
    load_screensaver_from_nvs();
    load_theme_from_nvs();
    load_time_from_nvs();
}

static void settings_rotate_screen(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    ctx->settings.screen_rotation_step = (ctx->settings.screen_rotation_step + 1) % SETTINGS_ROTATION_STEPS;
    apply_rotation_to_display(false);
}

static esp_err_t settings_build_date_time_dialog(settings_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Close previous overlay if still open. */
    if (ctx->datetime_overlay) {
        lv_obj_del(ctx->datetime_overlay);
        ctx->datetime_overlay = NULL;
        ctx->dt_month_ta = NULL;
        ctx->dt_day_ta = NULL;
        ctx->dt_year_ta = NULL;
        ctx->dt_hour_ta = NULL;
        ctx->dt_min_ta = NULL;
        ctx->dt_keyboard = NULL;
        ctx->dt_dialog = NULL;
        ctx->dt_row_time = NULL;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(overlay, settings_on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    ctx->datetime_overlay = overlay;

    lv_obj_t *dlg = lv_obj_create(overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 6, 0);
    lv_obj_set_style_pad_gap(dlg, 6, 0);
    lv_obj_set_size(dlg, lv_pct(82), lv_pct(69));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(dlg, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dlg, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_add_event_cb(dlg, settings_on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_bg_opa(dlg, LV_OPA_COVER, 0);
    styles_set_dialog(dlg);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_center(dlg);
    ctx->dt_dialog = dlg;

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Set Date/Time");
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_font(title, &Domine_16, 0);
    lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);

    /* Date row */
    lv_obj_t *row_date = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_date);
    lv_obj_set_flex_flow(row_date, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_date, 4, 0);
    lv_obj_set_style_pad_all(row_date, 0, 0);
    lv_obj_set_width(row_date, LV_PCT(100));
    lv_obj_set_height(row_date, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_date, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_date, 3, 0);
    lv_obj_add_flag(row_date, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *date_lbl = lv_label_create(row_date);
    lv_label_set_text(date_lbl, "Date:");
    lv_obj_add_flag(date_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->dt_month_ta = lv_textarea_create(row_date);
    lv_obj_set_width(ctx->dt_month_ta, 48);
    lv_textarea_set_one_line(ctx->dt_month_ta, true);
    lv_textarea_set_max_length(ctx->dt_month_ta, 2);
    lv_textarea_set_text(ctx->dt_month_ta, "MM");
    styles_set_textarea(ctx->dt_month_ta);
    lv_obj_add_event_cb(ctx->dt_month_ta, settings_on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->dt_month_ta, settings_on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->dt_month_ta, settings_on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    lv_obj_t *slash1 = lv_label_create(row_date);
    lv_label_set_text(slash1, "/");
    lv_obj_add_flag(slash1, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->dt_day_ta = lv_textarea_create(row_date);
    lv_obj_set_width(ctx->dt_day_ta, 48);
    lv_textarea_set_one_line(ctx->dt_day_ta, true);
    lv_textarea_set_max_length(ctx->dt_day_ta, 2);
    lv_textarea_set_text(ctx->dt_day_ta, "DD");
    styles_set_textarea(ctx->dt_day_ta);
    lv_obj_add_event_cb(ctx->dt_day_ta, settings_on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->dt_day_ta, settings_on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->dt_day_ta, settings_on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    lv_obj_t *slash2 = lv_label_create(row_date);
    lv_label_set_text(slash2, "/");
    lv_obj_add_flag(slash2, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->dt_year_ta = lv_textarea_create(row_date);
    lv_obj_set_width(ctx->dt_year_ta, 48);
    lv_textarea_set_one_line(ctx->dt_year_ta, true);
    lv_textarea_set_max_length(ctx->dt_year_ta, 2);
    lv_textarea_set_text(ctx->dt_year_ta, "YY");
    styles_set_textarea(ctx->dt_year_ta);
    lv_obj_add_event_cb(ctx->dt_year_ta, settings_on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->dt_year_ta, settings_on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->dt_year_ta, settings_on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    /* Time row */
    lv_obj_t *row_time = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_time);
    lv_obj_set_flex_flow(row_time, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_time, 4, 0);
    lv_obj_set_style_pad_all(row_time, 0, 0);
    lv_obj_set_width(row_time, LV_PCT(100));
    lv_obj_set_height(row_time, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_time, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(row_time, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->dt_row_time = row_time;

    lv_obj_t *time_lbl = lv_label_create(row_time);
    lv_label_set_text(time_lbl, "Time:");
    lv_obj_add_flag(time_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->dt_hour_ta = lv_textarea_create(row_time);
    lv_obj_set_width(ctx->dt_hour_ta, 48);
    lv_textarea_set_one_line(ctx->dt_hour_ta, true);
    lv_textarea_set_max_length(ctx->dt_hour_ta, 2);
    lv_textarea_set_text(ctx->dt_hour_ta, "HH");
    styles_set_textarea(ctx->dt_hour_ta);
    lv_obj_add_event_cb(ctx->dt_hour_ta, settings_on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->dt_hour_ta, settings_on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->dt_hour_ta, settings_on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    lv_obj_t *colon = lv_label_create(row_time);
    lv_label_set_text(colon, ":");
    lv_obj_add_flag(colon, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->dt_min_ta = lv_textarea_create(row_time);
    lv_obj_set_width(ctx->dt_min_ta, 48);
    lv_textarea_set_one_line(ctx->dt_min_ta, true);
    lv_textarea_set_max_length(ctx->dt_min_ta, 2);
    lv_textarea_set_text(ctx->dt_min_ta, "MM");
    styles_set_textarea(ctx->dt_min_ta);
    lv_obj_add_event_cb(ctx->dt_min_ta, settings_on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->dt_min_ta, settings_on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->dt_min_ta, settings_on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    /* Action row */
    lv_obj_t *row_actions = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_actions);
    lv_obj_set_flex_flow(row_actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_actions, 6, 0);
    lv_obj_set_style_pad_all(row_actions, 0, 0);
    lv_obj_set_width(row_actions, LV_PCT(100));
    lv_obj_set_height(row_actions, LV_SIZE_CONTENT);
    lv_obj_set_style_margin_top(row_actions, 8, 0);
    lv_obj_add_flag(row_actions, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *apply_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(apply_btn, 1);
    lv_obj_set_style_radius(apply_btn, 6, 0);
    lv_obj_t *apply_lbl = lv_label_create(apply_btn);
    lv_label_set_text(apply_lbl, "Apply");
    lv_obj_center(apply_lbl);
    lv_obj_add_flag(apply_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(apply_btn);
    lv_obj_add_event_cb(apply_btn, settings_apply_date_time, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_radius(cancel_btn, 6, 0);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);
    lv_obj_add_flag(cancel_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, settings_close_set_date_time, LV_EVENT_CLICKED, ctx);

    /* Keyboard anchored to bottom of overlay */
    ctx->dt_keyboard = lv_keyboard_create(overlay);
    styles_set_keyboard(ctx->dt_keyboard);
    lv_keyboard_set_mode(ctx->dt_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(ctx->dt_keyboard, NULL);
    lv_obj_add_flag(ctx->dt_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(ctx->dt_keyboard, LV_OBJ_FLAG_HIDDEN); /* show only after a field is tapped */
    lv_obj_add_event_cb(ctx->dt_keyboard, settings_on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->dt_keyboard, settings_on_dt_keyboard_event, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->dt_keyboard, settings_on_dt_keyboard_event, LV_EVENT_READY, ctx);
    lv_obj_align(ctx->dt_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

    return ESP_OK;
}

static void settings_set_date_time(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    settings_build_date_time_dialog(ctx);
}

static void settings_close_set_date_time(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);  
    if (ctx && ctx->datetime_overlay) {
        lv_obj_del(ctx->datetime_overlay);
        ctx->datetime_overlay = NULL;
        ctx->dt_month_ta = NULL;
        ctx->dt_day_ta = NULL;
        ctx->dt_year_ta = NULL;
        ctx->dt_hour_ta = NULL;
        ctx->dt_min_ta = NULL;
        ctx->dt_keyboard = NULL;
    }    
}

static void settings_apply_date_time(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    int month, day, year, hour, minute;
    const char *month_txt = ctx->dt_month_ta ? lv_textarea_get_text(ctx->dt_month_ta) : NULL;
    const char *day_txt = ctx->dt_day_ta ? lv_textarea_get_text(ctx->dt_day_ta) : NULL;
    const char *year_txt = ctx->dt_year_ta ? lv_textarea_get_text(ctx->dt_year_ta) : NULL;
    const char *hour_txt = ctx->dt_hour_ta ? lv_textarea_get_text(ctx->dt_hour_ta) : NULL;
    const char *min_txt = ctx->dt_min_ta ? lv_textarea_get_text(ctx->dt_min_ta) : NULL;

    if (!settings_parse_int_range(month_txt, 1, 12, &month) ||
        !settings_parse_int_range(day_txt, 1, 31, &day) ||
        !settings_parse_int_range(year_txt, 0, 99, &year) ||
        !settings_parse_int_range(hour_txt, 0, 23, &hour) ||
        !settings_parse_int_range(min_txt, 0, 59, &minute)) {
        settings_show_invalid_input();
        return;
    }

    int year_full = 2000 + year;
    if (!settings_is_valid_date(year_full, month, day)) {
        settings_show_invalid_input();
        return;
    }

    ctx->settings.dt_month = month;
    ctx->settings.dt_day = day;
    ctx->settings.dt_year = year;
    ctx->settings.dt_hour = hour;
    ctx->settings.dt_minute = minute;
    ctx->settings.time_valid = true;
    settings_persist_valid_time_flag_to_nvs(true);
    settings_notify_time_set();

    /* Set system time from the provided fields (no persistence). */
    struct tm tm_set = {
        .tm_year = year_full - 1900, /* YY -> 20YY */
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = 0,
    };
    time_t t = mktime(&tm_set);
    if (t != (time_t)-1) {
        struct timeval tv = {
            .tv_sec = t,
            .tv_usec = 0,
        };
        settimeofday(&tv, NULL);
    }

    settings_persist_time_to_nvs(t);

    if (ctx->datetime_overlay) {
        lv_obj_del(ctx->datetime_overlay);
        ctx->datetime_overlay = NULL;
    }
    ctx->dt_month_ta = NULL;
    ctx->dt_day_ta = NULL;
    ctx->dt_year_ta = NULL;
    ctx->dt_hour_ta = NULL;
    ctx->dt_min_ta = NULL;
    ctx->dt_keyboard = NULL;
    ctx->dt_dialog = NULL;
    ctx->dt_row_time = NULL;
}

static void build_splash_screen(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_clean(scr);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "File Manager");
    lv_obj_center(label);

    lv_screen_load(scr);
}

static void build_connection_result_message(esp_err_t result)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_clean(scr);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(scr);
    if (result == ESP_OK){
        lv_label_set_text(label, "Time set succesfully!");
    }else{
        lv_label_set_text(label, "Time setting failed.");
    }
    lv_obj_center(label);

    lv_screen_load(scr);
}

static void build_connecting_screen(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Connecting to Wi-Fi and SNTP server");
    lv_obj_center(label);

    lv_screen_load(scr);
}

static void show_splash_screen(void)
{

    bsp_display_lock(0);
    build_splash_screen();
    bsp_display_unlock();
    
    // Small delay before turning on the screen to avoid wipe effect
    vTaskDelay(pdMS_TO_TICKS(150));
    bsp_display_backlight_on();
    vTaskDelay(pdMS_TO_TICKS(1350));
}

static void show_connecting_message(void)
{

    bsp_display_lock(0);
    build_connecting_screen();
    bsp_display_unlock();
}

static void show_connection_result_message(esp_err_t result)
{

    bsp_display_lock(0);
    build_connection_result_message(result);
    bsp_display_unlock();
}

static void save_time_data(void)
{
    time_t now = 0;
    time(&now);

    struct tm tm_info = {0};
    localtime_r(&now, &tm_info);

    if (now <= 0 || tm_info.tm_year < (2016 - 1900)) {
        ESP_LOGW(TAG, "SNTP time not valid; keeping previous time settings");
        return;
    }

    s_settings_ctx.settings.dt_month  = tm_info.tm_mon + 1;
    s_settings_ctx.settings.dt_day    = tm_info.tm_mday;
    s_settings_ctx.settings.dt_year   = tm_info.tm_year % 100; /* YY */
    s_settings_ctx.settings.dt_hour   = tm_info.tm_hour;
    s_settings_ctx.settings.dt_minute = tm_info.tm_min;
    s_settings_ctx.settings.dt_second = tm_info.tm_sec;
    s_settings_ctx.settings.time_valid = true;

    settings_persist_time_to_nvs(now);
}

static void apply_timezone(void)
{
    setenv("TZ", SNTP_DEFAULT_TIMEZONE, 1);
    tzset();
}

static bool settings_parse_int_range(const char *txt, int min, int max, int *out_val)
{
    if (!txt || !out_val) {
        return false;
    }
    char *end = NULL;
    long v = strtol(txt, &end, 10);
    if (txt == end || (end && *end != '\0')) {
        return false;
    }
    if (v < min || v > max) {
        return false;
    }
    *out_val = (int)v;
    return true;
}

static void settings_invalid_ok(lv_event_t *e)
{
    lv_obj_t *mbox = lv_event_get_user_data(e);
    if (mbox) {
        lv_msgbox_close(mbox);
    }
}

static void settings_show_invalid_input(void)
{
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, LV_PCT(70), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Incorrect Input");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, settings_invalid_ok, LV_EVENT_CLICKED, mbox);
}

static void settings_on_dt_textarea_focus(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->dt_keyboard) {
        return;
    }
    lv_obj_t *ta = lv_event_get_target(e);
    const char *txt = lv_textarea_get_text(ta);
    if (txt && (strcmp(txt, "MM") == 0 || strcmp(txt, "DD") == 0 ||
                strcmp(txt, "YY") == 0 || strcmp(txt, "HH") == 0)) {
        lv_textarea_set_text(ta, "");
    }
    lv_keyboard_set_textarea(ctx->dt_keyboard, ta);
    lv_obj_clear_flag(ctx->dt_keyboard, LV_OBJ_FLAG_HIDDEN);
    settings_realign_dt_dialog(ctx, ta);
    settings_scroll_field_into_view(ctx, ta);
}

static void settings_on_dt_background_tap(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    lv_obj_t *target = lv_event_get_target(e);
    if (settings_is_descendant(target, ctx->dt_keyboard)) {
        return;
    }
    if (settings_is_descendant(target, ctx->dt_month_ta) ||
        settings_is_descendant(target, ctx->dt_day_ta) ||
        settings_is_descendant(target, ctx->dt_year_ta) ||
        settings_is_descendant(target, ctx->dt_hour_ta) ||
        settings_is_descendant(target, ctx->dt_min_ta)) {
        return;
    }

    settings_hide_dt_keyboard(ctx);
}

static void settings_hide_dt_keyboard(settings_ctx_t *ctx)
{
    if (!ctx || !ctx->dt_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->dt_keyboard, NULL);
    lv_obj_add_flag(ctx->dt_keyboard, LV_OBJ_FLAG_HIDDEN);
    settings_realign_dt_dialog(ctx, NULL);
}

static void settings_realign_dt_dialog(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->dt_dialog || !lv_obj_is_valid(ctx->dt_dialog)) {
        return;
    }

    int16_t kb_h = (ctx->dt_keyboard && lv_obj_is_valid(ctx->dt_keyboard)) ? lv_obj_get_height(ctx->dt_keyboard) : 0;
    if (kb_h <= 0) {
        kb_h = lv_obj_get_height(ctx->dt_dialog) / 3;
    }
    if (kb_h <= 0) {
        kb_h = 120; /* fallback */
    }

    int16_t date_row_lift = -(kb_h / 3);
    int16_t time_row_lift = -(kb_h / 2) - 20;
    if (date_row_lift == 0) {
        date_row_lift = -30;
    }
    if (time_row_lift == 0) {
        time_row_lift = -70;
    }

    int16_t offset = 0;
    if (ta) {
        if (ta == ctx->dt_month_ta || ta == ctx->dt_day_ta || ta == ctx->dt_year_ta) {
            offset = date_row_lift;
        } else {
            offset = time_row_lift;
        }
    }

    lv_obj_align(ctx->dt_dialog, LV_ALIGN_CENTER, 0, offset);
}

static void settings_hide_ss_keyboard(settings_ctx_t *ctx)
{
    if (!ctx || !ctx->ss_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->ss_keyboard, NULL);
    lv_obj_add_flag(ctx->ss_keyboard, LV_OBJ_FLAG_HIDDEN);
    settings_realign_screensaver_dialog(ctx, NULL);
}

static void settings_realign_screensaver_dialog(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->screensaver_dialog || !lv_obj_is_valid(ctx->screensaver_dialog)) {
        return;
    }

    int16_t kb_h = (ctx->ss_keyboard && lv_obj_is_valid(ctx->ss_keyboard)) ? lv_obj_get_height(ctx->ss_keyboard) : 0;
    if (kb_h <= 0 && ctx->screensaver_dialog) {
        kb_h = lv_obj_get_height(ctx->screensaver_dialog) / 3;
    }
    if (kb_h <= 0) {
        kb_h = 120; /* reasonable fallback */
    }

    int16_t dim_row_lift = -(kb_h / 2);
    int16_t off_row_lift = -(kb_h * 2 / 3) - 25;
    if (dim_row_lift == 0) {
        dim_row_lift = -40;
    }
    if (off_row_lift == 0) {
        off_row_lift = -95;
    }

    int16_t offset = 0;
    if (ta) {
        if (ta == ctx->ss_dim_after_ta || ta == ctx->ss_dim_pct_ta) {
            offset = dim_row_lift;
        } else {
            offset = off_row_lift;
        }
    }

    lv_obj_align(ctx->screensaver_dialog, LV_ALIGN_CENTER, 0, offset);
}

static void settings_on_dt_keyboard_event(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    settings_hide_dt_keyboard(ctx);
}

static void settings_on_ss_background_tap(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    lv_obj_t *target = lv_event_get_target(e);
    if (settings_is_descendant(target, ctx->ss_keyboard)) {
        return;
    }
    if (settings_is_descendant(target, ctx->ss_dim_after_ta) ||
        settings_is_descendant(target, ctx->ss_dim_pct_ta) ||
        settings_is_descendant(target, ctx->ss_off_after_ta)) {
        return;
    }

    settings_hide_ss_keyboard(ctx);
}

static void settings_on_ss_keyboard_event(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    settings_hide_ss_keyboard(ctx);
}

static void settings_on_ss_textarea_focus(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (!ctx || !ctx->ss_keyboard) {
        return;
    }

    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }

    lv_obj_t *ta = lv_event_get_target(e);
    if (lv_obj_has_state(ta, LV_STATE_DISABLED)) {
        return;
    }
    lv_keyboard_set_textarea(ctx->ss_keyboard, ta);
    lv_obj_clear_flag(ctx->ss_keyboard, LV_OBJ_FLAG_HIDDEN);
    settings_realign_screensaver_dialog(ctx, ta);
    lv_obj_scroll_to_view(ta, LV_ANIM_ON);
}

static void settings_on_dim_switch_changed(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    lv_obj_t *sw = lv_event_get_target(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (!enabled) {
        settings_hide_ss_keyboard(ctx);
    }
    settings_update_dim_controls_enabled(ctx, enabled);
}

static void settings_on_off_switch_changed(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    lv_obj_t *sw = lv_event_get_target(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (!enabled) {
        settings_hide_ss_keyboard(ctx);
    }
    settings_update_off_controls_enabled(ctx, enabled);
}

static void settings_update_dim_controls_enabled(settings_ctx_t *ctx, bool enabled)
{
    if (!ctx) {
        return;
    }

    lv_obj_t *labels[] = {
        ctx->ss_dim_after_lbl,
        ctx->ss_seconds_lbl,
        ctx->ss_at_lbl,
        ctx->ss_pct_lbl,
    };
    for (size_t i = 0; i < sizeof(labels)/sizeof(labels[0]); i++) {
        lv_obj_t *lbl = labels[i];
        if (!lbl) {
            continue;
        }
        if (enabled) {
            lv_obj_clear_state(lbl, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, LV_PART_MAIN);
        } else {
            lv_obj_add_state(lbl, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(lbl, LV_OPA_60, LV_PART_MAIN);
        }
    }

    lv_obj_t *textareas[] = {
        ctx->ss_dim_after_ta,
        ctx->ss_dim_pct_ta,
    };
    for (size_t i = 0; i < sizeof(textareas)/sizeof(textareas[0]); i++) {
        lv_obj_t *ta = textareas[i];
        if (!ta) {
            continue;
        }
        if (enabled) {
            lv_obj_clear_state(ta, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(ta, LV_OPA_COVER, LV_PART_MAIN);
            /* If empty, restore from placeholder so last value shows when re-enabled. */
            const char *txt = lv_textarea_get_text(ta);
            const char *ph = lv_textarea_get_placeholder_text(ta);
            if (txt && txt[0] == '\0' && ph && ph[0] != '\0') {
                lv_textarea_set_text(ta, ph);
            }
        } else {
            lv_obj_add_state(ta, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(ta, LV_OPA_60, LV_PART_MAIN);
            /* Clear text so placeholder (last known value) is visible while disabled. */
            lv_textarea_set_text(ta, "");
        }
    }

    if (!enabled && ctx->ss_keyboard) {
        lv_obj_t *attached = lv_keyboard_get_textarea(ctx->ss_keyboard);
        if (attached == ctx->ss_dim_after_ta || attached == ctx->ss_dim_pct_ta) {
            lv_keyboard_set_textarea(ctx->ss_keyboard, NULL);
            lv_obj_add_flag(ctx->ss_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void settings_update_off_controls_enabled(settings_ctx_t *ctx, bool enabled)
{
    if (!ctx) {
        return;
    }

    lv_obj_t *labels[] = {
        ctx->ss_off_after_lbl,
        ctx->ss_off_seconds_lbl,
    };
    for (size_t i = 0; i < sizeof(labels)/sizeof(labels[0]); i++) {
        lv_obj_t *lbl = labels[i];
        if (!lbl) {
            continue;
        }
        if (enabled) {
            lv_obj_clear_state(lbl, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, LV_PART_MAIN);
        } else {
            lv_obj_add_state(lbl, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(lbl, LV_OPA_60, LV_PART_MAIN);
        }
    }

    if (ctx->ss_off_after_ta) {
        if (enabled) {
            lv_obj_clear_state(ctx->ss_off_after_ta, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(ctx->ss_off_after_ta, LV_OPA_COVER, LV_PART_MAIN);
            const char *txt = lv_textarea_get_text(ctx->ss_off_after_ta);
            const char *ph = lv_textarea_get_placeholder_text(ctx->ss_off_after_ta);
            if (txt && txt[0] == '\0' && ph && ph[0] != '\0') {
                lv_textarea_set_text(ctx->ss_off_after_ta, ph);
            }
        } else {
            lv_obj_add_state(ctx->ss_off_after_ta, LV_STATE_DISABLED);
            lv_obj_set_style_text_opa(ctx->ss_off_after_ta, LV_OPA_60, LV_PART_MAIN);
            lv_textarea_set_text(ctx->ss_off_after_ta, "");
        }
    }

    if (!enabled && ctx->ss_keyboard) {
        lv_obj_t *attached = lv_keyboard_get_textarea(ctx->ss_keyboard);
        if (attached == ctx->ss_off_after_ta) {
            lv_keyboard_set_textarea(ctx->ss_keyboard, NULL);
            lv_obj_add_flag(ctx->ss_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void screensaver_dim_start(int seconds, int level_pct)
{
    if (s_ss_dim_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = settings_dim_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ss_dim",
        };
        esp_err_t err = esp_timer_create(&args, &s_ss_dim_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create dim timer: %s", esp_err_to_name(err));
            return;
        }
    } else {
        esp_timer_stop(s_ss_dim_timer);
    }

    int64_t us = (seconds < 0 ? 0 : seconds) * 1000000LL;
    esp_err_t err = esp_timer_start_once(s_ss_dim_timer, us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start dim timer: %s", esp_err_to_name(err));
    }
}

static void screensaver_dim_stop(void)
{
    if (!s_settings_ctx.changing_brightness){
        ESP_LOGD(TAG, "Stop dim timer");
    }
    if (s_ss_dim_timer) {
        esp_timer_stop(s_ss_dim_timer);
    }
}

static void screensaver_off_start(int seconds)
{
    ESP_LOGD(TAG, "Start screen-off timer: %ds", seconds);
    if (s_ss_off_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = settings_off_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ss_off",
        };
        esp_err_t err = esp_timer_create(&args, &s_ss_off_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create off timer: %s", esp_err_to_name(err));
            return;
        }
    } else {
        esp_timer_stop(s_ss_off_timer);
    }

    int64_t us = (seconds < 0 ? 0 : seconds) * 1000000LL;
    esp_err_t err = esp_timer_start_once(s_ss_off_timer, us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start off timer: %s", esp_err_to_name(err));
    }
}

static void screensaver_off_stop(void)
{
    if (!s_settings_ctx.changing_brightness){
        ESP_LOGD(TAG, "Stop screen-off timer");
    }
    if (s_ss_off_timer) {
        esp_timer_stop(s_ss_off_timer);
    }
    settings_fade_brightness(s_settings_ctx.settings.brightness, 0); /* stop any ongoing fade */
}

static void settings_off_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGD(TAG, "Off timer fired: fading screen off");
    settings_fade_brightness(0, SETTINGS_OFF_FADE_MS);
}

static void settings_dim_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGD(TAG, "Dim timer fired: fading to dim level");
    settings_fade_brightness(s_settings_ctx.settings.dim_level, SETTINGS_DIM_FADE_MS);
}

static void settings_fade_brightness(int target_pct, uint32_t duration_ms)
{
    if (target_pct > 100) target_pct = 100;
    if (target_pct < 0) target_pct = 0;

    settings_ctx_t *ctx = &s_settings_ctx;
    int start = ctx->settings.brightness;
    bool rising = target_pct > start;
    if (duration_ms == 0 || start == target_pct) {
        ctx->settings.brightness = target_pct;
        bsp_display_brightness_set(target_pct);
        settings_sync_brightness_ui(ctx, target_pct);
        if (!rising) {
            s_wake_in_progress = false;
        }
        return;
    }

    if (rising) {
        s_wake_in_progress = true;
    }

    /* Stop existing fade timer */
    if (s_fade_timer) {
        esp_timer_stop(s_fade_timer);
    } else {
        const esp_timer_create_args_t args = {
            .callback = settings_fade_step_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "fade",
        };
        if (esp_timer_create(&args, &s_fade_timer) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create fade timer");
            return;
        }
    }

    s_fade_target = target_pct;
    s_fade_direction = (target_pct > start) ? 1 : -1;
    s_fade_steps_left = (start > target_pct) ? (start - target_pct) : (target_pct - start);
    if (s_fade_steps_left == 0) {
        ctx->settings.brightness = target_pct;
        bsp_display_brightness_set(target_pct);
        settings_sync_brightness_ui(ctx, target_pct);
        return;
    }

    int64_t interval_us = (duration_ms * 1000ULL) / s_fade_steps_left;
    if (interval_us < 1000) interval_us = 1000;

    esp_err_t err = esp_timer_start_periodic(s_fade_timer, interval_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start fade timer: %s", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Fade start: %d -> %d over %ums (step %lldus)", start, target_pct, duration_ms, interval_us);
    }
}

static void settings_fade_step_cb(void *arg)
{
    (void)arg;
    if (s_fade_steps_left <= 0) {
        if (s_fade_timer) {
            esp_timer_stop(s_fade_timer);
        }
        s_settings_ctx.settings.brightness = s_fade_target;
        bsp_display_brightness_set(s_fade_target);
        settings_sync_brightness_ui(&s_settings_ctx, s_fade_target);
        ESP_LOGD(TAG, "Fade complete -> %d", s_fade_target);
        s_wake_in_progress = false;
        return;
    }

    int next = s_settings_ctx.settings.brightness + s_fade_direction;
    if (next < 0) next = 0;
    if (next > 100) next = 100;
    s_settings_ctx.settings.brightness = next;
    bsp_display_brightness_set(next);
    s_fade_steps_left--;
}

static void settings_sync_brightness_ui(settings_ctx_t *ctx, int val)
{
    (void)ctx;
    lv_async_call(settings_sync_brightness_ui_async, (void *)(uintptr_t)val);
}

static void settings_sync_brightness_ui_async(void *arg)
{
    int val = (int)(uintptr_t)arg;
    if (val < SETTINGS_MINIMUM_BRIGHTNESS) val = SETTINGS_MINIMUM_BRIGHTNESS;
    if (val > 100) val = 100;

    settings_ctx_t *ctx = &s_settings_ctx;
    /* Skip if settings screen is not active/visible or controls were deleted. */
    if (!ctx->active || !ctx->screen || !lv_obj_is_valid(ctx->screen) || lv_screen_active() != ctx->screen) {
        return;
    }

    if (ctx->brightness_slider && lv_obj_is_valid(ctx->brightness_slider)) {
        lv_slider_set_value(ctx->brightness_slider, val, LV_ANIM_OFF);
    }
    if (ctx->brightness_label && lv_obj_is_valid(ctx->brightness_label)) {
        char txt[32];
        lv_snprintf(txt, sizeof(txt), "Brightness: %d%%", val);
        lv_label_set_text(ctx->brightness_label, txt);
    }
}

static void settings_scroll_field_into_view(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->dt_dialog || !ta) {
        return;
    }
    lv_obj_t *target = ta;
    if ((ta == ctx->dt_hour_ta || ta == ctx->dt_min_ta) && ctx->dt_row_time) {
        target = ctx->dt_row_time;
    }
    lv_obj_scroll_to_view(target, LV_ANIM_ON);
}

static void settings_on_dt_textarea_defocus(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    lv_obj_t *ta = lv_event_get_target(e);
    const char *txt = lv_textarea_get_text(ta);
    if (txt && txt[0] != '\0') {
        return;
    }
    if (ta == ctx->dt_month_ta) {
        lv_textarea_set_text(ta, "MM");
    } else if (ta == ctx->dt_day_ta) {
        lv_textarea_set_text(ta, "DD");
    } else if (ta == ctx->dt_year_ta) {
        lv_textarea_set_text(ta, "YY");
    } else if (ta == ctx->dt_hour_ta) {
        lv_textarea_set_text(ta, "HH");
    } else if (ta == ctx->dt_min_ta) {
        lv_textarea_set_text(ta, "MM");
    }
    settings_scroll_field_into_view(ctx, ta);
}

static bool settings_is_descendant(lv_obj_t *obj, lv_obj_t *maybe_ancestor)
{
    if (!obj || !maybe_ancestor) {
        return false;
    }
    lv_obj_t *cur = obj;
    while (cur) {
        if (cur == maybe_ancestor) {
            return true;
        }
        cur = lv_obj_get_parent(cur);
    }
    return false;
}

static bool settings_is_valid_date(int year_full, int month, int day)
{
    if (month < 1 || month > 12 || day < 1) {
        return false;
    }

    int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool leap = ((year_full % 4 == 0) && (year_full % 100 != 0)) || (year_full % 400 == 0);
    if (leap) {
        days_in_month[1] = 29;
    }

    return day <= days_in_month[month - 1];
}

static void settings_notify_time_set(void)
{
    if (s_time_set_cb) {
        s_time_set_cb();
    }
}

static void settings_notify_time_reset(void)
{
    if (s_time_reset_cb) {
        s_time_reset_cb();
    }
}

static void settings_persist_time_to_nvs(time_t epoch)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i64(h, SETTINGS_NVS_TIME_KEY, (int64_t)epoch);
    nvs_commit(h);
    nvs_close(h);
}

static void settings_clear_time_in_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, SETTINGS_NVS_TIME_KEY);
    nvs_commit(h);
    nvs_close(h);
}

static void settings_persist_valid_time_flag_to_nvs(bool is_valid)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_VALID_TIME_FLAG_KEY, is_valid);
    nvs_commit(h);
    nvs_close(h);
}

static void settings_clear_valid_time_flag_in_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, SETTINGS_NVS_VALID_TIME_FLAG_KEY);
    nvs_commit(h);
    nvs_close(h);
}

static void settings_persist_sntp_result(bool is_successful)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_SNTP_RESULT_KEY, is_successful);
    nvs_commit(h);
    nvs_close(h);
}

static void load_sntp_result_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for previous sntp result: (%s)", esp_err_to_name(err));
        return;
    }

    int8_t raw = -1;
    if (nvs_get_i8(h, SETTINGS_NVS_SNTP_RESULT_KEY, &raw) == ESP_OK) {
        s_settings_ctx.settings.sntp_success = (raw != 0);
    }

    nvs_close(h);
}

static void settings_persist_manual_restart(bool manual_restart)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_MANUAL_RESTART_KEY, manual_restart);
    nvs_commit(h);
    nvs_close(h);
}

static void load_manual_restart_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for manual restart flag: (%s)", esp_err_to_name(err));
        return;
    }

    int8_t raw = -1;
    if (nvs_get_i8(h, SETTINGS_NVS_MANUAL_RESTART_KEY, &raw) == ESP_OK) {
        s_settings_ctx.settings.manual_restart = (raw != 0);
    }

    nvs_close(h);
}

static void load_time_from_nvs(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason != ESP_RST_SW) {
        settings_clear_time_in_nvs();
        s_settings_ctx.settings.time_valid = false;
        settings_clear_valid_time_flag_in_nvs();
        settings_notify_time_reset();
        return;
    }

    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    int64_t stored = 0;
    esp_err_t err = nvs_get_i64(h, SETTINGS_NVS_TIME_KEY, &stored);
    nvs_close(h);
    if (err != ESP_OK || stored <= 0) {
        return;
    }

    struct timeval tv = {
        .tv_sec = (time_t)stored,
        .tv_usec = 0,
    };
    settimeofday(&tv, NULL);
    s_settings_ctx.settings.time_valid = true;
    settings_persist_valid_time_flag_to_nvs(true);
    settings_notify_time_set();
}

static void settings_on_brightness_changed(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED && code != LV_EVENT_RELEASED && code != LV_EVENT_CLICKED) {
        return;
    }

    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->brightness_label || !ctx->brightness_slider) {
        return;
    }

    int val = lv_slider_get_value(ctx->brightness_slider);
    if (val < SETTINGS_MINIMUM_BRIGHTNESS) val = SETTINGS_MINIMUM_BRIGHTNESS;
    if (val > 100) val = 100;
    ctx->settings.brightness = val;
    
    /* Stop any screensaver dim/off fade using the latest brightness value. */
    screensaver_dim_stop();
    screensaver_off_stop();
    s_settings_ctx.changing_brightness = true;

    char txt[32];
    lv_snprintf(txt, sizeof(txt), "Brightness: %d%%", val);
    lv_label_set_text(ctx->brightness_label, txt);

    bsp_display_brightness_set(val);
}

static void settings_restart(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->restart_confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text_fmt(label, "Are you sure you want to restart?");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *yes_btn = lv_msgbox_add_footer_button(mbox, "Yes");
    lv_obj_set_user_data(yes_btn, (void *)1);
    styles_set_button(yes_btn);
    lv_obj_add_event_cb(yes_btn, settings_restart_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, settings_close_restart, LV_EVENT_CLICKED, ctx);
}

static void settings_restart_confirm(lv_event_t *e)
{
    bsp_display_backlight_off();
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx && ctx->brightness_slider) {
        int val = lv_slider_get_value(ctx->brightness_slider);
        if (val < SETTINGS_MINIMUM_BRIGHTNESS) val = SETTINGS_MINIMUM_BRIGHTNESS;
        if (val > 100) val = 100;
        ctx->settings.brightness = val;
        if (ctx->settings.brightness != ctx->settings.saved_brightness) {
            persist_brightness_to_nvs();
        }
    }
    if (ctx->settings.screen_rotation_step != ctx->settings.saved_rotation_step) {
        persist_rotation_to_nvs();
    }
    if (settings_is_time_valid()){
        settings_shutdown_save_time();
    }
    settings_persist_manual_restart(true);
    esp_restart();
}

static void settings_close_restart(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->restart_confirm_mbox)
    {
        return;
    }    
    if (ctx && ctx->restart_confirm_mbox) {
        lv_msgbox_close(ctx->restart_confirm_mbox);
        ctx->restart_confirm_mbox = NULL;
    }    
}

static void settings_reset(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->reset_confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text_fmt(label, "Are you sure you want to reset?");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *yes_btn = lv_msgbox_add_footer_button(mbox, "Yes");
    lv_obj_set_user_data(yes_btn, (void *)1);
    styles_set_button(yes_btn);
    lv_obj_add_event_cb(yes_btn, settings_reset_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, settings_close_reset, LV_EVENT_CLICKED, ctx);
}

static void settings_reset_confirm(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->reset_confirm_mbox)
    {
        return;
    }  

    init_default_configs();
    persist_brightness_to_nvs();
    persist_rotation_to_nvs();
    persist_screensaver_to_nvs();
    persist_calibration_prompt_to_nvs();
    settings_persist_manual_restart(false);
    init_settings();
    
    nvs_handle_t cal_nvs;
    if (nvs_open(TOUCH_CAL_NVS_NS, NVS_READWRITE, &cal_nvs) == ESP_OK) {
        nvs_erase_key(cal_nvs, TOUCH_CAL_NVS_KEY);
        nvs_commit(cal_nvs);
        nvs_close(cal_nvs);
    }

    settings_clear_time_in_nvs();
    s_settings_ctx.settings.time_valid = false;
    settings_persist_valid_time_flag_to_nvs(false);
    settings_notify_time_reset();

    lv_msgbox_close(ctx->reset_confirm_mbox);
    ctx->reset_confirm_mbox = NULL;
}

static void settings_close_reset(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);  
    if (ctx && ctx->reset_confirm_mbox) {
        lv_msgbox_close(ctx->reset_confirm_mbox);
        ctx->reset_confirm_mbox = NULL;
    }    
}

static void settings_toggle_theme(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    if (ctx->theme_confirm_mbox) {
        lv_msgbox_close(ctx->theme_confirm_mbox);
        ctx->theme_confirm_mbox = NULL;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->theme_confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Restart required to change theme");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *yes_btn = lv_msgbox_add_footer_button(mbox, "Yes");
    styles_set_button(yes_btn);
    lv_obj_add_event_cb(yes_btn, settings_theme_confirm_yes, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, settings_theme_confirm_no, LV_EVENT_CLICKED, ctx);
}

static void settings_theme_confirm_yes(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    /* Toggle theme and persist */
    bool new_dark = !settings_get_dark_theme_flag();
    settings_set_dark_theme_flag(new_dark);
    persist_theme_to_nvs();

    settings_restart_confirm(e);
}

static void settings_theme_confirm_no(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->theme_confirm_mbox) {
        return;
    }
    lv_msgbox_close(ctx->theme_confirm_mbox);
    ctx->theme_confirm_mbox = NULL;
}

static void settings_run_calibration(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->screen)
    {
        return;
    }

    if (settings_get_running_calibration())
    {
        ESP_LOGW(TAG, "Calibration already running; ignoring request");
        return;
    }

    settings_set_running_calibration(true);
    calibration_set_show_loader(false);

    lv_obj_clean(ctx->screen);
    settings_clear_ui_refs(ctx);

    /* Run calibration asynchronously to avoid blocking the LVGL task/UI thread. */
    BaseType_t task_ok = xTaskCreate(settings_calibration_task,
                                     "settings_calibration",
                                     SETTINGS_CALIBRATION_TASK_STACK,
                                     ctx,
                                     SETTINGS_CALIBRATION_TASK_PRIO,
                                     NULL);
    if (task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start calibration task");
        settings_set_running_calibration(false);
    }
}

static void settings_calibration_task(void *param)
{
    settings_ctx_t *ctx = (settings_ctx_t *)param;

    if (!ctx || !ctx->return_screen){
        settings_set_running_calibration(false);
        vTaskDelete(NULL);
        return;
    }

    /* Clear cached widget pointers because we delete/clean the screen. */
    settings_clear_ui_refs(ctx);

    int prev_rotation = ctx->settings.screen_rotation_step;
    
    if (ctx->settings.screen_rotation_step != SETTINGS_DEFAULT_ROTATION_STEP && ctx->settings.screen_rotation_step != SETTINGS_DEFAULT_ROTATION_STEP - 2){
        ctx->settings.screen_rotation_step = SETTINGS_DEFAULT_ROTATION_STEP;
        apply_rotation_to_display(true);
    }

    /* Stop Screensaver While Performing Calibration*/
    bsp_display_brightness_set(100);
    screensaver_dim_stop();
    screensaver_off_stop();
    esp_err_t calib_err = run_calibration(true);
    if (calib_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(calib_err));
    }
    s_settings_ctx.changing_brightness = false;  
    settings_start_screensaver_timers();

    ctx->settings.screen_rotation_step = prev_rotation;
    apply_rotation_to_display(true);

    bsp_display_lock(0);
    if (ctx->screen)
    {
        lv_obj_del(ctx->screen);
        ctx->screen = NULL;
    }
    ctx->active = false;
    settings_open_settings(ctx->return_screen);
    bsp_display_unlock();
    
    settings_set_running_calibration(false);
    vTaskDelete(NULL);
}

static void settings_clear_ui_refs(settings_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->toolbar = NULL;
    ctx->brightness_label = NULL;
    ctx->brightness_slider = NULL;
    ctx->restart_confirm_mbox = NULL;
    ctx->reset_confirm_mbox = NULL;
    ctx->theme_confirm_mbox = NULL;
    ctx->datetime_overlay = NULL;
    ctx->screensaver_overlay = NULL;
    ctx->dt_month_ta = NULL;
    ctx->dt_day_ta = NULL;
    ctx->dt_year_ta = NULL;
    ctx->dt_hour_ta = NULL;
    ctx->dt_min_ta = NULL;
    ctx->dt_keyboard = NULL;
    ctx->dt_dialog = NULL;
    ctx->screensaver_dialog = NULL;
    ctx->dt_row_time = NULL;
    ctx->ss_dim_lbl = NULL;
    ctx->ss_dim_switch = NULL;
    ctx->ss_dim_after_lbl = NULL;
    ctx->ss_seconds_lbl = NULL;
    ctx->ss_at_lbl = NULL;
    ctx->ss_pct_lbl = NULL;
    ctx->ss_dim_after_ta = NULL;
    ctx->ss_dim_pct_ta = NULL;
    ctx->ss_off_lbl = NULL;
    ctx->ss_off_switch = NULL;
    ctx->ss_off_after_lbl = NULL;
    ctx->ss_off_seconds_lbl = NULL;
    ctx->ss_off_after_ta = NULL;
    ctx->ss_keyboard = NULL;
}

static void settings_screensaver(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx && ctx->screen)
    {
        settings_build_screensaver_dialog(ctx);
    }
}


static esp_err_t settings_build_screensaver_dialog(settings_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Close previous overlay if still open. */
    if (ctx->screensaver_overlay) {
        lv_obj_del(ctx->screensaver_overlay);
        ctx->screensaver_overlay = NULL;
        ctx->screensaver_dialog = NULL;
        ctx->ss_dim_lbl = NULL;
        ctx->ss_dim_switch = NULL;
        ctx->ss_dim_after_lbl = NULL;
        ctx->ss_seconds_lbl = NULL;
        ctx->ss_at_lbl = NULL;
        ctx->ss_pct_lbl = NULL;
        ctx->ss_dim_after_ta = NULL;
        ctx->ss_dim_pct_ta = NULL;
        ctx->ss_off_lbl = NULL;
        ctx->ss_off_switch = NULL;
        ctx->ss_off_after_lbl = NULL;
        ctx->ss_off_seconds_lbl = NULL;
        ctx->ss_off_after_ta = NULL;
        ctx->ss_keyboard = NULL;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(overlay, settings_on_ss_background_tap, LV_EVENT_CLICKED, ctx);
    ctx->screensaver_overlay = overlay;

    lv_obj_t *dlg = lv_obj_create(overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 6, 0);
    lv_obj_set_style_pad_gap(dlg, 4, 0);
    lv_obj_set_size(dlg, lv_pct(85), lv_pct(95));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(dlg, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dlg, LV_SCROLLBAR_MODE_AUTO);
    styles_set_dialog(dlg);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_add_event_cb(dlg, settings_on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_center(dlg);
    ctx->screensaver_dialog = dlg;

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Screensaver");
    lv_obj_set_style_text_font(title, &Domine_16, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);

    /* Dim row */
    lv_obj_t *row_dim = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_dim);
    lv_obj_set_flex_flow(row_dim, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_dim, 4, 0);
    lv_obj_set_style_pad_all(row_dim, 0, 0);
    lv_obj_set_width(row_dim, LV_PCT(100));
    lv_obj_set_height(row_dim, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_dim, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(row_dim, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *dim_lbl = lv_label_create(row_dim);
    lv_label_set_text(dim_lbl, "Dimming");
    lv_obj_add_flag(dim_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_dim_lbl = dim_lbl;

    lv_obj_t *dim_switch = lv_switch_create(row_dim);
    lv_obj_set_style_pad_all(dim_switch, 4, 0);
    styles_set_switch(dim_switch);
    lv_obj_add_event_cb(dim_switch, settings_on_dim_switch_changed, LV_EVENT_VALUE_CHANGED, ctx);
    ctx->ss_dim_switch = dim_switch;

    /* Dim timing/level row */
    lv_obj_t *row_dim_cfg = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_dim_cfg);
    lv_obj_set_flex_flow(row_dim_cfg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_dim_cfg, 4, 0);
    lv_obj_set_style_pad_all(row_dim_cfg, 0, 0);
    lv_obj_set_width(row_dim_cfg, LV_PCT(100));
    lv_obj_set_height(row_dim_cfg, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_dim_cfg, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(row_dim_cfg, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *dim_after_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(dim_after_lbl, "Dim after");
    lv_obj_add_flag(dim_after_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_dim_after_lbl = dim_after_lbl;

    ctx->ss_dim_after_ta = lv_textarea_create(row_dim_cfg);
    lv_obj_set_width(ctx->ss_dim_after_ta, 35);
    lv_obj_clear_flag(ctx->ss_dim_after_ta, LV_OBJ_FLAG_SCROLLABLE);
    styles_set_textarea(ctx->ss_dim_after_ta);
    lv_textarea_set_one_line(ctx->ss_dim_after_ta, true);
    lv_textarea_set_max_length(ctx->ss_dim_after_ta, 3);
    if (ctx->settings.dim_time >= 0) {
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", ctx->settings.dim_time);
        lv_textarea_set_placeholder_text(ctx->ss_dim_after_ta, buf);
        if (ctx->settings.screen_dim) {
            lv_textarea_set_text(ctx->ss_dim_after_ta, buf);
        } else {
            lv_textarea_set_text(ctx->ss_dim_after_ta, "");
        }
    } else {
        lv_textarea_set_placeholder_text(ctx->ss_dim_after_ta, "");
        lv_textarea_set_text(ctx->ss_dim_after_ta, "");
    }
    lv_obj_add_event_cb(ctx->ss_dim_after_ta, settings_on_ss_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->ss_dim_after_ta, settings_on_ss_textarea_focus, LV_EVENT_CLICKED, ctx);

    lv_obj_t *seconds_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(seconds_lbl, "seconds");
    lv_obj_add_flag(seconds_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_seconds_lbl = seconds_lbl;

    lv_obj_t *at_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(at_lbl, "to");
    lv_obj_add_flag(at_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_at_lbl = at_lbl;

    ctx->ss_dim_pct_ta = lv_textarea_create(row_dim_cfg);
    lv_obj_set_width(ctx->ss_dim_pct_ta, 35);
    lv_obj_clear_flag(ctx->ss_dim_pct_ta, LV_OBJ_FLAG_SCROLLABLE);
    lv_textarea_set_one_line(ctx->ss_dim_pct_ta, true);
    lv_textarea_set_max_length(ctx->ss_dim_pct_ta, 3);
    styles_set_textarea(ctx->ss_dim_pct_ta); 
    if (ctx->settings.dim_level >= 0) {
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", ctx->settings.dim_level);
        lv_textarea_set_placeholder_text(ctx->ss_dim_pct_ta, buf);
        if (ctx->settings.screen_dim) {
            lv_textarea_set_text(ctx->ss_dim_pct_ta, buf);
        } else {
            lv_textarea_set_text(ctx->ss_dim_pct_ta, "");
        }
    } else {
        lv_textarea_set_placeholder_text(ctx->ss_dim_pct_ta, "");
        lv_textarea_set_text(ctx->ss_dim_pct_ta, "");
    }
    lv_obj_add_event_cb(ctx->ss_dim_pct_ta, settings_on_ss_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->ss_dim_pct_ta, settings_on_ss_textarea_focus, LV_EVENT_CLICKED, ctx);

    lv_obj_t *pct_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(pct_lbl, "%");
    lv_obj_add_flag(pct_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_pct_lbl = pct_lbl;

    if (ctx->settings.screen_dim) {
        lv_obj_add_state(dim_switch, LV_STATE_CHECKED);
        lv_obj_add_state(dim_after_lbl, LV_STATE_DISABLED);
        lv_obj_add_state(seconds_lbl, LV_STATE_DISABLED);
        lv_obj_add_state(at_lbl, LV_STATE_DISABLED);
        lv_obj_add_state(pct_lbl, LV_STATE_DISABLED);        
    } else {
        lv_obj_clear_state(dim_switch, LV_STATE_CHECKED);
        lv_obj_clear_state(dim_after_lbl, LV_STATE_DISABLED);
        lv_obj_clear_state(seconds_lbl, LV_STATE_DISABLED);
        lv_obj_clear_state(at_lbl, LV_STATE_DISABLED);
        lv_obj_clear_state(pct_lbl, LV_STATE_DISABLED);        
    }

    /* Off row */
    lv_obj_t *row_off = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_off);
    lv_obj_set_flex_flow(row_off, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_off, 4, 0);
    lv_obj_set_style_pad_all(row_off, 0, 0);
    lv_obj_set_width(row_off, LV_PCT(100));
    lv_obj_set_height(row_off, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_off, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(row_off, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *time_lbl = lv_label_create(row_off);
    lv_label_set_text(time_lbl, "Screen OFF");
    lv_obj_add_flag(time_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_off_lbl = time_lbl;

    lv_obj_t *off_switch = lv_switch_create(row_off);
    lv_obj_set_style_pad_all(off_switch, 4, 0);
    styles_set_switch(off_switch);  
    lv_obj_add_event_cb(off_switch, settings_on_off_switch_changed, LV_EVENT_VALUE_CHANGED, ctx);
    ctx->ss_off_switch = off_switch;

    /* Off timing row */
    lv_obj_t *row_off_cfg = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_off_cfg);
    lv_obj_set_flex_flow(row_off_cfg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_off_cfg, 4, 0);
    lv_obj_set_style_pad_all(row_off_cfg, 0, 0);
    lv_obj_set_width(row_off_cfg, LV_PCT(100));
    lv_obj_set_height(row_off_cfg, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_off_cfg, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(row_off_cfg, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *off_after_lbl = lv_label_create(row_off_cfg);
    lv_label_set_text(off_after_lbl, "Turn off after");
    lv_obj_add_flag(off_after_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_off_after_lbl = off_after_lbl;

    ctx->ss_off_after_ta = lv_textarea_create(row_off_cfg);
    lv_obj_set_width(ctx->ss_off_after_ta, 50);
    lv_obj_clear_flag(ctx->ss_off_after_ta, LV_OBJ_FLAG_SCROLLABLE);
    styles_set_textarea(ctx->ss_off_after_ta); 
    lv_textarea_set_one_line(ctx->ss_off_after_ta, true);
    lv_textarea_set_max_length(ctx->ss_off_after_ta, 4);
    if (ctx->settings.off_time >= 0) {
        char buf[12];
        lv_snprintf(buf, sizeof(buf), "%d", ctx->settings.off_time);
        lv_textarea_set_placeholder_text(ctx->ss_off_after_ta, buf);
        if (ctx->settings.screen_off) {
            lv_textarea_set_text(ctx->ss_off_after_ta, buf);
        } else {
            lv_textarea_set_text(ctx->ss_off_after_ta, "");
        }
    } else {
        lv_textarea_set_placeholder_text(ctx->ss_off_after_ta, "");
        lv_textarea_set_text(ctx->ss_off_after_ta, "");
    }
    lv_obj_add_event_cb(ctx->ss_off_after_ta, settings_on_ss_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->ss_off_after_ta, settings_on_ss_textarea_focus, LV_EVENT_CLICKED, ctx);

    lv_obj_t *off_seconds_lbl = lv_label_create(row_off_cfg);
    lv_label_set_text(off_seconds_lbl, "seconds.");
    lv_obj_add_flag(off_seconds_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->ss_off_seconds_lbl = off_seconds_lbl;

    /* Action row */
    lv_obj_t *row_actions = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_actions);
    lv_obj_set_flex_flow(row_actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_actions, 6, 0);
    lv_obj_set_style_pad_all(row_actions, 0, 0);
    lv_obj_set_width(row_actions, LV_PCT(100));
    lv_obj_set_height(row_actions, LV_SIZE_CONTENT);
    lv_obj_set_style_margin_top(row_actions, 10, 0);
    lv_obj_add_flag(row_actions, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *apply_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(apply_btn, 1);
    lv_obj_set_style_radius(apply_btn, 6, 0);
    lv_obj_t *apply_lbl = lv_label_create(apply_btn);
    lv_label_set_text(apply_lbl, "Apply");
    lv_obj_center(apply_lbl);
    lv_obj_add_flag(apply_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(apply_btn);
    lv_obj_add_event_cb(apply_btn, settings_apply_screensaver, LV_EVENT_CLICKED, ctx);
    
    lv_obj_t *cancel_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_radius(cancel_btn, 6, 0);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);
    lv_obj_add_flag(cancel_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, settings_close_screensaver, LV_EVENT_CLICKED, ctx);
    
    /* Keyboard anchored to bottom of overlay */
    ctx->ss_keyboard = lv_keyboard_create(overlay);
    styles_set_keyboard(ctx->ss_keyboard);
    lv_keyboard_set_mode(ctx->ss_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(ctx->ss_keyboard, NULL);
    lv_obj_add_flag(ctx->ss_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(ctx->ss_keyboard, LV_OBJ_FLAG_HIDDEN); /* show only after a field is tapped */
    lv_obj_add_event_cb(ctx->ss_keyboard, settings_on_ss_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->ss_keyboard, settings_on_ss_keyboard_event, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->ss_keyboard, settings_on_ss_keyboard_event, LV_EVENT_READY, ctx);
    lv_obj_align(ctx->ss_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

    if (ctx->settings.screen_off) {
        lv_obj_add_state(off_switch, LV_STATE_CHECKED);
        lv_obj_add_state(off_after_lbl, LV_STATE_DISABLED);
        lv_obj_add_state(off_seconds_lbl, LV_STATE_DISABLED);     
    } else {
        lv_obj_clear_state(off_switch, LV_STATE_CHECKED);
        lv_obj_clear_state(off_after_lbl, LV_STATE_DISABLED);
        lv_obj_clear_state(off_seconds_lbl, LV_STATE_DISABLED);       
    }

    settings_update_dim_controls_enabled(ctx, lv_obj_has_state(ctx->ss_dim_switch, LV_STATE_CHECKED));
    settings_update_off_controls_enabled(ctx, lv_obj_has_state(ctx->ss_off_switch, LV_STATE_CHECKED));

    return ESP_OK;
}

static void settings_apply_screensaver(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->screensaver_overlay) {
        return;
    }

    bool dim_on = ctx->ss_dim_switch && lv_obj_has_state(ctx->ss_dim_switch, LV_STATE_CHECKED);
    bool off_on = ctx->ss_off_switch && lv_obj_has_state(ctx->ss_off_switch, LV_STATE_CHECKED);

    int new_dim_time = ctx->settings.dim_time;
    int new_dim_level = ctx->settings.dim_level;
    int new_off_time = ctx->settings.off_time;

    /* Validate dim when enabled. */
    if (dim_on) {
        const char *dim_time_txt = ctx->ss_dim_after_ta ? lv_textarea_get_text(ctx->ss_dim_after_ta) : NULL;
        const char *dim_level_txt = ctx->ss_dim_pct_ta ? lv_textarea_get_text(ctx->ss_dim_pct_ta) : NULL;
        int parsed_time = 0;
        int parsed_level = 0;

        /* dim time: 1..9999 (textarea limited to 3 chars) */
        if (!settings_parse_int_range(dim_time_txt, 1, 9999, &parsed_time)) {
            settings_show_invalid_input();
            return;
        }

        /* Accept any 0..100 value, clamp later against brightness/minimum. */
        if (!settings_parse_int_range(dim_level_txt, 0, 100, &parsed_level)) {
            settings_show_invalid_input();
            return;
        }

        new_dim_time = parsed_time;
        new_dim_level = parsed_level;
    }

    /* Validate off when enabled. */
    if (off_on) {
        const char *off_time_txt = ctx->ss_off_after_ta ? lv_textarea_get_text(ctx->ss_off_after_ta) : NULL;
        int parsed_off = 0;
        if (!settings_parse_int_range(off_time_txt, 1, 99999, &parsed_off)) {
            settings_show_invalid_input();
            return;
        }
        new_off_time = parsed_off;
    }

    /* Apply in-memory state. Keep last valid values even when feature is off. */
    ctx->settings.screen_dim = dim_on;
    ctx->settings.dim_time = new_dim_time;
    /* Clamp dim level to brightness bounds even if feature disabled. */
    if (new_dim_level >= 0) {
        int max_level = ctx->settings.saved_brightness > 0 ? ctx->settings.saved_brightness : SETTINGS_DEFAULT_BRIGHTNESS;
        if (max_level < SETTINGS_MINIMUM_BRIGHTNESS) {
            max_level = SETTINGS_MINIMUM_BRIGHTNESS;
        }
        if (new_dim_level > max_level) new_dim_level = max_level;
        if (new_dim_level < SETTINGS_MINIMUM_BRIGHTNESS) new_dim_level = SETTINGS_MINIMUM_BRIGHTNESS;
    }

    ctx->settings.dim_level = new_dim_level;
    ctx->settings.screen_off = off_on;
    ctx->settings.off_time = new_off_time;

    /* Persist */
    persist_screensaver_to_nvs();
    settings_start_screensaver_timers();
    settings_close_screensaver(e);
}

static void settings_close_screensaver(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e); 
    if (ctx && ctx->screensaver_overlay) {
        lv_obj_del(ctx->screensaver_overlay);
        ctx->screensaver_overlay = NULL;
        ctx->screensaver_dialog = NULL;
        ctx->ss_dim_lbl = NULL;
        ctx->ss_dim_switch = NULL;
        ctx->ss_dim_after_lbl = NULL;
        ctx->ss_seconds_lbl = NULL;
        ctx->ss_at_lbl = NULL;
        ctx->ss_pct_lbl = NULL;
        ctx->ss_dim_after_ta = NULL;
        ctx->ss_dim_pct_ta = NULL;
        ctx->ss_off_lbl = NULL;
        ctx->ss_off_switch = NULL;
        ctx->ss_off_after_lbl = NULL;
        ctx->ss_off_seconds_lbl = NULL;
        ctx->ss_off_after_ta = NULL;
        ctx->ss_keyboard = NULL;
    }    
}
