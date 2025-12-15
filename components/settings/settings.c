#include "settings.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_rom_uart.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_lvgl_port.h"

#include "driver/rtc_io.h"
#include "driver/gpio.h"

#include "Goldman_Regular_35.h"
#include "bsp/esp-bsp.h"
#include "Domine_14.h"
#include "Domine_16.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "nvs.h"

#include "calibration_xpt2046.h"
#include "touch_xpt2046.h"
#include "sntp_header.h"
#include "styles.h"

#define SETTINGS_NVS_REFRESH_SNTP_STARTUP_KEY   "refresh_sntp"
#define SETTINGS_NVS_VALID_TIME_FLAG_KEY        "is_time_valid"
#define SETTINGS_NVS_MANUAL_RESTART_KEY         "manual_restart"
#define SETTINGS_NVS_AUTO_CONNECT_KEY           "auto_connect"
#define SETTINGS_NVS_CALIB_PROMPT_KEY           "calib_prompt"
#define SETTINGS_NVS_SNTP_RESULT_KEY            "sntp_result"
#define SETTINGS_NVS_BRIGHTNESS_KEY             "brightness_pct"
#define SETTINGS_NVS_DIM_LEVEL_KEY              "dim_level"
#define SETTINGS_NVS_DIM_TIME_KEY               "dim_time"
#define SETTINGS_NVS_OFF_TIME_KEY               "off_time"
#define SETTINGS_NVS_AP_SSID_KEY                "ap_ssid"
#define SETTINGS_NVS_AP_PWD_KEY                 "ap_pwd"
#define SETTINGS_NVS_OFF_EN_KEY                 "off_en"
#define SETTINGS_NVS_DIM_EN_KEY                 "dim_en"
#define SETTINGS_NVS_THEME_KEY                  "theme"
#define SETTINGS_NVS_TIME_KEY                   "time_epoch"
#define SETTINGS_NVS_ROT_KEY                    "rotation_step"
#define SETTINGS_NVS_SNTP_ERROR_KEY             "sntp_err"
#define SETTINGS_NVS_NS                         "settings"

#define SETTINGS_DEFAULT_STARTUP_SNTP_AUTO_CONNECT  false
#define SETTINGS_DEFAULT_REFRESH_SNTP_STARTUP       false
#define SETTINGS_DEFAULT_RUNNING_CALIBRATION        false
#define SETTINGS_DEFAULT_CALI_PROMPT_ENABLE         true
#define SETTINGS_DEFAULT_SCREEN_DIM_LEVEL           -1
#define SETTINGS_DEFAULT_MANUAL_RESTART             false
#define SETTINGS_DEFAULT_SCREEN_DIM_TIME            -1
#define SETTINGS_DEFAULT_SCREEN_OFF_TIME            -1
#define SETTINGS_DEFAULT_ROTATION_STEP              3
#define SETTINGS_DEFAULT_SNTP_SUCCESS               false 
#define SETTINGS_DEFAULT_BRIGHTNESS                 100
#define SETTINGS_DEFAULT_SCREEN_DIM                 false
#define SETTINGS_DEFAULT_SCREEN_OFF                 false
#define SETTINGS_DEFAULT_DARK_THEME                 true
#define SETTINGS_DEFAULT_TIME_VALID                 false
#define SETTINGS_DEFAULT_SNTP_ERR_CODE              ESP_ERR_INVALID_STATE

#define SETTINGS_CALIBRATION_TASK_STACK  (10 * 1024)
#define SETTINGS_CALIBRATION_TASK_PRIO   (5)
#define SETTINGS_MINIMUM_BRIGHTNESS      1   /**< Lowest brightness percentage to avoid black screen */
#define SETTINGS_AP_SSID_MAX_LEN         32
#define SETTINGS_AP_PWD_MAX_LEN          63
#define SETTINGS_ROTATION_STEPS          4
#define SETTINGS_DIM_FADE_MS             500
#define SETTINGS_OFF_FADE_MS             500
#define SETTINGS_UP_FADE_MS              250

#if CONFIG_APP_ENABLE_DUAL_CORE
    #define SETTINGS_LIGHT_SLEEP_TASK_CORE          1
#else
    #define SETTINGS_LIGHT_SLEEP_TASK_CORE          0
#endif                                
#define SETTINGS_LIGHT_SLEEP_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define SETTINGS_LIGHT_SLEEP_TASK_STACK_WORDS   (4 * 1024)
#define TOUCH_IRQ_WAKE_LEVEL                    0

_Static_assert(SETTINGS_CALIBRATION_TASK_STACK > 0, "Calibration task stack must be positive");
_Static_assert(SETTINGS_CALIBRATION_TASK_PRIO > 0, "Calibration task priority must be positive");
_Static_assert(SETTINGS_CALIBRATION_TASK_PRIO < configMAX_PRIORITIES, "Calibration task priority exceeds configMAX_PRIORITIES");
_Static_assert(SETTINGS_AP_SSID_MAX_LEN > 0, "AP SSID max length must be positive");
_Static_assert(SETTINGS_AP_PWD_MAX_LEN > 0, "AP password max length must be positive");
_Static_assert(SETTINGS_ROTATION_STEPS > 0, "Rotation steps must be positive");
_Static_assert((SETTINGS_DEFAULT_ROTATION_STEP >= 0) && (SETTINGS_DEFAULT_ROTATION_STEP < SETTINGS_ROTATION_STEPS),
               "Default rotation step out of range");
_Static_assert((SETTINGS_DEFAULT_BRIGHTNESS >= SETTINGS_MINIMUM_BRIGHTNESS) && (SETTINGS_DEFAULT_BRIGHTNESS <= 100),
               "Default brightness must be between minimum and 100");
_Static_assert(SETTINGS_DIM_FADE_MS > 0, "Dim fade time must be positive");
_Static_assert(SETTINGS_OFF_FADE_MS > 0, "Off fade time must be positive");
_Static_assert(SETTINGS_UP_FADE_MS > 0, "Up fade time must be positive");

#define STR_HELPER(x)               #x
#define STR(x)                      STR_HELPER(x)

static const char *TAG = "settings";

typedef struct{
    int screen_rotation_step;           /**< Current rotation step (0-3) applied to display */
    int saved_rotation_step;            /**< Last persisted rotation step */
    int brightness;                     /**< Current brightness percentage */
    int saved_brightness;               /**< Last persisted brightness percentage */
    int dim_time;
    int dim_level;
    int off_time;
    bool time_valid;                    /**< True if a valid time was set/restored */
    bool screen_dim;
    bool screen_off;
}settings_display_t;

typedef struct{
    bool startup_sntp_auto_connect;
    bool refresh_sntp_startup;
    esp_err_t sntp_last_err;
    bool sntp_success;
    int dt_month;
    int dt_day;
    int dt_year;
    int dt_hour;
    int dt_minute;
    int dt_second;
}settings_time_t;

typedef struct{
    char ap_ssid[SETTINGS_AP_SSID_MAX_LEN + 1];
    char ap_pwd[SETTINGS_AP_PWD_MAX_LEN + 1];
    bool calibration_prompt_enabled;    /**< True to ask for calibration at startup */
    bool running_calibration;
    bool manual_restart;
    bool dark_theme;
    settings_display_t display;
    settings_time_t time;
}settings_t;

typedef struct{
    lv_obj_t *return_screen;            /**< Screen to return to on close */
    lv_obj_t *screen;                   /**< Root LVGL screen object */
    lv_obj_t *toolbar;                  /**< Toolbar container */
    lv_obj_t *brightness_label;         /**< Label showing current brightness percent */
    lv_obj_t *brightness_slider;        /**< Slider to pick brightness percent */
    lv_obj_t *restart_confirm_mbox;     /**< Active restart confirmation dialog (NULL when closed) */
    lv_obj_t *reset_confirm_mbox;       /**< Active reset confirmation dialog (NULL when closed) */
    lv_obj_t *theme_confirm_mbox;       /**< Active theme change confirmation dialog (NULL when closed) */
    lv_obj_t *sntp_confirm_mbox;        /**< Active sntp confirmation dialog (NULL when closed) */
    lv_obj_t *datetime_overlay;         /**< Active date&time overlay (NULL when closed) */
    lv_obj_t *screensaver_overlay;      /**< Active screensaver overlay (NULL when closed) */
    lv_obj_t *wifi_sntp_overlay;        /**< Active Wi-Fi & SNTP overlay (NULL when closed) */
    lv_obj_t *dt_month_ta;              /**< Month input (MM) */
    lv_obj_t *dt_day_ta;                /**< Day input (DD) */
    lv_obj_t *dt_year_ta;               /**< Year input (YY) */
    lv_obj_t *dt_hour_ta;               /**< Hour input (HH) */
    lv_obj_t *dt_min_ta;                /**< Minute input (MM) */
    lv_obj_t *dt_keyboard;              /**< On-screen keyboard for date&time dialog */
    lv_obj_t *dt_dialog;                /**< Date&time dialog container */
    lv_obj_t *screensaver_dialog;       /**< Screensaver dialog container */
    lv_obj_t *wifi_sntp_dialog;         /**< Wi-Fi & SNTP dialog container */
    lv_obj_t *access_point_dialog;      /**< Wi-Fi & SNTP dialog container */
    lv_obj_t *access_point_keyboard;    /**< On-screen keyboard for access point dialog */
    lv_obj_t *access_point_ssid_ta;     /**< SSID input for access point dialog */
    lv_obj_t *access_point_pwd_ta;      /**< Password input for access point dialog */    
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
} settings_ctx_graphics_t;

typedef struct{
    bool active;                        /**< True while the settings screen is active */
    bool changing_brightness;           /**< True while changing values to the brightness slider */
    settings_ctx_graphics_t graphics;   /**< UI lvgl objects */
    settings_t settings;                /**< Information about the current session */
}settings_ctx_t;

static settings_ctx_t s_settings_ctx;
static esp_timer_handle_t s_ss_off_timer = NULL;
static esp_timer_handle_t s_ss_dim_timer = NULL;
static esp_timer_handle_t s_fade_timer = NULL;
static int s_fade_target = 0;
static int s_fade_steps_left = 0;
static int s_fade_direction = 0;
static bool s_wake_in_progress = false;
static bool s_lv_timers_paused = false;
static volatile bool s_brightness_ui_pending = false;
static TaskHandle_t s_light_sleep_task_handle = NULL;
static volatile bool s_light_sleep_pending = false;
static bool s_display_rst_hold = false;

/**
 * @brief Turn the backlight on after a short delay without clearing the screen.
 */
static void backlight_on_without_wipe_effect(void);

/**
 * @brief Show the startup splash screen and keep it visible briefly.
 */
static void startup_splash_screen(void);
 
/**
 * @brief Notify the light-sleep task to enter sleep after fade-to-off completes.
 */
static void notify_light_sleep_task(void);

/**
 * @brief Configure and hold the display reset line high for light sleep.
 */
static void hold_display_reset_high(void);

/**
 * @brief Enable EXT0 wakeup on the touch IRQ GPIO.
 *
 * @return true if EXT0 wakeup was configured, false otherwise.
 */
static bool enable_touch_ext0_wakeup(void);

/**
 * @brief Release the held display reset line after waking from light sleep.
 */
static void release_display_reset_hold(void);

/**
 * @brief Check if the configured touch IRQ GPIO supports EXT0 wakeup.
 *
 * @return true if the GPIO is RTC-capable and valid for wakeup.
 */
static bool touch_interrupt_can_wakeup(void);

/**
 * @brief Restore display state and restart screensaver timers after light sleep.
 */
static void refresh_display_after_light_sleep(void);

/**
 * @brief Pause LVGL timers while entering light sleep to avoid heavy callbacks.
 */
static void pause_lvgl_timers_for_sleep(void);

/**
 * @brief Resume LVGL timers after waking from light sleep.
 */
static void resume_lvgl_timers_after_sleep(void);

/**
 * @brief Task that waits for a notification then enters light sleep until touch wakeup.
 *
 * @param param Unused task parameter.
 */
static void screensaver_light_sleep_task(void *param);

/**
 * @brief Create the screensaver light-sleep task if it is not already running.
 */
static void initialize_screensaver_light_sleep_task(void);

/**
 * @brief Connect to Wi-Fi, sync time via SNTP, persist the result, and restart.
 */
static void get_sntp_time(void);

/**
 * @brief Connect to Wi-Fi (STA), perform SNTP sync, and persist the outcome.
 *
 * Attempts to start the station Wi-Fi stack, then runs SNTP. Updates
 * in-memory status flags and persists the last SNTP result/error code.
 */
static void sntp_connect(void);

/**
 * @brief Build the settings screen (header + scrollable settings list).
 *
 * Creates the root screen, toolbar (Back/About), and the scrollable list of settings.
 *
 * @param ctx Active settings context.
 */
static void build_settings_menu(settings_ctx_t *ctx);

/**
 * @brief Show the About overlay with setting descriptions.
 *
 * Opens a modal overlay on the top layer with descriptive labels and an OK button.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void build_on_about_dlg(lv_event_t *e);

/**
 * @brief Close handler for the About overlay.
 *
 * Deletes the overlay provided via event user data.
 *
 * @param e LVGL event (CLICKED) with user data = overlay obj.
 */
static void close_about_dlg(lv_event_t *e);

/**
 * @brief Update brightness level when the slider value changes.
 *
 * Refreshes the brightness label and drives the backlight to the new level.
 *
 * @param e LVGL event (VALUE_CHANGED) with user data = settings_ctx_t*.
 */
static void on_brightness_changed(lv_event_t *e);

/**
 * @brief Back button handler for the settings screen.
 *
 * Retrieves the settings context from event user data and closes the settings UI.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void back_from_settings(lv_event_t *e);

/**
 * @brief Close the settings screen and restore the previous screen.
 *
 * Persists brightness/rotation changes to NVS when needed, marks the context inactive,
 * and loads @ref settings_ctx_t::return_screen if set.
 *
 * @param ctx Active settings context.
 */
static void close_settings(settings_ctx_t *ctx);

/**
 * @brief Show a restart confirmation overlay.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void build_restart_ui(lv_event_t *e);

/**
 * @brief Read brightness slider value, clamp to safe bounds, and cache it in context.
 *
 * This only updates the in-memory brightness; callers decide if/when to persist.
 *
 * @param ctx Active settings context.
 */
static void update_brightness_value(settings_ctx_t *ctx);

/**
 * @brief Handler for confirming restart from the overlay.
 *
 * Persists pending brightness/rotation changes and then triggers a restart.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void confirm_restart(lv_event_t *e);

/**
 * @brief Close the restart overlay without restarting.
 *
 * Dismisses the confirmation dialog and clears the stored pointer.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_restart(lv_event_t *e);

/**
 * @brief Show a reset confirmation overlay.
 *
 * Creates a confirmation dialog for resetting settings and stores it in the context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void build_reset_ui(lv_event_t *e);

/**
 * @brief Confirm reset, restore defaults, and reinitialize settings.
 *
 * Resets all configurable settings to defaults, persists them, reinitializes runtime state,
 * and closes the reset dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void confirm_reset(lv_event_t *e);

/**
 * @brief Toggle theme (light/dark) and restart system.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void toggle_theme(lv_event_t *e);

/**
 * @brief Open the Wi-Fi & SNTP dialog from the settings screen.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void wifi_sntp_dialog(lv_event_t *e);

/**
 * @brief Returns whether Wi-Fi auto-connect at startup is enabled.
 *
 * @return true if auto-connect is persisted as enabled; false otherwise.
 */
static bool get_auto_connect_state(void);

/**
 * @brief Persist the Wi-Fi auto-connect preference.
 *
 * @param enable True to enable auto-connect on boot, false to disable.
 */
static void set_auto_connect_state(bool enable);

/**
 * @brief Event handler for the startup auto-connect switch.
 *
 * Reads the switch state and updates the stored preference.
 *
 * @param e LVGL event (VALUE_CHANGED) with target = switch.
 */
static void ui_on_startup_switch_changed(lv_event_t *e);

/**
 * @brief Build the Wi-Fi & SNTP overlay dialog UI.
 *
 * Destroys any previous dialog and recreates the overlay with AP, refresh, and auto-connect controls.
 *
 * @param ctx Active settings context.
 */
static void build_wifi_sntp_dialog(settings_ctx_t *ctx);

/**
 * @brief Build the confirmation message box for SNTP refresh.
 *
 * Closes any existing SNTP confirmation box, then creates a new one with Yes/Cancel actions.
 *
 * @param ctx Active settings context.
 */
static void build_refresh_sntp_msgbox(settings_ctx_t *ctx);

/**
 * @brief Prompt user confirmation to refresh SNTP (requires restart).
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void refresh_sntp(lv_event_t *e);

/**
 * @brief Build the Access Point configuration dialog inside the Wi-Fi overlay.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void build_wifi_connection_dialog(lv_event_t *e);

/**
 * @brief Saves theme and other the rest of the custom flags to NVS and restarts.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void theme_confirm(lv_event_t *e);

/**
 * @brief Confirm SNTP refresh and flag the request for restart processing.
 *
 * Persists the refresh-on-startup flag and triggers a restart dialog flow.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void sntp_confirm(lv_event_t *e);

/**
 * @brief Closes the theme change dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_theme_msgbox(lv_event_t *e);

/**
 * @brief Cancel SNTP refresh request and close the confirmation dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void cancel_sntp(lv_event_t *e);

/**
 * @brief Show and attach the AP keyboard when SSID/password fields gain focus.
 *
 * @param e LVGL event (FOCUSED or CLICKED) with user data = settings_ctx_t*.
 */
static void on_ap_textarea_focus(lv_event_t *e);

/**
 * @brief Hide the AP keyboard when tapping outside editable areas.
 *
 * Ignores taps on the keyboard itself or the currently focused textarea.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void on_ap_background_tap(lv_event_t *e);

/**
 * @brief Handle AP keyboard cancel/ready events and hide it when finished.
 *
 * @param e LVGL event (CANCEL or READY) with user data = settings_ctx_t*.
 */
static void on_ap_keyboard_event(lv_event_t *e);

/**
 * @brief Detach and hide the AP keyboard, then reset dialog alignment.
 *
 * @param ctx Active settings context.
 */
static void hide_ap_keyboard(settings_ctx_t *ctx);

/**
 * @brief Realign the AP dialog to keep the active textarea visible above the keyboard.
 *
 * @param ctx Active settings context.
 * @param ta  Focused textarea requesting alignment (NULL to reset).
 */
static void realign_ap_dialog(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Close the reset confirmation overlay without applying changes.
 *
 * Dismisses the reset dialog and clears its pointer from the context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_reset(lv_event_t *e);

/**
 * @brief Launch touch calibration from Settings (async).
 *
 * Cleans the current settings screen, marks the context inactive, and spawns a
 * FreeRTOS task to run the calibration flow without blocking the LVGL handler.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void start_calibration_task(lv_event_t *e);

/**
 * @brief Open the screensaver dialog from settings.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void open_screensaver(lv_event_t *e);

/**
 * @brief Build the screensaver dialog UI and attach it to the top layer.
 *
 * @param ctx Active settings context.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ctx is NULL.
 */
static esp_err_t build_screensaver_dialog(settings_ctx_t *ctx);

/**
 * @brief Apply screensaver settings from the dialog (Dim/Off).
 *
 * Parses inputs, validates, persists to NVS, (re)starts timers and closes dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void apply_screensaver(lv_event_t *e);

/**
 * @brief Validate dim inputs and populate parsed values.
 *
 * Parses dim delay and level text fields, enforcing allowed ranges; shows
 * an "Incorrect Input" message on failure.
 *
 * @param[out] new_dim_time Parsed dim delay (seconds).
 * @param[out] new_dim_level Parsed dim level percent.
 * @param[in]  ctx Settings context.
 * @return true if parsing succeeds; false otherwise.
 */
static bool dim_valid(int *new_dim_time, int *new_dim_level, settings_ctx_t *ctx);

/**
 * @brief Validate off input and populate parsed value.
 *
 * Parses screen-off delay field; shows an "Incorrect Input" message on failure.
 *
 * @param[out] new_off_time Parsed off delay (seconds).
 * @param[in]  ctx Settings context.
 * @return true if parsing succeeds; false otherwise.
 */
static bool off_valid(int *new_off_time, settings_ctx_t *ctx);

/**
 * @brief Apply dim values to in-memory settings with clamping.
 *
 * Updates dim flags/time/level in the context and clamps level to brightness bounds.
 *
 * @param ctx Settings context.
 * @param dim_on Whether dimming is enabled.
 * @param new_dim_level Parsed dim level (in/out, clamped).
 * @param new_dim_time Parsed dim delay (seconds).
 */
static void apply_in_memory_state(settings_ctx_t *ctx, bool dim_on, int *new_dim_level, int new_dim_time);

/**
 * @brief Collect and validate screensaver values from the dialog.
 *
 * Reads UI controls for dim/off, validates inputs, applies clamped values to
 * the settings context, and returns success/failure.
 *
 * @param ctx Settings context.
 * @return true on success; false if validation failed.
 */
static bool obtain_screensaver_values(settings_ctx_t *ctx);

/**
 * @brief Persist Access Point credentials from the dialog and close it.
 *
 * Copies SSID/password text, saves to NVS, and dismisses the AP dialog.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void apply_ap_data(lv_event_t *e);

/**
 * @brief Close the screensaver dialog without applying changes.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_screensaver(lv_event_t *e);

/**
 * @brief Close the Wi-Fi/SNTP connection dialog overlay.
 *
 * Deletes the overlay and clears cached pointers in the context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_connection_dialog(lv_event_t *e);

/**
 * @brief Close the Access Point configuration dialog and rebuild the Wi-Fi dialog.
 *
 * Deletes the overlay, clears stored pointers, and recreates the Wi-Fi/SNTP dialog to return
 * the user to the previous view.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_access_point_dialog(lv_event_t *e);

/**
 * @brief Background task to run touch calibration and restore UI state.
 *
 * Temporarily forces default rotation for calibration, runs @ref calibration_run_cal,
 * restores the previous rotation, and reopens the settings screen.
 *
 * @param param settings_ctx_t* passed from @ref start_calibration_task.
 */
static void calibration_task(void *param);

/**
 * @brief Clear cached LVGL object pointers in the settings context.
 *
 * Call after deleting/cleaning the settings UI to avoid dangling pointers being
 * used by async callbacks (e.g., brightness updates).
 */
static void clear_ui_refs(settings_ctx_t *ctx);

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
 * @param power_reset Current reset reason (hard vs software/manual restart).
 */
static void sntp_startup(bool power_reset);

/**
 * @brief Apply the desired timezone to the C library clock.
 */
static void apply_timezone(void);

/**
 * @brief Apply the current rotation step to the active LVGL display.
 *
 * Maps @ref s_settings_ctx.settings.display.screen_rotation_step to an LVGL display rotation and sets it,
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
 * Writes @ref s_settings_ctx.settings.display.screen_rotation_step to @ref SETTINGS_NVS_ROT_KEY inside
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
static void rotate_screen(lv_event_t *e);

/**
 * @brief Build the date&time dialog overlay and wire its events.
 *
 * Destroys any existing dialog, constructs the overlay, text areas, action buttons,
 * and numeric keyboard, and stores pointers in the shared settings context.
 *
 * @param ctx Active settings context (must be non-NULL).
 */
static void build_date_time_dialog(settings_ctx_t *ctx);

/**
 * @brief Settings button handler to open the date&time dialog.
 *
 * Uses @ref build_date_time_dialog() to display the picker.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void set_date_time(lv_event_t *e);

/**
 * @brief Build the splash screen shown at startup.
 */
static void build_splash_screen(void);

/**
 * @brief Translate the last SNTP/Wi-Fi error into a short user-facing reason.
 *
 * Provides readable text for common connection and time-sync failures. When
 * the code is not handled explicitly, the esp_err_to_name string is used as a
 * fallback.
 *
 * @param err Last esp_err_t recorded during SNTP/Wi-Fi initialization.
 * @return const char* Static description of the failure reason.
 */
static const char *get_time_failure_reason(esp_err_t err);

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
static void apply_date_time(lv_event_t *e);

/**
 * @brief Parse date/time inputs from the dialog, validate, and return as time_t.
 *
 * Reads all date/time text fields, validates ranges and calendar date, persists
 * the "time valid" flag, updates the in-memory time fields, and notifies listeners.
 * Returns -1 and shows an "Incorrect Input" message box on validation failure.
 *
 * @param ctx Active settings context.
 * @return time_t seconds since epoch (UTC) on success, -1 on failure.
 */
static time_t build_date_time_data(settings_ctx_t *ctx);

/**
 * @brief Close handler for the date&time dialog (Cancel or overlay tap).
 *
 * Deletes the overlay and clears dialog-related pointers in the settings context.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void close_set_date_time(lv_event_t *e);

/**
 * @brief OK handler for the invalid-input message box.
 *
 * @param e LVGL event (CLICKED) with user data = msgbox obj.
 */
static void close_invalid_message_mbox(lv_event_t *e);

/**
 * @brief Show a simple "Incorrect Input" message box.
 */
static void show_invalid_input_mbox(void);

/**
 * @brief Parse an integer from text and clamp to a [min, max] range.
 *
 * @param txt Input string.
 * @param min Minimum accepted value.
 * @param max Maximum accepted value.
 * @param out_val Parsed integer on success.
 * @return true if parse and range check succeed; false otherwise.
 */
static bool parse_int_range(const char *txt, int min, int max, int *out_val);

/**
 * @brief Textarea focus/click handler to prep the keyboard and clear placeholders.
 *
 * @param e LVGL event (FOCUSED/CLICKED) with user data = settings_ctx_t*.
 */
static void on_dt_textarea_focus(lv_event_t *e);

/**
 * @brief Overlay/dialog tap handler to hide the keyboard when tapping outside fields.
 *
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void on_dt_background_tap(lv_event_t *e);

/**
 * @brief Keyboard CANCEL/READY handler to hide the keyboard.
 *
 * @param e LVGL event (CANCEL/READY) with user data = settings_ctx_t*.
 */
static void on_dt_keyboard_event(lv_event_t *e);

/**
 * @brief Textarea defocus handler to restore placeholders when left empty.
 *
 * @param e LVGL event (DEFOCUSED) with user data = settings_ctx_t*.
 */
static void on_dt_textarea_defocus(lv_event_t *e);

/**
 * @brief Scroll the dialog so the given field (or its row) stays visible.
 *
 * @param ctx Settings context.
 * @param ta  Target textarea to bring into view.
 */
static void scroll_field_into_view(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Hide the date&time keyboard and detach it from any textarea.
 *
 * @param ctx Settings context.
 */
static void hide_dt_keyboard(settings_ctx_t *ctx);

/**
 * @brief Re-align the date&time dialog based on which textarea is active.
 * @param ctx Settings context.
 * @param ta  Active textarea (NULL to reset to default position).
 */
static void realign_dt_dialog(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Focus/click handler for screensaver numeric fields (dim delay/percent).
 * @param e LVGL event with user data = settings_ctx_t*.
 */
static void on_ss_textarea_focus(lv_event_t *e);

/**
 * @brief Toggle handler for screensaver dim switch to enable/disable related fields.
 * @param e LVGL event (VALUE_CHANGED) with user data = settings_ctx_t*.
 */
static void on_dim_switch_changed(lv_event_t *e);

/**
 * @brief Apply enabled/disabled state to dimming controls based on switch state.
 * @param ctx Settings context.
 * @param enabled True to enable fields, false to disable.
 */
static void update_dim_controls_enabled(settings_ctx_t *ctx, bool enabled);

/**
 * @brief Toggle label style/opacity based on enabled state.
 *
 * Applies disabled styling when @p enabled is false; no-op if label is NULL.
 *
 * @param lbl Label object to update (nullable).
 * @param enabled True to show as enabled, false to dim/disable.
 */
static void update_label(lv_obj_t *lbl, bool enabled);

/**
 * @brief Toggle textarea enabled state and style.
 *
 * Disables input, clears focus, and applies disabled style when @p enabled is false.
 * No-op if textarea is NULL.
 *
 * @param ta Textarea object to update (nullable).
 * @param enabled True to enable input, false to disable.
 */
static void update_textarea(lv_obj_t *ta, bool enabled);

/**
 * @brief Hide the screensaver keyboard and detach it from any textarea.
 * @param ctx Settings context.
 */
static void hide_ss_keyboard(settings_ctx_t *ctx);

/**
 * @brief Re-align the screensaver dialog based on which textarea is active.
 * @param ctx Settings context.
 * @param ta  Active textarea (NULL to reset to default position).
 */
static void realign_screensaver_dialog(settings_ctx_t *ctx, lv_obj_t *ta);

/**
 * @brief Overlay/dialog tap handler for screensaver dialog.
 * @param e LVGL event (CLICKED) with user data = settings_ctx_t*.
 */
static void on_ss_background_tap(lv_event_t *e);

/**
 * @brief Screensaver keyboard CANCEL/READY handler.
 * @param e LVGL event (CANCEL/READY) with user data = settings_ctx_t*.
 */
static void on_ss_keyboard_event(lv_event_t *e);

/**
 * @brief Start dim timer; delays and target level for screensaver dim.
 * @param seconds Delay in seconds before dim.
 */
static void screensaver_dim_start(int seconds);

/**
 * @brief Shared helper to (re)create and start a one-shot screensaver timer.
 *
 * @param timer_handle Pointer to timer handle (created on-demand).
 * @param cb           Timer callback.
 * @param name         Timer name (for esp_timer_create).
 * @param seconds      Delay before firing (negative treated as 0).
 */
static void screensaver_start_timer(esp_timer_handle_t *timer_handle, esp_timer_cb_t cb, const char *name, int seconds);

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
static void on_off_switch_changed(lv_event_t *e);

/**
 * @brief Apply enabled/disabled state to off controls based on switch state.
 * @param ctx Settings context.
 * @param enabled True to enable, false to disable.
 */
static void update_off_controls_enabled(settings_ctx_t *ctx, bool enabled);

/**
 * @brief esp_timer callback for delayed screen off.
 * @param arg Unused.
 */
static void off_timer_cb(void *arg);

/**
 * @brief esp_timer callback for delayed screen dim.
 * @param arg Unused.
 */
static void dim_timer_cb(void *arg);

/**
 * @brief Helper to animate brightness to a target percentage over a duration using esp_timer.
 * @param target_pct Target brightness percent.
 * @param duration_ms Fade duration in milliseconds.
 */
static void fade_brightness(int target_pct, uint32_t duration_ms);

/**
 * @brief Handle instant fades (duration 0 or no delta); returns true if handled.
 *
 * Applies brightness immediately when no animation is needed. Clears wake-in-progress
 * for downward fades.
 *
 * @param duration_ms Fade duration in milliseconds.
 * @param target_pct Target brightness percent.
 * @param start Current brightness percent.
 * @param rising True if fading upward.
 * @param ctx Active settings context.
 * @return true if the fade was handled instantly; false otherwise.
 */
static bool fade_handle_instant_update(uint32_t duration_ms, int target_pct, int start, bool rising, settings_ctx_t *ctx);

/**
 * @brief Ensure the fade timer exists and is stopped; returns false on failure.
 *
 * Creates the timer on demand; stops any previous run before reuse.
 *
 * @return true if the timer is ready; false on creation failure.
 */
static bool fade_ensure_timer_ready(void);

/**
 * @brief Configure fade globals; returns false if no steps are needed.
 *
 * Sets target, direction, and remaining steps; applies immediate brightness when
 * already at target.
 *
 * @param target_pct Target brightness percent.
 * @param start Current brightness percent.
 * @param ctx Active settings context.
 * @return true if steps were configured and animation should proceed; false if no steps.
 */
static bool fade_setup_steps(int target_pct, int start, settings_ctx_t *ctx);

/**
 * @brief Fade step timer callback for brightness animation.
 * @param arg Unused.
 */
static void fade_step_cb(void *arg);

/**
 * @brief Sync brightness slider/label to the current brightness value.
 * @param ctx Settings context.
 * @param val Brightness percent to display.
 */
static void sync_brightness_ui(settings_ctx_t *ctx, int val);

/**
 * @brief Async worker to update brightness UI elements (debounced).
 * @param arg Brightness value (cast from uintptr_t).
 */
static void sync_brightness_ui_async(void *arg);

/**
 * @brief Utility to check if an object is a descendant of another.
 *
 * @param obj            Candidate child object.
 * @param maybe_ancestor Candidate ancestor object.
 * @return true if obj is a descendant (or same) as maybe_ancestor; false otherwise.
 */
static bool is_descendant(lv_obj_t *obj, lv_obj_t *maybe_ancestor);

/**
 * @brief Validate a date considering leap years and month lengths.
 *
 * @param year_full Full year (e.g., 2025).
 * @param month     Month 1-12.
 * @param day       Day (1..n based on month and leap year).
 * @return true if the date is valid, false otherwise.
 */
static bool is_date_valid(int year_full, int month, int day);

/**
 * @brief Notify registered listeners that time was set via dialog Apply.
 */
static void notify_time_set(void);

/**
 * @brief Notify registered listeners that time was reset via settings reset.
 */
static void notify_time_reset(void);

/**
 * @brief Restore system time from NVS only after a software reset; clear otherwise.
 */
static void load_time_from_nvs(void);

/**
 * @brief Persist the given epoch seconds to NVS.
 *
 * @param epoch Epoch seconds to store.
 */
static void persist_time_to_nvs(time_t epoch);

/**
 * @brief Erase the stored time key from NVS.
 */
static void clear_time_in_nvs(void);

/**
 * @brief Remove the stored "time valid" flag from NVS.
 */
static void clear_valid_time_flag_in_nvs(void);

/**
 * @brief Persist the "time valid" flag to NVS.
 */
static void persist_valid_time_flag_to_nvs(void);

/**
 * @brief Persist the last SNTP synchronization result to NVS.
 */
static void persist_sntp_result(void);

/**
 * @brief Persist the access point credentials to NVS.
 */
static void persist_ap_credentials_to_nvs(void);

/**
 * @brief Load the access point credentials from NVS.
 */
static void load_ap_credentials_from_nvs(void);

/**
 * @brief Load the stored AP SSID into the settings context.
 *
 * @param h Open NVS handle for @ref SETTINGS_NVS_NS.
 */
static void load_ap_ssid(nvs_handle_t h);

/**
 * @brief Load the stored AP password into the settings context.
 *
 * @param h Open NVS handle for @ref SETTINGS_NVS_NS.
 */
static void load_ap_pwd(nvs_handle_t h);

/**
 * @brief Helper to load an NVS string value into a caller-provided buffer.
 *
 * Allocates a temporary buffer if the stored string is larger than @p buf_size,
 * then copies/truncates into the destination and null-terminates.
 *
 * @param h        Open NVS handle for @ref SETTINGS_NVS_NS.
 * @param key      NVS key to read (e.g., @ref SETTINGS_NVS_AP_SSID_KEY).
 * @param buf      Destination buffer.
 * @param buf_size Size of @p buf in bytes.
 */
static void load_user_data(nvs_handle_t h, const char *key, char *buf, size_t buf_size);

/**
 * @brief Persist the SNTP refresh-on-startup preference to NVS.
 */
static void persist_sntp_refresh(void);

/**
 * @brief Load the SNTP refresh-on-startup preference from NVS.
 */
static void load_sntp_refresh_from_nvs(void);

/**
 * @brief Load the last SNTP synchronization result from NVS.
 */
static void load_sntp_result_from_nvs(void);

/**
 * @brief Persist the manual restart flag to NVS.
 */
static void persist_manual_restart(void);

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

void settings_starting_routine(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    bool power_reset = (reason == ESP_RST_POWERON);
    
    /* ----- NSV ----- */
    ESP_LOGI(TAG, "Initializing NVS");
    ESP_ERROR_CHECK(init_nvs());

    /* ----- Display and LVGL ----- */
    ESP_LOGI(TAG, "Starting bsp for ILI9341 display");
    ESP_ERROR_CHECK(bsp_display_start_result());
    if (power_reset) bsp_display_backlight_off();
    styles_init_colors();

    /* ----- Power Saving ----- */
    initialize_screensaver_light_sleep_task();

    /* ----- Configurations ----- */
    ESP_LOGI(TAG, "Loading configurations");
    init_settings();
    apply_default_font_theme(true);

    /* ----- Wi-Fi & SNTP ----- */
    if (power_reset){
        ESP_LOGI(TAG, "Showing splash & connection screens");   
        startup_splash_screen();  
        if (s_settings_ctx.settings.time.startup_sntp_auto_connect) get_sntp_time();
    }else{
        sntp_startup(power_reset);
        s_settings_ctx.settings.time.refresh_sntp_startup = false;
        persist_sntp_refresh();
    }

    /* ----- XPT2046 Driver Init ----- */
    ESP_LOGI(TAG, "Initializing XPT2046 touch driver");
    ESP_ERROR_CHECK(touch_init()); 
    ESP_LOGI(TAG, "Registering touch driver to LVGL");
    ESP_ERROR_CHECK(touch_register_to_lvgl());
    ESP_LOGI(TAG, "Load touch driver calibration data");
    bool calibration_found;
    calibration_load_cal_data(&calibration_found);

    /* ----- XPT2046 Calibration ----- */
    if (s_settings_ctx.settings.calibration_prompt_enabled){
        ESP_LOGI(TAG, "Start calibration dialog");
        calibration_set_show_loader(true);
        settings_set_running_calibration(true);
        ESP_ERROR_CHECK(calibration_run_cal(calibration_found));
        settings_set_running_calibration(false);
    }
}

esp_err_t settings_open_settings(lv_obj_t *return_screen)
{
    if (!return_screen){
        return ESP_ERR_INVALID_ARG;
    }

    settings_ctx_t *ctx = &s_settings_ctx;
    if (!ctx->graphics.screen){
        build_settings_menu(ctx);
    }

    ctx->active = true;
    ctx->graphics.return_screen = return_screen;
    lv_screen_load(ctx->graphics.screen);

    return ESP_OK;
}

void settings_show_date_time_dialog(lv_obj_t *return_screen)
{
    settings_ctx_t *ctx = &s_settings_ctx;
    ctx->graphics.return_screen = return_screen;
    build_date_time_dialog(ctx);
}

void settings_show_sntp_dialog(lv_obj_t *return_screen)
{
    settings_ctx_t *ctx = &s_settings_ctx;
    ctx->graphics.return_screen = return_screen;
    build_wifi_sntp_dialog(ctx);
}

void settings_register_time_callbacks(void (*on_time_set)(void),
                                      void (*on_time_reset)(void))
{
    s_time_set_cb = on_time_set;
    s_time_reset_cb = on_time_reset;

    if (s_settings_ctx.settings.display.time_valid) {
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
        persist_time_to_nvs(now);
    }
}

bool settings_is_time_valid(void)
{
    return s_settings_ctx.settings.display.time_valid == true;
}

void settings_fade_to_saved_brightness(void)
{
    int target = s_settings_ctx.settings.display.saved_brightness;
    if (target < SETTINGS_MINIMUM_BRIGHTNESS) target = SETTINGS_MINIMUM_BRIGHTNESS;
    if (target > 100) target = 100;
    fade_brightness(target, SETTINGS_UP_FADE_MS);
}

void settings_start_screensaver_timers(void)
{
    bool dim_allowed = s_settings_ctx.settings.display.screen_dim &&
                        (!s_settings_ctx.settings.display.screen_off ||
                        s_settings_ctx.settings.display.off_time <= 0 ||
                        s_settings_ctx.settings.display.dim_time <= 0 ||
                        s_settings_ctx.settings.display.dim_time < s_settings_ctx.settings.display.off_time);

    if (dim_allowed) {
        screensaver_dim_start(s_settings_ctx.settings.display.dim_time);
    } else {
        screensaver_dim_stop();
    }

    if (s_settings_ctx.settings.display.screen_off) {
        screensaver_off_start(s_settings_ctx.settings.display.off_time);
    } else {
        screensaver_off_stop();
    }
}

int settings_get_active_brightness(void){
    return s_settings_ctx.settings.display.brightness;
}

bool settings_is_wake_in_progress(void)
{
    return s_wake_in_progress;
}

bool settings_is_brightness_changing(void)
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

bool settings_is_theme_dark(void)
{
    return s_settings_ctx.settings.dark_theme;
}

void settings_set_dark_theme_flag(bool is_dark)
{
    s_settings_ctx.settings.dark_theme = is_dark;
}

char* settings_get_ap_ssid()
{
    return s_settings_ctx.settings.ap_ssid;
}

char* settings_get_ap_pwd(void)
{
    return s_settings_ctx.settings.ap_pwd;
}

static void get_sntp_time(void)
{
    show_connecting_message();
    lv_timer_handler();
    if (!s_settings_ctx.settings.display.time_valid && s_settings_ctx.settings.manual_restart){
        vTaskDelay(150);
        bsp_display_backlight_on(); 
    }
    bsp_display_stop();

    sntp_connect();

    s_settings_ctx.settings.manual_restart = false;
    persist_manual_restart();
    esp_restart();
}

static void sntp_connect(void)
{
    esp_err_t err = wifi_init_sta();
    if (err == ESP_OK){
        err = sntp_initialize();    
    }
    s_settings_ctx.settings.time.sntp_last_err = err;
    if (err == ESP_OK){
        s_settings_ctx.settings.time.sntp_success = true;
        save_time_data();
    }else{
        s_settings_ctx.settings.time.sntp_success = false;
    }
    persist_sntp_result();
}

static void backlight_on_without_wipe_effect(void)
{
    vTaskDelay(pdMS_TO_TICKS(150));
    bsp_display_backlight_on();
}

static void startup_splash_screen(void)
{
    show_splash_screen();
    backlight_on_without_wipe_effect();
    vTaskDelay(pdMS_TO_TICKS(1350));   
}

static void build_settings_menu(settings_ctx_t *ctx)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 2, 0);
    lv_obj_set_style_pad_gap(scr, 5, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    ctx->graphics.screen = scr;

    lv_obj_t *toolbar = lv_obj_create(scr);
    lv_obj_remove_style_all(toolbar);
    styles_set_card_color(toolbar, 0);
    lv_obj_set_size(toolbar, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(toolbar, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(toolbar, 3, 0);
    lv_obj_set_flex_align(toolbar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(toolbar, LV_OPA_COVER, 0);
    ctx->graphics.toolbar = toolbar;    

    lv_obj_t *back_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_set_style_pad_all(back_btn, 6, 0);    
    styles_set_button(back_btn);
    lv_obj_add_event_cb(back_btn, back_from_settings, LV_EVENT_CLICKED, ctx);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_center(back_lbl);

    lv_obj_t *about_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(about_btn, 6, 0);
    lv_obj_set_style_pad_all(about_btn, 6, 0);   
    styles_set_button(about_btn); 
    lv_obj_add_event_cb(about_btn, build_on_about_dlg, LV_EVENT_CLICKED, ctx);
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

    ctx->graphics.brightness_label = lv_label_create(brightness_card);
    lv_obj_set_width(ctx->graphics.brightness_label, LV_PCT(100));
    lv_obj_set_style_text_align(ctx->graphics.brightness_label, LV_TEXT_ALIGN_CENTER, 0);
    styles_set_text_color(ctx->graphics.brightness_label, 0);

    ctx->graphics.brightness_slider = lv_slider_create(brightness_card);
    lv_obj_set_width(ctx->graphics.brightness_slider, LV_PCT(90));
    lv_slider_set_range(ctx->graphics.brightness_slider, SETTINGS_MINIMUM_BRIGHTNESS, 100);
    lv_slider_set_value(ctx->graphics.brightness_slider, ctx->settings.display.brightness, LV_ANIM_OFF);
    lv_obj_add_event_cb(ctx->graphics.brightness_slider, on_brightness_changed, LV_EVENT_VALUE_CHANGED, ctx);
    styles_set_slider(ctx->graphics.brightness_slider);
    lv_obj_set_style_bg_opa(ctx->graphics.brightness_slider, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ctx->graphics.brightness_slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ctx->graphics.brightness_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(ctx->graphics.brightness_slider, 1, LV_PART_KNOB);
    lv_obj_set_style_radius(ctx->graphics.brightness_slider, 6, 0);
    lv_obj_set_style_radius(ctx->graphics.brightness_slider, 6, LV_PART_INDICATOR);
    lv_obj_set_style_radius(ctx->graphics.brightness_slider, 5, LV_PART_KNOB);

    int init_val = lv_slider_get_value(ctx->graphics.brightness_slider);
    char init_txt[32];
    lv_snprintf(init_txt, sizeof(init_txt), "Brightness: %d%%", init_val);
    lv_label_set_text(ctx->graphics.brightness_label, init_txt);

    /* Row: Screensaver + Change Theme*/
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
    lv_obj_add_event_cb(screen_saver_button, open_screensaver, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(screen_saver_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *screen_saver_lbl = lv_label_create(screen_saver_button);
    lv_label_set_text(screen_saver_lbl, "Screensaver");
    lv_obj_center(screen_saver_lbl);  

    lv_obj_t *theme_button = lv_button_create(row_actions0);
    lv_obj_set_flex_grow(theme_button, 1);
    lv_obj_set_style_radius(theme_button, 8, 0);
    lv_obj_set_style_pad_all(theme_button, 6, 0);  
    styles_set_button(theme_button);  
    lv_obj_add_event_cb(theme_button, toggle_theme, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(theme_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *theme_lbl = lv_label_create(theme_button);
    lv_label_set_text(theme_lbl, "Change Theme");
    lv_obj_center(theme_lbl);      

    /* Row: Manual Date&Time + Wi-Fi & SNTP */
    lv_obj_t *row_actions1 = lv_obj_create(settings_list);
    lv_obj_remove_style_all(row_actions1);
    lv_obj_set_flex_flow(row_actions1, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_actions1, LV_PCT(100));
    lv_obj_set_style_pad_gap(row_actions1, 2, 0);
    lv_obj_set_style_pad_all(row_actions1, 0, 0);
    lv_obj_set_height(row_actions1, LV_SIZE_CONTENT);

    lv_obj_t *set_date_time_button = lv_button_create(row_actions1);
    lv_obj_set_flex_grow(set_date_time_button, 1);
    lv_obj_set_style_radius(set_date_time_button, 8, 0);
    lv_obj_set_style_pad_all(set_date_time_button, 6, 0);  
    styles_set_button(set_date_time_button);  
    lv_obj_add_event_cb(set_date_time_button, set_date_time, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(set_date_time_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *set_date_time_lbl = lv_label_create(set_date_time_button);
    lv_label_set_text(set_date_time_lbl, "Manual Date&Time");
    lv_obj_center(set_date_time_lbl);    

    lv_obj_t *connection_button = lv_button_create(row_actions1);
    lv_obj_set_flex_grow(connection_button, 1);
    lv_obj_set_style_radius(connection_button, 8, 0);
    lv_obj_set_style_pad_all(connection_button, 6, 0);
    styles_set_button(connection_button);    
    lv_obj_add_event_cb(connection_button, wifi_sntp_dialog, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(connection_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *connection_lbl = lv_label_create(connection_button);
    lv_label_set_text(connection_lbl, "Wi-Fi & SNTP");
    lv_obj_center(connection_lbl);      

    /* Row: Rotate Screen + Run Calibration */
    lv_obj_t *row_actions2 = lv_obj_create(settings_list);
    lv_obj_remove_style_all(row_actions2);
    lv_obj_set_flex_flow(row_actions2, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_actions2, LV_PCT(100));
    lv_obj_set_style_pad_gap(row_actions2, 2, 0);
    lv_obj_set_style_pad_all(row_actions2, 0, 0);
    lv_obj_set_height(row_actions2, LV_SIZE_CONTENT);

    lv_obj_t *rotate_button = lv_button_create(row_actions2);
    lv_obj_set_flex_grow(rotate_button, 1);
    lv_obj_set_style_radius(rotate_button, 8, 0);
    lv_obj_set_style_pad_all(rotate_button, 6, 0);    
    styles_set_button(rotate_button);
    lv_obj_add_event_cb(rotate_button, rotate_screen, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(rotate_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *rotate_lbl = lv_label_create(rotate_button);
    lv_label_set_text(rotate_lbl, "Rotate Screen");
    lv_obj_center(rotate_lbl);   

    lv_obj_t *calibration_button = lv_button_create(row_actions2);
    lv_obj_set_flex_grow(calibration_button, 1);
    lv_obj_set_style_radius(calibration_button, 8, 0);
    lv_obj_set_style_pad_all(calibration_button, 6, 0); 
    styles_set_button(calibration_button);
    lv_obj_add_event_cb(calibration_button, start_calibration_task, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(calibration_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *calibration_lbl = lv_label_create(calibration_button);
    lv_label_set_text(calibration_lbl, "Run Calibration");
    lv_obj_center(calibration_lbl);   

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
    lv_obj_add_event_cb(restart_button, build_restart_ui, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(restart_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *restart_lbl = lv_label_create(restart_button);
    lv_label_set_text(restart_lbl, "Restart");
    lv_obj_center(restart_lbl);

    lv_obj_t *reset_button = lv_button_create(row_actions3);
    lv_obj_set_flex_grow(reset_button, 1);
    lv_obj_set_style_radius(reset_button, 8, 0);
    lv_obj_set_style_pad_all(reset_button, 6, 0);
    styles_set_button(reset_button);    
    lv_obj_add_event_cb(reset_button, build_reset_ui, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(reset_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *reset_lbl = lv_label_create(reset_button);
    lv_label_set_text(reset_lbl, "Reset");
    lv_obj_center(reset_lbl);      
}

static void build_on_about_dlg(lv_event_t *e)
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
        "Change Theme: toggles between dark and light system theme, saves other unsaved configs and restarts.",
        "Manual Date&Time: opens the date&time picker to manually set values in this format: HH:MM MM/DD/YY.",
        "Wi-Fi & SNTP: configure the SSID and password of the Wi-Fi connection for precise and automatic time, also with a switch for auto-connect at startup.",
        "Rotate Screen: rotates the display 90 degrees each time.",
        "Run Calibration: starts the touch calibration wizard and saves the new calibration data. Also offers startup calibration toggle.",
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

    lv_obj_add_event_cb(close_btn, close_about_dlg, LV_EVENT_CLICKED, overlay);
}

static void close_about_dlg(lv_event_t *e)
{
    lv_obj_t *overlay = lv_event_get_user_data(e);
    if (overlay) {
        lv_obj_del(overlay);
    }
}

static void back_from_settings(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    settings_start_screensaver_timers();
    close_settings(ctx);
}

static void close_settings(settings_ctx_t *ctx)
{
    if (ctx && ctx->graphics.brightness_slider) {
        update_brightness_value(ctx);
        s_settings_ctx.changing_brightness = false; 
        if (ctx->settings.display.brightness != ctx->settings.display.saved_brightness) {
            persist_brightness_to_nvs();
        }
    }

    if (ctx->settings.display.screen_rotation_step != ctx->settings.display.saved_rotation_step) {
        persist_rotation_to_nvs();
    }

    ctx->active = false;
    if (ctx->graphics.return_screen)
    {
        lv_screen_load(ctx->graphics.return_screen);
    }    
    lv_obj_del(ctx->graphics.screen);
    clear_ui_refs(ctx);
    ctx->active = false;
    ctx->graphics.screen = NULL;
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
    esp_err_t last_err = s_settings_ctx.settings.time.sntp_last_err;
    if (s_settings_ctx.settings.time.sntp_success){
        show_connection_result_message(ESP_OK);
    }else{
        last_err = (last_err == ESP_OK) ? SETTINGS_DEFAULT_SNTP_ERR_CODE : last_err;
        show_connection_result_message(last_err);
    }
    backlight_on_without_wipe_effect();
    vTaskDelay(pdMS_TO_TICKS(s_settings_ctx.settings.time.sntp_success ? 2500 : 3000));
    
    bsp_display_lock(0);
    lv_obj_clean(lv_screen_active());
    bsp_display_unlock();
}

static void manual_restart_flow(void)
{
    if(s_settings_ctx.settings.time.startup_sntp_auto_connect ||
            s_settings_ctx.settings.time.refresh_sntp_startup){
                get_sntp_time();
    }
}

static void sntp_startup(bool power_reset)
{
    if (power_reset)
        return;

    if (s_settings_ctx.settings.manual_restart){
        startup_splash_screen();  
        manual_restart_flow();
    }else{
        sntp_restart_flow();
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
    switch (s_settings_ctx.settings.display.screen_rotation_step % SETTINGS_ROTATION_STEPS) {
        case 0: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_270); break;
        case 1: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_180); break;
        case 2: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);  break;
        case 3: lv_display_set_rotation(display, LV_DISPLAY_ROTATION_0);   break;
        default:
            s_settings_ctx.settings.display.screen_rotation_step = SETTINGS_ROTATION_STEPS - 1;
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

    int32_t stored = s_settings_ctx.settings.display.screen_rotation_step;
    err = nvs_get_i32(h, SETTINGS_NVS_ROT_KEY, &stored);
    nvs_close(h);

    if (err == ESP_OK && stored >= 0 && stored < SETTINGS_ROTATION_STEPS) {
        s_settings_ctx.settings.display.screen_rotation_step = (int)stored;
        s_settings_ctx.settings.display.saved_rotation_step = s_settings_ctx.settings.display.screen_rotation_step;
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

    err = nvs_set_i32(h, SETTINGS_NVS_ROT_KEY, s_settings_ctx.settings.display.screen_rotation_step);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save rotation to NVS: %s", esp_err_to_name(err));
    } else {
        s_settings_ctx.settings.display.saved_rotation_step = s_settings_ctx.settings.display.screen_rotation_step;
    }
}

static void load_brightness_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        s_settings_ctx.settings.display.brightness = SETTINGS_DEFAULT_BRIGHTNESS;
        s_settings_ctx.settings.display.saved_brightness = s_settings_ctx.settings.display.brightness;
        return;
    }

    int32_t stored = SETTINGS_DEFAULT_BRIGHTNESS;
    err = nvs_get_i32(h, SETTINGS_NVS_BRIGHTNESS_KEY, &stored);
    nvs_close(h);

    if (err == ESP_OK && stored >= SETTINGS_MINIMUM_BRIGHTNESS && stored <= 100) {
        s_settings_ctx.settings.display.brightness = (int)stored;
        s_settings_ctx.settings.display.saved_brightness = s_settings_ctx.settings.display.brightness;
    } else {
        s_settings_ctx.settings.display.brightness = SETTINGS_DEFAULT_BRIGHTNESS;
        s_settings_ctx.settings.display.saved_brightness = s_settings_ctx.settings.display.brightness;
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

    err = nvs_set_i32(h, SETTINGS_NVS_BRIGHTNESS_KEY, s_settings_ctx.settings.display.brightness);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save brightness to NVS: %s", esp_err_to_name(err));
    } else {
        s_settings_ctx.settings.display.saved_brightness = s_settings_ctx.settings.display.brightness;

        /* Adjust dim level to stay within saved brightness and above minimum. */
        if (s_settings_ctx.settings.display.dim_level >= 0) {
            int max_level = s_settings_ctx.settings.display.saved_brightness;
            int clamped = s_settings_ctx.settings.display.dim_level;
            if (max_level < SETTINGS_MINIMUM_BRIGHTNESS) {
                max_level = SETTINGS_MINIMUM_BRIGHTNESS;
            }
            if (clamped > max_level) {
                clamped = max_level;
            }
            if (clamped < SETTINGS_MINIMUM_BRIGHTNESS) {
                clamped = SETTINGS_MINIMUM_BRIGHTNESS;
            }
            if (clamped != s_settings_ctx.settings.display.dim_level) {
                s_settings_ctx.settings.display.dim_level = clamped;
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
        s_settings_ctx.settings.display.screen_dim = dim_en ? true : false;
    }

    int32_t dim_time = -1;
    if (nvs_get_i32(h, SETTINGS_NVS_DIM_TIME_KEY, &dim_time) == ESP_OK) {
        if (dim_time >= -1) {
            s_settings_ctx.settings.display.dim_time = (int)dim_time;
        }
    }

    int32_t dim_level = -1;
    if (nvs_get_i32(h, SETTINGS_NVS_DIM_LEVEL_KEY, &dim_level) == ESP_OK) {
        if (dim_level >= -1 && dim_level <= 100) {
            s_settings_ctx.settings.display.dim_level = (int)dim_level;
        }
    }

    int8_t off_en = 0;
    if (nvs_get_i8(h, SETTINGS_NVS_OFF_EN_KEY, &off_en) == ESP_OK) {
        s_settings_ctx.settings.display.screen_off = off_en ? true : false;
    }

    int32_t off_time = -1;
    if (nvs_get_i32(h, SETTINGS_NVS_OFF_TIME_KEY, &off_time) == ESP_OK) {
        if (off_time >= -1) {
            s_settings_ctx.settings.display.off_time = (int)off_time;
        }
    }

    nvs_close(h);

    if (s_settings_ctx.settings.display.off_time <= 0){
        s_settings_ctx.settings.display.screen_off = false;
    }

    if(s_settings_ctx.settings.display.dim_time <= 0){
        s_settings_ctx.settings.display.screen_dim = false;
    }

    /* Clamp dim level against current saved brightness and minimum brightness. */
    if (s_settings_ctx.settings.display.dim_level >= 0) {
        int max_level = s_settings_ctx.settings.display.saved_brightness > 0 ? s_settings_ctx.settings.display.saved_brightness : SETTINGS_DEFAULT_BRIGHTNESS;
        int clamped = s_settings_ctx.settings.display.dim_level;
        if (max_level < SETTINGS_MINIMUM_BRIGHTNESS) {
            max_level = SETTINGS_MINIMUM_BRIGHTNESS;
        }
        if (clamped > max_level) {
            clamped = max_level;
        }
        if (clamped < SETTINGS_MINIMUM_BRIGHTNESS) {
            clamped = SETTINGS_MINIMUM_BRIGHTNESS;
        }
        s_settings_ctx.settings.display.dim_level = clamped;
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

    esp_err_t res = nvs_set_i8(h, SETTINGS_NVS_DIM_EN_KEY, s_settings_ctx.settings.display.screen_dim ? 1 : 0);
    res |= nvs_set_i32(h, SETTINGS_NVS_DIM_TIME_KEY, s_settings_ctx.settings.display.dim_time);
    res |= nvs_set_i32(h, SETTINGS_NVS_DIM_LEVEL_KEY, s_settings_ctx.settings.display.dim_level);
    res |= nvs_set_i8(h, SETTINGS_NVS_OFF_EN_KEY, s_settings_ctx.settings.display.screen_off ? 1 : 0);
    res |= nvs_set_i32(h, SETTINGS_NVS_OFF_TIME_KEY, s_settings_ctx.settings.display.off_time);
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

static void persist_auto_connect_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for auto-connect: (%s)", esp_err_to_name(err));
        return;
    }

    esp_err_t res = nvs_set_i8(h, SETTINGS_NVS_AUTO_CONNECT_KEY, s_settings_ctx.settings.time.startup_sntp_auto_connect ? 1 : 0);
    if (res == ESP_OK) {
        res = nvs_commit(h);
    }
    nvs_close(h);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save auto-connect preference: (%s)", esp_err_to_name(res));
    }
}

static void load_auto_connect_from_nvs(void)
{
    /* Default: disabled */
    s_settings_ctx.settings.time.startup_sntp_auto_connect = SETTINGS_DEFAULT_STARTUP_SNTP_AUTO_CONNECT;

    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    int8_t raw = -1;
    if (nvs_get_i8(h, SETTINGS_NVS_AUTO_CONNECT_KEY, &raw) == ESP_OK) {
        s_settings_ctx.settings.time.startup_sntp_auto_connect = (raw != 0);
    }

    nvs_close(h);
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
    s_settings_ctx.settings.time.startup_sntp_auto_connect = SETTINGS_DEFAULT_STARTUP_SNTP_AUTO_CONNECT;
    s_settings_ctx.settings.calibration_prompt_enabled = SETTINGS_DEFAULT_CALI_PROMPT_ENABLE;
    s_settings_ctx.settings.time.refresh_sntp_startup = SETTINGS_DEFAULT_REFRESH_SNTP_STARTUP;
    s_settings_ctx.settings.running_calibration = SETTINGS_DEFAULT_RUNNING_CALIBRATION;    
    s_settings_ctx.settings.display.screen_rotation_step = SETTINGS_DEFAULT_ROTATION_STEP;
    s_settings_ctx.settings.display.saved_rotation_step = SETTINGS_DEFAULT_ROTATION_STEP;
    s_settings_ctx.settings.manual_restart = SETTINGS_DEFAULT_MANUAL_RESTART; 
    s_settings_ctx.settings.time.sntp_last_err = SETTINGS_DEFAULT_SNTP_ERR_CODE;
    s_settings_ctx.settings.display.saved_brightness = SETTINGS_DEFAULT_BRIGHTNESS;
    s_settings_ctx.settings.display.dim_level = SETTINGS_DEFAULT_SCREEN_DIM_LEVEL;
    s_settings_ctx.settings.time.sntp_success = SETTINGS_DEFAULT_SNTP_SUCCESS; 
    s_settings_ctx.settings.display.off_time = SETTINGS_DEFAULT_SCREEN_OFF_TIME;
    s_settings_ctx.settings.display.dim_time = SETTINGS_DEFAULT_SCREEN_DIM_TIME;
    s_settings_ctx.settings.display.screen_dim = SETTINGS_DEFAULT_SCREEN_DIM;
    s_settings_ctx.settings.display.brightness = SETTINGS_DEFAULT_BRIGHTNESS;
    s_settings_ctx.settings.display.screen_off = SETTINGS_DEFAULT_SCREEN_OFF;
    s_settings_ctx.settings.dark_theme = SETTINGS_DEFAULT_DARK_THEME;
    s_settings_ctx.settings.display.time_valid = SETTINGS_DEFAULT_TIME_VALID; 
    s_settings_ctx.changing_brightness = false; 
    s_settings_ctx.settings.ap_ssid[0] = '\0';
    s_settings_ctx.settings.ap_pwd[0] = '\0';    
}

static void load_last_saved_configs()
{
    load_calibration_prompt_from_nvs();
    load_ap_credentials_from_nvs();
    load_manual_restart_from_nvs();
    load_sntp_refresh_from_nvs();
    load_auto_connect_from_nvs();
    load_brightness_from_nvs();
    load_rotation_from_nvs();
    apply_rotation_to_display(true);
    load_screensaver_from_nvs();
    load_theme_from_nvs();
    load_time_from_nvs();
}

static void rotate_screen(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    ctx->settings.display.screen_rotation_step = (ctx->settings.display.screen_rotation_step + 1) % SETTINGS_ROTATION_STEPS;
    apply_rotation_to_display(false);
}

static void build_date_time_dialog(settings_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    /* Close previous overlay if still open. */
    if (ctx->graphics.datetime_overlay) {
        lv_obj_del(ctx->graphics.datetime_overlay);
        ctx->graphics.datetime_overlay = NULL;
        ctx->graphics.dt_month_ta = NULL;
        ctx->graphics.dt_day_ta = NULL;
        ctx->graphics.dt_year_ta = NULL;
        ctx->graphics.dt_hour_ta = NULL;
        ctx->graphics.dt_min_ta = NULL;
        ctx->graphics.dt_keyboard = NULL;
        ctx->graphics.dt_dialog = NULL;
        ctx->graphics.dt_row_time = NULL;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(overlay, on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    ctx->graphics.datetime_overlay = overlay;

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
    lv_obj_add_event_cb(dlg, on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_bg_opa(dlg, LV_OPA_COVER, 0);
    styles_set_dialog(dlg);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_center(dlg);
    ctx->graphics.dt_dialog = dlg;

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Manual Date&Time");
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

    ctx->graphics.dt_month_ta = lv_textarea_create(row_date);
    lv_obj_set_width(ctx->graphics.dt_month_ta, 48);
    lv_textarea_set_one_line(ctx->graphics.dt_month_ta, true);
    lv_textarea_set_max_length(ctx->graphics.dt_month_ta, 2);
    lv_textarea_set_text(ctx->graphics.dt_month_ta, "MM");
    styles_set_textarea(ctx->graphics.dt_month_ta);
    lv_obj_add_event_cb(ctx->graphics.dt_month_ta, on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_month_ta, on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_month_ta, on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    lv_obj_t *slash1 = lv_label_create(row_date);
    lv_label_set_text(slash1, "/");
    lv_obj_add_flag(slash1, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->graphics.dt_day_ta = lv_textarea_create(row_date);
    lv_obj_set_width(ctx->graphics.dt_day_ta, 48);
    lv_textarea_set_one_line(ctx->graphics.dt_day_ta, true);
    lv_textarea_set_max_length(ctx->graphics.dt_day_ta, 2);
    lv_textarea_set_text(ctx->graphics.dt_day_ta, "DD");
    styles_set_textarea(ctx->graphics.dt_day_ta);
    lv_obj_add_event_cb(ctx->graphics.dt_day_ta, on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_day_ta, on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_day_ta, on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    lv_obj_t *slash2 = lv_label_create(row_date);
    lv_label_set_text(slash2, "/");
    lv_obj_add_flag(slash2, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->graphics.dt_year_ta = lv_textarea_create(row_date);
    lv_obj_set_width(ctx->graphics.dt_year_ta, 48);
    lv_textarea_set_one_line(ctx->graphics.dt_year_ta, true);
    lv_textarea_set_max_length(ctx->graphics.dt_year_ta, 2);
    lv_textarea_set_text(ctx->graphics.dt_year_ta, "YY");
    styles_set_textarea(ctx->graphics.dt_year_ta);
    lv_obj_add_event_cb(ctx->graphics.dt_year_ta, on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_year_ta, on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_year_ta, on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

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
    ctx->graphics.dt_row_time = row_time;

    lv_obj_t *time_lbl = lv_label_create(row_time);
    lv_label_set_text(time_lbl, "Time:");
    lv_obj_add_flag(time_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->graphics.dt_hour_ta = lv_textarea_create(row_time);
    lv_obj_set_width(ctx->graphics.dt_hour_ta, 48);
    lv_textarea_set_one_line(ctx->graphics.dt_hour_ta, true);
    lv_textarea_set_max_length(ctx->graphics.dt_hour_ta, 2);
    lv_textarea_set_text(ctx->graphics.dt_hour_ta, "HH");
    styles_set_textarea(ctx->graphics.dt_hour_ta);
    lv_obj_add_event_cb(ctx->graphics.dt_hour_ta, on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_hour_ta, on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_hour_ta, on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

    lv_obj_t *colon = lv_label_create(row_time);
    lv_label_set_text(colon, ":");
    lv_obj_add_flag(colon, LV_OBJ_FLAG_EVENT_BUBBLE);

    ctx->graphics.dt_min_ta = lv_textarea_create(row_time);
    lv_obj_set_width(ctx->graphics.dt_min_ta, 48);
    lv_textarea_set_one_line(ctx->graphics.dt_min_ta, true);
    lv_textarea_set_max_length(ctx->graphics.dt_min_ta, 2);
    lv_textarea_set_text(ctx->graphics.dt_min_ta, "MM");
    styles_set_textarea(ctx->graphics.dt_min_ta);
    lv_obj_add_event_cb(ctx->graphics.dt_min_ta, on_dt_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_min_ta, on_dt_textarea_focus, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_min_ta, on_dt_textarea_defocus, LV_EVENT_DEFOCUSED, ctx);

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
    lv_obj_add_event_cb(apply_btn, apply_date_time, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_radius(cancel_btn, 6, 0);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);
    lv_obj_add_flag(cancel_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, close_set_date_time, LV_EVENT_CLICKED, ctx);

    /* Keyboard anchored to bottom of overlay */
    ctx->graphics.dt_keyboard = lv_keyboard_create(overlay);
    styles_set_keyboard(ctx->graphics.dt_keyboard);
    lv_keyboard_set_mode(ctx->graphics.dt_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(ctx->graphics.dt_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.dt_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(ctx->graphics.dt_keyboard, LV_OBJ_FLAG_HIDDEN); /* show only after a field is tapped */
    lv_obj_add_event_cb(ctx->graphics.dt_keyboard, on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_keyboard, on_dt_keyboard_event, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->graphics.dt_keyboard, on_dt_keyboard_event, LV_EVENT_READY, ctx);
    lv_obj_align(ctx->graphics.dt_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
}

static void set_date_time(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    build_date_time_dialog(ctx);
}

static void close_set_date_time(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);  
    if (ctx && ctx->graphics.datetime_overlay) {
        lv_obj_del(ctx->graphics.datetime_overlay);
        ctx->graphics.datetime_overlay = NULL;
        ctx->graphics.dt_month_ta = NULL;
        ctx->graphics.dt_day_ta = NULL;
        ctx->graphics.dt_year_ta = NULL;
        ctx->graphics.dt_hour_ta = NULL;
        ctx->graphics.dt_min_ta = NULL;
        ctx->graphics.dt_keyboard = NULL;
    }    
}

static time_t build_date_time_data(settings_ctx_t *ctx)
{
    int month;
    int day;
    int year;
    int hour;
    int minute;
    const char *month_txt = ctx->graphics.dt_month_ta ? lv_textarea_get_text(ctx->graphics.dt_month_ta) : NULL;
    const char *day_txt = ctx->graphics.dt_day_ta ? lv_textarea_get_text(ctx->graphics.dt_day_ta) : NULL;
    const char *year_txt = ctx->graphics.dt_year_ta ? lv_textarea_get_text(ctx->graphics.dt_year_ta) : NULL;
    const char *hour_txt = ctx->graphics.dt_hour_ta ? lv_textarea_get_text(ctx->graphics.dt_hour_ta) : NULL;
    const char *min_txt = ctx->graphics.dt_min_ta ? lv_textarea_get_text(ctx->graphics.dt_min_ta) : NULL;
    
    if (!parse_int_range(month_txt, 1, 12, &month) ||
        !parse_int_range(day_txt, 1, 31, &day) ||
        !parse_int_range(year_txt, 0, 99, &year) ||
        !parse_int_range(hour_txt, 0, 23, &hour) ||
        !parse_int_range(min_txt, 0, 59, &minute)) {
        show_invalid_input_mbox();
        return -1;
    }
    
    int year_full = 2000 + year;
    if (!is_date_valid(year_full, month, day)) {
        show_invalid_input_mbox();
        return -1;
    }
    
    ctx->settings.display.time_valid = true;
    persist_valid_time_flag_to_nvs();
    ctx->settings.time.dt_day = day;
    ctx->settings.time.dt_year = year;
    ctx->settings.time.dt_hour = hour;
    ctx->settings.time.dt_month = month;
    ctx->settings.time.dt_minute = minute;
    notify_time_set();
    
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

    return t;
}

static void apply_date_time(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    time_t t = build_date_time_data(ctx);
    if (t != -1){
        persist_time_to_nvs(t);
    }

    if (ctx->graphics.datetime_overlay) {
        lv_obj_del(ctx->graphics.datetime_overlay);
        ctx->graphics.datetime_overlay = NULL;
    }
    ctx->graphics.dt_month_ta = NULL;
    ctx->graphics.dt_day_ta = NULL;
    ctx->graphics.dt_year_ta = NULL;
    ctx->graphics.dt_hour_ta = NULL;
    ctx->graphics.dt_min_ta = NULL;
    ctx->graphics.dt_keyboard = NULL;
    ctx->graphics.dt_dialog = NULL;
    ctx->graphics.dt_row_time = NULL;
}

static void build_splash_screen(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_clean(scr);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "FILE MANAGER");
    lv_obj_set_style_text_font(label, &Goldman_Regular_35, 0);
    lv_obj_center(label);

    lv_screen_load(scr);
}

static const char *get_time_failure_reason(esp_err_t err)
{
    switch (err) {
        case ESP_ERR_INVALID_ARG:
            return "Wi-Fi credentials missing or invalid.\nCHECK CREDENTIALS.";
#ifdef ESP_ERR_WIFI_TIMEOUT
        case ESP_ERR_WIFI_TIMEOUT:
            return "Wi-Fi connection timed out.\nCHECK CREDENTIALS OR SIGNAL.";
#endif
        case ESP_ERR_INVALID_STATE:
            return "No recorded failure reason.";
        case ESP_FAIL:
            return "SNTP sync timed out.";
        default: {
            const char *name = esp_err_to_name(err);
            return name ? name : "Unknown error";
        }
    }
}

static void build_connection_result_message(esp_err_t result)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_clean(scr);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    if (result == ESP_OK){
        lv_label_set_text(label, "Time set succesfully!");
    } else {
        lv_label_set_text(label, "Time setting failed.");
    }
    lv_obj_center(label);
    if (result != ESP_OK) {
        lv_obj_t *reason_label = lv_label_create(scr);
        lv_obj_set_width(reason_label, LV_PCT(100));
        lv_obj_set_style_text_align(reason_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text_fmt(reason_label, "Reason: %s", get_time_failure_reason(result));
        lv_obj_center(reason_label);
        lv_obj_align_to(label, reason_label, LV_ALIGN_OUT_TOP_MID, 0, -8);
    }

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

    s_settings_ctx.settings.time.dt_year   = tm_info.tm_year % 100; /* YY */
    s_settings_ctx.settings.time.dt_month  = tm_info.tm_mon + 1;
    s_settings_ctx.settings.time.dt_day    = tm_info.tm_mday;
    s_settings_ctx.settings.time.dt_hour   = tm_info.tm_hour;
    s_settings_ctx.settings.time.dt_minute = tm_info.tm_min;
    s_settings_ctx.settings.time.dt_second = tm_info.tm_sec;
    s_settings_ctx.settings.display.time_valid = true;
    persist_time_to_nvs(now);
}

static void apply_timezone(void)
{
    setenv("TZ", SNTP_DEFAULT_TIMEZONE, 1);
    tzset();
}

static bool parse_int_range(const char *txt, int min, int max, int *out_val)
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

static void close_invalid_message_mbox(lv_event_t *e)
{
    lv_obj_t *mbox = lv_event_get_user_data(e);
    if (mbox) {
        lv_msgbox_close(mbox);
    }
}

static void show_invalid_input_mbox(void)
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
    lv_obj_add_event_cb(ok_btn, close_invalid_message_mbox, LV_EVENT_CLICKED, mbox);
}

static void on_dt_textarea_focus(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.dt_keyboard) {
        return;
    }
    lv_obj_t *ta = lv_event_get_target(e);
    const char *txt = lv_textarea_get_text(ta);
    if (txt && (strcmp(txt, "MM") == 0 || strcmp(txt, "DD") == 0 ||
                strcmp(txt, "YY") == 0 || strcmp(txt, "HH") == 0)) {
        lv_textarea_set_text(ta, "");
    }
    lv_keyboard_set_textarea(ctx->graphics.dt_keyboard, ta);
    lv_obj_clear_flag(ctx->graphics.dt_keyboard, LV_OBJ_FLAG_HIDDEN);
    realign_dt_dialog(ctx, ta);
    scroll_field_into_view(ctx, ta);
}

static void on_dt_background_tap(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    lv_obj_t *target = lv_event_get_target(e);
    if (is_descendant(target, ctx->graphics.dt_keyboard)) {
        return;
    }
    if (is_descendant(target, ctx->graphics.dt_month_ta) ||
        is_descendant(target, ctx->graphics.dt_day_ta) ||
        is_descendant(target, ctx->graphics.dt_year_ta) ||
        is_descendant(target, ctx->graphics.dt_hour_ta) ||
        is_descendant(target, ctx->graphics.dt_min_ta)) {
        return;
    }

    hide_dt_keyboard(ctx);
}

static void hide_dt_keyboard(settings_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.dt_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.dt_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.dt_keyboard, LV_OBJ_FLAG_HIDDEN);
    realign_dt_dialog(ctx, NULL);
}

static void realign_dt_dialog(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->graphics.dt_dialog || !lv_obj_is_valid(ctx->graphics.dt_dialog)) {
        return;
    }

    int16_t kb_h = (ctx->graphics.dt_keyboard && lv_obj_is_valid(ctx->graphics.dt_keyboard)) ? lv_obj_get_height(ctx->graphics.dt_keyboard) : 0;
    if (kb_h <= 0) {
        kb_h = lv_obj_get_height(ctx->graphics.dt_dialog) / 3;
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
        if (ta == ctx->graphics.dt_month_ta || ta == ctx->graphics.dt_day_ta || ta == ctx->graphics.dt_year_ta) {
            offset = date_row_lift;
        } else {
            offset = time_row_lift;
        }
    }

    lv_obj_align(ctx->graphics.dt_dialog, LV_ALIGN_CENTER, 0, offset);
}

static void hide_ss_keyboard(settings_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.ss_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.ss_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.ss_keyboard, LV_OBJ_FLAG_HIDDEN);
    realign_screensaver_dialog(ctx, NULL);
}

static void realign_screensaver_dialog(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->graphics.screensaver_dialog || !lv_obj_is_valid(ctx->graphics.screensaver_dialog)) {
        return;
    }

    int16_t kb_h = (ctx->graphics.ss_keyboard && lv_obj_is_valid(ctx->graphics.ss_keyboard)) ? lv_obj_get_height(ctx->graphics.ss_keyboard) : 0;
    if (kb_h <= 0 && ctx->graphics.screensaver_dialog) {
        kb_h = lv_obj_get_height(ctx->graphics.screensaver_dialog) / 3;
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
        if (ta == ctx->graphics.ss_dim_after_ta || ta == ctx->graphics.ss_dim_pct_ta) {
            offset = dim_row_lift;
        } else {
            offset = off_row_lift;
        }
    }

    lv_obj_align(ctx->graphics.screensaver_dialog, LV_ALIGN_CENTER, 0, offset);
}

static void realign_ap_dialog(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->graphics.access_point_dialog || !lv_obj_is_valid(ctx->graphics.access_point_dialog)) {
        return;
    }

    int16_t kb_h = (ctx->graphics.access_point_keyboard && lv_obj_is_valid(ctx->graphics.access_point_keyboard)) ? lv_obj_get_height(ctx->graphics.access_point_keyboard) : 0;
    if (kb_h <= 0) {
        kb_h = lv_obj_get_height(ctx->graphics.access_point_dialog) / 3;
    }
    if (kb_h <= 0) {
        kb_h = 120; /* fallback */
    }

    int16_t offset = 0;
    if (ta) {
        /* Lift more for password row to keep it visible. */
        if (ta == ctx->graphics.access_point_pwd_ta) {
            offset = -(kb_h * 2 / 3);
            if (offset == 0) offset = -80;
        } else {
            offset = -(kb_h / 2);
            if (offset == 0) offset = -60;
        }
    }

    lv_obj_align(ctx->graphics.access_point_dialog, LV_ALIGN_CENTER, 0, offset);
}

static void on_dt_keyboard_event(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    hide_dt_keyboard(ctx);
}

static void on_ss_background_tap(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    lv_obj_t *target = lv_event_get_target(e);
    if (is_descendant(target, ctx->graphics.ss_keyboard)) {
        return;
    }
    if (is_descendant(target, ctx->graphics.ss_dim_after_ta) ||
        is_descendant(target, ctx->graphics.ss_dim_pct_ta) ||
        is_descendant(target, ctx->graphics.ss_off_after_ta)) {
        return;
    }

    hide_ss_keyboard(ctx);
}

static void on_ss_keyboard_event(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    hide_ss_keyboard(ctx);
}

static void hide_ap_keyboard(settings_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.access_point_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.access_point_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.access_point_keyboard, LV_OBJ_FLAG_HIDDEN);
    realign_ap_dialog(ctx, NULL);
}

static void on_ap_background_tap(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    lv_obj_t *target = lv_event_get_target(e);
    if (is_descendant(target, ctx->graphics.access_point_keyboard)) {
        return;
    }
    lv_obj_t *attached = ctx->graphics.access_point_keyboard ? lv_keyboard_get_textarea(ctx->graphics.access_point_keyboard) : NULL;
    if (is_descendant(target, attached)) {
        return;
    }

    hide_ap_keyboard(ctx);
}

static void on_ap_keyboard_event(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CANCEL) {
        hide_ap_keyboard(ctx);
        return;
    }

    if (code == LV_EVENT_READY){
        hide_ap_keyboard(ctx);
    }
}

static void on_ap_textarea_focus(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (!ctx || !ctx->graphics.access_point_keyboard) {
        return;
    }

    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }

    lv_obj_t *ta = lv_event_get_target(e);
    if (!ta || lv_obj_has_state(ta, LV_STATE_DISABLED)) {
        return;
    }

    lv_keyboard_set_textarea(ctx->graphics.access_point_keyboard, ta);
    lv_obj_clear_flag(ctx->graphics.access_point_keyboard, LV_OBJ_FLAG_HIDDEN);
    realign_ap_dialog(ctx, ta);
    lv_obj_scroll_to_view(ta, LV_ANIM_ON);
}

static void on_ss_textarea_focus(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (!ctx || !ctx->graphics.ss_keyboard) {
        return;
    }

    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) {
        return;
    }

    lv_obj_t *ta = lv_event_get_target(e);
    if (lv_obj_has_state(ta, LV_STATE_DISABLED)) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.ss_keyboard, ta);
    lv_obj_clear_flag(ctx->graphics.ss_keyboard, LV_OBJ_FLAG_HIDDEN);
    realign_screensaver_dialog(ctx, ta);
    lv_obj_scroll_to_view(ta, LV_ANIM_ON);
}

static void on_dim_switch_changed(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    lv_obj_t *sw = lv_event_get_target(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (!enabled) {
        hide_ss_keyboard(ctx);
    }
    update_dim_controls_enabled(ctx, enabled);
}

static void on_off_switch_changed(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    lv_obj_t *sw = lv_event_get_target(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (!enabled) {
        hide_ss_keyboard(ctx);
    }
    update_off_controls_enabled(ctx, enabled);
}

static void update_label(lv_obj_t *lbl, bool enabled)
{
    if (!lbl) {
        return;
    }
    if (enabled) {
        lv_obj_clear_state(lbl, LV_STATE_DISABLED);
        lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, LV_PART_MAIN);
    } else {
        lv_obj_add_state(lbl, LV_STATE_DISABLED);
        lv_obj_set_style_text_opa(lbl, LV_OPA_60, LV_PART_MAIN);
    }
}

static void update_textarea(lv_obj_t *ta, bool enabled)
{
    if (!ta) {
        return;
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

static void update_dim_controls_enabled(settings_ctx_t *ctx, bool enabled)
{
    if (!ctx) {
        return;
    }

    lv_obj_t *labels[] = {
        ctx->graphics.ss_dim_after_lbl,
        ctx->graphics.ss_seconds_lbl,
        ctx->graphics.ss_at_lbl,
        ctx->graphics.ss_pct_lbl,
    };
    for (size_t i = 0; i < sizeof(labels)/sizeof(labels[0]); i++) {
        lv_obj_t *lbl = labels[i];
        update_label(lbl, enabled);
    }

    lv_obj_t *textareas[] = {
        ctx->graphics.ss_dim_after_ta,
        ctx->graphics.ss_dim_pct_ta,
    };
    for (size_t i = 0; i < sizeof(textareas)/sizeof(textareas[0]); i++) {
        lv_obj_t *ta = textareas[i];
        update_textarea(ta, enabled);
    }

    if (!enabled && ctx->graphics.ss_keyboard) {
        lv_obj_t *attached = lv_keyboard_get_textarea(ctx->graphics.ss_keyboard);
        if (attached == ctx->graphics.ss_dim_after_ta || attached == ctx->graphics.ss_dim_pct_ta) {
            lv_keyboard_set_textarea(ctx->graphics.ss_keyboard, NULL);
            lv_obj_add_flag(ctx->graphics.ss_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void update_off_controls_enabled(settings_ctx_t *ctx, bool enabled)
{
    if (!ctx) {
        return;
    }

    lv_obj_t *labels[] = {
        ctx->graphics.ss_off_after_lbl,
        ctx->graphics.ss_off_seconds_lbl,
    };
    for (size_t i = 0; i < sizeof(labels)/sizeof(labels[0]); i++) {
        lv_obj_t *lbl = labels[i];
        update_label(lbl, enabled);
    }

    if (ctx->graphics.ss_off_after_ta) {
        update_textarea(ctx->graphics.ss_off_after_ta, enabled);
    }

    if (!enabled && ctx->graphics.ss_keyboard) {
        lv_obj_t *attached = lv_keyboard_get_textarea(ctx->graphics.ss_keyboard);
        if (attached == ctx->graphics.ss_off_after_ta) {
            lv_keyboard_set_textarea(ctx->graphics.ss_keyboard, NULL);
            lv_obj_add_flag(ctx->graphics.ss_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void screensaver_start_timer(esp_timer_handle_t *timer_handle, esp_timer_cb_t cb, const char *name, int seconds)
{
    if (!timer_handle || !cb || !name) {
        ESP_LOGE(TAG, "Invalid screensaver timer parameters");
        return;
    }

    if (*timer_handle == NULL) {
        const esp_timer_create_args_t args = {
            .callback = cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = name,
        };
        esp_err_t err = esp_timer_create(&args, timer_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create %s timer: %s", name, esp_err_to_name(err));
            return;
        }
    } else {
        esp_timer_stop(*timer_handle);
    }

    int64_t us = (seconds < 0 ? 0 : seconds) * 1000000LL;
    esp_err_t err = esp_timer_start_once(*timer_handle, us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start %s timer: %s", name, esp_err_to_name(err));
    }
}

static void screensaver_dim_start(int seconds)
{
    ESP_LOGD(TAG, "Start dim timer: %ds", seconds);
    screensaver_start_timer(&s_ss_dim_timer, dim_timer_cb, "ss_dim", seconds);
}

static void screensaver_off_start(int seconds)
{
    ESP_LOGD(TAG, "Start screen-off timer: %ds", seconds);
    screensaver_start_timer(&s_ss_off_timer, off_timer_cb, "ss_off", seconds);
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

static void screensaver_off_stop(void)
{
    if (!s_settings_ctx.changing_brightness){
        ESP_LOGD(TAG, "Stop screen-off timer");
    }
    if (s_ss_off_timer) {
        esp_timer_stop(s_ss_off_timer);
    }
    fade_brightness(s_settings_ctx.settings.display.brightness, 0); /* stop any ongoing fade */
}

static void dim_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGD(TAG, "Dim timer fired: fading to dim level");
    fade_brightness(s_settings_ctx.settings.display.dim_level, SETTINGS_DIM_FADE_MS);
}

static void off_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGD(TAG, "Off timer fired: fading screen off");
    s_light_sleep_pending = true;
    fade_brightness(0, SETTINGS_OFF_FADE_MS);
}

static void initialize_screensaver_light_sleep_task(void)
{
    if (s_light_sleep_task_handle) {
        return;
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(screensaver_light_sleep_task,
                                     "ss_light_sleep",
                                     SETTINGS_LIGHT_SLEEP_TASK_STACK_WORDS,
                                     NULL,
                                     SETTINGS_LIGHT_SLEEP_TASK_PRIORITY,
                                     &s_light_sleep_task_handle,
                                     SETTINGS_LIGHT_SLEEP_TASK_CORE
                                    );
    if (task_ok != pdPASS) {
        ESP_LOGW(TAG, "Failed to create screensaver light sleep task");
        s_light_sleep_task_handle = NULL;
    }
}

static bool touch_interrupt_can_wakeup(void)
{
#if defined(CONFIG_TOUCH_IRQ_GPIO)
    return rtc_gpio_is_valid_gpio(CONFIG_TOUCH_IRQ_GPIO);
#else
    return false;
#endif
}

static bool enable_touch_ext0_wakeup(void)
{
#if !defined(CONFIG_TOUCH_IRQ_GPIO) || !defined(TOUCH_IRQ_WAKE_LEVEL)
    ESP_LOGW(TAG, "Touch IRQ GPIO is not configured; cannot enable EXT0 wakeup");
    return false;
#else
    if (!touch_interrupt_can_wakeup()) {
        ESP_LOGW(TAG, "Touch IRQ GPIO %d cannot be used for EXT0 wakeup", CONFIG_TOUCH_IRQ_GPIO);
        return false;
    }

    esp_err_t err = esp_sleep_enable_ext0_wakeup((gpio_num_t)CONFIG_TOUCH_IRQ_GPIO, TOUCH_IRQ_WAKE_LEVEL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable EXT0 wakeup: %s", esp_err_to_name(err));
        return false;
    }

    return true;
#endif
}

static void screensaver_light_sleep_task(void *param)
{
    (void)param;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (settings_get_active_brightness() > 0) {
            continue;
        }
        if (!enable_touch_ext0_wakeup()) {
            continue;
        }

        hold_display_reset_high();

        pause_lvgl_timers_for_sleep();
        screensaver_off_stop();
        screensaver_dim_stop();

        ESP_LOGI(TAG, "Entering light sleep until touch interrupt");
#ifdef CONFIG_ESP_CONSOLE_UART_NUM
        esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
#endif        

        esp_err_t err = esp_light_sleep_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Light sleep start failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Light sleep ended");
            refresh_display_after_light_sleep();
        }
        
        resume_lvgl_timers_after_sleep();
        
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
    }
}

static void refresh_display_after_light_sleep(void)
{
    release_display_reset_hold();
    if (settings_get_active_brightness() <= 0) {
        settings_fade_to_saved_brightness();
    }
    settings_start_screensaver_timers();
}

static void pause_lvgl_timers_for_sleep(void)
{
    if (s_lv_timers_paused) {
        return;
    }
    if (!bsp_display_lock(50)) {
        ESP_LOGW(TAG, "Failed to lock LVGL to pause timers");
        return;
    }

    /* Stop LVGL timer handler and tick source so long sleeps don't queue huge backlogs. */
    lvgl_port_stop();
    bsp_display_unlock();
    s_lv_timers_paused = true;
}

static void resume_lvgl_timers_after_sleep(void)
{
    if (!s_lv_timers_paused) {
        return;
    }
    if (!bsp_display_lock(50)) {
        ESP_LOGW(TAG, "Failed to lock LVGL to resume timers");
        return;
    }

    lvgl_port_resume();
    bsp_display_unlock();
    s_lv_timers_paused = false;
}

static void hold_display_reset_high(void)
{
#if defined(CONFIG_BSP_DISPLAY_RST_GPIO) && (CONFIG_BSP_DISPLAY_RST_GPIO >= 0)
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_BSP_DISPLAY_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&rst_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure display RST GPIO %d: %s", CONFIG_BSP_DISPLAY_RST_GPIO, esp_err_to_name(err));
        return;
    }
    gpio_set_level(CONFIG_BSP_DISPLAY_RST_GPIO, 1);
    if (rtc_gpio_is_valid_gpio(CONFIG_BSP_DISPLAY_RST_GPIO)) {
        esp_err_t hold_err = gpio_hold_en(CONFIG_BSP_DISPLAY_RST_GPIO); // for deep sleep use gpio_deep_sleep_hold_en
        if (hold_err == ESP_OK) {
            s_display_rst_hold = true;
        } else {
            ESP_LOGW(TAG, "Failed to hold display RST high: %s", esp_err_to_name(hold_err));
        }
    } else {
        ESP_LOGW(TAG, "Display RST GPIO %d is not RTC-capable; cannot hold during sleep", CONFIG_BSP_DISPLAY_RST_GPIO);
    }
#endif
}

static void release_display_reset_hold(void)
{
#if defined(CONFIG_BSP_DISPLAY_RST_GPIO) && (CONFIG_BSP_DISPLAY_RST_GPIO >= 0)
    if (s_display_rst_hold) {
        gpio_hold_dis(CONFIG_BSP_DISPLAY_RST_GPIO);
        s_display_rst_hold = false;
    }
#endif
}

static void notify_light_sleep_task(void)
{
    if (s_fade_target == 0 && s_light_sleep_pending) {
        s_light_sleep_pending = false;
        if (s_light_sleep_task_handle) {
            ESP_LOGD(TAG, "Requesting light sleep after the screen is off");
            xTaskNotifyGive(s_light_sleep_task_handle);
        } else {
            ESP_LOGW(TAG, "Light sleep task unavailable; skipping sleep request");
        }
    }
}

static bool fade_ensure_timer_ready(void)
{
    if (s_fade_timer) {
        esp_timer_stop(s_fade_timer);
    } else {
        const esp_timer_create_args_t args = {
            .callback = fade_step_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "fade",
        };
        if (esp_timer_create(&args, &s_fade_timer) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create fade timer");
            return false;
        }
    }

    return true;
}

static bool fade_setup_steps(int target_pct, int start, settings_ctx_t *ctx)
{
    s_fade_target = target_pct;
    s_fade_direction = (target_pct > start) ? 1 : -1;
    s_fade_steps_left = (start > target_pct) ? (start - target_pct) : (target_pct - start);
    if (s_fade_steps_left == 0) {
        ctx->settings.display.brightness = target_pct;
        bsp_display_brightness_set(target_pct);
        sync_brightness_ui(ctx, target_pct);
        return false;
    }

    return true;
}

static bool fade_handle_instant_update(uint32_t duration_ms, int target_pct, int start, bool rising, settings_ctx_t *ctx)
{
    if (duration_ms == 0 || start == target_pct) {
        ctx->settings.display.brightness = target_pct;
        bsp_display_brightness_set(target_pct);
        sync_brightness_ui(ctx, target_pct);
        if (!rising) {
            s_wake_in_progress = false;
        }
        return true;
    }

    return false;
}

static void fade_brightness(int target_pct, uint32_t duration_ms)
{
    settings_ctx_t *ctx = &s_settings_ctx;
    if (target_pct > 100) target_pct = 100;
    if (target_pct < 0) target_pct = 0;
    if (target_pct > 0) {
        s_light_sleep_pending = false;
    }
    int start = ctx->settings.display.brightness;
    bool rising = target_pct > start;

    if (fade_handle_instant_update(duration_ms, target_pct, start, rising, ctx)) return;
    if (rising) {
        s_wake_in_progress = true;
    }
    if (!fade_ensure_timer_ready()) return;
    if (!fade_setup_steps(target_pct, start, ctx)) return;

    int64_t interval_us = (duration_ms * 1000ULL) / s_fade_steps_left;
    if (interval_us < 1000) interval_us = 1000;

    esp_err_t err = esp_timer_start_periodic(s_fade_timer, interval_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start fade timer: %s", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Fade start: %d -> %d over %ums (step %lldus)", start, target_pct, duration_ms, interval_us);
    }
}

static void fade_step_cb(void *arg)
{
    (void)arg;
    if (s_fade_steps_left <= 0) {
        if (s_fade_timer) {
            esp_timer_stop(s_fade_timer);
        }

        s_settings_ctx.settings.display.brightness = s_fade_target;
        bsp_display_brightness_set(s_fade_target);
        sync_brightness_ui(&s_settings_ctx, s_fade_target);
        ESP_LOGD(TAG, "Fade complete -> %d", s_fade_target);

        notify_light_sleep_task();
        s_wake_in_progress = false;
        return;
    }

    int next = s_settings_ctx.settings.display.brightness + s_fade_direction;
    if (next < 0) next = 0;
    if (next > 100) next = 100;
    s_settings_ctx.settings.display.brightness = next;
    bsp_display_brightness_set(next);

    s_fade_steps_left--;
}

static void sync_brightness_ui(settings_ctx_t *ctx, int val)
{
    (void)ctx;
    /* Debounce: only one pending UI update at a time from esp_timer context. */
    if (s_brightness_ui_pending) {
        return;
    }
    s_brightness_ui_pending = true;
    lv_async_call(sync_brightness_ui_async, (void *)(uintptr_t)val);
}

static void sync_brightness_ui_async(void *arg)
{
    int val = (int)(uintptr_t)arg;
    if (val < SETTINGS_MINIMUM_BRIGHTNESS) val = SETTINGS_MINIMUM_BRIGHTNESS;
    if (val > 100) val = 100;

    settings_ctx_t *ctx = &s_settings_ctx;
    /* Skip if settings screen is not active/visible or controls were deleted. */
    if (!ctx->active || !ctx->graphics.screen || !lv_obj_is_valid(ctx->graphics.screen) || lv_screen_active() != ctx->graphics.screen) {
        return;
    }

    if (ctx->graphics.brightness_slider && lv_obj_is_valid(ctx->graphics.brightness_slider)) {
        lv_slider_set_value(ctx->graphics.brightness_slider, val, LV_ANIM_OFF);
    }
    if (ctx->graphics.brightness_label && lv_obj_is_valid(ctx->graphics.brightness_label)) {
        char txt[32];
        lv_snprintf(txt, sizeof(txt), "Brightness: %d%%", val);
        lv_label_set_text(ctx->graphics.brightness_label, txt);
    }

    s_brightness_ui_pending = false;
}

static void scroll_field_into_view(settings_ctx_t *ctx, lv_obj_t *ta)
{
    if (!ctx || !ctx->graphics.dt_dialog || !ta) {
        return;
    }
    lv_obj_t *target = ta;
    if ((ta == ctx->graphics.dt_hour_ta || ta == ctx->graphics.dt_min_ta) && ctx->graphics.dt_row_time) {
        target = ctx->graphics.dt_row_time;
    }
    lv_obj_scroll_to_view(target, LV_ANIM_ON);
}

static void on_dt_textarea_defocus(lv_event_t *e)
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
    if (ta == ctx->graphics.dt_month_ta) {
        lv_textarea_set_text(ta, "MM");
    } else if (ta == ctx->graphics.dt_day_ta) {
        lv_textarea_set_text(ta, "DD");
    } else if (ta == ctx->graphics.dt_year_ta) {
        lv_textarea_set_text(ta, "YY");
    } else if (ta == ctx->graphics.dt_hour_ta) {
        lv_textarea_set_text(ta, "HH");
    } else if (ta == ctx->graphics.dt_min_ta) {
        lv_textarea_set_text(ta, "MM");
    }
    scroll_field_into_view(ctx, ta);
}

static bool is_descendant(lv_obj_t *obj, lv_obj_t *maybe_ancestor)
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

static bool is_date_valid(int year_full, int month, int day)
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

static void notify_time_set(void)
{
    if (s_time_set_cb) {
        s_time_set_cb();
    }
}

static void notify_time_reset(void)
{
    if (s_time_reset_cb) {
        s_time_reset_cb();
    }
}

static void persist_time_to_nvs(time_t epoch)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i64(h, SETTINGS_NVS_TIME_KEY, (int64_t)epoch);
    nvs_commit(h);
    nvs_close(h);
}

static void clear_time_in_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, SETTINGS_NVS_TIME_KEY);
    nvs_commit(h);
    nvs_close(h);
}

static void persist_valid_time_flag_to_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_VALID_TIME_FLAG_KEY, s_settings_ctx.settings.display.time_valid ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

static void clear_valid_time_flag_in_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, SETTINGS_NVS_VALID_TIME_FLAG_KEY);
    nvs_commit(h);
    nvs_close(h);
}

static void persist_sntp_result(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_SNTP_RESULT_KEY, s_settings_ctx.settings.time.sntp_success ? 1 : 0);
    nvs_set_i32(h, SETTINGS_NVS_SNTP_ERROR_KEY, (int32_t)s_settings_ctx.settings.time.sntp_last_err);
    nvs_commit(h);
    nvs_close(h);
}

static void persist_ap_credentials_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for AP creds: (%s)", esp_err_to_name(err));
        return;
    }

    esp_err_t res = nvs_set_str(h, SETTINGS_NVS_AP_SSID_KEY, s_settings_ctx.settings.ap_ssid);
    if (res == ESP_OK) {
        res = nvs_set_str(h, SETTINGS_NVS_AP_PWD_KEY, s_settings_ctx.settings.ap_pwd);
    }
    if (res == ESP_OK) {
        res = nvs_commit(h);
    }
    nvs_close(h);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save AP credentials: (%s)", esp_err_to_name(res));
    }
}

static void persist_sntp_refresh(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_REFRESH_SNTP_STARTUP_KEY, s_settings_ctx.settings.time.refresh_sntp_startup? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

static void load_sntp_refresh_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for sntp refresh result: (%s)", esp_err_to_name(err));
        return;
    }

    int8_t raw = -1;
    if (nvs_get_i8(h, SETTINGS_NVS_REFRESH_SNTP_STARTUP_KEY, &raw) == ESP_OK) {
        s_settings_ctx.settings.time.refresh_sntp_startup = (raw != 0);
    }

    nvs_close(h);
}

static void load_ap_credentials_from_nvs(void)
{
    s_settings_ctx.settings.ap_ssid[0] = '\0';
    s_settings_ctx.settings.ap_pwd[0] = '\0';

    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    load_ap_ssid(h);
    load_ap_pwd(h);

    nvs_close(h);
}

static void load_ap_ssid(nvs_handle_t h)
{
    load_user_data(h, SETTINGS_NVS_AP_SSID_KEY, s_settings_ctx.settings.ap_ssid,
                   sizeof(s_settings_ctx.settings.ap_ssid));
}

static void load_ap_pwd(nvs_handle_t h)
{
    load_user_data(h, SETTINGS_NVS_AP_PWD_KEY, s_settings_ctx.settings.ap_pwd,
                   sizeof(s_settings_ctx.settings.ap_pwd));
}

static void load_user_data(nvs_handle_t h, const char *key, char *buf, size_t buf_size)
{
    size_t len = buf_size;
    esp_err_t res = nvs_get_str(h, key, buf, &len);
    if (res == ESP_ERR_NVS_INVALID_LENGTH) {
        /* String stored in NVS is larger than our buffer; treating as invalid to avoid large alloc. */
        ESP_LOGW(TAG, "%s too long in NVS (%u > %u); clearing", key ? key : "key", (unsigned)len, (unsigned)buf_size);
        buf[0] = '\0';
        return;
    }

    if (res != ESP_OK) {
        buf[0] = '\0';
        return;
    }

    /* Ensure null-termination even if NVS returned exactly buf_size. */
    buf[buf_size - 1] = '\0';
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
        s_settings_ctx.settings.time.sntp_success = (raw != 0);
    }
    int32_t raw_err = (int32_t)SETTINGS_DEFAULT_SNTP_ERR_CODE;
    if (nvs_get_i32(h, SETTINGS_NVS_SNTP_ERROR_KEY, &raw_err) == ESP_OK) {
        s_settings_ctx.settings.time.sntp_last_err = (esp_err_t)raw_err;
    } else {
        s_settings_ctx.settings.time.sntp_last_err = SETTINGS_DEFAULT_SNTP_ERR_CODE;
    }

    nvs_close(h);
}

static void persist_manual_restart(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i8(h, SETTINGS_NVS_MANUAL_RESTART_KEY, s_settings_ctx.settings.manual_restart ? 1 : 0);
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
        clear_time_in_nvs();
        s_settings_ctx.settings.display.time_valid = false;
        clear_valid_time_flag_in_nvs();
        notify_time_reset();
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
    s_settings_ctx.settings.display.time_valid = true;
    persist_valid_time_flag_to_nvs();
    notify_time_set();
}

static void on_brightness_changed(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED && code != LV_EVENT_RELEASED && code != LV_EVENT_CLICKED) {
        return;
    }

    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.brightness_label || !ctx->graphics.brightness_slider) {
        return;
    }

    update_brightness_value(ctx);
    
    /* Stop any screensaver dim/off fade using the latest brightness value. */
    screensaver_dim_stop();
    screensaver_off_stop();
    s_settings_ctx.changing_brightness = true;

    char txt[32];
    lv_snprintf(txt, sizeof(txt), "Brightness: %d%%", ctx->settings.display.brightness);
    lv_label_set_text(ctx->graphics.brightness_label, txt);

    bsp_display_brightness_set(ctx->settings.display.brightness);
}

static void build_restart_ui(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.restart_confirm_mbox = mbox;
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
    lv_obj_add_event_cb(yes_btn, confirm_restart, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, close_restart, LV_EVENT_CLICKED, ctx);
}

static void update_brightness_value(settings_ctx_t *ctx)
{
    int val = lv_slider_get_value(ctx->graphics.brightness_slider);
    if (val < SETTINGS_MINIMUM_BRIGHTNESS) val = SETTINGS_MINIMUM_BRIGHTNESS;
    if (val > 100) val = 100;
    ctx->settings.display.brightness = val;
}

static void confirm_restart(lv_event_t *e)
{
    bsp_display_backlight_off();
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx && ctx->graphics.brightness_slider) {
        update_brightness_value(ctx);
        if (ctx->settings.display.brightness != ctx->settings.display.saved_brightness) {
            persist_brightness_to_nvs();
        }
    }
    if (ctx->settings.display.screen_rotation_step != ctx->settings.display.saved_rotation_step) {
        persist_rotation_to_nvs();
    }
    if (settings_is_time_valid()){
        settings_shutdown_save_time();
    }
    s_settings_ctx.settings.manual_restart = true;
    persist_manual_restart();
    esp_restart();
}

static void close_restart(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.restart_confirm_mbox)
    {
        return;
    }    
    if (ctx && ctx->graphics.restart_confirm_mbox) {
        lv_msgbox_close(ctx->graphics.restart_confirm_mbox);
        ctx->graphics.restart_confirm_mbox = NULL;
    }    
}

static void build_reset_ui(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.reset_confirm_mbox = mbox;
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
    lv_obj_add_event_cb(yes_btn, confirm_reset, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, close_reset, LV_EVENT_CLICKED, ctx);
}

static void confirm_reset(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.reset_confirm_mbox)
    {
        return;
    }  

    init_default_configs();
    persist_manual_restart();
    persist_rotation_to_nvs();
    persist_brightness_to_nvs();
    persist_screensaver_to_nvs();
    persist_auto_connect_to_nvs();
    persist_ap_credentials_to_nvs();
    persist_calibration_prompt_to_nvs();

    load_last_saved_configs();
    
    nvs_handle_t cal_nvs;
    if (nvs_open(TOUCH_CAL_NVS_NS, NVS_READWRITE, &cal_nvs) == ESP_OK) {
        nvs_erase_key(cal_nvs, TOUCH_CAL_NVS_KEY);
        nvs_commit(cal_nvs);
        nvs_close(cal_nvs);
    }

    clear_time_in_nvs();
    s_settings_ctx.settings.display.time_valid = false;
    persist_valid_time_flag_to_nvs();
    notify_time_reset();

    lv_msgbox_close(ctx->graphics.reset_confirm_mbox);
    ctx->graphics.reset_confirm_mbox = NULL;
}

static void close_reset(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);  
    if (ctx && ctx->graphics.reset_confirm_mbox) {
        lv_msgbox_close(ctx->graphics.reset_confirm_mbox);
        ctx->graphics.reset_confirm_mbox = NULL;
    }    
}

static void toggle_theme(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    if (ctx->graphics.theme_confirm_mbox) {
        lv_msgbox_close(ctx->graphics.theme_confirm_mbox);
        ctx->graphics.theme_confirm_mbox = NULL;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.theme_confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Restart required to change theme");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "Ok");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, theme_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, close_theme_msgbox, LV_EVENT_CLICKED, ctx);
}

static void build_wifi_sntp_dialog(settings_ctx_t *ctx)
{
    if (ctx->graphics.wifi_sntp_dialog){
        lv_obj_delete(ctx->graphics.wifi_sntp_dialog);
        ctx->graphics.wifi_sntp_dialog = NULL;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    ctx->graphics.wifi_sntp_overlay = overlay;

    lv_obj_t *dlg = lv_obj_create(overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 6, 0);
    lv_obj_set_style_pad_gap(dlg, 4, 0);
    lv_obj_set_size(dlg, lv_pct(85), lv_pct(90));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(dlg, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dlg, LV_SCROLLBAR_MODE_AUTO);
    styles_set_dialog(dlg);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_center(dlg);
    ctx->graphics.wifi_sntp_dialog = dlg;

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Wi-Fi & SNTP");
    lv_obj_set_style_text_font(title, &Domine_16, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);    

    /* Access Point data row */
    lv_obj_t *row_ap_data = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_ap_data);
    lv_obj_set_flex_flow(row_ap_data, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_ap_data, 6, 0);
    lv_obj_set_style_pad_all(row_ap_data, 0, 0);
    lv_obj_set_width(row_ap_data, LV_PCT(90));
    lv_obj_set_height(row_ap_data, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_ap_data, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_ap_data, 10, 0);
    lv_obj_add_flag(row_ap_data, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *ap_data_button = lv_button_create(row_ap_data);
    lv_obj_set_flex_grow(ap_data_button, 1);
    lv_obj_set_style_radius(ap_data_button, 8, 0);
    lv_obj_set_style_pad_all(ap_data_button, 6, 0); 
    styles_set_button(ap_data_button);
    lv_obj_add_event_cb(ap_data_button, build_wifi_connection_dialog, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(ap_data_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *ap_data_lbl = lv_label_create(ap_data_button);
    lv_label_set_text(ap_data_lbl, "Access Point Credentials");
    lv_obj_center(ap_data_lbl);      

    /* Refresh time row */
    lv_obj_t *row_refresh_time = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_refresh_time);
    lv_obj_set_flex_flow(row_refresh_time, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_refresh_time, 6, 0);
    lv_obj_set_style_pad_all(row_refresh_time, 0, 0);
    lv_obj_set_width(row_refresh_time, LV_PCT(90));
    lv_obj_set_height(row_refresh_time, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_refresh_time, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_refresh_time, 10, 0);
    lv_obj_add_flag(row_refresh_time, LV_OBJ_FLAG_EVENT_BUBBLE);        

    lv_obj_t *refresh_time_button = lv_button_create(row_refresh_time);
    lv_obj_set_flex_grow(refresh_time_button, 1);
    lv_obj_set_style_radius(refresh_time_button, 8, 0);
    lv_obj_set_style_pad_all(refresh_time_button, 6, 0); 
    styles_set_button(refresh_time_button);
    lv_obj_add_event_cb(refresh_time_button, refresh_sntp, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(refresh_time_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *refresh_time_lbl = lv_label_create(refresh_time_button);
    lv_label_set_text(refresh_time_lbl, "Refresh Date&Time");
    lv_obj_center(refresh_time_lbl);
    
    /* Toggle startup refresh */
    lv_obj_t *row_startup_refresh = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_startup_refresh);
    lv_obj_set_flex_flow(row_startup_refresh, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_startup_refresh, 6, 0);
    lv_obj_set_style_pad_all(row_startup_refresh, 0, 0);
    lv_obj_set_width(row_startup_refresh, LV_PCT(90));
    lv_obj_set_height(row_startup_refresh, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_startup_refresh, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_startup_refresh, 10, 0);
    lv_obj_add_flag(row_startup_refresh, LV_OBJ_FLAG_EVENT_BUBBLE);      

    lv_obj_t *startup_label = lv_label_create(row_startup_refresh);
    lv_label_set_text(startup_label, "Auto connect at startup");
    lv_obj_set_style_text_align(startup_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(startup_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(startup_label, LV_PCT(100));
    styles_set_text_color(startup_label, 0);
    lv_obj_set_flex_grow(startup_label, 1); /* push switch to the far right */

    lv_obj_t *startup_switch = lv_switch_create(row_startup_refresh);
    styles_set_switch(startup_switch);
    bool startup_enabled = get_auto_connect_state();
    if (startup_enabled) {
        lv_obj_add_state(startup_switch, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(startup_switch, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(startup_switch, ui_on_startup_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    /* Action row */
    lv_obj_t *row_actions = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_actions);
    lv_obj_set_flex_flow(row_actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_actions, 6, 0);
    lv_obj_set_style_pad_all(row_actions, 0, 0);
    lv_obj_set_width(row_actions, LV_PCT(100));
    lv_obj_set_height(row_actions, LV_SIZE_CONTENT);
    lv_obj_set_style_margin_top(row_actions, 10, 0);
    lv_obj_set_flex_align(row_actions, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(row_actions, LV_OBJ_FLAG_EVENT_BUBBLE);
    
    lv_obj_t *close_btn = lv_button_create(row_actions);
    styles_set_button(close_btn);
    lv_obj_set_width(close_btn, LV_PCT(55));
    lv_obj_set_style_radius(close_btn, 6, 0);
    lv_obj_add_event_cb(close_btn, close_connection_dialog, LV_EVENT_CLICKED, ctx);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "Close");
    lv_obj_center(close_lbl);
    lv_obj_add_flag(close_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
}

static void build_refresh_sntp_msgbox(settings_ctx_t *ctx)
{
    if (ctx->graphics.sntp_confirm_mbox) {
        lv_msgbox_close(ctx->graphics.sntp_confirm_mbox);
        ctx->graphics.sntp_confirm_mbox = NULL;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.sntp_confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Restart required to refresh SNTP");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "Ok");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, sntp_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, cancel_sntp, LV_EVENT_CLICKED, ctx);
}

static void refresh_sntp(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.wifi_sntp_dialog) {
        return;
    }   

    build_refresh_sntp_msgbox(ctx);
}

static void build_wifi_connection_dialog(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.wifi_sntp_dialog) {
        return;
    }    
    lv_obj_delete(ctx->graphics.wifi_sntp_dialog);
    ctx->graphics.wifi_sntp_dialog = NULL;

    if (ctx->graphics.access_point_dialog){
        lv_obj_delete(ctx->graphics.access_point_dialog);
        ctx->graphics.access_point_dialog = NULL;
    }

    ctx->graphics.access_point_ssid_ta = NULL;
    ctx->graphics.access_point_pwd_ta = NULL;
    if (ctx->graphics.access_point_keyboard) {
        lv_obj_del(ctx->graphics.access_point_keyboard);
        ctx->graphics.access_point_keyboard = NULL;
    }    

    lv_obj_t *dlg = lv_obj_create(ctx->graphics.wifi_sntp_overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 6, 0);
    lv_obj_set_style_pad_gap(dlg, 4, 0);
    lv_obj_set_size(dlg, lv_pct(85), lv_pct(75));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(dlg, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dlg, LV_SCROLLBAR_MODE_AUTO);
    styles_set_dialog(dlg);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_center(dlg);
    ctx->graphics.access_point_dialog = dlg;
    lv_obj_add_event_cb(ctx->graphics.wifi_sntp_overlay, on_ap_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(dlg, on_ap_background_tap, LV_EVENT_CLICKED, ctx);    

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Access Point Credentials");
    lv_obj_set_style_text_font(title, &Domine_16, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);    

    /* Access Point SSID row */
    lv_obj_t *row_ap_ssid = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_ap_ssid);
    lv_obj_set_flex_flow(row_ap_ssid, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_ap_ssid, 6, 0);
    lv_obj_set_style_pad_all(row_ap_ssid, 0, 0);
    lv_obj_set_width(row_ap_ssid, LV_PCT(90));
    lv_obj_set_height(row_ap_ssid, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_ap_ssid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_ap_ssid, 10, 0);
    lv_obj_add_flag(row_ap_ssid, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *ssid_lbl = lv_label_create(row_ap_ssid);
    lv_label_set_text(ssid_lbl, "SSID:");
    lv_obj_add_flag(ssid_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_width(ssid_lbl, 90);
    lv_obj_set_style_text_align(ssid_lbl, LV_TEXT_ALIGN_RIGHT, 0);    

    lv_obj_t *ssid_ta = lv_textarea_create(row_ap_ssid);
    lv_obj_set_flex_grow(ssid_ta, 1);
    lv_obj_set_height(ssid_ta, 32);
    lv_textarea_set_one_line(ssid_ta, true);
    lv_textarea_set_max_length(ssid_ta, SETTINGS_AP_SSID_MAX_LEN);
    lv_textarea_set_placeholder_text(ssid_ta, "");
    styles_set_textarea(ssid_ta);
    lv_obj_add_event_cb(ssid_ta, on_ap_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ssid_ta, on_ap_textarea_focus, LV_EVENT_CLICKED, ctx);
    if (ctx->settings.ap_ssid[0] != '\0') {
        lv_textarea_set_text(ssid_ta, ctx->settings.ap_ssid);
    }
    ctx->graphics.access_point_ssid_ta = ssid_ta;    

    /* Access Point password row */
    lv_obj_t *row_ap_password = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_ap_password);
    lv_obj_set_flex_flow(row_ap_password, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_ap_password, 6, 0);
    lv_obj_set_style_pad_all(row_ap_password, 0, 0);
    lv_obj_set_width(row_ap_password, LV_PCT(90));
    lv_obj_set_height(row_ap_password, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_ap_password, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_ap_password, 10, 0);
    lv_obj_add_flag(row_ap_password, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *pwd_lbl = lv_label_create(row_ap_password);
    lv_label_set_text(pwd_lbl, "Password:");
    lv_obj_add_flag(pwd_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_width(pwd_lbl, 90);
    lv_obj_set_style_text_align(pwd_lbl, LV_TEXT_ALIGN_RIGHT, 0);    

    lv_obj_t *pwd_ta = lv_textarea_create(row_ap_password);
    lv_obj_set_flex_grow(pwd_ta, 1);
    lv_obj_set_height(pwd_ta, 32);    
    lv_textarea_set_one_line(pwd_ta, true);
    lv_textarea_set_max_length(pwd_ta, SETTINGS_AP_PWD_MAX_LEN);
    lv_textarea_set_password_mode(pwd_ta, true);
    lv_textarea_set_placeholder_text(pwd_ta, "");
    styles_set_textarea(pwd_ta);
    lv_obj_add_event_cb(pwd_ta, on_ap_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(pwd_ta, on_ap_textarea_focus, LV_EVENT_CLICKED, ctx);
    if (ctx->settings.ap_pwd[0] != '\0') {
        lv_textarea_set_text(pwd_ta, ctx->settings.ap_pwd);
    }
    ctx->graphics.access_point_pwd_ta = pwd_ta;    

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
    lv_obj_add_event_cb(apply_btn, apply_ap_data, LV_EVENT_CLICKED, ctx);
    
    lv_obj_t *cancel_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_radius(cancel_btn, 6, 0);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);
    lv_obj_add_flag(cancel_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, close_access_point_dialog, LV_EVENT_CLICKED, ctx);
    
    ctx->graphics.access_point_keyboard = lv_keyboard_create(ctx->graphics.wifi_sntp_overlay);
    styles_set_keyboard(ctx->graphics.access_point_keyboard);
    lv_keyboard_set_mode(ctx->graphics.access_point_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_keyboard_set_textarea(ctx->graphics.access_point_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.access_point_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(ctx->graphics.access_point_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(ctx->graphics.access_point_keyboard, on_ap_keyboard_event, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->graphics.access_point_keyboard, on_ap_keyboard_event, LV_EVENT_READY, ctx);
    lv_obj_align(ctx->graphics.access_point_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
}

static void wifi_sntp_dialog(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    build_wifi_sntp_dialog(ctx);
}

static bool get_auto_connect_state(void)
{
    return s_settings_ctx.settings.time.startup_sntp_auto_connect;
}

static void set_auto_connect_state(bool enable)
{
    if (s_settings_ctx.settings.time.startup_sntp_auto_connect == enable) {
        return;
    }
    s_settings_ctx.settings.time.startup_sntp_auto_connect = enable;
    persist_auto_connect_to_nvs();
}

static void ui_on_startup_switch_changed(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    set_auto_connect_state(enabled);
}

static void theme_confirm(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    /* Toggle theme and persist */
    bool new_dark = !settings_is_theme_dark();
    settings_set_dark_theme_flag(new_dark);
    persist_theme_to_nvs();

    confirm_restart(e);
}

static void sntp_confirm(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    s_settings_ctx.settings.time.refresh_sntp_startup = true;
    persist_sntp_refresh();
    s_settings_ctx.settings.display.time_valid = false;
    persist_valid_time_flag_to_nvs();

    confirm_restart(e);
}

static void close_theme_msgbox(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.theme_confirm_mbox) {
        return;
    }
    lv_msgbox_close(ctx->graphics.theme_confirm_mbox);
    ctx->graphics.theme_confirm_mbox = NULL;
}

static void cancel_sntp(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.sntp_confirm_mbox) {
        return;
    }
    lv_msgbox_close(ctx->graphics.sntp_confirm_mbox);
    ctx->graphics.sntp_confirm_mbox = NULL;
}

static void start_calibration_task(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.screen)
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

    lv_obj_clean(ctx->graphics.screen);
    clear_ui_refs(ctx);

    /* Run calibration asynchronously to avoid blocking the LVGL task/UI thread. */
    BaseType_t task_ok = xTaskCreate(calibration_task,
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

static void calibration_task(void *param)
{
    settings_ctx_t *ctx = (settings_ctx_t *)param;

    if (!ctx || !ctx->graphics.return_screen){
        settings_set_running_calibration(false);
        vTaskDelete(NULL);
        return;
    }

    /* Clear cached widget pointers because we delete/clean the screen. */
    clear_ui_refs(ctx);

    int prev_rotation = ctx->settings.display.screen_rotation_step;
    
    if (ctx->settings.display.screen_rotation_step != SETTINGS_DEFAULT_ROTATION_STEP && ctx->settings.display.screen_rotation_step != SETTINGS_DEFAULT_ROTATION_STEP - 2){
        ctx->settings.display.screen_rotation_step = SETTINGS_DEFAULT_ROTATION_STEP;
        apply_rotation_to_display(true);
    }

    /* Stop Screensaver While Performing Calibration*/
    bsp_display_brightness_set(100);
    screensaver_dim_stop();
    screensaver_off_stop();
    esp_err_t calib_err = calibration_run_cal(true);
    if (calib_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(calib_err));
    }
    s_settings_ctx.changing_brightness = false;  
    settings_start_screensaver_timers();

    ctx->settings.display.screen_rotation_step = prev_rotation;
    apply_rotation_to_display(true);

    bsp_display_lock(0);
    if (ctx->graphics.screen)
    {
        lv_obj_del(ctx->graphics.screen);
        ctx->graphics.screen = NULL;
    }
    ctx->active = false;
    settings_open_settings(ctx->graphics.return_screen);
    bsp_display_unlock();
    
    settings_set_running_calibration(false);
    vTaskDelete(NULL);
}

static void clear_ui_refs(settings_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->graphics.toolbar = NULL;
    ctx->graphics.brightness_label = NULL;
    ctx->graphics.brightness_slider = NULL;
    ctx->graphics.restart_confirm_mbox = NULL;
    ctx->graphics.reset_confirm_mbox = NULL;
    ctx->graphics.theme_confirm_mbox = NULL;
    ctx->graphics.sntp_confirm_mbox = NULL;
    ctx->graphics.datetime_overlay = NULL;
    ctx->graphics.screensaver_overlay = NULL;
    ctx->graphics.wifi_sntp_overlay = NULL;
    ctx->graphics.dt_month_ta = NULL;
    ctx->graphics.dt_day_ta = NULL;
    ctx->graphics.dt_year_ta = NULL;
    ctx->graphics.dt_hour_ta = NULL;
    ctx->graphics.dt_min_ta = NULL;
    ctx->graphics.dt_keyboard = NULL;
    ctx->graphics.access_point_keyboard = NULL;
    ctx->graphics.access_point_ssid_ta = NULL;
    ctx->graphics.access_point_pwd_ta = NULL;    
    ctx->graphics.dt_dialog = NULL;
    ctx->graphics.screensaver_dialog = NULL;
    ctx->graphics.wifi_sntp_dialog = NULL;
    ctx->graphics.access_point_dialog = NULL;
    ctx->graphics.access_point_ssid_ta = NULL;
    ctx->graphics.access_point_pwd_ta = NULL;    
    ctx->graphics.dt_row_time = NULL;
    ctx->graphics.ss_dim_lbl = NULL;
    ctx->graphics.ss_dim_switch = NULL;
    ctx->graphics.ss_dim_after_lbl = NULL;
    ctx->graphics.ss_seconds_lbl = NULL;
    ctx->graphics.ss_at_lbl = NULL;
    ctx->graphics.ss_pct_lbl = NULL;
    ctx->graphics.ss_dim_after_ta = NULL;
    ctx->graphics.ss_dim_pct_ta = NULL;
    ctx->graphics.ss_off_lbl = NULL;
    ctx->graphics.ss_off_switch = NULL;
    ctx->graphics.ss_off_after_lbl = NULL;
    ctx->graphics.ss_off_seconds_lbl = NULL;
    ctx->graphics.ss_off_after_ta = NULL;
    ctx->graphics.ss_keyboard = NULL;
}

static void open_screensaver(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx && ctx->graphics.screen)
    {
        build_screensaver_dialog(ctx);
    }
}


static esp_err_t build_screensaver_dialog(settings_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Close previous overlay if still open. */
    if (ctx->graphics.screensaver_overlay) {
        lv_obj_del(ctx->graphics.screensaver_overlay);
        ctx->graphics.screensaver_overlay = NULL;
        ctx->graphics.screensaver_dialog = NULL;
        ctx->graphics.ss_dim_lbl = NULL;
        ctx->graphics.ss_dim_switch = NULL;
        ctx->graphics.ss_dim_after_lbl = NULL;
        ctx->graphics.ss_seconds_lbl = NULL;
        ctx->graphics.ss_at_lbl = NULL;
        ctx->graphics.ss_pct_lbl = NULL;
        ctx->graphics.ss_dim_after_ta = NULL;
        ctx->graphics.ss_dim_pct_ta = NULL;
        ctx->graphics.ss_off_lbl = NULL;
        ctx->graphics.ss_off_switch = NULL;
        ctx->graphics.ss_off_after_lbl = NULL;
        ctx->graphics.ss_off_seconds_lbl = NULL;
        ctx->graphics.ss_off_after_ta = NULL;
        ctx->graphics.ss_keyboard = NULL;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(overlay, on_ss_background_tap, LV_EVENT_CLICKED, ctx);
    ctx->graphics.screensaver_overlay = overlay;

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
    lv_obj_add_event_cb(dlg, on_dt_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_center(dlg);
    ctx->graphics.screensaver_dialog = dlg;

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
    ctx->graphics.ss_dim_lbl = dim_lbl;

    lv_obj_t *dim_switch = lv_switch_create(row_dim);
    lv_obj_set_style_pad_all(dim_switch, 4, 0);
    styles_set_switch(dim_switch);
    lv_obj_add_event_cb(dim_switch, on_dim_switch_changed, LV_EVENT_VALUE_CHANGED, ctx);
    ctx->graphics.ss_dim_switch = dim_switch;

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
    ctx->graphics.ss_dim_after_lbl = dim_after_lbl;

    ctx->graphics.ss_dim_after_ta = lv_textarea_create(row_dim_cfg);
    lv_obj_set_width(ctx->graphics.ss_dim_after_ta, 35);
    lv_obj_clear_flag(ctx->graphics.ss_dim_after_ta, LV_OBJ_FLAG_SCROLLABLE);
    styles_set_textarea(ctx->graphics.ss_dim_after_ta);
    lv_textarea_set_one_line(ctx->graphics.ss_dim_after_ta, true);
    lv_textarea_set_max_length(ctx->graphics.ss_dim_after_ta, 3);
    if (ctx->settings.display.dim_time >= 0) {
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", ctx->settings.display.dim_time);
        lv_textarea_set_placeholder_text(ctx->graphics.ss_dim_after_ta, buf);
        if (ctx->settings.display.screen_dim) {
            lv_textarea_set_text(ctx->graphics.ss_dim_after_ta, buf);
        } else {
            lv_textarea_set_text(ctx->graphics.ss_dim_after_ta, "");
        }
    } else {
        lv_textarea_set_placeholder_text(ctx->graphics.ss_dim_after_ta, "");
        lv_textarea_set_text(ctx->graphics.ss_dim_after_ta, "");
    }
    lv_obj_add_event_cb(ctx->graphics.ss_dim_after_ta, on_ss_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.ss_dim_after_ta, on_ss_textarea_focus, LV_EVENT_CLICKED, ctx);

    lv_obj_t *seconds_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(seconds_lbl, "seconds");
    lv_obj_add_flag(seconds_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->graphics.ss_seconds_lbl = seconds_lbl;

    lv_obj_t *at_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(at_lbl, "to");
    lv_obj_add_flag(at_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->graphics.ss_at_lbl = at_lbl;

    ctx->graphics.ss_dim_pct_ta = lv_textarea_create(row_dim_cfg);
    lv_obj_set_width(ctx->graphics.ss_dim_pct_ta, 35);
    lv_obj_clear_flag(ctx->graphics.ss_dim_pct_ta, LV_OBJ_FLAG_SCROLLABLE);
    lv_textarea_set_one_line(ctx->graphics.ss_dim_pct_ta, true);
    lv_textarea_set_max_length(ctx->graphics.ss_dim_pct_ta, 3);
    styles_set_textarea(ctx->graphics.ss_dim_pct_ta); 
    if (ctx->settings.display.dim_level >= 0) {
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", ctx->settings.display.dim_level);
        lv_textarea_set_placeholder_text(ctx->graphics.ss_dim_pct_ta, buf);
        if (ctx->settings.display.screen_dim) {
            lv_textarea_set_text(ctx->graphics.ss_dim_pct_ta, buf);
        } else {
            lv_textarea_set_text(ctx->graphics.ss_dim_pct_ta, "");
        }
    } else {
        lv_textarea_set_placeholder_text(ctx->graphics.ss_dim_pct_ta, "");
        lv_textarea_set_text(ctx->graphics.ss_dim_pct_ta, "");
    }
    lv_obj_add_event_cb(ctx->graphics.ss_dim_pct_ta, on_ss_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.ss_dim_pct_ta, on_ss_textarea_focus, LV_EVENT_CLICKED, ctx);

    lv_obj_t *pct_lbl = lv_label_create(row_dim_cfg);
    lv_label_set_text(pct_lbl, "%");
    lv_obj_add_flag(pct_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->graphics.ss_pct_lbl = pct_lbl;

    if (ctx->settings.display.screen_dim) {
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
    ctx->graphics.ss_off_lbl = time_lbl;

    lv_obj_t *off_switch = lv_switch_create(row_off);
    lv_obj_set_style_pad_all(off_switch, 4, 0);
    styles_set_switch(off_switch);  
    lv_obj_add_event_cb(off_switch, on_off_switch_changed, LV_EVENT_VALUE_CHANGED, ctx);
    ctx->graphics.ss_off_switch = off_switch;

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
    ctx->graphics.ss_off_after_lbl = off_after_lbl;

    ctx->graphics.ss_off_after_ta = lv_textarea_create(row_off_cfg);
    lv_obj_set_width(ctx->graphics.ss_off_after_ta, 50);
    lv_obj_clear_flag(ctx->graphics.ss_off_after_ta, LV_OBJ_FLAG_SCROLLABLE);
    styles_set_textarea(ctx->graphics.ss_off_after_ta); 
    lv_textarea_set_one_line(ctx->graphics.ss_off_after_ta, true);
    lv_textarea_set_max_length(ctx->graphics.ss_off_after_ta, 4);
    if (ctx->settings.display.off_time >= 0) {
        char buf[12];
        lv_snprintf(buf, sizeof(buf), "%d", ctx->settings.display.off_time);
        lv_textarea_set_placeholder_text(ctx->graphics.ss_off_after_ta, buf);
        if (ctx->settings.display.screen_off) {
            lv_textarea_set_text(ctx->graphics.ss_off_after_ta, buf);
        } else {
            lv_textarea_set_text(ctx->graphics.ss_off_after_ta, "");
        }
    } else {
        lv_textarea_set_placeholder_text(ctx->graphics.ss_off_after_ta, "");
        lv_textarea_set_text(ctx->graphics.ss_off_after_ta, "");
    }
    lv_obj_add_event_cb(ctx->graphics.ss_off_after_ta, on_ss_textarea_focus, LV_EVENT_FOCUSED, ctx);
    lv_obj_add_event_cb(ctx->graphics.ss_off_after_ta, on_ss_textarea_focus, LV_EVENT_CLICKED, ctx);

    lv_obj_t *off_seconds_lbl = lv_label_create(row_off_cfg);
    lv_label_set_text(off_seconds_lbl, "seconds.");
    lv_obj_add_flag(off_seconds_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    ctx->graphics.ss_off_seconds_lbl = off_seconds_lbl;

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
    lv_obj_add_event_cb(apply_btn, apply_screensaver, LV_EVENT_CLICKED, ctx);
    
    lv_obj_t *cancel_btn = lv_button_create(row_actions);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_radius(cancel_btn, 6, 0);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);
    lv_obj_add_flag(cancel_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, close_screensaver, LV_EVENT_CLICKED, ctx);
    
    /* Keyboard anchored to bottom of overlay */
    ctx->graphics.ss_keyboard = lv_keyboard_create(overlay);
    styles_set_keyboard(ctx->graphics.ss_keyboard);
    lv_keyboard_set_mode(ctx->graphics.ss_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(ctx->graphics.ss_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.ss_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(ctx->graphics.ss_keyboard, LV_OBJ_FLAG_HIDDEN); /* show only after a field is tapped */
    lv_obj_add_event_cb(ctx->graphics.ss_keyboard, on_ss_background_tap, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.ss_keyboard, on_ss_keyboard_event, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->graphics.ss_keyboard, on_ss_keyboard_event, LV_EVENT_READY, ctx);
    lv_obj_align(ctx->graphics.ss_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

    if (ctx->settings.display.screen_off) {
        lv_obj_add_state(off_switch, LV_STATE_CHECKED);
        lv_obj_add_state(off_after_lbl, LV_STATE_DISABLED);
        lv_obj_add_state(off_seconds_lbl, LV_STATE_DISABLED);     
    } else {
        lv_obj_clear_state(off_switch, LV_STATE_CHECKED);
        lv_obj_clear_state(off_after_lbl, LV_STATE_DISABLED);
        lv_obj_clear_state(off_seconds_lbl, LV_STATE_DISABLED);       
    }

    update_dim_controls_enabled(ctx, lv_obj_has_state(ctx->graphics.ss_dim_switch, LV_STATE_CHECKED));
    update_off_controls_enabled(ctx, lv_obj_has_state(ctx->graphics.ss_off_switch, LV_STATE_CHECKED));

    return ESP_OK;
}

static void apply_screensaver(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.screensaver_overlay) {
        return;
    }

    if (!obtain_screensaver_values(ctx)){
        return;
    }

    persist_screensaver_to_nvs();
    settings_start_screensaver_timers();
    close_screensaver(e);
}

static bool dim_valid(int *new_dim_time, int *new_dim_level, settings_ctx_t *ctx)
{
    const char *dim_time_txt = ctx->graphics.ss_dim_after_ta ? lv_textarea_get_text(ctx->graphics.ss_dim_after_ta) : NULL;
    const char *dim_level_txt = ctx->graphics.ss_dim_pct_ta ? lv_textarea_get_text(ctx->graphics.ss_dim_pct_ta) : NULL;
    int parsed_time = 0;
    int parsed_level = 0;

    /* dim time: 1..9999 (textarea limited to 3 chars) */
    if (!parse_int_range(dim_time_txt, 1, 9999, &parsed_time)) {
        show_invalid_input_mbox();
        return false;
    }

    /* Accept any 0..100 value, clamp later against brightness/minimum. */
    if (!parse_int_range(dim_level_txt, 0, 100, &parsed_level)) {
        show_invalid_input_mbox();
        return false;
    }

    *new_dim_time = parsed_time;
    *new_dim_level = parsed_level;

    return true;
}

static bool off_valid(int *new_off_time, settings_ctx_t *ctx)
{
    const char *off_time_txt = ctx->graphics.ss_off_after_ta ? lv_textarea_get_text(ctx->graphics.ss_off_after_ta) : NULL;
    int parsed_off = 0;
    if (!parse_int_range(off_time_txt, 1, 99999, &parsed_off)) {
        show_invalid_input_mbox();
        return false;
    }
    *new_off_time = parsed_off;

    return true;
}

static void apply_in_memory_state(settings_ctx_t *ctx, bool dim_on, int *new_dim_level, int new_dim_time)
{
    ctx->settings.display.screen_dim = dim_on;
    ctx->settings.display.dim_time = new_dim_time;

    if (*new_dim_level >= 0) {
        int max_level = ctx->settings.display.saved_brightness > 0 ? ctx->settings.display.saved_brightness : SETTINGS_DEFAULT_BRIGHTNESS;
        if (max_level < SETTINGS_MINIMUM_BRIGHTNESS) {
            max_level = SETTINGS_MINIMUM_BRIGHTNESS;
        }
        if (*new_dim_level > max_level) *new_dim_level = max_level;
        if (*new_dim_level < SETTINGS_MINIMUM_BRIGHTNESS) *new_dim_level = SETTINGS_MINIMUM_BRIGHTNESS;
    }
}

static bool obtain_screensaver_values(settings_ctx_t *ctx)
{
    bool dim_on = ctx->graphics.ss_dim_switch && lv_obj_has_state(ctx->graphics.ss_dim_switch, LV_STATE_CHECKED);
    bool off_on = ctx->graphics.ss_off_switch && lv_obj_has_state(ctx->graphics.ss_off_switch, LV_STATE_CHECKED);

    int new_dim_time = ctx->settings.display.dim_time;
    int new_dim_level = ctx->settings.display.dim_level;
    int new_off_time = ctx->settings.display.off_time;

    if (dim_on) {
        if (!dim_valid(&new_dim_time, &new_dim_level, ctx)){
            return false;
        }    
    }

    if (off_on) {
        if (!off_valid(&new_off_time, ctx)){
            return false;
        }
    }

    apply_in_memory_state(ctx, dim_on, &new_dim_level, new_dim_time);

    ctx->settings.display.dim_level = new_dim_level;
    ctx->settings.display.screen_off = off_on;
    ctx->settings.display.off_time = new_off_time;

    return true;
}

static void apply_ap_data(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.access_point_dialog) {
        return;
    }

    const char *ssid_txt = ctx->graphics.access_point_ssid_ta ? lv_textarea_get_text(ctx->graphics.access_point_ssid_ta) : "";
    const char *pwd_txt = ctx->graphics.access_point_pwd_ta ? lv_textarea_get_text(ctx->graphics.access_point_pwd_ta) : "";

    /* Copy into bounded buffers */
    strncpy(ctx->settings.ap_ssid, ssid_txt ? ssid_txt : "", sizeof(ctx->settings.ap_ssid) - 1);
    ctx->settings.ap_ssid[sizeof(ctx->settings.ap_ssid) - 1] = '\0';
    strncpy(ctx->settings.ap_pwd, pwd_txt ? pwd_txt : "", sizeof(ctx->settings.ap_pwd) - 1);
    ctx->settings.ap_pwd[sizeof(ctx->settings.ap_pwd) - 1] = '\0';

    persist_ap_credentials_to_nvs();
    close_access_point_dialog(e);
}

static void close_screensaver(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e); 
    if (ctx && ctx->graphics.screensaver_overlay) {
        lv_obj_del(ctx->graphics.screensaver_overlay);
        ctx->graphics.screensaver_overlay = NULL;
        ctx->graphics.screensaver_dialog = NULL;
        ctx->graphics.ss_dim_lbl = NULL;
        ctx->graphics.ss_dim_switch = NULL;
        ctx->graphics.ss_dim_after_lbl = NULL;
        ctx->graphics.ss_seconds_lbl = NULL;
        ctx->graphics.ss_at_lbl = NULL;
        ctx->graphics.ss_pct_lbl = NULL;
        ctx->graphics.ss_dim_after_ta = NULL;
        ctx->graphics.ss_dim_pct_ta = NULL;
        ctx->graphics.ss_off_lbl = NULL;
        ctx->graphics.ss_off_switch = NULL;
        ctx->graphics.ss_off_after_lbl = NULL;
        ctx->graphics.ss_off_seconds_lbl = NULL;
        ctx->graphics.ss_off_after_ta = NULL;
        ctx->graphics.ss_keyboard = NULL;
    }    
}

static void close_connection_dialog(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e); 
    if (ctx && ctx->graphics.wifi_sntp_overlay) {
        lv_obj_del(ctx->graphics.wifi_sntp_overlay);
        ctx->graphics.wifi_sntp_dialog = NULL;
        ctx->graphics.wifi_sntp_overlay = NULL;
        
        ctx->graphics.access_point_dialog = NULL;
        ctx->graphics.access_point_ssid_ta = NULL;
        ctx->graphics.access_point_pwd_ta = NULL;
        ctx->graphics.access_point_keyboard = NULL;
    }    
}

static void close_access_point_dialog(lv_event_t *e)
{
    settings_ctx_t *ctx = lv_event_get_user_data(e); 
    if (ctx && ctx->graphics.wifi_sntp_overlay) {
        lv_obj_del(ctx->graphics.wifi_sntp_overlay);
        ctx->graphics.wifi_sntp_overlay = NULL;
        ctx->graphics.access_point_dialog = NULL;
        ctx->graphics.access_point_ssid_ta = NULL;
        ctx->graphics.access_point_pwd_ta = NULL;
        ctx->graphics.access_point_keyboard = NULL;
    }   
    
    build_wifi_sntp_dialog(ctx);
}
