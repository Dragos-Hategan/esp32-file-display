#include "calibration_xpt2046.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "nvs.h"

#include "touch_xpt2046.h"
#include "settings.h"
#include "styles.h"

typedef struct
{
    float xA, xB, xC; // for x' = xA*x + xB*y + xC
    float yA, yB, yC; // for y' = yA*x + yB*y + yC
    bool valid;
    uint32_t magic; // 0xC411B007
    uint32_t crc32; // simple, for integrity
} touch_cal_t;

typedef struct
{
    int tx;
    int ty;
    int rx;
    int ry;
} cal_point_t;

typedef struct
{
    SemaphoreHandle_t xSemaphore;
    bool response;
} msg_box_response_t;

static const int CALIBRATION_MESSAGE_DISPLAY_TIME_MS = 3000;
static touch_cal_t s_cal = {0};
static bool s_show_loader = true;

/** @brief 5-point calibration target set (screen-space). */
static cal_point_t s_cal_points[5] = {
    {20, 20, 0, 0},                             // top left
    {TOUCH_X_MAX - 20, 20, 0, 0},               // top right
    {TOUCH_X_MAX - 20, TOUCH_Y_MAX - 20, 0, 0}, // bottom right
    {20, TOUCH_Y_MAX - 20, 0, 0},               // bottom left
    {TOUCH_X_MAX / 2, TOUCH_Y_MAX / 2, 0, 0}    // center
};

/**
 * @brief Load touch calibration from NVS into the internal state.
 *
 * Reads the calibration blob, validates magic and CRC, and updates @ref s_cal.
 *
 * @param existing_cal Non-NULL pointer (unused for output here; required to keep signature uniform).
 * @return true if a valid calibration was loaded, false otherwise.
 */
static bool touch_cal_load_nvs(const touch_cal_t *existing_cal);

/**
 * @brief Save a valid touch calibration to NVS with CRC protection.
 *
 * Writes a blob identified by @c TOUCH_CAL_NVS_NS / @c TOUCH_CAL_NVS_KEY.
 * A magic and CRC-32 are included for integrity checks.
 *
 * @param cal Pointer to a valid calibration structure (cal->valid must be true).
 * @return esp_err_t ESP_OK on success or an error code from NVS APIs.
 */
static esp_err_t touch_cal_save_nvs(const touch_cal_t *cal);

/**
 * @brief Compute CRC-32 (poly 0xEDB88320, reflected) over a byte buffer.
 *
 * @param data Pointer to data.
 * @param len  Data length in bytes.
 * @return uint32_t CRC-32 of the buffer.
 */
static uint32_t crc32_fast(const void *data, size_t len);

/**
 * @brief Run a 5-point on-screen touch calibration and persist the result to NVS.
 *
 * Displays a temporary LVGL calibration screen, draws crosshair targets at five
 * predefined positions, and for each target samples raw touch coordinates using
 * @ref sample_raw(). From the 5 (raw, target) pairs it computes an affine
 * transform:
 * @code
 * x' = xA * x + xB * y + xC
 * y' = yA * x + yB * y + yC
 * @endcode
 * via a least-squares fit (normal equations). If the system is near-singular,
 * the calibration is marked invalid and an error is returned.
 *
 * On success, the computed coefficients are stored in the global calibration
 * structure @ref s_cal, marked as valid, and saved to NVS with
 * touch_cal_save_nvs().
 *
 * During calibration, the function:
 *  - replaces the current LVGL screen with a calibration screen,
 *  - temporarily disables the LVGL touch input device, and
 *  - restores the original screen and re-enables input before returning.
 * 
 * @param[in] calibration_found  True if a valid calibration was loaded from NVS,
 *                               false if no calibration data is available.
 *
 * @warning Assumes the caller does not hold the LVGL/BSP display lock.
 *          This function acquires and releases bsp_display_lock() internally.
 *
 * @return ESP_OK                 Calibration completed and saved successfully.
 * @return ESP_FAIL               Calibration failed due to a singular/ill-conditioned matrix.
 * @return Other esp_err_t codes  If sampling or saving to NVS fails.
 */
static esp_err_t run_5point_touch_calibration(bool calibration_found);

/**
 * @brief Read averaged raw touch coordinates from the XPT2046 controller.
 *
 * This function performs up to 12 sampling attempts, spaced by ~15 ms.
 * Only samples where the touch controller reports valid coordinates are used.
 * The function accumulates the valid samples and returns the averaged raw
 * (x, y) coordinates through the output parameters.
 *
 * @param[out] rx Pointer to an integer where the averaged raw X value will be stored.
 * @param[out] ry Pointer to an integer where the averaged raw Y value will be stored.
 *
 * @return ESP_OK on success - ESP_ERR_INVALID_ARG parameter error
 * @return ESP_FAIL sending command error, slave hasn't ACK the transfer
 * @return ESP_ERR_TIMEOUT operation timeout because the bus is busy
 */
static esp_err_t sample_raw(int *rx, int *ry);

/**
 * @brief Display a full screen with a centered text message.
 *
 * Places a centered label, updates layout, and forces an immediate refresh.
 *
 * @param[in] calibration_found  True if a valid calibration was loaded from NVS,
 *                               false if no calibration data is available.
 *
 * @note Runs on the LVGL thread/context; make sure your platform’s display
 *       locking rules are respected before calling if required.
 */
static void ui_show_calibration_message(bool calibration_found);

/**
 * @brief Show a modal Yes/No dialog with optional 10-second auto-yes countdown.
 *
 * Builds a centered vertical stack on the top layer containing:
 *  - an LVGL message box with the fixed question "Run Touch Screen Calibration?"
 *    and two buttons: **Yes** and **No**
 *  - optionally a loader row showing "Performing Calibration In:" with a circular progress
 *    arc and numeric countdown (10 → 1) when the loader is enabled
 *  - a row with the label "Ask For Calibration At Startup" and a switch (default ON)
 *
 * The function blocks the calling task:
 *  - When the loader/countdown is enabled: until the user presses a button or the
 *    10-second timeout elapses (timeout treated as **Yes**).
 *  - When the loader is disabled: until the user explicitly presses Yes/No (no timeout).
 *
 * Thread-safety: internal calls to @ref bsp_display_lock / @ref bsp_display_unlock
 * protect LVGL operations. This function must be called from a task context
 * (not from an ISR) after LVGL has been initialized and a display is active.
 *
 * External dependencies:
 *  - FreeRTOS (semaphores, delays, ticks)
 *  - LVGL (objects, message box, arc, labels, refresh)
 *  - A user-provided event callback `event_cb` that writes the button result to
 *    a @c msg_box_response_t passed via @c user_data and gives the binary semaphore.
 *
 * @return bool
 * @retval true  if the user pressed **Yes** or the countdown timed out
 * @retval false if the user pressed **No**
 *
 * @note When the loader is enabled, the countdown duration is fixed at 10000ms. The
 *       toggle state persists to Settings (NVS) via @ref settings_set_calibration_prompt_enabled.
 * @warning The function performs blocking waits (semaphore / vTaskDelay).
 * @warning This function assumes there is no LVGL display lock already acquired.
 */
static bool ui_yes_no_dialog(void);

/**
 * @brief Draw a four-arrow crosshair pointing to a target coordinate.
 *
 * Clears the active screen and draws four black arrows (up/down/left/right)
 * converging toward (@p x, @p y). A shared line style is lazily initialized
 * on first call. Forces an immediate display refresh when done.
 *
 * @param x Target X coordinate in screen space.
 * @param y Target Y coordinate in screen space.
 *
 * @note Designed for LVGL v9. Adjust types/APIs if using a different LVGL major version.
 */
static void draw_cross(int x, int y);

/**
 * @brief Create an LVGL line object with a given point array and optional style.
 *
 * Convenience wrapper around @c lv_line_create and @c lv_line_set_points.
 *
 * @param parent Parent LVGL object.
 * @param pts    Pointer to an array of points (in screen coordinates).
 * @param cnt    Number of points in @p pts.
 * @param style  Optional style to apply to the line (can be NULL).
 *
 * @note Static helper; not intended to be used outside this module.
 */
static void make_line(lv_obj_t *parent, const lv_point_precise_t *pts, uint16_t cnt, const lv_style_t *style);

/**
 * @brief LVGL button click event handler for the Yes/No dialog.
 *
 * Interprets the clicked button by reading its first child label’s text.
 * Sets the @ref msg_box_response_t::response flag accordingly and gives
 * the semaphore to wake the waiting task.
 *
 * @param e LVGL event descriptor (expects LV_EVENT_CLICKED).
 *
 * @pre The event’s user data must be a valid pointer to @ref msg_box_response_t.
 * @pre The target object must be a button whose first child is a label with text "Yes" or "No".
 *
 * @note Static helper; not intended to be called directly outside this module.
 */
static void event_cb(lv_event_t *e);

/**
 * @brief Clamp an integer to a [lo, hi] range.
 *
 * @param v  Value to clamp.
 * @param lo Lower bound (inclusive).
 * @param hi Upper bound (inclusive).
 * @return int Clamped value.
 */
static inline int clampi(int v, int lo, int hi);

void load_nvs_calibration(bool *calibration_found)
{
    const touch_cal_t *existing_cal = &s_cal;
    *calibration_found = touch_cal_load_nvs(existing_cal);
    ESP_LOGI("touch_calibration", "%s", *calibration_found ? "Touch driver is already calibrated" : "Touch driver needs calibration");
}

esp_err_t run_calibration(bool calibration_found)
{
    esp_err_t run_calibration_err;
    if (!calibration_found)
    {
        // No calibration saved: runs calibration directly
        run_calibration_err = run_5point_touch_calibration(calibration_found);
        if (run_calibration_err != ESP_OK){
            ESP_LOGE("Calibration Test", "5 point calibration failed: (%s)", esp_err_to_name(run_calibration_err));
            return run_calibration_err;
        }
    }
    else
    {
        // Run calibration?
        bool run;
        run = ui_yes_no_dialog();

        if (run)
        {
            run_calibration_err = run_5point_touch_calibration(calibration_found);
            if (run_calibration_err != ESP_OK){
                ESP_LOGE("Calibration Test", "5 point calibration failed: (%s)", esp_err_to_name(run_calibration_err));
                return run_calibration_err;
            }
        }
        else
        {
            // We keep s_cal from NVS; we just clear the screen
            bsp_display_lock(0);
            lv_obj_clean(lv_screen_active());
            bsp_display_unlock();
        }
    }

    return ESP_OK;
}

void calibration_set_show_loader(bool enable)
{
    s_show_loader = enable;
}

void apply_touch_calibration(uint16_t raw_x, uint16_t raw_y, lv_point_t *out_point, int xmax, int ymax)
{
    if (!s_cal.valid)
    {
        out_point->x = clampi(raw_x, 0, xmax - 1);
        out_point->y = clampi(raw_y, 0, ymax - 1);
        return;
    }

    float xf = s_cal.xA * raw_x + s_cal.xB * raw_y + s_cal.xC;
    float yf = s_cal.yA * raw_x + s_cal.yB * raw_y + s_cal.yC;

    out_point->x = clampi((int)(xf + 0.5f), 0, xmax - 1);
    out_point->y = clampi((int)(yf + 0.5f), 0, ymax - 1);
}

static bool touch_cal_load_nvs(const touch_cal_t *existing_cal)
{
    if (!existing_cal)
        return false;

    nvs_handle_t h;
    if (nvs_open(TOUCH_CAL_NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return false;

    touch_cal_t blob;
    size_t sz = sizeof(blob);
    esp_err_t err = nvs_get_blob(h, TOUCH_CAL_NVS_KEY, &blob, &sz);
    nvs_close(h);
    if (err != ESP_OK || sz != sizeof(blob))
        return false;

    if (blob.magic != 0xC411B007)
        return false;

    uint32_t crc = crc32_fast(&blob, sizeof(blob) - sizeof(blob.crc32));
    if (crc != blob.crc32)
        return false;

    s_cal.xA = blob.xA;
    s_cal.xB = blob.xB;
    s_cal.xC = blob.xC;
    s_cal.yA = blob.yA;
    s_cal.yB = blob.yB;
    s_cal.yC = blob.yC;
    s_cal.valid = true;

    return true;
}

static esp_err_t touch_cal_save_nvs(const touch_cal_t *cal)
{
    if (!cal || !cal->valid)
        return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(TOUCH_CAL_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    touch_cal_t blob = {
        .xA = cal->xA, .xB = cal->xB, .xC = cal->xC, .yA = cal->yA, .yB = cal->yB, .yC = cal->yC, .magic = 0xC411B007};
    blob.crc32 = crc32_fast(&blob, sizeof(blob) - sizeof(blob.crc32));

    err = nvs_set_blob(h, TOUCH_CAL_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK)
        err = nvs_commit(h);
    
    nvs_close(h);
    return err;
}

static uint32_t crc32_fast(const void *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *p = (const uint8_t *)data;
    while (len--)
    {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
    }
    return ~crc;
}

static esp_err_t run_5point_touch_calibration(bool calibration_found)
{
    const char *TAG = "5 Point Calibration";
    esp_err_t calibration_err;

    /* ----- Update screen for calibration ----- */
    bsp_display_lock(0);
    ui_show_calibration_message(calibration_found);
    bsp_display_unlock();

    calibration_found ? vTaskDelay(pdMS_TO_TICKS(CALIBRATION_MESSAGE_DISPLAY_TIME_MS)) :
                        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_MESSAGE_DISPLAY_TIME_MS) + 500);

    lv_indev_t *indev = touch_get_indev();
    if (indev)
    {
        bsp_display_lock(0);
        lv_indev_enable(indev, false);
        bsp_display_unlock();
    }

    /* ----- Collect raw data ----- */
    for (int i = 0; i < 5; i++)
    {
        bsp_display_lock(0);
        draw_cross(s_cal_points[i].tx, s_cal_points[i].ty);
        bsp_display_unlock();

        calibration_err = sample_raw(&s_cal_points[i].rx, &s_cal_points[i].ry);
        if (calibration_err != ESP_OK){
            ESP_LOGE(TAG, "Failed to sample calibration data from touch driver: (%s)", esp_err_to_name(calibration_err));
            return calibration_err;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (indev)
    {
        bsp_display_lock(0);
        lv_obj_clean(lv_screen_active());
        lv_indev_enable(indev, true);
        bsp_display_unlock();
    }

    /* ----- Calculate the coefficients for the affine transformation ----- */
    float Sx = 0;
    float Sy = 0;
    float Sxx = 0;
    float Syy = 0;
    float Sxy = 0;

    float Sx_tx = 0;
    float Sy_tx = 0;
    float Sx_ty = 0;
    float Sy_ty = 0;
    float S1 = 0;
    float Stx = 0;
    float Sty = 0;

    for (int i = 0; i < 5; i++)
    {
        float x = s_cal_points[i].rx;
        float y = s_cal_points[i].ry;
        float tx = s_cal_points[i].tx;
        float ty = s_cal_points[i].ty;

        Sx += x;
        Sy += y;
        Sxx += x * x;
        Syy += y * y;
        Sxy += x * y;
        S1 += 1.0f;

        Sx_tx += x * tx;
        Sy_tx += y * tx;
        Stx += tx;

        Sx_ty += x * ty;
        Sy_ty += y * ty;
        Sty += ty;
    }

    float denom = (Sxx * Syy - Sxy * Sxy);
    if (fabsf(denom) < 1e-6f)
    {
        s_cal.valid = false;

        ESP_LOGE(TAG, "Calibration failed: singular matrix");
        return ESP_FAIL;
    }

    // Coefficients for X
    s_cal.xA = (Sx_tx * Syy - Sy_tx * Sxy) / denom;
    s_cal.xB = (Sy_tx * Sxx - Sx_tx * Sxy) / denom;
    s_cal.xC = (Stx - s_cal.xA * Sx - s_cal.xB * Sy) / S1;

    // Coefficients for Y
    s_cal.yA = (Sx_ty * Syy - Sy_ty * Sxy) / denom;
    s_cal.yB = (Sy_ty * Sxx - Sx_ty * Sxy) / denom;
    s_cal.yC = (Sty - s_cal.yA * Sx - s_cal.yB * Sy) / S1;

    s_cal.valid = true;

    calibration_err = touch_cal_save_nvs(&s_cal);
    if (calibration_err != ESP_OK){
        ESP_LOGE(TAG, "Failed to save calibration data to NVS: (%s)", esp_err_to_name(calibration_err));
        return calibration_err;
    }

    ESP_LOGI(TAG, "Touch cal saved to NVS");
    return ESP_OK;
}

static esp_err_t sample_raw(int *rx, int *ry)
{
    esp_err_t sample_raw_err = ESP_OK;
    esp_lcd_touch_handle_t touch_handle = touch_get_handle();

    uint32_t sx = 0;
    uint32_t sy = 0;
    uint32_t n = 0;

    while (n < 12)
    {
        uint16_t x, y;
        uint8_t btn;
        sample_raw_err = esp_lcd_touch_read_data(touch_handle);
        if (sample_raw_err != ESP_OK){
            ESP_LOGE("Raw Calibration Sample", "Failed to read lcd touch data: (%s)", esp_err_to_name(sample_raw_err));
        }

        if (esp_lcd_touch_get_coordinates(touch_handle, &x, &y, NULL, &btn, 1))
        {
            sx += x;
            sy += y;
            n++;
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
    *rx = sx / n;
    *ry = sy / n;

    return ESP_OK;
}

static void ui_show_calibration_message(bool calibration_found)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_clean(scr);

    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_scroll_dir(scr, LV_DIR_NONE);
    styles_set_bg_color(scr, 0);

    lv_obj_t *lbl = lv_label_create(scr);
    styles_set_text_color(lbl, 0);
    if (!calibration_found){
        lv_label_set_text(lbl, "No Previous Calibration Found\n\nGet Ready For Touch Screen Calibration\n\nClick Inside The Pointing Arrows");
    }else{
        lv_label_set_text(lbl, "Get Ready For Touch Screen Calibration\n\nClick Inside The Pointing Arrows");
    }
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(lbl, LV_PCT(100));
    lv_obj_center(lbl);

    lv_obj_update_layout(scr);
    lv_refr_now(NULL);
}

static void ui_on_ask_switch_changed(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    settings_set_calibration_prompt_enabled(enabled);
}

static bool ui_yes_no_dialog(void)
{
    bsp_display_lock(0);

    /* Use the top layer so flex/grid layouts on the active screen don't affect positioning. */
    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);

    /* Centered vertical stack to keep equal top/bottom margins. */
    lv_obj_t *stack = lv_obj_create(overlay);
    lv_obj_remove_style_all(stack);
    lv_obj_set_size(stack, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(stack, 0, 0);
    lv_obj_set_style_pad_gap(stack, 12, 0);
    lv_obj_set_flex_flow(stack, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(stack, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_center(stack);

    lv_obj_t *mbox = lv_msgbox_create(stack);
    lv_obj_set_style_radius(mbox, 6, LV_PART_MAIN);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, lv_pct(90), 0);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Run Touch Screen Calibration?");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(label, lv_pct(100));

    msg_box_response_t msg_box_response = {
        .xSemaphore = xSemaphoreCreateBinary(),
        .response = false};

    lv_obj_t *btn;
    btn = lv_msgbox_add_footer_button(mbox, "Yes");
    styles_set_button(btn);
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_CLICKED, &msg_box_response);
    btn = lv_msgbox_add_footer_button(mbox, "No");
    styles_set_button(btn);
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_CLICKED, &msg_box_response);

    lv_obj_t *loader_wrap = NULL;
    lv_obj_t *loading_arc = NULL;
    lv_obj_t *countdown_label = NULL;
    if (s_show_loader) {
        /* Compact row aligned under the dialog: text on the left, countdown on the right. */
        loader_wrap = lv_obj_create(stack);
        lv_obj_remove_style_all(loader_wrap);
        lv_obj_set_style_radius(loader_wrap, 6, LV_PART_MAIN);
        lv_obj_set_style_border_width(loader_wrap, 1, LV_PART_MAIN);
        styles_set_border_color(loader_wrap, LV_PART_MAIN);
        styles_set_bg_color(loader_wrap, 0);
        lv_obj_set_style_bg_opa(loader_wrap, LV_OPA_COVER, 0);         
        lv_obj_set_style_pad_all(loader_wrap, 4, 0);
        lv_obj_set_style_pad_gap(loader_wrap, 10, 0);
        lv_obj_set_flex_flow(loader_wrap, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(loader_wrap, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_width(loader_wrap, 260);
        lv_obj_set_height(loader_wrap, LV_SIZE_CONTENT);

        lv_obj_t *performing_label = lv_label_create(loader_wrap);
        lv_label_set_text(performing_label, "Performing Calibration In:");
        lv_obj_set_style_text_align(performing_label, LV_TEXT_ALIGN_LEFT, 0);
        styles_set_text_color(performing_label, 0);
        lv_obj_set_width(performing_label, LV_SIZE_CONTENT);
        lv_obj_set_flex_grow(performing_label, 1);

        loading_arc = lv_arc_create(loader_wrap);
        lv_obj_set_size(loading_arc, 48, 48); // slightly smaller for better proportions
        lv_arc_set_range(loading_arc, 0, 100);
        lv_arc_set_bg_angles(loading_arc, 0, 360);
        lv_arc_set_rotation(loading_arc, 270);
        lv_arc_set_value(loading_arc, 100);
        lv_obj_remove_style(loading_arc, NULL, LV_PART_KNOB);
        styles_set_arc(loading_arc);

        countdown_label = lv_label_create(loading_arc);
        lv_obj_set_style_text_font(countdown_label, LV_FONT_DEFAULT, 0);
        lv_label_set_text(countdown_label, "10");
        styles_set_text_color(countdown_label, 0);
        lv_obj_center(countdown_label);
    }

    /* Toggle row under the loader: text on the left, switch on the right. */
    lv_obj_t *toggle_row = lv_obj_create(stack);
    lv_obj_remove_style_all(toggle_row);
    lv_obj_set_style_radius(toggle_row, 6, LV_PART_MAIN);
    styles_set_border_color(toggle_row, LV_PART_MAIN);
    lv_obj_set_style_border_width(toggle_row, 1, LV_PART_MAIN);
    styles_set_bg_color(toggle_row, 0);
    lv_obj_set_style_bg_opa(toggle_row, LV_OPA_COVER, 0);    
    lv_obj_set_style_pad_all(toggle_row, 6, 0);
    lv_obj_set_style_pad_gap(toggle_row, 12, 0);
    lv_obj_set_flex_flow(toggle_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(toggle_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_width(toggle_row, lv_pct(80)); /* keep centered and within screen */
    lv_obj_set_height(toggle_row, LV_SIZE_CONTENT);

    lv_obj_t *ask_label = lv_label_create(toggle_row);
    lv_label_set_text(ask_label, "Ask For Calibration At Startup");
    lv_obj_set_style_text_align(ask_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(ask_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(ask_label, LV_PCT(100));
    styles_set_text_color(ask_label, 0);
    lv_obj_set_flex_grow(ask_label, 1); /* push switch to the far right */

    lv_obj_t *ask_switch = lv_switch_create(toggle_row);
    styles_set_switch(ask_switch);
    bool prompt_enabled = settings_get_calibration_prompt_enabled();
    if (prompt_enabled) {
        lv_obj_add_state(ask_switch, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ask_switch, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(ask_switch, ui_on_ask_switch_changed, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_invalidate(overlay);
    lv_refr_now(lv_disp_get_default());

    bsp_display_unlock();

    if (!s_show_loader)
    {
        /* Settings flow: no countdown/auto-yes. Wait for explicit Yes/No. */
        xSemaphoreTake(msg_box_response.xSemaphore, portMAX_DELAY);
    }
    else
    {
        const uint32_t countdown_ms = 10000;
        TickType_t start_ticks = xTaskGetTickCount();
        TickType_t timeout_ticks = pdMS_TO_TICKS(countdown_ms);
        int last_second_displayed = 5;
        int last_arc_value = 100;

        while (true)
        {
            if (xSemaphoreTake(msg_box_response.xSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                break;
            }

            TickType_t now_ticks = xTaskGetTickCount();
            TickType_t elapsed_ticks = now_ticks - start_ticks;
            uint32_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;

            if (elapsed_ticks >= timeout_ticks)
            {
                msg_box_response.response = true;
                break;
            }

            int seconds_left = 10 - (elapsed_ms / 1000);
            if (seconds_left < 1)
            {
                seconds_left = 1;
            }

            int arc_value = 100 - (int)((elapsed_ms * 100) / countdown_ms);
            if (arc_value < 0)
                arc_value = 0;

            if (seconds_left != last_second_displayed || arc_value != last_arc_value)
            {
                bsp_display_lock(0);
                if (s_show_loader && countdown_label && seconds_left != last_second_displayed)
                {
                    lv_label_set_text_fmt(countdown_label, "%d", seconds_left);
                    last_second_displayed = seconds_left;
                }
                if (s_show_loader && loading_arc && arc_value != last_arc_value)
                {
                    lv_arc_set_value(loading_arc, arc_value);
                    last_arc_value = arc_value;
                }
                bsp_display_unlock();
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    bsp_display_lock(0);
    lv_obj_del(overlay); /* deletes dialog and loader wrap */
    bsp_display_unlock();

    vSemaphoreDelete(msg_box_response.xSemaphore);

    return msg_box_response.response;
}

static void draw_cross(int x, int y)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr); // remove old children

    /* ---- Shared line style (black, width 3) ----- */
    static lv_style_t st;
    static bool inited = false;
    if (!inited)
    {
        lv_style_init(&st);
        lv_style_set_line_width(&st, 3);
        bool dark_theme = settings_get_dark_theme_flag();
        lv_color_t line_color = dark_theme ? UI_COLOR_TEXT_DARK : UI_COLOR_TEXT_LIGHT;
        lv_style_set_line_color(&st, line_color);
        lv_style_set_line_rounded(&st, false);
        inited = true;
    }

    const int gap = 5;  // tip distance from center
    const int len = 24; // shaft length
    const int head = 7; // arrow head size

    /* ---- Up arrow (points down toward center) ----- */
    lv_point_precise_t up_shaft[] = {{x, y - gap - len}, {x, y - gap}};
    lv_point_precise_t up_head_l[] = {{x, y - gap}, {x - head, y - gap - head}};
    lv_point_precise_t up_head_r[] = {{x, y - gap}, {x + head, y - gap - head}};
    make_line(scr, up_shaft, 2, &st);
    make_line(scr, up_head_l, 2, &st);
    make_line(scr, up_head_r, 2, &st);

    /* ---- Down arrow (points up toward center) ----- */
    lv_point_precise_t down_shaft[] = {{x, y + gap + len}, {x, y + gap}};
    lv_point_precise_t down_head_l[] = {{x, y + gap}, {x - head, y + gap + head}};
    lv_point_precise_t down_head_r[] = {{x, y + gap}, {x + head, y + gap + head}};
    make_line(scr, down_shaft, 2, &st);
    make_line(scr, down_head_l, 2, &st);
    make_line(scr, down_head_r, 2, &st);

    /* ---- Left arrow (points right toward center) ----- */
    lv_point_precise_t left_shaft[] = {{x - gap - len, y}, {x - gap, y}};
    lv_point_precise_t left_head_u[] = {{x - gap, y}, {x - gap - head, y - head}};
    lv_point_precise_t left_head_d[] = {{x - gap, y}, {x - gap - head, y + head}};
    make_line(scr, left_shaft, 2, &st);
    make_line(scr, left_head_u, 2, &st);
    make_line(scr, left_head_d, 2, &st);

    /* ---- Right arrow (points left toward center) ----- */
    lv_point_precise_t right_shaft[] = {{x + gap + len, y}, {x + gap, y}};
    lv_point_precise_t right_head_u[] = {{x + gap, y}, {x + gap + head, y - head}};
    lv_point_precise_t right_head_d[] = {{x + gap, y}, {x + gap + head, y + head}};
    make_line(scr, right_shaft, 2, &st);
    make_line(scr, right_head_u, 2, &st);
    make_line(scr, right_head_d, 2, &st);

    lv_refr_now(NULL);
}

static void make_line(lv_obj_t *parent, const lv_point_precise_t *pts, uint16_t cnt, const lv_style_t *style)
{
    lv_obj_t *l = lv_line_create(parent);
    if (style)
    {
        lv_obj_add_style(l, style, 0);
    }

    lv_line_set_points(l, pts, cnt);
}

static void event_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    msg_box_response_t *msg_response = (msg_box_response_t *)lv_event_get_user_data(e);

    if (strcmp(lv_label_get_text(label), "Yes") == 0)
    {
        msg_response->response = true;
    }
    else if (strcmp(lv_label_get_text(label), "No") == 0)
    {
        msg_response->response = false;
    }

    xSemaphoreGive(msg_response->xSemaphore);
}

static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
