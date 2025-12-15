#include "touch_xpt2046.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "bsp/esp-bsp.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_lcd_touch_xpt2046.h"
#include "calibration_xpt2046.h"
#include "settings.h"

static const char *TAG = "touch_driver";
static esp_lcd_touch_handle_t touch_handle = NULL;
static lv_indev_t *touch_indev = NULL;

/**
 * @brief Register the touch controller as an LVGL pointer device.
 *
 * Creates an LVGL input device, sets it to pointer type, and attaches the
 * @ref lvgl_touch_read_cb callback.
 *
 * @return lv_indev_t* LVGL input device handle.
 */
static lv_indev_t *register_touch_with_lvgl(void);

/**
 * @brief LVGL input device read callback for the touch controller.
 *
 * Reads the latest touch sample from the XPT2046 via esp_lcd_touch, applies calibration,
 * and fills @p data with pointer position and state.
 *
 * @param indev Unused LVGL input device handle.
 * @param data  LVGL input data to fill.
 */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);

/**
 * @brief Process a raw touch sample and update press state/calibrated point.
 *
 * Handles wake/suppress logic when brightness is off or waking, and marks
 * @p pressed true only when the touch should be consumed by LVGL.
 *
 * @param x            Raw X coordinate from controller.
 * @param y            Raw Y coordinate from controller.
 * @param pressed      Output flag set true when touch is valid.
 * @param prev_pressed Previous pressed flag (may be cleared on suppression).
 * @param data         LVGL input data to fill with calibrated point.
 */
static void handle_touch_press(uint16_t x, uint16_t y, bool *pressed, bool *prev_pressed, lv_indev_data_t *data);

/**
 * @brief Finalize LVGL state/logging after touch processing.
 *
 * Sets LVGL state, logs edge presses, triggers brightness wake when needed,
 * and updates @p prev_pressed.
 *
 * @param pressed      Current pressed state.
 * @param x            Calibrated X coordinate.
 * @param y            Calibrated Y coordinate.
 * @param prev_pressed Pointer to previous pressed flag (will be updated).
 * @param data         LVGL input data to update state in.
 */
static void finalize_touch_state(bool pressed, uint16_t x, uint16_t y, bool *prev_pressed, lv_indev_data_t *data);

esp_err_t touch_init(void)
{
    esp_err_t touch_init_err = ESP_OK;
    const bool shared_bus = (CONFIG_TOUCH_SPI_HOST == BSP_LCD_SPI_NUM ||
                             CONFIG_TOUCH_SPI_HOST == CONFIG_SDSPI_BUS_HOST);

    ESP_LOGI(TAG, "Initializing SPI bus%s", shared_bus ? " (shared)" : "");
    if (!shared_bus) {
        spi_bus_config_t buscfg = {
            .sclk_io_num = CONFIG_TOUCH_SPI_SCLK_GPIO,
            .mosi_io_num = CONFIG_TOUCH_SPI_MOSI_GPIO,
            .miso_io_num = CONFIG_TOUCH_SPI_MISO_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
            .flags = SPICOMMON_BUSFLAG_MASTER,
        };
        touch_init_err = spi_bus_initialize(CONFIG_TOUCH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (touch_init_err != ESP_OK && touch_init_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(touch_init_err));
            return touch_init_err;
        }
    } else {
        ESP_LOGI(TAG, "SPI bus already initialized by another driver");
    }

    ESP_LOGI(TAG, "Creating panel IO (esp_lcd)");
    esp_lcd_panel_io_spi_config_t tp_io_cfg = {
        .cs_gpio_num = CONFIG_TOUCH_CS_GPIO,
        .dc_gpio_num = -1,
        .spi_mode = 0,
        .pclk_hz = CONFIG_TOUCH_SPI_HZ,
        .trans_queue_depth = 3,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {.lsb_first = 0, .cs_high_active = 0},
    };
    esp_lcd_panel_io_handle_t tp_io = NULL;
    touch_init_err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CONFIG_TOUCH_SPI_HOST, &tp_io_cfg, &tp_io);
    if (touch_init_err != ESP_OK) {
        if (!shared_bus) {
            spi_bus_free(CONFIG_TOUCH_SPI_HOST);
        }
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(touch_init_err));
        return touch_init_err;
    }

    ESP_LOGI(TAG, "Configuring XPT2046 driver");
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = TOUCH_X_MAX,
        .y_max = TOUCH_Y_MAX,
        .rst_gpio_num = CONFIG_TOUCH_RST_GPIO,
        .int_gpio_num = CONFIG_TOUCH_IRQ_GPIO,
        .levels = {
            .reset = 0,    // LOW reset (if RST used)
            .interrupt = 0 // IRQ active LOW on XPT2046
        },
        .flags = {
#ifdef CONFIG_TOUCH_SWAP_XY
            .swap_xy = 1,
#else
            .swap_xy = 0,
#endif

#ifdef CONFIG_TOUCH_MIRROR_X
            .mirror_x = 1,
#else
            .mirror_x = 0,
#endif

#ifdef CONFIG_TOUCH_MIRROR_Y
            .mirror_y = 1,
#else
            .mirror_y = 0,
#endif
        },
    };
    touch_init_err = esp_lcd_touch_new_spi_xpt2046(tp_io, &tp_cfg, &touch_handle);
    if (touch_init_err != ESP_OK) {
        if (!shared_bus) {
            spi_bus_free(CONFIG_TOUCH_SPI_HOST);
        }
        esp_lcd_panel_io_del(tp_io);
        touch_handle = NULL;
        ESP_LOGE(TAG, "Failed to configure driver XPT2046: %s", esp_err_to_name(touch_init_err));
        return touch_init_err;
    }

    ESP_LOGI(TAG, "XPT2046 initialized");
    return ESP_OK;
}

esp_err_t touch_register_to_lvgl(void)
{
    if (!bsp_display_lock(0)) {
        ESP_LOGE(TAG, "Failed to lock display for LVGL touch registration");
        return ESP_FAIL;
    }

    touch_indev = register_touch_with_lvgl();
    if (touch_indev == NULL) {
        bsp_display_unlock();
        ESP_LOGE(TAG, "Failed to register XPT2046 touch input");
        return ESP_FAIL;
    }
    bsp_display_unlock();
    ESP_LOGI(TAG, "XPT2046 touch registered to LVGL");

    return ESP_OK;
}

lv_indev_t *touch_get_indev(void)
{
    return touch_indev;
}

esp_lcd_touch_handle_t touch_get_handle(void)
{
    return touch_handle;
}

void touch_log_press(uint16_t x, uint16_t y)
{
    ESP_LOGD(TAG, "Touch press: x=%u y=%u", (unsigned)x, (unsigned)y);
}

static lv_indev_t *register_touch_with_lvgl(void)
{
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);
    return indev;
}

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (!touch_handle) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    uint16_t x = 0;
    uint16_t y = 0;
    static bool prev_pressed = false;
    bool pressed = false;

    esp_lcd_touch_point_data_t points[1] = {0};
    uint8_t point_cnt = 0;

    if (esp_lcd_touch_read_data(touch_handle) == ESP_OK &&
        esp_lcd_touch_get_data(touch_handle, points, &point_cnt, 1) == ESP_OK &&
        point_cnt > 0) {

        x = points[0].x;
        y = points[0].y;

        handle_touch_press(x, y, &pressed, &prev_pressed, data);
    }

    finalize_touch_state(pressed, x, y, &prev_pressed, data);
}

static void handle_touch_press(uint16_t x, uint16_t y, bool *pressed, bool *prev_pressed, lv_indev_data_t *data)
{
    const bool display_off = settings_get_active_brightness() <= 0;
    const bool waking_up = settings_is_wake_in_progress();

    if (display_off || waking_up) {
        /* Wake screen but ignore this press for LVGL until fade-up completes */
        if (!*prev_pressed && display_off) {
            settings_fade_to_saved_brightness();
            settings_start_screensaver_timers();
        }
        data->state = LV_INDEV_STATE_RELEASED;
        *prev_pressed = false;
        return;
    }

    *pressed = true;
    calibration_apply_cal_data(x, y, &data->point, TOUCH_X_MAX, TOUCH_Y_MAX);
}

static void finalize_touch_state(bool pressed, uint16_t x, uint16_t y, bool *prev_pressed, lv_indev_data_t *data)
{
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    if (pressed && !*prev_pressed) {
        touch_log_press(x, y);
        if (!settings_is_brightness_changing()) {
            settings_fade_to_saved_brightness();
            settings_start_screensaver_timers();
        }
    }
    *prev_pressed = pressed;
}
