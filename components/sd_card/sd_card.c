#include "sd_card.h"

#include <stdbool.h>
#include <stdio.h>

#include "bsp/esp-bsp.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "lvgl.h"
#include "sdmmc_cmd.h"
#include "settings.h"
#include "styles.h"

#define SDSPI_RETRY_UI_STEP_MS  50U
#define SDSPI_RETRY_DELAY_MS    500U
#define SDSPI_MAX_RETRIES       10U

#define SD_RETRY_STACK   (6 * 1024)
#define SD_RETRY_PRIO    (4)

typedef struct {
    SemaphoreHandle_t semaphore;
} sdspi_retry_prompt_ctx_t;

typedef struct {
    lv_obj_t *container;
    lv_obj_t *message_label;
    lv_obj_t *attempt_label;
    lv_obj_t *arc;
    uint32_t total_duration_ms;
} sdspi_retry_ui_t;

static const char *TAG = "sd_card";
static sdmmc_card_t *sd_card_handle = NULL;
static bool sd_spi_bus_ready = false;
static TaskHandle_t s_sd_retry_task = NULL;

SemaphoreHandle_t reconnection_success = NULL;

/**
 * @brief Blocking worker task used to retry SDSPI init without stalling LVGL callbacks.
 *
 * Running the retry flow in its own FreeRTOS task keeps the UI responsive while the
 * modal dialog waits for user confirmation and the SDSPI driver reinitializes.
 */
static void sd_retry_task(void *param);

/**
 * @brief LVGL callback fired when the retry dialog button is tapped.
 *
 * Gives the semaphore that wakes @ref sdspi_retry_wait_for_confirmation().
 *
 * @param e LVGL event descriptor (expects @c LV_EVENT_CLICKED).
 */
static void sdspi_retry_prompt_event_cb(lv_event_t *e);

/**
 * @brief Show a modal prompt asking the user to check SD wiring.
 *
 * Blocks until the user taps the OK button. The dialog is created on the top
 * LVGL layer so it stays visible above any other UI.
 */
static void sdspi_retry_wait_for_confirmation(void);

/**
 * @brief Update the retry overlay message label.
 *
 * @param ui   Overlay descriptor.
 * @param text New text (must be null-terminated).
 */
static void sdspi_retry_ui_set_message(sdspi_retry_ui_t *ui, const char *text);

/**
 * @brief Update the attempt label so the user sees current progress.
 *
 * @param ui      Overlay descriptor.
 * @param attempt Attempt number starting at 1.
 */
static void sdspi_retry_ui_set_attempt(sdspi_retry_ui_t *ui, uint32_t attempt);

/**
 * @brief Drive the arc progress to match elapsed retry time.
 *
 * @param ui         Overlay descriptor.
 * @param elapsed_ms Time already spent waiting.
 */
static void sdspi_retry_ui_set_progress(sdspi_retry_ui_t *ui, uint32_t elapsed_ms);

/**
 * @brief Block for @p wait_ms while keeping the overlay animation fluid.
 *
 * @param ui         Overlay descriptor (optional).
 * @param elapsed_ms Running millisecond counter shared across retries.
 * @param wait_ms    Milliseconds to wait.
 */
static void sdspi_retry_ui_wait(sdspi_retry_ui_t *ui, uint32_t *elapsed_ms, uint32_t wait_ms);

/**
 * @brief Tear down the retry overlay UI.
 *
 * @param ui Overlay descriptor to clean.
 */
static void sdspi_retry_ui_destroy(sdspi_retry_ui_t *ui);

/**
 * @brief Build the LVGL overlay used while retrying SDSPI init.
 *
 * @param ui                 Descriptor to populate.
 * @param total_duration_ms  Total wait duration (used to scale the arc).
 */
static void sdspi_retry_ui_create(sdspi_retry_ui_t *ui, uint32_t total_duration_ms);

esp_err_t init_sdspi(void)
{
    const char *TAG_INIT_SDSPI = "init_sdspi";

    if (sd_card_handle){
        esp_vfs_fat_sdcard_unmount(CONFIG_SDSPI_MOUNT_POINT, sd_card_handle);
        sd_card_handle = NULL;
    }

    if (sd_spi_bus_ready){
        spi_bus_free(CONFIG_SDSPI_BUS_HOST);
        sd_spi_bus_ready = false;
    }

    if (!sd_spi_bus_ready) {
        ESP_LOGI(TAG_INIT_SDSPI, "Initializing SPI bus");
        spi_bus_config_t spi_bus_config = {
            .mosi_io_num = CONFIG_SDSPI_BUS_MOSI_PIN,
            .miso_io_num = CONFIG_SDSPI_BUS_MISO_PIN,
            .sclk_io_num = CONFIG_SPSPI_BUS_SCL_PIN,
            .max_transfer_sz = 4096,
        };
        esp_err_t err = spi_bus_initialize(CONFIG_SDSPI_BUS_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_INIT_SDSPI, "Failed to init SDSPI bus: (%s)", esp_err_to_name(err));
            return err;
        }
        sd_spi_bus_ready = true;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = CONFIG_SDSPI_MAX_FREQ_KHZ;
    host.slot = CONFIG_SDSPI_BUS_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_SDSPI_DEVICE_CS_PIN;
    slot_config.host_id = CONFIG_SDSPI_BUS_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .allocation_unit_size = 16 * 1024,
        .format_if_mount_failed = false,
        .max_files = 5,
    };

    ESP_LOGI(TAG_INIT_SDSPI, "Mounting SDSPI filesystem at %s", CONFIG_SDSPI_MOUNT_POINT);
    esp_err_t err = esp_vfs_fat_sdspi_mount(CONFIG_SDSPI_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INIT_SDSPI, "Failed to init SD card: (%s). Check wiring/pull-ups.", esp_err_to_name(err));
        return err;
    }

    sdmmc_card_print_info(stdout, sd_card_handle);
    ESP_LOGI(TAG_INIT_SDSPI, "SDSPI ready");

    if (!reconnection_success){
        reconnection_success = xSemaphoreCreateBinary();
        xSemaphoreTake(reconnection_success, 0);
    }

    return ESP_OK;
}

void retry_init_sdspi(void)
{
    sdspi_retry_wait_for_confirmation();

    esp_err_t err = ESP_OK;

    const uint32_t total_wait_ms = SDSPI_MAX_RETRIES * SDSPI_RETRY_DELAY_MS;
    sdspi_retry_ui_t retry_ui = {0};
    sdspi_retry_ui_create(&retry_ui, total_wait_ms);

    uint32_t elapsed_ms = 0;

    for (int attempt = 1; attempt <= SDSPI_MAX_RETRIES; attempt++) {
        ESP_LOGW(TAG, "Retrying SD card init %d/%d...", attempt, SDSPI_MAX_RETRIES);
        sdspi_retry_ui_set_attempt(&retry_ui, attempt);
        sdspi_retry_ui_wait(&retry_ui, &elapsed_ms, SDSPI_RETRY_DELAY_MS);

        err = init_sdspi();
        if (err == ESP_OK) {
            sdspi_retry_ui_set_message(&retry_ui, "SD card recovered");
            sdspi_retry_ui_set_progress(&retry_ui, total_wait_ms);
            ESP_LOGW(TAG, "SD card recovered after %d attempt(s)", attempt);
            vTaskDelay(pdMS_TO_TICKS(1500));
            sdspi_retry_ui_destroy(&retry_ui);
            xSemaphoreGive(reconnection_success);
            return;
        }
    }

    sdspi_retry_ui_set_message(&retry_ui, "SD card retry failed, restarting...");
    sdspi_retry_ui_set_progress(&retry_ui, total_wait_ms);
    vTaskDelay(pdMS_TO_TICKS(1500));
    sdspi_retry_ui_destroy(&retry_ui);

    ESP_LOGE(TAG, "SD card init failed after %d retries. Last ESP err: %s",
             SDSPI_MAX_RETRIES,
             esp_err_to_name(err));

    if (settings_is_time_valid()){
        settings_shutdown_save_time();
    }
    esp_restart();
}

void sdspi_schedule_sd_retry(void)
{
    if (s_sd_retry_task) {
        return;
    }

    BaseType_t res = xTaskCreatePinnedToCore(sd_retry_task,
                                             "sd_retry",
                                             SD_RETRY_STACK,
                                             NULL,
                                             SD_RETRY_PRIO,
                                             &s_sd_retry_task,
                                             tskNO_AFFINITY);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SD retry task");
        s_sd_retry_task = NULL;
    }
}

static void sd_retry_task(void *param)
{
    retry_init_sdspi();
    s_sd_retry_task = NULL;
    vTaskDelete(NULL);
}

static void sdspi_retry_prompt_event_cb(lv_event_t *e)
{
    sdspi_retry_prompt_ctx_t *ctx = (sdspi_retry_prompt_ctx_t *)lv_event_get_user_data(e);
    if (!ctx || !ctx->semaphore) {
        return;
    }

    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    xSemaphoreGive(ctx->semaphore);
}

static void sdspi_retry_wait_for_confirmation(void)
{
    sdspi_retry_prompt_ctx_t ctx = {
        .semaphore = xSemaphoreCreateBinary(),
    };
    if (!ctx.semaphore) {
        ESP_LOGW(TAG, "Failed to allocate semaphore for SDSPI retry prompt");
        return;
    }

    if (!bsp_display_lock(0)) {
        vSemaphoreDelete(ctx.semaphore);
        ESP_LOGW(TAG, "Unable to acquire display lock for SDSPI retry prompt");
        return;
    }

    lv_obj_t *layer = lv_layer_top();
    lv_obj_t *mbox = lv_msgbox_create(layer);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, lv_pct(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Check SD card connection and hit OK");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(label, UI_COLOR_TEXT_DARK, 0);
    lv_obj_set_width(label, lv_pct(100));

    lv_obj_t *btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(btn);
    lv_obj_add_event_cb(btn, sdspi_retry_prompt_event_cb, LV_EVENT_CLICKED, &ctx);

    lv_obj_invalidate(mbox);
    lv_refr_now(NULL);
    bsp_display_unlock();

    if (xSemaphoreTake(ctx.semaphore, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "SDSPI retry prompt wait aborted");
    }

    if (bsp_display_lock(0)) {
        lv_obj_del(mbox);
        bsp_display_unlock();
    }

    vSemaphoreDelete(ctx.semaphore);
}

static void sdspi_retry_ui_set_message(sdspi_retry_ui_t *ui, const char *text)
{
    if (!ui || !ui->container || !ui->message_label || !text) {
        return;
    }

    if (!bsp_display_lock(0)) {
        return;
    }
    lv_label_set_text(ui->message_label, text);
    bsp_display_unlock();
}

static void sdspi_retry_ui_set_attempt(sdspi_retry_ui_t *ui, uint32_t attempt)
{
    if (!ui || !ui->container || !ui->attempt_label) {
        return;
    }

    if (!bsp_display_lock(0)) {
        return;
    }
    lv_label_set_text_fmt(ui->attempt_label, "Attempt %lu/%d", attempt, SDSPI_MAX_RETRIES);
    bsp_display_unlock();
}

static void sdspi_retry_ui_set_progress(sdspi_retry_ui_t *ui, uint32_t elapsed_ms)
{
    if (!ui || !ui->container || !ui->arc || ui->total_duration_ms == 0) {
        return;
    }

    if (elapsed_ms > ui->total_duration_ms) {
        elapsed_ms = ui->total_duration_ms;
    }

    if (!bsp_display_lock(0)) {
        return;
    }
    lv_arc_set_value(ui->arc, elapsed_ms);
    bsp_display_unlock();
}

static void sdspi_retry_ui_wait(sdspi_retry_ui_t *ui, uint32_t *elapsed_ms, uint32_t wait_ms)
{
    if (!elapsed_ms) {
        return;
    }

    uint32_t target = *elapsed_ms + wait_ms;

    if (!ui || !ui->container || !ui->arc) {
        TickType_t ticks = pdMS_TO_TICKS(wait_ms);
        if (ticks == 0) {
            ticks = 1;
        }
        vTaskDelay(ticks);
        *elapsed_ms = target;
        return;
    }

    while (*elapsed_ms < target) {
        uint32_t chunk_ms = target - *elapsed_ms;
        if (chunk_ms > SDSPI_RETRY_UI_STEP_MS) {
            chunk_ms = SDSPI_RETRY_UI_STEP_MS;
        }
        TickType_t ticks = pdMS_TO_TICKS(chunk_ms);
        if (ticks == 0) {
            ticks = 1;
        }
        vTaskDelay(ticks);
        *elapsed_ms += chunk_ms;
        sdspi_retry_ui_set_progress(ui, *elapsed_ms);
    }
}

static void sdspi_retry_ui_destroy(sdspi_retry_ui_t *ui)
{
    if (!ui || !ui->container) {
        return;
    }

    if (!bsp_display_lock(0)) {
        return;
    }
    lv_obj_del(ui->container);
    bsp_display_unlock();

    ui->container = NULL;
    ui->message_label = NULL;
    ui->attempt_label = NULL;
    ui->arc = NULL;
}

static void sdspi_retry_ui_create(sdspi_retry_ui_t *ui, uint32_t total_duration_ms)
{
    if (!ui) {
        return;
    }

    ui->total_duration_ms = total_duration_ms;

    if (!bsp_display_lock(0)) {
        ESP_LOGW(TAG, "Unable to acquire display lock for SDSPI retry UI");
        return;
    }

    lv_obj_t *parent = lv_layer_top();
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_style_pad_all(container, 16, 0);
    lv_obj_set_style_pad_row(container, 12, 0);
    lv_obj_set_style_radius(container, 12, 0);
    lv_obj_set_style_bg_color(container, UI_COLOR_CARD_DARK, 0);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(container, 2, 0);
    lv_obj_set_style_border_color(container, UI_COLOR_BORDER_DARK, 0);
    lv_obj_set_width(container, lv_pct(80));
    lv_obj_set_height(container, LV_SIZE_CONTENT);
    lv_obj_center(container);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(container, 12, 0);
    lv_obj_set_flex_align(container,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *message = lv_label_create(container);
    lv_label_set_long_mode(message, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(message, UI_COLOR_TEXT_DARK, 0);
    lv_obj_set_style_text_align(message, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(message, lv_pct(100));
    lv_label_set_text(message, "SD card failed, retrying...");

    lv_obj_t *arc = lv_arc_create(container);
    lv_obj_set_size(arc, 100, 100);
    lv_arc_set_range(arc, 0, total_duration_ms);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_value(arc, 0);
    lv_obj_set_style_bg_color(arc, UI_COLOR_INDICATOR_OFF_DARK, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_center(arc);

    lv_obj_t *attempt = lv_label_create(container);
    lv_obj_set_width(attempt, lv_pct(100));
    lv_obj_set_style_text_align(attempt, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(attempt, UI_COLOR_TEXT_DARK, 0);
    lv_label_set_text_fmt(attempt, "Attempt 0/%d", SDSPI_MAX_RETRIES);

    ui->container = container;
    ui->message_label = message;
    ui->attempt_label = attempt;
    ui->arc = arc;

    bsp_display_unlock();
}
