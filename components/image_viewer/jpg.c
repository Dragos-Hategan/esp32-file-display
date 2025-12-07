#include "jpg.h"

#include <stdbool.h>
#include <string.h>

#include "lvgl/src/libs/tjpgd/tjpgd.h"
#include "lvgl/src/misc/lv_fs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "bsp/esp-bsp.h"
#include "esp_err.h"
#include "esp_log.h"
#include "styles.h"

#define TAG "jpg_viewer"
#define IMG_VIEWER_MAX_PATH 256

typedef struct {
    lv_fs_file_t file;
    esp_lcd_panel_handle_t panel;
    uint16_t *stripe;               /* DMA-capable stripe buffer */
    uint32_t stripe_w;
    uint32_t stripe_h;
    uint16_t disp_w;
    uint16_t disp_h;
    uint8_t scale;
} jpg_stripe_ctx_t;

typedef struct {
    bool active;
    lv_obj_t *screen;
    lv_obj_t *image;
    lv_obj_t *close_btn;
    lv_obj_t *path_label;
    lv_obj_t *return_screen;
    lv_obj_t *previous_screen;
    char path[IMG_VIEWER_MAX_PATH];
} jpg_viewer_ctx_t;

static jpg_viewer_ctx_t s_jpg_viewer;

/**
 * @brief Destroy the currently active JPG viewer screen and reset its context.
 *
 * This helper deletes the LVGL screen associated with the viewer (if any)
 * under a display lock, then calls jpg_viewer_reset() to clear the context.
 * If the context is NULL or not active, it simply resets the context.
 *
 * @param ctx Pointer to the viewer context to destroy.
 */
static void jpg_viewer_destroy_active(jpg_viewer_ctx_t *ctx);

/**
 * @brief Build the LVGL UI for the JPG viewer.
 *
 * This creates a new LVGL screen with a black transparent background,
 * an image object centered on the screen and a close button aligned
 * in the top-right corner. The close button is wired to jpg_viewer_on_close().
 *
 * @param ctx  Pointer to the viewer context to populate.
 */
static void jpg_viewer_build_ui(jpg_viewer_ctx_t *ctx);

/**
 * @brief Render the JPEG at the given path to the display panel.
 *
 * This function validates the LVGL image object and path, retrieves the
 * display panel from the BSP and calls jpg_draw_striped() to decode and
 * draw the JPEG in stripes.
 *
 * @param img  LVGL image object associated with the viewer (not used for
 *             rendering in this implementation, but kept for API symmetry).
 * @param path Path to the JPEG file to be rendered.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if img or path is invalid
 *      - ESP_ERR_INVALID_STATE if no valid panel is available
 *      - ESP_ERR_NOT_SUPPORTED if the jpg file is corrupted or it's specific type is not supported
 *      - ESP_ERR_INVALID_SIZE if the image can't fit in the screen even after the downscale
 */
static esp_err_t jpg_handler_set_src(lv_obj_t *img, const char *path);

/**
 * @brief Reset the JPG viewer context to a clean state.
 *
 * This function stops and deletes the dim timer (if any) and clears the
 * entire context structure to zero. It is safe to call with a NULL ctx
 * pointer (no action is taken in that case).
 *
 * @param ctx Pointer to the viewer context to reset.
 */
static void jpg_viewer_reset(jpg_viewer_ctx_t *ctx);

/**
 * @brief LVGL event callback used to close the JPG viewer.
 *
 * This callback is attached to the close button. It retrieves the viewer
 * context from the event user data, switches back to the return screen
 * (or the previous screen if no explicit return screen is set), deletes
 * the viewer screen and resets the context.
 *
 * @param e Pointer to the LVGL event descriptor.
 */
static void jpg_viewer_on_close(lv_event_t *e);

/**
 * @brief TJpgDec input callback for reading from LVGL filesystem.
 *
 * If @p buff is non-NULL, this function reads up to @p nbytes from the
 * current file position into the buffer. If @p buff is NULL, it advances
 * the file position by @p nbytes (seek).
 *
 * @param jd     Pointer to the TJpgDec decoder object.
 * @param buff   Destination buffer to read into, or NULL to skip data.
 * @param nbytes Number of bytes to read or skip.
 *
 * @return Number of bytes actually read or skipped, or 0 on error.
 */
static size_t input_cb(JDEC *jd, uint8_t *buff, size_t nbytes);

/**
 * @brief TJpgDec output callback to convert and push decoded pixels to the panel.
 *
 * The decoder provides a rectangular block of pixels in RGB888 format. This
 * callback converts the block to RGB565, applying the required BGR swap, and
 * stores it into a stripe buffer. The buffer is then drawn to the display
 * using esp_lcd_panel_draw_bitmap().
 *
 * @param jd      Pointer to the TJpgDec decoder object.
 * @param bitmap  Pointer to the decoded pixel data (RGB888).
 * @param rect    Pointer to the rectangle describing the region in the image.
 *
 * @return
 *      - 1 to continue decoding
 *      - 0 to abort decoding due to error or invalid parameters
 */
static int output_cb(JDEC *jd, void *bitmap, JRECT *rect);

/**
 * @brief Decode and draw a JPEG image in stripes directly to an LCD panel.
 *
 * This function opens the JPEG file using LVGL's filesystem API, prepares
 * a TJpgDec decoder instance and allocates a stripe buffer sized according
 * to the MCU height and image width. The image is then decompressed via
 * jd_decomp(), which repeatedly calls input_cb() and output_cb() to stream
 * the image to the panel without loading it fully into memory.
 *
 * @param path  Path to the JPEG file in the LVGL filesystem.
 * @param panel Handle to the LCD panel used for drawing.
 *
 * @return
 *      - ESP_OK on successful decode and draw
 *      - ESP_FAIL on file open, decoder prepare or decode failure
 *      - ESP_ERR_NO_MEM if the stripe buffer allocation fails
 *      - ESP_ERR_NOT_SUPPORTED if the jpg file is corrupted or it's specific type is not supported
 *      - ESP_ERR_INVALID_SIZE if the image can't fit in the screen even after the downscale
 */
static esp_err_t jpg_draw_striped(const char *path, esp_lcd_panel_handle_t panel);

esp_err_t jpg_viewer_open(const jpg_viewer_open_opts_t *opts)
{
    if (!opts || !opts->path || opts->path[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    jpg_viewer_ctx_t *ctx = &s_jpg_viewer;

    if (ctx->active) {
        jpg_viewer_destroy_active(ctx);
    }

    ctx->return_screen = opts->return_screen;
    strlcpy(ctx->path, opts->path, sizeof(ctx->path));

    if (!bsp_display_lock(0)) {
        return ESP_ERR_TIMEOUT;
    }

    ctx->previous_screen = lv_screen_active();
    jpg_viewer_build_ui(ctx);

    /* Load the screen before drawing so LVGL flushes its background/UI first */
    lv_screen_load(ctx->screen);
    /* Force a refresh now so subsequent LVGL cycles don't clear our direct draw */
    lv_refr_now(NULL);

    esp_err_t err = jpg_handler_set_src(ctx->image, opts->path);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to render image: (%s)", esp_err_to_name(err));
        if (ctx->previous_screen) {
            lv_screen_load(ctx->previous_screen);
        }
        lv_obj_del(ctx->screen);
        ctx->screen = NULL;
        bsp_display_unlock();
        jpg_viewer_reset(ctx);
        return err;
    }

    lv_obj_set_style_opa(ctx->close_btn, LV_OPA_100, LV_PART_MAIN);

    bsp_display_unlock();

    ctx->active = true;
    return ESP_OK;
}

static void jpg_viewer_destroy_active(jpg_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->active) {
        jpg_viewer_reset(ctx);
        return;
    }

    if (bsp_display_lock(0)) {
        if (ctx->screen) {
            lv_obj_del(ctx->screen);
        }
        bsp_display_unlock();
    }

    jpg_viewer_reset(ctx);
}

static void jpg_viewer_build_ui(jpg_viewer_ctx_t *ctx)
{
    ctx->screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(ctx->screen, UI_COLOR_BG_DARK, 0);
    lv_obj_set_style_bg_opa(ctx->screen, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(ctx->screen, UI_COLOR_TEXT_DARK, 0);
    lv_obj_set_style_pad_all(ctx->screen, 0, 0);

    ctx->image = lv_image_create(ctx->screen);
    lv_obj_center(ctx->image);

    lv_obj_t *close_btn = lv_button_create(ctx->screen);
    ctx->close_btn = close_btn;
    //lv_obj_remove_style_all(close_btn);
    lv_obj_set_size(close_btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(close_btn, 3, 0);
    lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    styles_set_button(close_btn);
    lv_obj_set_style_radius(close_btn, 20, 0); // enough radius to make the button a circle
    lv_obj_add_event_cb(close_btn, jpg_viewer_on_close, LV_EVENT_CLICKED, ctx);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(close_lbl, UI_COLOR_TEXT_DARK, 0);
    lv_obj_center(close_lbl);
}

static esp_err_t jpg_handler_set_src(lv_obj_t *img, const char *path)
{
    if (!img || !path || path[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_lcd_panel_handle_t panel = bsp_display_get_panel();
    if (!panel) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err = jpg_draw_striped(path, panel);

    return err;
}

static void jpg_viewer_reset(jpg_viewer_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
}

static void jpg_viewer_on_close(lv_event_t *e)
{
    jpg_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->active) {
        return;
    }

    if (!bsp_display_lock(0)) {
        return;
    }

    lv_obj_t *old_screen = ctx->screen;
    lv_obj_t *target = ctx->return_screen ? ctx->return_screen : ctx->previous_screen;
    if (target) {
        lv_screen_load(target);
    }

    if (old_screen) {
        lv_obj_del(old_screen);
    }

    bsp_display_unlock();
    jpg_viewer_reset(ctx);
}

static size_t input_cb(JDEC *jd, uint8_t *buff, size_t nbytes)
{
    jpg_stripe_ctx_t *ctx = (jpg_stripe_ctx_t *)jd->device;
    if (!ctx) {
        return 0;
    }
    lv_fs_file_t *f = &ctx->file;

    if (buff) {
        uint32_t rn = 0;
        lv_fs_res_t res = lv_fs_read(f, buff, (uint32_t)nbytes, &rn);
        return (res == LV_FS_RES_OK) ? rn : 0;
    }

    uint32_t pos = 0;
    if (lv_fs_tell(f, &pos) != LV_FS_RES_OK) {
        return 0;
    }
    if (lv_fs_seek(f, (uint32_t)(pos + nbytes), LV_FS_SEEK_SET) != LV_FS_RES_OK) {
        return 0;
    }
    return nbytes;
}

static int output_cb(JDEC *jd, void *bitmap, JRECT *rect)
{
    jpg_stripe_ctx_t *ctx = (jpg_stripe_ctx_t *)jd->device; /* user context */
    if (!ctx || !bitmap || !rect) {
        return 0;
    }

    const int w = rect->right - rect->left + 1;
    const int h = rect->bottom - rect->top + 1;

    /* Ensure stripe buffer is large enough */
    if ((uint32_t)w > ctx->stripe_w || (uint32_t)h > ctx->stripe_h) {
        return 0;
    }

    uint8_t *src = (uint8_t *)bitmap; /* RGB888 from tjpgd (JD_FORMAT=0) */
    uint16_t *dst = ctx->stripe;
    const int src_stride = (int)(jd->msx * 8); /* tjpgd always outputs full MCU width */
    const int scale = ctx->scale;
    for (int y = 0; y < h; y++) {
        const int sy = y << scale; /* top-left sampling for downscale */
        uint8_t *src_row = src + (sy * src_stride * 3);
        uint16_t *dst_row = dst + y * w;
        for (int x = 0; x < w; x++) {
            const int sx = x << scale;
            int idx = sx * 3;
            /* Panel is configured BGR; swap R and B, then pack RGB565 and swap bytes */
            uint8_t b = src_row[idx];
            uint8_t g = src_row[idx + 1];
            uint8_t r = src_row[idx + 2];
            uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            c = (uint16_t)((c >> 8) | (c << 8));
            dst_row[x] = c;
        }
    }

    /* Clip to panel bounds to avoid wrap-around when images exceed display size */
    if (rect->left >= ctx->disp_w || rect->top >= ctx->disp_h) {
        return 1;
    }

    const int draw_left = rect->left;
    const int draw_top = rect->top;
    const int draw_right = (rect->right < (ctx->disp_w - 1)) ? rect->right : (int)(ctx->disp_w - 1);
    const int draw_bottom = (rect->bottom < (ctx->disp_h - 1)) ? rect->bottom : (int)(ctx->disp_h - 1);
    const int clipped_w = draw_right - draw_left + 1;
    const int clipped_h = draw_bottom - draw_top + 1;

    if (clipped_w <= 0 || clipped_h <= 0) {
        return 1;
    }

    if (clipped_w == w && clipped_h == h) {
        esp_lcd_panel_draw_bitmap(ctx->panel,
                                  draw_left, draw_top,
                                  draw_right + 1, draw_bottom + 1,
                                  dst);
    } else {
        for (int row = 0; row < clipped_h; row++) {
            esp_lcd_panel_draw_bitmap(ctx->panel,
                                      draw_left, draw_top + row,
                                      draw_left + clipped_w, draw_top + row + 1,
                                      dst + (row * w));
        }
    }
    return 1; /* continue */
}

static esp_err_t jpg_draw_striped(const char *path, esp_lcd_panel_handle_t panel)
{
    esp_err_t err = ESP_OK;
    jpg_stripe_ctx_t ctx = {
        .panel = panel,
        .stripe = NULL,
        .stripe_w = 0,
        .stripe_h = 0,
        .disp_w = BSP_LCD_H_RES,
        .disp_h = BSP_LCD_V_RES,
        .scale = 0,
    };

    lv_fs_res_t res = lv_fs_open(&ctx.file, path, LV_FS_MODE_RD);
    if (res != LV_FS_RES_OK) {
        ESP_LOGE(TAG, "Failed to open image file, lv_fs_res: (%d)", res);
        return ESP_FAIL;
    }

    uint8_t workb[4096];      /* tjpgd work buffer */

    JDEC jd;
    JRESULT rc = jd_prepare(&jd, input_cb, workb, sizeof(workb), &ctx);
    if (rc != JDR_OK) {
        ESP_LOGE(TAG, "Failed to initialize tjpgd decoder, JRESULT: (%d)", rc);
        if (rc == JDR_INP || rc == JDR_FMT1 || rc == JDR_FMT2 || rc == JDR_FMT3){
            err = ESP_ERR_NOT_SUPPORTED;
        }else{
            err = ESP_FAIL;
        }
        goto cleanup;
    }

    /* Choose the smallest power-of-two downscale that fits the panel */
    uint32_t scaled_w = jd.width;
    uint32_t scaled_h = jd.height;
    while (ctx.scale < 3 && (scaled_w > ctx.disp_w || scaled_h > ctx.disp_h)) {
        ctx.scale++;
        scaled_w = (scaled_w + 1) >> 1;
        scaled_h = (scaled_h + 1) >> 1;
    }

    if (scaled_w > ctx.disp_w || scaled_h > ctx.disp_h) {
        ESP_LOGE(TAG, "Image %ux%u is too large to fit display %ux%u even at 1/%u scale",
                 jd.width, jd.height, ctx.disp_w, ctx.disp_h, 1U << ctx.scale);
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }

    ESP_LOGD(TAG, "Drawing JPEG %ux%u scaled 1/%u -> %lux%lu",
             jd.width, jd.height, 1U << ctx.scale,
             (unsigned long)scaled_w, (unsigned long)scaled_h);

    /* MCU height = msy * 8 lines; width capped to scaled image width */
    ctx.stripe_w = scaled_w;
    ctx.stripe_h = (uint32_t)((jd.msy * 8u) >> ctx.scale);
    if (ctx.stripe_h == 0) {
        ctx.stripe_h = 1;
    }
    size_t stripe_size = ctx.stripe_w * ctx.stripe_h * sizeof(uint16_t);
    ESP_LOGW(TAG, "Stripe size is %lu", stripe_size);
    ctx.stripe = heap_caps_malloc(stripe_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!ctx.stripe) {
        ESP_LOGE(TAG, "Failed to allocate memory for the stripe buffer used for image draw");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    rc = jd_decomp(&jd, output_cb, ctx.scale); /* scale: 0=full, 1=1/2, 2=1/4, 3=1/8 */
    if (rc != JDR_OK) {
        ESP_LOGE(TAG, "Failed to draw image, JRESULT: (%d)", rc);
        err = ESP_FAIL;
    }

cleanup:
    lv_fs_close(&ctx.file);
    if (ctx.stripe) {
        free(ctx.stripe);
    }
    return err;
}
