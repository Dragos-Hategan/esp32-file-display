#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"
#include "lvgl.h"

/**
 * @brief Callback invoked when the viewer screen closes.
 *
 * @param content_changed true if the file was saved during the session.
 * @param user_ctx        User-supplied pointer passed through the open options.
 */
typedef void (*text_viewer_close_cb_t)(bool content_changed, void *user_ctx);

/**
 * @brief Options describing how to open the text viewer.
 */
typedef struct {
    const char *path;                 /**< Absolute path to existing file (NULL/empty for new files). */
    const char *directory;            /**< Directory used when creating a new file. */
    const char *suggested_name;       /**< Suggested filename when creating a new file. */
    lv_obj_t *return_screen;          /**< Screen to restore when the viewer closes. */
    bool editable;                    /**< true to enable editing (cursor, keyboard, save). */
    text_viewer_close_cb_t on_close;  /**< Optional callback invoked on close. */
    void *user_ctx;                   /**< Optional context passed to @p on_close. */
} text_viewer_open_opts_t;

/**
 * @brief Load the viewer screen and display the requested file.
 *
 * Reads the file at @p opts->path, builds the screen (on first use),
 * sets edit/view mode, populates the text area and status, and loads
 * the screen. Original content is snapshotted to support dirty tracking.
 *
 * @param[in] opts Options:
 *   - @c path: full path to the text file to view/edit (omit or empty for new files)
 *   - @c directory: parent directory used when @p path is NULL/empty
 *   - @c suggested_name: default filename populated when creating a new file
 *   - @c return_screen (required): screen to return to on close
 *   - @c editable: true to enable edit mode
 *   - @c on_close: optional callback invoked on close
 *   - @c user_ctx: user context for the close callback
 *
 * @return
 *   - ESP_OK on success
 *   - ESP_ERR_INVALID_ARG if required options are missing
 *   - fs_text_read_range error codes if reading fails
 */
esp_err_t text_viewer_open(const text_viewer_open_opts_t *opts);

#ifdef __cplusplus
}
#endif
