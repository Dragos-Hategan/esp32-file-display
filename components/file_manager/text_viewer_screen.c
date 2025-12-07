#include "text_viewer_screen.h"

#include <sys/stat.h>
#include <strings.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "fs_navigator.h"
#include "fs_text_ops.h"
#include "Domine_16.h"
#include "esp_log.h"
#include "sd_card.h"
#include "styles.h"

#define TEXT_VIEWER_PATH_SCROLL_DELAY_MS 2000

/**
 * @brief Actions in the chunk-change prompt.
 */
typedef enum
{
    TEXT_VIEWER_CHUNK_SAVE = 1,    /**< Save before loading new chunk */
    TEXT_VIEWER_CHUNK_DISCARD = 2, /**< Discard changes and load new chunk */
} text_viewer_chunk_action_t;

/**
 * @brief Actions to resume after SD reconnection.
 */
typedef enum
{
    TEXT_VIEWER_SD_NONE = 0,
    TEXT_VIEWER_SD_SAVE,
    TEXT_VIEWER_SD_CHUNK,
} text_viewer_sd_action_t;

/**
 * @brief Runtime state for the singleton text viewer/editor screen.
 */
typedef struct
{
    bool active;                                /**< True while the viewer screen is active */
    bool dirty;                                 /**< True if current text differs from original */
    bool editable;                              /**< True if edit mode is enabled */
    bool new_file;                              /**< True if creating a new file */
    bool at_top_edge;                           /**< Tracks if the scroll is currently at the top edge */
    bool at_bottom_edge;                        /**< Tracks if the scroll is currently at the bottom edge */
    bool suppress_events;                       /**< Temporarily disable change detection */
    size_t lasf_file_offset_kb;                 /**< Offset (in KB) used for the last read chunk */
    size_t current_file_offset_kb;              /**< Offset (in KB) used for the current/next chunk */
    size_t max_file_offset_kb;                  /**< Maximum readable offset (in KB) for the loaded file */
    lv_obj_t *screen;                           /**< Root LVGL screen object */
    lv_obj_t *toolbar;                          /**< Toolbar container */
    lv_obj_t *path_label;                       /**< Label showing the file path */
    lv_obj_t *status_label;                     /**< Label showing transient status messages */
    lv_obj_t *save_btn;                         /**< Save button (hidden/disabled in view mode) */
    lv_obj_t *text_area;                        /**< Text area for viewing/editing content */
    lv_obj_t *keyboard;                         /**< On-screen keyboard */
    lv_obj_t *chunk_slider;                     /**< Vertical slider for chunk navigation */
    lv_obj_t *return_screen;                    /**< Screen to return to on close */
    lv_obj_t *confirm_mbox;                     /**< Confirmation message box (save/discard) */
    lv_obj_t *chunk_mbox;                       /**< Chunk-change confirmation message box */
    lv_obj_t *name_dialog;                      /**< Filename prompt dialog */
    lv_obj_t *name_textarea;                    /**< Text area used inside filename dialog */
    lv_timer_t *sd_retry_timer;                 /**< Timer to poll SD reconnection */
    lv_timer_t *path_scroll_timer;              /**< Timer to delay the scrolling of paths */
    text_viewer_close_cb_t close_cb;            /**< Optional close callback */
    void *close_ctx;                            /**< User context for close callback */
    char path[FS_TEXT_MAX_PATH];                /**< Current file path */
    char directory[FS_TEXT_MAX_PATH];           /**< Directory used for new files */
    char pending_name[FS_NAV_MAX_NAME];         /**< Suggested filename for new files */
    char *original_text;                        /**< Snapshot of text at load/save time */
    size_t pending_first_offset_kb;             /**< Pending first chunk offset when prompting */
    size_t pending_second_offset_kb;            /**< Pending second chunk offset when prompting */
    bool pending_scroll_up;                     /**< True if pending load comes from top edge */
    bool pending_chunk;                         /**< True if a chunk load is pending confirmation */
    bool waiting_sd;                            /**< True while waiting SD reconnection */
    text_viewer_sd_action_t sd_retry_action;    /**< Pending action after SD reconnect */
    bool content_changed;                       /**< True if file was saved during session */
    bool slider_suppress_change;                /**< Guard slider callbacks while syncing */
    bool slider_drag_active;                    /**< True while slider knob is dragged */
    size_t slider_pending_step;                 /**< Pending slider step during drag */
} text_viewer_ctx_t;

/**
 * @brief Confirmation actions used in the save/discard dialog.
 */
typedef enum
{
    TEXT_VIEWER_CONFIRM_SAVE = 1,    /**< Confirm saving changes */
    TEXT_VIEWER_CONFIRM_DISCARD = 2, /**< Confirm discarding changes */
} text_viewer_confirm_action_t;

static const char *TAG = "text_viewer";
static text_viewer_ctx_t s_viewer;

/************************************** UI Setup & State *************************************/

/**
 * @brief Build all LVGL widgets for the viewer/editor screen.
 *
 * Creates toolbar, labels, text area, and on-screen keyboard.
 * Does not load content or set mode; see @ref text_viewer_open and
 * @ref text_viewer_apply_mode.
 *
 * @param ctx Viewer context (must be non-NULL).
 */
static void text_viewer_build_screen(text_viewer_ctx_t *ctx);

/**
 * @brief Apply current mode (view vs edit) to widgets and controls.
 *
 * Enables/disables text area, toggles keyboard and save button,
 * then updates button states.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_apply_mode(text_viewer_ctx_t *ctx);

/**
 * @brief Set a short status message in the toolbar.
 *
 * @param ctx Viewer context.
 * @param msg Null-terminated message string (ignored if NULL).
 */
static void text_viewer_set_status(text_viewer_ctx_t *ctx, const char *msg);

/**
 * @brief Set the path label using a UI-friendly path (hide mountpoint).
 *
 * Replaces leading CONFIG_SDSPI_MOUNT_POINT with "/" for display purposes.
 *
 * @param ctx Viewer context.
 * @param path Filesystem path (may be NULL/empty).
 */
static void text_viewer_set_path_label(text_viewer_ctx_t *ctx, const char *path);

/**
 * @brief Restarts the delayed scrolling animation for the path label.
 *
 * This function cancels any existing scroll-start timer, forces the path label into
 * clipped mode, and creates a new one-shot timer that will re-enable circular
 * scrolling after TEXT_VIEWER_PATH_SCROLL_DELAY_MS milliseconds.
 *
 * It is typically used whenever the displayed path changes, ensuring the scroll
 * animation restarts cleanly and does not begin immediately.
 *
 * @param ctx Pointer to the text viewer UI context. Must contain a valid path_label.
 */
static void text_viewer_restart_path_scroll(text_viewer_ctx_t *ctx);

/**
 * @brief Timer callback used to enable scrolling for the text viewer path label.
 *
 * This function is invoked after a short delay to switch the path label's long mode
 * from clipped (LV_LABEL_LONG_CLIP) to circular scrolling (LV_LABEL_LONG_SCROLL_CIRCULAR).
 * The delay prevents immediate scrolling and makes the UI feel smoother when paths change.
 *
 * @param timer Pointer to the LVGL timer that triggered the callback.
 *              Its user_data must contain a valid text_viewer_ctx_t*.
 */
static void text_viewer_path_scroll_timer_cb(lv_timer_t *timer);

/**
 * @brief Replace the stored original text snapshot.
 *
 * Frees the previous snapshot and stores a duplicate of @p text.
 *
 * @param ctx  Viewer context.
 * @param text New baseline text (may be NULL, which clears the snapshot).
 */
static void text_viewer_set_original(text_viewer_ctx_t *ctx, const char *text);

/**
 * @brief Resolve slider window size and step (in KB chunks) with defaults.
 *
 * @param[in]  ctx          Viewer context (currently unused).
 * @param[out] window_size  Effective window size (chunks per window, >=1).
 * @param[out] step         Effective step size (chunks per step, >=1).
 */
static void text_viewer_get_slider_params(text_viewer_ctx_t *ctx, size_t *window_size, size_t *step);

/**
 * @brief Sync the chunk slider with the current window and file size.
 *
 * Updates range/value, snaps to the active window (including the last), disables when
 * a single window fits, and stores the pending step for drag handling.
 *
 * @param[in,out] ctx Viewer context containing slider state.
 */
static void text_viewer_update_slider(text_viewer_ctx_t *ctx);

/**
 * @brief Handle slider press/drag/release to jump between chunk windows.
 *
 * Tracks the target step while dragging and applies the chunk load on release; no-ops
 * if blocked or if the knob returns to the current step.
 *
 * @param e LVGL slider event with user data = text_viewer_ctx_t*.
 */
static void text_viewer_on_slider_value_changed(lv_event_t *e);

/**
 * @brief Load two consecutive chunks into the textarea and position the cursor at the boundary.
 *
 * @param ctx             Viewer context.
 * @param first_offset_kb Offset (KB) of the first chunk.
 * @param second_offset_kb Offset (KB) of the second chunk.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t text_viewer_load_window(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb);

/**
 * @brief Enable/disable the Save button based on @c editable and @c dirty.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_update_buttons(text_viewer_ctx_t *ctx);

/*********************************************************************************************/

/******************************* Keyboard & interaction helpers ******************************/

/**
 * @brief Explicitly show the keyboard in edit mode.
 *
 * @param ctx    Viewer context.
 * @param target Text area to focus (falls back to main text area if NULL).
 */
static void text_viewer_show_keyboard(text_viewer_ctx_t *ctx, lv_obj_t *target);

/**
 * @brief Explicitly hide the keyboard and detach it from the textarea.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_hide_keyboard(text_viewer_ctx_t *ctx);

/**
 * @brief Show the keyboard when the text area is tapped in edit mode.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_text_area_clicked(lv_event_t *e);
/**
 * @brief Immediately jump the text area's scroll to the cursor position.
 *
 * Skips LVGL's default smooth scroll animation and forces the text area
 * to scroll instantly to the final cursor location. Useful when loading
 * a new text chunk and repositioning the cursor manually.
 *
 * @param ctx Pointer to the text viewer context. Must not be NULL.
 */
static void text_viewer_skip_cursor_animation(text_viewer_ctx_t *ctx);

/**
 * @brief Handle scroll events and load new text chunks when reaching edges.
 *
 * Triggered whenever the LVGL text area scrolls.  
 * Detects when the user reaches the top or bottom of the current buffer window
 * and loads the previous/next chunk of the file accordingly.
 *
 * Behavior:
 * - When scrolled to the top, loads the previous file chunk (if available).
 * - When scrolled to the bottom, loads the next chunk (if available).
 * - Repositions the cursor after new content is inserted and skips animation.
 *
 * @param e LVGL event structure (LV_EVENT_SCROLL).  
 *          User data must contain a valid `text_viewer_ctx_t *`.
 */
static void text_viewer_on_text_scrolled(lv_event_t *e);

/**
 * @brief Hide the keyboard when its cancel/close button is pressed.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_keyboard_cancel(lv_event_t *e);

/**
 * @brief Show the keyboard when the filename dialog textarea is tapped.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_name_textarea_clicked(lv_event_t *e);

/**
 * @brief Trigger Save when the keyboard's OK button is pressed.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_keyboard_ready(lv_event_t *e);

/**
 * @brief Hide the keyboard when tapping outside editable widgets.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_screen_clicked(lv_event_t *e);

/*********************************************************************************************/

/************************************** Editing workflow *************************************/

/**
 * @brief Text change handler: updates @c dirty, Save button, and status.
 *
 * Ignored when not editable or when @c suppress_events is true.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_text_changed(lv_event_t *e);

/**
 * @brief Save the currently loaded text chunk back to the underlying file.
 *
 * This function writes the contents of the LVGL textarea in @p ctx->text_area
 * into the backing file at @p ctx->path, only within the byte window
 * corresponding to the currently loaded chunks (defined by
 * ctx->lasf_file_offset_kb and ctx->current_file_offset_kb).
 *
 * Save strategy:
 * - If @p ctx is NULL, the function returns immediately.
 * - If this is a new file with no name yet, a name dialog is shown and
 *   the function returns without writing.
 * - If the file name is still missing, a "Missing file name" status is set.
 * - Computes a byte window [window_start, window_end) for the loaded text
 *   (based on chunk offsets and READ_CHUNK_SIZE_B), with overflow checks.
 * - Clamps the window to the existing file size to avoid seeking past EOF.
 * - Builds a temporary file path in the same directory as @p dest_path.
 * - Opens the existing file (if any) as @p src and a temporary file as @p tmp.
 * - Writes:
 *      1) Prefix (bytes [0, window_start)) from @p src into @p tmp.
 *      2) The current textarea contents into @p tmp.
 *      3) Suffix (bytes [window_end, file_end)) from @p src into @p tmp.
 * - Renames the temporary file over the destination file for an atomic-ish
 *   replacement.
 *
 * Error handling:
 * - On I/O errors (open/read/write/seek/rename) it:
 *      - Sets a human-readable status string via text_viewer_set_status().
 *      - Logs an error via ESP_LOGE().
 *      - Calls sdspi_schedule_sd_retry() to trigger SD-card recovery logic.
 *      - Cleans up open FILE handles and removes the temporary file.
 *
 * On success:
 * - Updates the "original" text snapshot via text_viewer_set_original().
 * - Clears @p ctx->dirty (sets it to false).
 * - Sets status to "Saved".
 *
 * @param ctx Pointer to the text viewer context. May be NULL, in which case
 *            the function returns immediately without side effects.
 */
static void text_viewer_handle_save(text_viewer_ctx_t *ctx);

/**
 * @brief "Save" button event handler.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_save(lv_event_t *e);

/**
 * @brief "Back" button handler: closes the screen or prompts to save/discard.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_back(lv_event_t *e);

/**
 * @brief Show prompt before changing chunk when dirty.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_show_chunk_prompt(text_viewer_ctx_t *ctx);

/**
 * @brief Close chunk-change prompt if present.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_close_chunk_prompt(text_viewer_ctx_t *ctx);

/**
 * @brief Handle chunk-change prompt buttons.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_chunk_prompt(lv_event_t *e);

/**
 * @brief Apply a pending chunk load after save/discard decision.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_apply_pending_chunk(text_viewer_ctx_t *ctx);

/**
 * @brief Schedule loading a new chunk window (with optional prompt if dirty).
 *
 * @param ctx Viewer context.
 * @param first_offset_kb First chunk offset to load.
 * @param second_offset_kb Second chunk offset to load.
 * @param from_top True if triggered from top edge scroll.
 */
static void text_viewer_request_chunk_load(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb, bool from_top);

/**
 * @brief Poll SD reconnection and retry pending actions.
 *
 * @param timer LVGL timer.
 */
static void text_viewer_on_sd_retry_timer(lv_timer_t *timer);

/**
 * @brief Schedule SD reconnection prompt and retry logic.
 *
 * @param ctx Viewer context.
 * @param action Action to retry after reconnection.
 */
static void text_viewer_schedule_sd_retry(text_viewer_ctx_t *ctx, text_viewer_sd_action_t action);

/*********************************************************************************************/

/**
 * @brief Validate candidate filename (must end with .txt and contain safe chars).
 */
static bool text_viewer_validate_name(const char *name);

/**
 * @brief Ensure \".txt\" suffix (adds or fixes trailing dot cases).
 */
static void text_viewer_ensure_txt_extension(char *name, size_t len);

/**
 * @brief Compose absolute path for a new file.
 *
 * @param ctx     Viewer context (requires valid directory).
 * @param name    Filename to append.
 * @param out     Destination buffer for resulting path.
 * @param out_len Size of @p out in bytes.
 * @return ESP_OK on success or ESP_ERR_* on failure.
 */
static esp_err_t text_viewer_compose_new_path(text_viewer_ctx_t *ctx, const char *name, char *out, size_t out_len);

/**
 * @brief Check if a filesystem path already exists.
 *
 * @param path Absolute path to test.
 * @return true if the path exists, false otherwise.
 */
static bool text_viewer_path_exists(const char *path);

/**
 * @brief Show the filename dialog used when saving a new file.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_show_name_dialog(text_viewer_ctx_t *ctx);

/**
 * @brief Close the filename dialog (if present) and restore edit state.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_close_name_dialog(text_viewer_ctx_t *ctx);

/**
 * @brief Filename dialog button handler.
 *
 * @param e LVGL event.
 */
static void text_viewer_on_name_dialog(lv_event_t *e);

/*********************************************************************************************/

/************************************ Confirmation dialog ************************************/

/**
 * @brief Show the save/discard/cancel confirmation dialog.
 *
 * No-op if a dialog is already open.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_show_confirm(text_viewer_ctx_t *ctx);

/**
 * @brief Check if a target object is inside (or is) a given parent object.
 *
 * @param parent  The LVGL object considered as the potential ancestor.
 * @param target  The LVGL object whose ancestry is checked.
 */
static bool text_viewer_target_in(lv_obj_t *parent, lv_obj_t *target);

/**
 * @brief Close and clear the confirmation dialog if present.
 *
 * @param ctx Viewer context.
 */
static void text_viewer_close_confirm(text_viewer_ctx_t *ctx);

/**
 * @brief Confirmation dialog button handler (Save / Discard / Cancel).
 *
 * @param e LVGL event.
 */
static void text_viewer_on_confirm(lv_event_t *e);

/**
 * @brief Close the viewer, unload the screen, and invoke the close callback.
 *
 * Resets mode and frees resources (keyboard target, original snapshot).
 *
 * @param ctx     Viewer context.
 * @param changed True if file content was saved/changed (passed to callback).
 */
static void text_viewer_close(text_viewer_ctx_t *ctx, bool changed);

/*********************************************************************************************/

esp_err_t text_viewer_open(const text_viewer_open_opts_t *opts)
{
    if (!opts || !opts->return_screen)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bool new_file = !opts->path || opts->path[0] == '\0';
    if (!new_file && !opts->path)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (new_file && (!opts->directory || opts->directory[0] == '\0'))
    {
        return ESP_ERR_INVALID_ARG;
    }

    char *content = NULL;
    size_t file_size_kb = 0;
    size_t first_offset_kb = 0;
    size_t second_offset_kb = 0;
    if (new_file)
    {
        content = strdup("");
        if (!content)
        {
            return ESP_ERR_NO_MEM;
        }
    }
    else
    {
        char *chunk_a = NULL;
        char *chunk_b = NULL;
        size_t len_a = 0;
        size_t len_b = 0;
        struct stat st = {0};
        if (stat(opts->path, &st) == 0 && S_ISREG(st.st_mode))
        {
            file_size_kb = (st.st_size > 0) ? ((size_t)st.st_size - 1u) / 1024u : 0;
        }
        second_offset_kb = (file_size_kb > 0) ? 1 : 0;

        esp_err_t err = fs_text_read_range(opts->path, first_offset_kb, &chunk_a, &len_a);
        if (err != ESP_OK)
        {
            free(chunk_a);
            return err;
        }

        if (second_offset_kb != first_offset_kb)
        {
            err = fs_text_read_range(opts->path, second_offset_kb, &chunk_b, &len_b);
            if (err != ESP_OK)
            {
                free(chunk_a);
                free(chunk_b);
                return err;
            }
        }

        size_t total = len_a + len_b;
        content = (char *)malloc(total + 1);
        if (!content)
        {
            free(chunk_a);
            free(chunk_b);
            return ESP_ERR_NO_MEM;
        }
        if (len_a)
        {
            memcpy(content, chunk_a, len_a);
        }
        if (len_b)
        {
            memcpy(content + len_a, chunk_b, len_b);
        }
        content[total] = '\0';

        free(chunk_a);
        free(chunk_b);
    }

    text_viewer_ctx_t *ctx = &s_viewer;
    if (!ctx->screen)
    {
        text_viewer_build_screen(ctx);
    }

    text_viewer_close_confirm(ctx);
    ctx->active = true;
    ctx->editable = new_file ? true : opts->editable;
    ctx->new_file = new_file;
    ctx->dirty = new_file ? true : false;
    ctx->suppress_events = true;
    ctx->return_screen = opts->return_screen;
    ctx->close_cb = opts->on_close;
    ctx->close_ctx = opts->user_ctx;

    ctx->current_file_offset_kb = second_offset_kb;
    ctx->lasf_file_offset_kb = first_offset_kb;
    ctx->max_file_offset_kb = file_size_kb;

    ctx->name_dialog = NULL;
    ctx->name_textarea = NULL;
    ctx->chunk_mbox = NULL;
    ctx->sd_retry_timer = NULL;
    ctx->at_top_edge = false;
    ctx->at_bottom_edge = false;
    ctx->pending_chunk = false;
    ctx->pending_first_offset_kb = 0;
    ctx->pending_second_offset_kb = 0;
    ctx->pending_scroll_up = false;
    ctx->waiting_sd = false;
    ctx->sd_retry_action = TEXT_VIEWER_SD_NONE;
    ctx->content_changed = false;
    ctx->slider_suppress_change = false;
    ctx->slider_drag_active = false;
    ctx->slider_pending_step = SIZE_MAX;

    if (new_file)
    {
        ctx->path[0] = '\0';
        strlcpy(ctx->directory, opts->directory, sizeof(ctx->directory));
        strlcpy(ctx->pending_name, ".txt", sizeof(ctx->pending_name));
        text_viewer_set_path_label(ctx, ctx->directory);
    }
    else
    {
        ctx->directory[0] = '\0';
        ctx->pending_name[0] = '\0';
        strlcpy(ctx->path, opts->path, sizeof(ctx->path));
        text_viewer_set_path_label(ctx, ctx->path);
    }

    lv_textarea_set_text(ctx->text_area, content);
    text_viewer_set_original(ctx, content);
    free(content);
    ctx->suppress_events = false;
    if (ctx->new_file)
    {
        text_viewer_set_status(ctx, "New TXT");
    }
    else
    {
        text_viewer_set_status(ctx, ctx->editable ? "Edit mode" : "View mode");
    }
    text_viewer_apply_mode(ctx);
    text_viewer_update_slider(ctx);
    lv_screen_load(ctx->screen);
    if (ctx->new_file)
    {
        lv_textarea_set_cursor_pos(ctx->text_area, 0);
        lv_obj_add_state(ctx->text_area, LV_STATE_FOCUSED);
        text_viewer_show_keyboard(ctx, ctx->text_area);
    }
    return ESP_OK;
}

static void text_viewer_build_screen(text_viewer_ctx_t *ctx)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 2, 0);
    lv_obj_set_style_pad_gap(scr, 5, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, text_viewer_on_screen_clicked, LV_EVENT_CLICKED, ctx);
    ctx->screen = scr;

    lv_obj_t *toolbar = lv_obj_create(scr);
    lv_obj_remove_style_all(toolbar);
    lv_obj_set_size(toolbar, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(toolbar, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(toolbar, 3, 0);
    lv_obj_set_flex_align(toolbar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    styles_set_card_color(toolbar, 0);
    lv_obj_set_style_bg_opa(toolbar, LV_OPA_COVER, 0);
    ctx->toolbar = toolbar;

    lv_obj_t *back_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_set_style_pad_all(back_btn, 6, 0);   
    styles_set_button(back_btn);  
    lv_obj_add_event_cb(back_btn, text_viewer_on_back, LV_EVENT_CLICKED, ctx);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_center(back_lbl);

    ctx->save_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(ctx->save_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->save_btn, 6, 0);     
    styles_set_button(ctx->save_btn);   
    lv_obj_add_event_cb(ctx->save_btn, text_viewer_on_save, LV_EVENT_CLICKED, ctx);
    lv_obj_t *save_lbl = lv_label_create(ctx->save_btn);
    lv_label_set_text(save_lbl, LV_SYMBOL_SAVE " Save");
    lv_obj_center(save_lbl);

    lv_obj_t *status_spacer_left = lv_obj_create(toolbar);
    lv_obj_remove_style_all(status_spacer_left);
    lv_obj_set_flex_grow(status_spacer_left, 1);
    lv_obj_set_height(status_spacer_left, 1);

    ctx->status_label = lv_label_create(toolbar);
    lv_label_set_text(ctx->status_label, "");
    lv_label_set_long_mode(ctx->status_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(ctx->status_label, LV_TEXT_ALIGN_CENTER, 0);
    styles_set_text_color(ctx->status_label, 0);
    lv_obj_set_style_text_font(ctx->status_label, &Domine_16, 0);
    const lv_font_t *status_font = lv_obj_get_style_text_font(ctx->status_label, LV_PART_MAIN);
    lv_coord_t status_height = status_font ? status_font->line_height : 18;
    lv_obj_set_style_min_height(ctx->status_label, status_height, 0);
    lv_obj_set_style_max_height(ctx->status_label, status_height, 0);

    lv_obj_t *status_spacer_right = lv_obj_create(toolbar);
    lv_obj_remove_style_all(status_spacer_right);
    lv_obj_set_flex_grow(status_spacer_right, 1);
    lv_obj_set_height(status_spacer_right, 1);

    lv_obj_t *path_row = lv_obj_create(scr);
    lv_obj_remove_style_all(path_row);
    lv_obj_set_size(path_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(path_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(path_row, 4, 0);

    lv_obj_t *path_prefix = lv_label_create(path_row);
    lv_label_set_text(path_prefix, "Path: ");
    lv_obj_set_style_text_align(path_prefix, LV_TEXT_ALIGN_LEFT, 0);

    ctx->path_label = lv_label_create(path_row);
    lv_label_set_long_mode(ctx->path_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_flex_grow(ctx->path_label, 1);
    lv_obj_set_width(ctx->path_label, LV_PCT(100));
    lv_obj_set_style_text_align(ctx->path_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(ctx->path_label, "");

    lv_coord_t slider_gap = 6;

    lv_obj_t *text_row = lv_obj_create(scr);
    lv_obj_remove_style_all(text_row);
    lv_obj_set_size(text_row, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(text_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(text_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(text_row, slider_gap, 0);
    lv_obj_set_style_pad_right(text_row, slider_gap, 0);
    lv_obj_set_flex_grow(text_row, 1);    

    ctx->text_area = lv_textarea_create(text_row);
    lv_obj_set_flex_grow(ctx->text_area, 1);
    lv_obj_set_height(ctx->text_area, LV_PCT(100));
    lv_obj_set_style_pad_all(ctx->text_area, 0, 0);
    styles_set_textarea(ctx->text_area);
    lv_textarea_set_cursor_click_pos(ctx->text_area, false);
    lv_obj_set_scrollbar_mode(ctx->text_area, LV_SCROLLBAR_MODE_AUTO);
    styles_set_textarea(ctx->text_area);
    lv_obj_add_event_cb(ctx->text_area, text_viewer_on_text_changed, LV_EVENT_VALUE_CHANGED, ctx);
    lv_obj_add_event_cb(ctx->text_area, text_viewer_on_text_area_clicked, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->text_area, text_viewer_on_text_scrolled, LV_EVENT_SCROLL, ctx);
    
    lv_obj_t *list_slider = lv_slider_create(text_row);
    lv_slider_set_orientation(list_slider, LV_SLIDER_ORIENTATION_VERTICAL);
    lv_slider_set_range(list_slider, 100, 0); /* Min at top, max at bottom */
    lv_slider_set_value(list_slider, 0, LV_ANIM_OFF);
    lv_obj_set_width(list_slider, 14);
    lv_obj_set_height(list_slider, LV_PCT(85));
    lv_obj_set_style_pad_top(list_slider, 0, 0);
    lv_obj_set_style_pad_bottom(list_slider, 0, 0);
    lv_obj_set_style_pad_left(list_slider, 0, 0);
    lv_obj_set_style_pad_right(list_slider, 0, 0);
    lv_obj_set_style_translate_y(list_slider, 2, 0);
    styles_set_slider(list_slider);
    lv_obj_set_style_bg_opa(list_slider, LV_OPA_60, 0);
    lv_obj_set_style_radius(list_slider, 8, 0);
    lv_obj_set_style_bg_opa(list_slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(list_slider, 8, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(list_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(list_slider, 1, LV_PART_KNOB);
    lv_obj_set_style_radius(list_slider, 6, LV_PART_KNOB);
    lv_obj_set_style_width(list_slider, 12, LV_PART_KNOB);
    lv_obj_set_style_height(list_slider, 12, LV_PART_KNOB);
    lv_obj_add_event_cb(list_slider, text_viewer_on_slider_value_changed, LV_EVENT_PRESSED, ctx);
    lv_obj_add_event_cb(list_slider, text_viewer_on_slider_value_changed, LV_EVENT_VALUE_CHANGED, ctx);
    lv_obj_add_event_cb(list_slider, text_viewer_on_slider_value_changed, LV_EVENT_RELEASED, ctx);
    lv_obj_add_event_cb(list_slider, text_viewer_on_slider_value_changed, LV_EVENT_PRESS_LOST, ctx);
    lv_obj_clear_flag(list_slider, LV_OBJ_FLAG_SCROLL_CHAIN);
    ctx->chunk_slider = list_slider;
    
    ctx->keyboard = lv_keyboard_create(scr);
    styles_set_keyboard(ctx->keyboard);
    lv_keyboard_set_textarea(ctx->keyboard, ctx->text_area);
    lv_obj_add_flag(ctx->keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(ctx->keyboard, text_viewer_on_keyboard_cancel, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->keyboard, text_viewer_on_keyboard_ready, LV_EVENT_READY, ctx);
}

static void text_viewer_apply_mode(text_viewer_ctx_t *ctx)
{
    if (ctx->editable)
    {
        lv_obj_clear_state(ctx->text_area, LV_STATE_DISABLED);
        lv_textarea_set_cursor_click_pos(ctx->text_area, true);
        lv_obj_add_flag(ctx->text_area, LV_OBJ_FLAG_CLICK_FOCUSABLE);
        text_viewer_hide_keyboard(ctx);
        lv_obj_clear_flag(ctx->save_btn, LV_OBJ_FLAG_HIDDEN);
        lv_textarea_set_cursor_pos(ctx->text_area, 0);
    }
    else
    {
        lv_textarea_set_cursor_click_pos(ctx->text_area, false);
        lv_obj_clear_flag(ctx->text_area, LV_OBJ_FLAG_CLICK_FOCUSABLE);
        text_viewer_hide_keyboard(ctx);
        lv_obj_add_flag(ctx->save_btn, LV_OBJ_FLAG_HIDDEN);
        lv_textarea_clear_selection(ctx->text_area);
        lv_textarea_set_cursor_pos(ctx->text_area, 0);
    }
    lv_obj_scroll_to_y(ctx->text_area, 0, LV_ANIM_OFF);
    text_viewer_update_buttons(ctx);
}

static void text_viewer_set_status(text_viewer_ctx_t *ctx, const char *msg)
{
    if (ctx->status_label && msg)
    {
        lv_label_set_text(ctx->status_label, msg);
    }
}

static void text_viewer_set_path_label(text_viewer_ctx_t *ctx, const char *path)
{
    if (!ctx || !ctx->path_label)
    {
        return;
    }

    const char *mount = CONFIG_SDSPI_MOUNT_POINT;
    char display[FS_TEXT_MAX_PATH + 8];

    if (path && mount && strncmp(path, mount, strlen(mount)) == 0)
    {
        const char *rest = path + strlen(mount);
        if (*rest == '/')
        {
            rest++;
        }
        if (*rest == '\0')
        {
            strlcpy(display, "/", sizeof(display));
        }
        else
        {
            snprintf(display, sizeof(display), "/%s", rest);
        }
    }
    else
    {
        snprintf(display, sizeof(display), "%s", path ? path : "");
    }

    lv_label_set_text(ctx->path_label, display);
    text_viewer_restart_path_scroll(ctx);
}

static void text_viewer_path_scroll_timer_cb(lv_timer_t *timer)
{
    text_viewer_ctx_t *ctx = (text_viewer_ctx_t *)lv_timer_get_user_data(timer);
    if (ctx) {
        ctx->path_scroll_timer = NULL;
        if (ctx->path_label && lv_obj_is_valid(ctx->path_label)) {
            lv_label_set_long_mode(ctx->path_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        }
    }
    lv_timer_del(timer);
}

static void text_viewer_restart_path_scroll(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->path_label) {
        return;
    }

    if (ctx->path_scroll_timer) {
        lv_timer_del(ctx->path_scroll_timer);
        ctx->path_scroll_timer = NULL;
    }

    /* Start clipped, then enable scroll after a short delay. */
    lv_label_set_long_mode(ctx->path_label, LV_LABEL_LONG_CLIP);
    ctx->path_scroll_timer = lv_timer_create(text_viewer_path_scroll_timer_cb, TEXT_VIEWER_PATH_SCROLL_DELAY_MS, ctx);
    if (ctx->path_scroll_timer) {
        lv_timer_set_repeat_count(ctx->path_scroll_timer, 1);
    }
}

static void text_viewer_set_original(text_viewer_ctx_t *ctx, const char *text)
{
    free(ctx->original_text);
    ctx->original_text = text ? strdup(text) : NULL;
}

static void text_viewer_get_slider_params(text_viewer_ctx_t *ctx, size_t *window_size, size_t *step)
{
    (void)ctx;
    if (!window_size || !step) {
        return;
    }
    *window_size = 2; /* two adjacent chunks = one window */
    *step = 1;        /* one chunk per step */
}

static void text_viewer_update_slider(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->chunk_slider) {
        return;
    }

    size_t window_size = 1;
    size_t step = 1;
    text_viewer_get_slider_params(ctx, &window_size, &step);

    size_t total_chunks = ctx->max_file_offset_kb + 1; /* offsets are in KB chunks */
    if (total_chunks == 0) {
        total_chunks = 1;
    }

    if (total_chunks <= window_size) {
        bool prev = ctx->slider_suppress_change;
        ctx->slider_suppress_change = true;
        lv_slider_set_range(ctx->chunk_slider, 0, 0);
        lv_slider_set_value(ctx->chunk_slider, 0, LV_ANIM_OFF);
        ctx->slider_suppress_change = prev;
        ctx->slider_pending_step = 0;
        ctx->slider_drag_active = false;
        lv_obj_add_state(ctx->chunk_slider, LV_STATE_DISABLED);
        return;
    }

    size_t max_start = total_chunks - window_size;
    size_t max_step_index = step ? ((max_start + step - 1) / step) : 0;
    int32_t max_val = (int32_t)max_step_index;

    size_t current_start = ctx->lasf_file_offset_kb;
    if (current_start > max_start) {
        current_start = max_start;
    }
    size_t current_step = step ? (current_start / step) : 0;
    if (current_step > max_step_index) {
        current_step = max_step_index;
    }

    bool prev = ctx->slider_suppress_change;
    ctx->slider_suppress_change = true;
    lv_slider_set_range(ctx->chunk_slider, max_val, 0); /* min at top, max at bottom */
    lv_slider_set_value(ctx->chunk_slider, (int32_t)current_step, LV_ANIM_OFF);
    ctx->slider_suppress_change = prev;
    ctx->slider_pending_step = current_step;
    lv_obj_remove_state(ctx->chunk_slider, LV_STATE_DISABLED);
}

static void text_viewer_on_slider_value_changed(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || ctx->slider_suppress_change) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(e);
    bool blocked = ctx->waiting_sd || ctx->chunk_mbox || ctx->pending_chunk;

    size_t window_size = 1;
    size_t step = 1;
    text_viewer_get_slider_params(ctx, &window_size, &step);
    size_t total_chunks = ctx->max_file_offset_kb + 1;
    if (total_chunks == 0) {
        total_chunks = 1;
    }
    if (total_chunks <= window_size) {
        return; /* Nothing to scroll */
    }

    size_t max_start = total_chunks - window_size;
    size_t max_step_index = step ? ((max_start + step - 1) / step) : 0;

    int32_t slider_val = lv_slider_get_value(lv_event_get_target(e));
    if (slider_val < 0) {
        slider_val = 0;
    }

    size_t clamped_step = (size_t)slider_val;
    if (clamped_step > max_step_index) {
        clamped_step = max_step_index;
    }

    if (code == LV_EVENT_PRESSED) {
        if (blocked) {
            return;
        }
        ctx->slider_drag_active = true;
        ctx->slider_pending_step = clamped_step;
        return;
    }

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (blocked) {
            return;
        }
        ctx->slider_pending_step = clamped_step;
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (blocked) {
            ctx->slider_pending_step = SIZE_MAX;
            ctx->slider_drag_active = false;
            text_viewer_update_slider(ctx);
            return;
        }

        size_t target_step = (ctx->slider_pending_step != SIZE_MAX) ? ctx->slider_pending_step : clamped_step;
        if (target_step > max_step_index) {
            target_step = max_step_index;
        }

        size_t current_start = ctx->lasf_file_offset_kb;
        if (current_start > max_start) {
            current_start = max_start;
        }
        size_t current_step = step ? (current_start / step) : 0;
        if (current_step > max_step_index) {
            current_step = max_step_index;
        }

        if (target_step == current_step) {
            ctx->slider_pending_step = SIZE_MAX;
            ctx->slider_drag_active = false;
            return;
        }

        size_t new_start = (target_step >= max_step_index) ? max_start : (target_step * step);
        if (new_start > max_start) {
            new_start = max_start;
        }

        size_t first_offset = new_start;
        size_t second_offset = first_offset + (window_size > 1 ? (window_size - 1) : 0);
        if (second_offset > ctx->max_file_offset_kb) {
            second_offset = ctx->max_file_offset_kb;
        }
        if (window_size > 1 && second_offset == first_offset && first_offset > 0) {
            first_offset -= 1;
        }

        bool from_top = target_step < current_step;
        ctx->slider_pending_step = SIZE_MAX;
        ctx->slider_drag_active = false;
        text_viewer_request_chunk_load(ctx, first_offset, second_offset, from_top);
    }
}

static esp_err_t text_viewer_load_window(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb)
{
    if (!ctx || ctx->path[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }

    char *chunk_a = NULL;
    char *chunk_b = NULL;
    char *joined = NULL;
    size_t len_a = 0;
    size_t len_b = 0;

    esp_err_t err = fs_text_read_range(ctx->path, first_offset_kb, &chunk_a, &len_a);
    if (err != ESP_OK)
    {
        goto cleanup;
    }

    if (second_offset_kb != first_offset_kb)
    {
        err = fs_text_read_range(ctx->path, second_offset_kb, &chunk_b, &len_b);
        if (err != ESP_OK)
        {
            goto cleanup;
        }
    }

    size_t total = len_a + len_b;
    joined = (char *)malloc(total + 1);
    if (!joined)
    {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    if (len_a)
    {
        memcpy(joined, chunk_a, len_a);
    }
    if (len_b)
    {
        memcpy(joined + len_a, chunk_b, len_b);
    }
    joined[total] = '\0';

    bool prev_suppress = ctx->suppress_events;
    ctx->suppress_events = true;
    lv_textarea_set_text(ctx->text_area, joined);
    text_viewer_set_original(ctx, joined);
    ctx->dirty = false;
    text_viewer_update_buttons(ctx);

    ctx->suppress_events = prev_suppress;

cleanup:
    free(joined);
    free(chunk_a);
    free(chunk_b);
    return err;
}

static void text_viewer_update_buttons(text_viewer_ctx_t *ctx)
{
    if (!ctx->editable)
    {
        return;
    }
    if (ctx->dirty)
    {
        lv_obj_clear_state(ctx->save_btn, LV_STATE_DISABLED);
    }
    else
    {
        lv_obj_add_state(ctx->save_btn, LV_STATE_DISABLED);
    }
}

static void text_viewer_show_keyboard(text_viewer_ctx_t *ctx, lv_obj_t *target)
{
    if (!ctx || !ctx->editable)
    {
        return;
    }
    if (target)
    {
        lv_keyboard_set_textarea(ctx->keyboard, target);
    }
    else if (!lv_keyboard_get_textarea(ctx->keyboard))
    {
        lv_keyboard_set_textarea(ctx->keyboard, ctx->text_area);
    }
    lv_obj_clear_flag(ctx->keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void text_viewer_hide_keyboard(text_viewer_ctx_t *ctx)
{
    if (!ctx)
    {
        return;
    }
    if (!lv_obj_has_flag(ctx->keyboard, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(ctx->keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if (lv_keyboard_get_textarea(ctx->keyboard))
    {
        lv_keyboard_set_textarea(ctx->keyboard, NULL);
    }
}

static void text_viewer_on_text_area_clicked(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->editable)
    {
        return;
    }
    text_viewer_show_keyboard(ctx, ctx->text_area);
}

static void text_viewer_skip_cursor_animation(text_viewer_ctx_t *ctx)
{
    // Jump to the new cursor position immediately (skip the default scroll animation)
    lv_point_t target_scroll = {0};
    lv_obj_get_scroll_end(ctx->text_area, &target_scroll);
    lv_obj_scroll_to(ctx->text_area, target_scroll.x, target_scroll.y, LV_ANIM_OFF);    
}

static void text_viewer_on_text_scrolled(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    if (ctx->waiting_sd)
    {
        return;
    }
    if (ctx->chunk_mbox || ctx->pending_chunk)
    {
        return;
    }

    bool at_top = lv_obj_get_scroll_top(ctx->text_area) <= 0;
    bool at_bottom = lv_obj_get_scroll_bottom(ctx->text_area) <= 0;

    if (at_top && !ctx->at_top_edge)
    {
        ctx->at_top_edge = true;

        if (!ctx->new_file && ctx->lasf_file_offset_kb > 0)
        {
            size_t new_first = ctx->lasf_file_offset_kb - 1;
            size_t new_second = ctx->lasf_file_offset_kb;
            text_viewer_request_chunk_load(ctx, new_first, new_second, true);
        }
    }
    else if (!at_top)
    {
        ctx->at_top_edge = false;
    }

    if (at_bottom && !ctx->at_bottom_edge)
    {
        ctx->at_bottom_edge = true;

        if (!ctx->new_file && ctx->current_file_offset_kb < ctx->max_file_offset_kb)
        {
            size_t next_offset = ctx->current_file_offset_kb + 1;
            size_t first_offset = ctx->current_file_offset_kb;
            text_viewer_request_chunk_load(ctx, first_offset, next_offset, false);
        }
    }
    else if (!at_bottom)
    {
        ctx->at_bottom_edge = false;
    }
}

static void text_viewer_on_keyboard_cancel(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    text_viewer_hide_keyboard(ctx);
}

static void text_viewer_on_name_textarea_clicked(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->name_textarea)
    {
        return;
    }
    text_viewer_show_keyboard(ctx, ctx->name_textarea);
}

static void text_viewer_on_keyboard_ready(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->editable)
    {
        return;
    }
    text_viewer_handle_save(ctx);
}

static void text_viewer_on_screen_clicked(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->editable)
    {
        return;
    }
    if (ctx->name_dialog)
    {
        return;
    }
    if (lv_obj_has_flag(ctx->keyboard, LV_OBJ_FLAG_HIDDEN))
    {
        return;
    }
    lv_obj_t *target = lv_event_get_target(e);
    if (text_viewer_target_in(ctx->text_area, target) ||
        text_viewer_target_in(ctx->keyboard, target))
    {
        return;
    }
    text_viewer_hide_keyboard(ctx);
}

static void text_viewer_on_text_changed(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->editable || ctx->suppress_events)
    {
        return;
    }
    const char *text = lv_textarea_get_text(ctx->text_area);
    const char *orig = ctx->original_text ? ctx->original_text : "";
    bool dirty = strcmp(text, orig) != 0;
    if (dirty != ctx->dirty)
    {
        ctx->dirty = dirty;
        text_viewer_update_buttons(ctx);
        text_viewer_set_status(ctx, dirty ? "Modified" : "Saved");
    }
}

static void text_viewer_handle_save(text_viewer_ctx_t *ctx)
{
    if (!ctx)
    {
        return;
    }
    if (ctx->waiting_sd)
    {
        text_viewer_set_status(ctx, "Reconnect SD");
        return;
    }
    if (ctx->new_file && ctx->path[0] == '\0')
    {
        text_viewer_show_name_dialog(ctx);
        return;
    }

    const char *text = lv_textarea_get_text(ctx->text_area);
    if (!text)
    {
        text = "";
    }

    if (ctx->path[0] == '\0')
    {
        text_viewer_set_status(ctx, "Missing file name");
        return;
    }

    const char *dest_path = ctx->path;
    size_t first_kb = ctx->lasf_file_offset_kb;
    size_t second_kb = ctx->current_file_offset_kb;
    size_t chunk_count = (second_kb > first_kb) ? (second_kb - first_kb + 1u) : 1u;

    /* Compute byte window for the currently loaded textarea (two chunks) */
    if (first_kb > SIZE_MAX / 1024u || chunk_count > SIZE_MAX / READ_CHUNK_SIZE_B)
    {
        text_viewer_set_status(ctx, "Range overflow");
        return;
    }
    size_t window_start = first_kb * 1024u;
    size_t window_span = chunk_count * READ_CHUNK_SIZE_B;
    size_t window_end = window_start + window_span;
    if (window_end < window_start)
    {
        text_viewer_set_status(ctx, "Range overflow");
        return;
    }

    struct stat st = {0};
    bool have_existing = (stat(dest_path, &st) == 0 && S_ISREG(st.st_mode));
    size_t file_size = have_existing ? (size_t)st.st_size : 0u;

    /* Clamp window to current file size to avoid seeking past EOF */
    if (window_start > file_size)
    {
        window_start = file_size;
    }
    if (window_end > file_size)
    {
        window_end = file_size;
    }

    size_t prefix_size = window_start;
    size_t suffix_start = window_end;
    size_t suffix_size = (suffix_start < file_size) ? (file_size - suffix_start) : 0u;

    /* Build temp path in same dir for atomic-ish replacement */
    char dir[FS_TEXT_MAX_PATH];
    const char *slash = strrchr(dest_path, '/');
    if (slash)
    {
        size_t dir_len = (size_t)(slash - dest_path);
        if (dir_len == 0)
        {
            dir[0] = '/';
            dir[1] = '\0';
        }
        else if (dir_len < sizeof(dir))
        {
            memcpy(dir, dest_path, dir_len);
            dir[dir_len] = '\0';
        }
        else
        {
            text_viewer_set_status(ctx, "Path too long");
            return;
        }
    }
    else
    {
        strlcpy(dir, ".", sizeof(dir));
    }

    char tmp_path[FS_TEXT_MAX_PATH];
    int needed = snprintf(tmp_path, sizeof(tmp_path), "%s/tmpwrt.tmp", dir);
    if (needed < 0 || needed >= (int)sizeof(tmp_path))
    {
        text_viewer_set_status(ctx, "Path too long");
        return;
    }
    remove(tmp_path);

    FILE *src = NULL;
    if (have_existing)
    {
        src = fopen(dest_path, "rb");
        if (!src)
        {
            text_viewer_set_status(ctx, "Open failed");
            ESP_LOGE(TAG, "Failed to open %s for patching", dest_path);
            text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return;
        }
    }

    FILE *tmp = fopen(tmp_path, "wb");
    if (!tmp)
    {
        if (src)
        {
            fclose(src);
        }
        text_viewer_set_status(ctx, "Temp open failed");
        ESP_LOGE(TAG, "Failed to open %s", tmp_path);
        text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
        return;
    }

    char buf[READ_CHUNK_SIZE_B];
    size_t remaining = prefix_size;
    while (remaining > 0)
    {
        size_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
        if (!src || fread(buf, 1, chunk, src) != chunk)
        {
            text_viewer_set_status(ctx, "Read failed");
            ESP_LOGE(TAG, "Failed to read prefix from %s", dest_path);
            text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            goto save_cleanup;
        }
        if (fwrite(buf, 1, chunk, tmp) != chunk)
        {
            text_viewer_set_status(ctx, "Write failed");
            ESP_LOGE(TAG, "Failed to write prefix to %s", tmp_path);
            text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            goto save_cleanup;
        }
        remaining -= chunk;
    }

    size_t text_len = strlen(text);
    if (text_len > 0)
    {
        if (fwrite(text, 1, text_len, tmp) != text_len)
        {
            text_viewer_set_status(ctx, "Write failed");
            ESP_LOGE(TAG, "Failed to write textarea to %s", tmp_path);
            text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            goto save_cleanup;
        }
    }

    if (suffix_size > 0 && src)
    {
        if (fseek(src, (long)suffix_start, SEEK_SET) != 0)
        {
            text_viewer_set_status(ctx, "Seek failed");
            ESP_LOGE(TAG, "Failed to seek %s to %zu", dest_path, suffix_start);
            text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            goto save_cleanup;
        }

        remaining = suffix_size;
        while (remaining > 0)
        {
            size_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
            size_t got = fread(buf, 1, chunk, src);
            if (got != chunk)
            {
                text_viewer_set_status(ctx, "Read failed");
                ESP_LOGE(TAG, "Failed to read suffix from %s", dest_path);
                text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
                goto save_cleanup;
            }
            if (fwrite(buf, 1, chunk, tmp) != chunk)
            {
                text_viewer_set_status(ctx, "Write failed");
                ESP_LOGE(TAG, "Failed to write suffix to %s", tmp_path);
                text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
                goto save_cleanup;
            }
            remaining -= chunk;
        }
    }

    if (src)
    {
        fclose(src);
        src = NULL;
    }
    fclose(tmp);
    tmp = NULL;

    if (rename(tmp_path, dest_path) != 0)
    {
        if (errno == EEXIST && remove(dest_path) == 0 && rename(tmp_path, dest_path) == 0)
        {
            /* success after replacing existing */
        }
        else
        {
            text_viewer_set_status(ctx, "Rename failed");
            ESP_LOGE(TAG, "rename(%s -> %s) failed (errno=%d)", tmp_path, dest_path, errno);
            remove(tmp_path);
            text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return;
        }
    }

    size_t new_size = prefix_size + text_len + suffix_size;
    ctx->max_file_offset_kb = (new_size > 0) ? ((new_size - 1u) / 1024u) : 0u;
    if (ctx->lasf_file_offset_kb > ctx->max_file_offset_kb)
    {
        ctx->lasf_file_offset_kb = ctx->max_file_offset_kb;
    }
    if (ctx->current_file_offset_kb > ctx->max_file_offset_kb)
    {
        ctx->current_file_offset_kb = ctx->max_file_offset_kb;
    }
    ctx->at_top_edge = false;
    ctx->at_bottom_edge = false;

    text_viewer_set_original(ctx, text);
    ctx->dirty = false;
    ctx->content_changed = true;
    text_viewer_set_status(ctx, "Saved");
    text_viewer_update_slider(ctx);
    return;

save_cleanup:
    if (src)
    {
        fclose(src);
    }
    if (tmp)
    {
        fclose(tmp);
    }
    remove(tmp_path);
}

static void text_viewer_on_save(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    text_viewer_handle_save(ctx);
}

static void text_viewer_on_back(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    if (ctx->editable && ctx->dirty)
    {
        text_viewer_show_confirm(ctx);
        return;
    }
    text_viewer_close(ctx, false);
}

static void text_viewer_show_chunk_prompt(text_viewer_ctx_t *ctx)
{
    if (!ctx || ctx->chunk_mbox || !ctx->pending_chunk)
    {
        return;
    }
    lv_obj_t *mbox = lv_msgbox_create(ctx->screen);
    styles_set_msgbox(mbox);
    lv_obj_add_flag(mbox, LV_OBJ_FLAG_FLOATING);
    ctx->chunk_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_set_width(mbox, LV_PCT(80));
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Save changes before loading new text?");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *save_btn = lv_msgbox_add_footer_button(mbox, "Save");
    lv_obj_set_user_data(save_btn, (void *)TEXT_VIEWER_CHUNK_SAVE);
    lv_obj_set_flex_grow(save_btn, 1);
    lv_obj_add_event_cb(save_btn, text_viewer_on_chunk_prompt, LV_EVENT_CLICKED, ctx);

    lv_obj_t *discard_btn = lv_msgbox_add_footer_button(mbox, "Discard");
    lv_obj_set_user_data(discard_btn, (void *)TEXT_VIEWER_CHUNK_DISCARD);
    lv_obj_set_flex_grow(discard_btn, 1);
    lv_obj_add_event_cb(discard_btn, text_viewer_on_chunk_prompt, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, NULL);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_add_event_cb(cancel_btn, text_viewer_on_chunk_prompt, LV_EVENT_CLICKED, ctx);
}

static void text_viewer_close_chunk_prompt(text_viewer_ctx_t *ctx)
{
    if (ctx && ctx->chunk_mbox)
    {
        lv_msgbox_close(ctx->chunk_mbox);
        ctx->chunk_mbox = NULL;
    }
}

static void text_viewer_apply_pending_chunk(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->pending_chunk)
    {
        return;
    }
    if (ctx->waiting_sd)
    {
        return;
    }

    esp_err_t err = text_viewer_load_window(ctx, ctx->pending_first_offset_kb, ctx->pending_second_offset_kb);
    if (err == ESP_OK)
    {
        lv_coord_t content_h = lv_obj_get_content_height(ctx->text_area);
        if (ctx->pending_scroll_up)
        {
            lv_textarea_set_cursor_pos(ctx->text_area, (int32_t)READ_CHUNK_SIZE_B + content_h);
            text_viewer_skip_cursor_animation(ctx);
        }
        else
        {
            lv_textarea_set_cursor_pos(ctx->text_area, (int32_t)READ_CHUNK_SIZE_B - content_h);
            text_viewer_skip_cursor_animation(ctx);
        }
        ctx->lasf_file_offset_kb = ctx->pending_first_offset_kb;
        ctx->current_file_offset_kb = ctx->pending_second_offset_kb;
        ctx->at_top_edge = false;
        ctx->at_bottom_edge = false;
        text_viewer_update_slider(ctx);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to load chunk: %s", esp_err_to_name(err));
        text_viewer_schedule_sd_retry(ctx, TEXT_VIEWER_SD_CHUNK);
        ctx->at_top_edge = false;
        ctx->at_bottom_edge = false;
        return;
    }
    ctx->pending_chunk = false;
}

static void text_viewer_on_chunk_prompt(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    void *ud = lv_obj_get_user_data(lv_event_get_target(e));
    text_viewer_close_chunk_prompt(ctx);

    if (ud == (void *)TEXT_VIEWER_CHUNK_SAVE)
    {
        text_viewer_handle_save(ctx);
        if (!ctx->dirty)
        {
            text_viewer_apply_pending_chunk(ctx);
        }
        else if (!ctx->waiting_sd)
        {
            ctx->pending_chunk = false;
            ctx->at_top_edge = false;
            ctx->at_bottom_edge = false;
            text_viewer_update_slider(ctx);
        }
    }
    else if (ud == (void *)TEXT_VIEWER_CHUNK_DISCARD)
    {
        ctx->dirty = false;
        text_viewer_update_buttons(ctx);
        text_viewer_apply_pending_chunk(ctx);
    }
    else
    {
        ctx->pending_chunk = false; // Cancel
        ctx->at_top_edge = false;
        ctx->at_bottom_edge = false;
        text_viewer_update_slider(ctx);
    }
}

static void text_viewer_request_chunk_load(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb, bool from_top)
{
    if (!ctx || ctx->chunk_mbox)
    {
        return;
    }
    if (ctx->waiting_sd)
    {
        ctx->pending_first_offset_kb = first_offset_kb;
        ctx->pending_second_offset_kb = second_offset_kb;
        ctx->pending_scroll_up = from_top;
        ctx->pending_chunk = true;
        return;
    }

    ctx->pending_first_offset_kb = first_offset_kb;
    ctx->pending_second_offset_kb = second_offset_kb;
    ctx->pending_scroll_up = from_top;
    ctx->pending_chunk = true;

    if (ctx->dirty)
    {
        text_viewer_show_chunk_prompt(ctx);
    }
    else
    {
        text_viewer_apply_pending_chunk(ctx);
    }
}

static void text_viewer_on_sd_retry_timer(lv_timer_t *timer)
{
    text_viewer_ctx_t *ctx = lv_timer_get_user_data(timer);
    if (!ctx || !ctx->waiting_sd)
    {
        return;
    }
    if (!reconnection_success)
    {
        text_viewer_set_status(ctx, "Reconnect SD");
        return;
    }
    if (xSemaphoreTake(reconnection_success, 0) != pdTRUE)
    {
        text_viewer_set_status(ctx, "Reconnect SD");
        return;
    }

    ctx->waiting_sd = false;
    text_viewer_sd_action_t action = ctx->sd_retry_action;
    ctx->sd_retry_action = TEXT_VIEWER_SD_NONE;
    text_viewer_set_status(ctx, "SD reconnected");

    if (action == TEXT_VIEWER_SD_SAVE)
    {
        text_viewer_handle_save(ctx);
        if (ctx->pending_chunk && !ctx->dirty && !ctx->waiting_sd)
        {
            text_viewer_apply_pending_chunk(ctx);
        }
    }
    else if (action == TEXT_VIEWER_SD_CHUNK)
    {
        text_viewer_apply_pending_chunk(ctx);
    }
}

static void text_viewer_schedule_sd_retry(text_viewer_ctx_t *ctx, text_viewer_sd_action_t action)
{
    if (!ctx)
    {
        return;
    }
    if (ctx->waiting_sd)
    {
        ctx->sd_retry_action = action;
        return;
    }
    ctx->waiting_sd = true;
    ctx->sd_retry_action = action;
    text_viewer_set_status(ctx, "Reconnect SD");
    sdspi_schedule_sd_retry();

    if (!ctx->sd_retry_timer)
    {
        ctx->sd_retry_timer = lv_timer_create(text_viewer_on_sd_retry_timer, 250, ctx);
    }
}


/************************************* New-file utilities *************************************/

static bool text_viewer_validate_name(const char *name)
{
    if (!name || name[0] == '\0')
    {
        return false;
    }
    for (const char *p = name; *p; ++p)
    {
        if (
            *p == '\\' || *p == '/' || *p == ':' ||
            *p == '*' || *p == '?' || *p == '"' ||
            *p == '<' || *p == '>' || *p == '|')
        {
            return false;
        }
    }
    return fs_text_is_txt(name);
}

static void text_viewer_ensure_txt_extension(char *name, size_t len)
{
    if (!name || len == 0)
    {
        return;
    }
    size_t n = strlen(name);
    if (n == 0)
    {
        strlcpy(name, ".txt", len);
        return;
    }
    const char *dot = strrchr(name, '.');
    if (dot && dot[1] != '\0')
    {
        if (strcasecmp(dot, ".txt") == 0)
        {
            return;
        }
        return;
    }
    if (n + 4 >= len)
    {
        return;
    }
    if (dot && dot[1] == '\0')
    {
        strlcpy(name + n, "txt", len - n);
    }
    else
    {
        strlcat(name, ".txt", len);
    }
}

static esp_err_t text_viewer_compose_new_path(text_viewer_ctx_t *ctx, const char *name, char *out, size_t out_len)
{
    if (!ctx || !name || !out || out_len == 0 || ctx->directory[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }
    int needed = snprintf(out, out_len, "%s/%s", ctx->directory, name);
    if (needed < 0 || needed >= (int)out_len)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static bool text_viewer_path_exists(const char *path)
{
    if (!path || path[0] == '\0')
    {
        return false;
    }
    struct stat st = {0};
    return stat(path, &st) == 0;
}

static void text_viewer_show_name_dialog(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->new_file || !ctx->editable || ctx->name_dialog)
    {
        return;
    }
    lv_obj_t *dlg = lv_msgbox_create(ctx->screen);
    styles_set_msgbox(dlg);
    ctx->name_dialog = dlg;
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_max_width(dlg, LV_PCT(65), 0);
    lv_obj_set_width(dlg, LV_PCT(65));

    lv_obj_t *content = lv_msgbox_get_content(dlg);
    lv_obj_t *label = lv_label_create(content);
    lv_label_set_text(label, "File name");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);

    ctx->name_textarea = lv_textarea_create(content);
    lv_textarea_set_one_line(ctx->name_textarea, true);
    lv_textarea_set_max_length(ctx->name_textarea, FS_NAV_MAX_NAME - 1);
    const char *initial = ctx->pending_name[0] ? ctx->pending_name : ".txt";
    lv_textarea_set_text(ctx->name_textarea, initial);
    lv_textarea_set_cursor_pos(ctx->name_textarea, 0);
    lv_obj_add_state(ctx->name_textarea, LV_STATE_FOCUSED);
    lv_obj_clear_state(ctx->text_area, LV_STATE_FOCUSED);
    lv_obj_add_state(ctx->text_area, LV_STATE_DISABLED);
    styles_set_textarea(ctx->name_textarea);
    lv_obj_set_width(ctx->name_textarea, LV_PCT(100));
    lv_textarea_set_cursor_click_pos(ctx->text_area, false);

    lv_obj_t *save_btn = lv_msgbox_add_footer_button(dlg, "Save");
    lv_obj_set_user_data(save_btn, (void *)1);
    styles_set_button(save_btn);
    lv_obj_add_event_cb(save_btn, text_viewer_on_name_dialog, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(dlg, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, text_viewer_on_name_dialog, LV_EVENT_CLICKED, ctx);

    text_viewer_show_keyboard(ctx, ctx->name_textarea);
    lv_obj_add_event_cb(ctx->name_textarea, text_viewer_on_name_textarea_clicked, LV_EVENT_CLICKED, ctx);

    lv_obj_update_layout(ctx->keyboard);
    lv_obj_update_layout(dlg);
    lv_coord_t keyboard_top = lv_obj_get_y(ctx->keyboard);
    lv_coord_t dialog_h = lv_obj_get_height(dlg);
    lv_coord_t margin = 10;
    if (keyboard_top > dialog_h)
    {
        lv_coord_t candidate = (keyboard_top - dialog_h) / 2;
        if (candidate > 0)
        {
            margin = candidate;
        }
    }
    lv_obj_align(dlg, LV_ALIGN_TOP_MID, 0, margin);
}

static void text_viewer_close_name_dialog(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->name_dialog)
    {
        return;
    }
    if (ctx->name_textarea)
    {
        const char *current = lv_textarea_get_text(ctx->name_textarea);
        if (current)
        {
            strlcpy(ctx->pending_name, current, sizeof(ctx->pending_name));
        }
    }
    lv_msgbox_close(ctx->name_dialog);
    ctx->name_dialog = NULL;
    ctx->name_textarea = NULL;
    lv_obj_clear_state(ctx->text_area, LV_STATE_DISABLED);
    lv_textarea_set_cursor_click_pos(ctx->text_area, true);
    text_viewer_hide_keyboard(ctx);
}

static void text_viewer_on_name_dialog(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->name_dialog)
    {
        return;
    }
    bool confirm = (bool)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    if (!confirm)
    {
        text_viewer_close_name_dialog(ctx);
        return;
    }

    const char *raw = ctx->name_textarea ? lv_textarea_get_text(ctx->name_textarea) : "";
    char name_buf[FS_NAV_MAX_NAME];
    strlcpy(name_buf, raw ? raw : "", sizeof(name_buf));
    text_viewer_ensure_txt_extension(name_buf, sizeof(name_buf));
    if (!text_viewer_validate_name(name_buf))
    {
        text_viewer_set_status(ctx, "Invalid .txt name");
        return;
    }
    esp_err_t compose_err = text_viewer_compose_new_path(ctx, name_buf, ctx->path, sizeof(ctx->path));
    if (compose_err != ESP_OK)
    {
        text_viewer_set_status(ctx, "Path too long");
        return;
    }
    if (text_viewer_path_exists(ctx->path))
    {
        text_viewer_set_status(ctx, "File already exists");
        return;
    }

    strlcpy(ctx->pending_name, name_buf, sizeof(ctx->pending_name));
    ctx->directory[0] = '\0';
    ctx->new_file = false;
    text_viewer_set_path_label(ctx, ctx->path);
    text_viewer_close_name_dialog(ctx);
    text_viewer_handle_save(ctx);
}

static void text_viewer_show_confirm(text_viewer_ctx_t *ctx)
{
    if (ctx->confirm_mbox)
    {
        return;
    }
    lv_obj_t *mbox = lv_msgbox_create(ctx->screen);
    styles_set_msgbox(mbox);
    lv_obj_add_flag(mbox, LV_OBJ_FLAG_FLOATING);
    ctx->confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_set_width(mbox, LV_PCT(80));
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Save changes?");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *save_btn = lv_msgbox_add_footer_button(mbox, "Save");
    lv_obj_set_user_data(save_btn, (void *)TEXT_VIEWER_CONFIRM_SAVE);
    lv_obj_set_flex_grow(save_btn, 1);
    styles_set_button(save_btn);
    lv_obj_add_event_cb(save_btn, text_viewer_on_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *discard_btn = lv_msgbox_add_footer_button(mbox, "Discard");
    lv_obj_set_user_data(discard_btn, (void *)TEXT_VIEWER_CONFIRM_DISCARD);
    lv_obj_set_flex_grow(discard_btn, 1);
    styles_set_button(discard_btn);
    lv_obj_add_event_cb(discard_btn, text_viewer_on_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, NULL);
    lv_obj_set_flex_grow(cancel_btn, 1);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, text_viewer_on_confirm, LV_EVENT_CLICKED, ctx);
}

static bool text_viewer_target_in(lv_obj_t *parent, lv_obj_t *target)
{
    while (target)
    {
        if (target == parent)
        {
            return true;
        }
        target = lv_obj_get_parent(target);
    }
    return false;
}

static void text_viewer_close_confirm(text_viewer_ctx_t *ctx)
{
    if (ctx->confirm_mbox)
    {
        lv_msgbox_close(ctx->confirm_mbox);
        ctx->confirm_mbox = NULL;
    }
}

static void text_viewer_on_confirm(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    void *ud = lv_obj_get_user_data(lv_event_get_target(e));
    text_viewer_close_confirm(ctx);
    if (ud == (void *)TEXT_VIEWER_CONFIRM_SAVE)
    {
        text_viewer_handle_save(ctx);
    }
    else if (ud == (void *)TEXT_VIEWER_CONFIRM_DISCARD)
    {
        text_viewer_close(ctx, false);
    }
}

static void text_viewer_close(text_viewer_ctx_t *ctx, bool changed)
{
    text_viewer_close_confirm(ctx);
    text_viewer_close_chunk_prompt(ctx);
    text_viewer_close_name_dialog(ctx);
    if (ctx->sd_retry_timer)
    {
        lv_timer_del(ctx->sd_retry_timer);
        ctx->sd_retry_timer = NULL;
    }
    ctx->active = false;
    ctx->editable = false;
    ctx->dirty = false;
    ctx->suppress_events = false;
    ctx->new_file = false;
    ctx->directory[0] = '\0';
    ctx->pending_name[0] = '\0';
    ctx->pending_chunk = false;
    ctx->waiting_sd = false;
    ctx->sd_retry_action = TEXT_VIEWER_SD_NONE;
    ctx->content_changed = false;
    lv_keyboard_set_textarea(ctx->keyboard, NULL);
    lv_obj_add_flag(ctx->keyboard, LV_OBJ_FLAG_HIDDEN);
    /* Drop heavy UI tree (text area buffer) so large files release heap after close. */
    if (ctx->screen) {
        lv_obj_del(ctx->screen);
        ctx->screen = NULL;
        ctx->toolbar = NULL;
        ctx->path_label = NULL;
        ctx->status_label = NULL;
        ctx->save_btn = NULL;
        ctx->text_area = NULL;
        ctx->keyboard = NULL;
        ctx->chunk_slider = NULL;
    }
    free(ctx->original_text);
    ctx->original_text = NULL;
    if (ctx->return_screen)
    {
        lv_screen_load(ctx->return_screen);
    }
    if (ctx->close_cb)
    {
        ctx->close_cb(true, ctx->close_ctx); // Force refresh in caller (e.g., file browser list)
    }
}
