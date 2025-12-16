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

typedef struct{
    lv_obj_t *screen;                           /**< Root LVGL screen object */
    lv_obj_t *toolbar;                          /**< Toolbar container */
    lv_obj_t *save_btn;                         /**< Save button (hidden/disabled in view mode) */
    lv_obj_t *keyboard;                         /**< On-screen keyboard */
    lv_obj_t *text_area;                        /**< Text area for viewing/editing content */
    lv_obj_t *chunk_mbox;                       /**< Chunk-change confirmation message box */
    lv_obj_t *path_label;                       /**< Label showing the file path */
    lv_obj_t *status_label;                     /**< Label showing transient status messages */
    lv_obj_t *name_dialog;                      /**< Filename prompt dialog */
    lv_obj_t *confirm_mbox;                     /**< Confirmation message box (save/discard) */
    lv_obj_t *save_conflict_mbox;               /**< Conflict dialog shown when a new name already exists */
    lv_obj_t *chunk_slider;                     /**< Vertical slider for chunk navigation */
    lv_obj_t *name_textarea;                    /**< Text area used inside filename dialog */
    lv_obj_t *return_screen;                    /**< Screen to return to on close */
    lv_timer_t *sd_retry_timer;                 /**< Timer to poll SD reconnection */
    lv_timer_t *path_scroll_timer;              /**< Timer to delay the scrolling of paths */
}text_viewer_graphics_t;

typedef struct{
    bool active;                                /**< True while the viewer screen is active */
    bool dirty;                                 /**< True if current text differs from original */
    bool editable;                              /**< True if edit mode is enabled */
    bool new_file;                              /**< True if creating a new file */
    bool waiting_sd;                            /**< True while waiting SD reconnection */
    bool at_top_edge;                           /**< Tracks if the scroll is currently at the top edge */
    bool pending_chunk;                         /**< True if a chunk load is pending confirmation */
    bool at_bottom_edge;                        /**< Tracks if the scroll is currently at the bottom edge */
    bool content_changed;                       /**< True if file was saved during session */
    bool suppress_events;                       /**< Temporarily disable change detection */
    bool pending_scroll_up;                     /**< True if pending load comes from top edge */
    bool slider_drag_active;                    /**< True while slider knob is dragged */
    bool slider_suppress_change;                /**< Guard slider callbacks while syncing */
}text_viewer_flags_t;

/**
 * @brief Runtime state for the singleton text viewer/editor screen.
 */
typedef struct
{
    void *close_ctx;                            /**< User context for close callback */
    char *original_text;                        /**< Snapshot of text at load/save time */
    char path[FS_TEXT_MAX_PATH];                /**< Current file path */
    char directory[FS_TEXT_MAX_PATH];           /**< Directory used for new files */
    char pending_name[FS_NAV_MAX_NAME];         /**< Suggested filename for new files */
    char conflict_path[FS_TEXT_MAX_PATH];       /**< Cached conflicting path for overwrite prompts */
    char conflict_name[FS_NAV_MAX_NAME];        /**< Cached conflicting name for overwrite prompts */
    size_t slider_pending_step;                 /**< Pending slider step during drag */
    size_t max_file_offset_kb;                  /**< Maximum readable offset (in KB) for the loaded file */
    size_t last_file_offset_kb;                 /**< Offset (in KB) used for the last read chunk */
    size_t current_file_offset_kb;              /**< Offset (in KB) used for the current/next chunk */
    size_t pending_first_offset_kb;             /**< Pending first chunk offset when prompting */
    size_t pending_second_offset_kb;            /**< Pending second chunk offset when prompting */
    text_viewer_flags_t flags;                  /**< All purpose flags for the text viewer context */
    text_viewer_graphics_t graphics;            /**< LVGL UI objects used for drawing */
    text_viewer_close_cb_t close_cb;            /**< Optional close callback */
    text_viewer_sd_action_t sd_retry_action;    /**< Pending action after SD reconnect */
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
 * @ref apply_mode.
 *
 * @param ctx Viewer context (must be non-NULL).
 */
static void build_screen(text_viewer_ctx_t *ctx);

/**
 * @brief Apply current mode (view vs edit) to widgets and controls.
 *
 * Enables/disables text area, toggles keyboard and save button,
 * then updates button states.
 *
 * @param ctx Viewer context.
 */
static void apply_mode(text_viewer_ctx_t *ctx);

/**
 * @brief Set a short status message in the toolbar.
 *
 * @param ctx Viewer context.
 * @param msg Null-terminated message string (ignored if NULL).
 */
static void set_status(text_viewer_ctx_t *ctx, const char *msg);

/**
 * @brief Set the path label using a UI-friendly path (hide mountpoint).
 *
 * Replaces leading CONFIG_SDSPI_MOUNT_POINT with "/" for display purposes.
 *
 * @param ctx Viewer context.
 * @param path Filesystem path (may be NULL/empty).
 */
static void set_path_label(text_viewer_ctx_t *ctx, const char *path);

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
static void restart_path_scroll(text_viewer_ctx_t *ctx);

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
static void path_scroll_timer_cb(lv_timer_t *timer);

/**
 * @brief Replace the stored original text snapshot.
 *
 * Frees the previous snapshot and stores a duplicate of @p text.
 *
 * @param ctx  Viewer context.
 * @param text New baseline text (may be NULL, which clears the snapshot).
 */
static void set_original(text_viewer_ctx_t *ctx, const char *text);

/**
 * @brief Resolve slider window size and step (in KB chunks) with defaults.
 *
 * @param[in]  ctx          Viewer context (currently unused).
 * @param[out] window_size  Effective window size (chunks per window, >=1).
 * @param[out] step         Effective step size (chunks per step, >=1).
 */
static void get_slider_params(text_viewer_ctx_t *ctx, size_t *window_size, size_t *step);

/**
 * @brief Sync the chunk slider with the current window and file size.
 *
 * Updates range/value, snaps to the active window (including the last), disables when
 * a single window fits, and stores the pending step for drag handling.
 *
 * @param[in,out] ctx Viewer context containing slider state.
 */
static void update_slider(text_viewer_ctx_t *ctx);

/**
 * @brief Disable the slider when only one window fits.
 *
 * Resets range/value, clears drag state, and disables the widget.
 *
 * @param ctx Viewer context.
 */
static void update_slider_disabled(text_viewer_ctx_t *ctx);

/**
 * @brief Update slider range/value based on total chunks and window size.
 *
 * Configures slider range (top=0, bottom=max), sets current position, and clears disabled state.
 *
 * @param ctx          Viewer context.
 * @param window_size  Chunks per window.
 * @param step         Chunks per slider step.
 * @param total_chunks Total chunks available.
 */
static void update_slider_range(text_viewer_ctx_t *ctx, size_t window_size, size_t step, size_t total_chunks);

/**
 * @brief Clamp the current step based on offsets and limits.
 *
 * @param ctx            Viewer context.
 * @param step           Chunks per slider step.
 * @param max_start      Maximum starting chunk index.
 * @param max_step_index Maximum slider step index.
 * @return Clamped current step.
 */
static size_t clamp_current_step(const text_viewer_ctx_t *ctx, size_t step, size_t max_start, size_t max_step_index);

/**
 * @brief Return whether slider actions are blocked (SD wait, dialogs, pending chunk).
 *
 * @param ctx Viewer context.
 * @return true if slider interactions should be ignored; false otherwise.
 */
static bool slider_is_blocked(const text_viewer_ctx_t *ctx);

/**
 * @brief Compute slider bounds (window/step/total/max values); returns false if no scroll.
 *
 * @param ctx             Viewer context.
 * @param[out] window_size Chunks per window.
 * @param[out] step        Chunks per step.
 * @param[out] total_chunks Total chunk count (>=1).
 * @param[out] max_start    Max starting chunk.
 * @param[out] max_step_index Max slider index.
 * @return true if scrolling is possible; false if slider should no-op.
 */
static bool get_slider_bounds(text_viewer_ctx_t *ctx, size_t *window_size, size_t *step, size_t *total_chunks,
                              size_t *max_start, size_t *max_step_index);

/**
 * @brief Clamp slider value to valid range.
 *
 * @param e LVGL event containing slider target.
 * @param max_step_index Maximum allowed step index.
 * @return Clamped step value.
 */
static size_t clamp_slider_value(lv_event_t *e, size_t max_step_index);

/**
 * @brief Handle slider release to trigger chunk load or reset.
 *
 * @param ctx            Viewer context.
 * @param blocked        True if operations are blocked (timer/dialog).
 * @param clamped_step   Current clamped slider step.
 * @param max_step_index Maximum slider step index.
 * @param max_start      Maximum starting chunk index.
 * @param window_size    Chunks per window.
 * @param step           Chunks per slider step.
 */
static void handle_slider_release(text_viewer_ctx_t *ctx, bool blocked, size_t clamped_step,
                                  size_t max_step_index, size_t max_start, size_t window_size, size_t step);

/**
 * @brief Resolve target slider step (pending vs current clamped) within bounds.
 *
 * @param ctx            Viewer context.
 * @param clamped_step   Current clamped slider value.
 * @param max_step_index Maximum slider step index.
 * @return Target step after applying bounds.
 */
static size_t slider_target_step(const text_viewer_ctx_t *ctx, size_t clamped_step, size_t max_step_index);

/**
 * @brief Compute first/second chunk offsets for a given target step.
 *
 * @param target_step       Target slider step.
 * @param max_step_index    Maximum slider step index.
 * @param max_start         Maximum starting chunk index.
 * @param step              Chunks per slider step.
 * @param window_size       Chunks per window.
 * @param max_file_offset_kb Maximum file offset (in KB).
 * @param[out] first_offset  Computed first chunk offset.
 * @param[out] second_offset Computed second chunk offset.
 */
static void compute_window_offsets(size_t target_step, size_t max_step_index, size_t max_start, size_t step,
                                   size_t window_size, size_t max_file_offset_kb,
                                   size_t *first_offset, size_t *second_offset);
/**
 * @brief Read one or two chunks for the current window.
 *
 * @param ctx              Viewer context.
 * @param first_offset_kb  First chunk offset.
 * @param second_offset_kb Second chunk offset.
 * @param[out] chunk_a     Buffer for first chunk.
 * @param[out] len_a       Length of first chunk.
 * @param[out] chunk_b     Buffer for second chunk.
 * @param[out] len_b       Length of second chunk.
 * @return ESP_OK on success; error code otherwise.
 */
static esp_err_t read_window_chunks(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb,
                                    char **chunk_a, size_t *len_a, char **chunk_b, size_t *len_b);
/**
 * @brief Join up to two chunks into a single null-terminated buffer.
 *
 * @param chunk_a    First chunk buffer.
 * @param len_a      Length of first chunk.
 * @param chunk_b    Second chunk buffer.
 * @param len_b      Length of second chunk.
 * @param[out] joined_out Allocated joined buffer (caller frees).
 * @return ESP_OK on success; ESP_ERR_NO_MEM on allocation failure.
 */
static esp_err_t join_window_chunks(char *chunk_a, size_t len_a, char *chunk_b, size_t len_b, char **joined_out);

/**
 * @brief Apply joined text to textarea, reset dirty/original, and update buttons.
 *
 * @param ctx   Viewer context.
 * @param joined Joined text buffer.
 */
static void apply_joined_window(text_viewer_ctx_t *ctx, const char *joined);
/**
 * @brief Perform the full save flow: open streams, copy prefix/body/suffix, rename temp.
 *
 * @param ctx            Viewer context.
 * @param text           Text to write.
 * @param tmp_path       Temporary file path.
 * @param have_existing  True if destination file exists.
 * @param window_start   Byte offset where the current window begins.
 * @param window_end     Byte offset right after the current window.
 * @param prefix_size    Bytes to copy before current window.
 * @param suffix_start   Offset where suffix begins.
 * @param suffix_size    Bytes to copy after current window.
 * @return true on success; false on any error.
 */
static bool file_save_success(text_viewer_ctx_t *ctx, const char *text, const char *tmp_path, bool have_existing,
                              size_t window_start, size_t window_end,
                              size_t prefix_size, size_t suffix_start, size_t suffix_size);

/**
 * @brief Check whether scroll handling should be skipped (e.g., blocked).
 *
 * @param ctx Viewer context.
 * @return true if scroll events should be ignored.
 */
static bool text_scroll_should_ignore(const text_viewer_ctx_t *ctx);

/**
 * @brief Handle reaching/leaving the top edge of the text area.
 *
 * @param ctx    Viewer context.
 * @param at_top True if currently at the top.
 */
static void handle_scroll_top_edge(text_viewer_ctx_t *ctx, bool at_top);

/**
 * @brief Handle reaching/leaving the bottom edge of the text area.
 *
 * @param ctx       Viewer context.
 * @param at_bottom True if currently at the bottom.
 */
static void handle_scroll_bottom_edge(text_viewer_ctx_t *ctx, bool at_bottom);

/**
 * @brief Validate save preconditions and return textarea text.
 *
 * Checks SD wait, missing name/new file name dialog, and empty path.
 *
 * @param ctx      Viewer context.
 * @param[out] text_out Pointer to textarea text (never NULL on success).
 * @return true if save can proceed; false otherwise.
 */
static bool save_prechecks(text_viewer_ctx_t *ctx, const char **text_out);

/**
 * @brief Compute save window offsets and sizes based on current chunks.
 *
 * Computes byte window, clamps to file size, and reports prefix/suffix sizes.
 *
 * @param ctx           Viewer context.
 * @param[out] window_start Start offset in bytes.
 * @param[out] window_end   End offset in bytes.
 * @param[out] prefix_size  Bytes before window.
 * @param[out] suffix_start Start of suffix in bytes.
 * @param[out] suffix_size  Bytes after window.
 * @param[out] have_existing True if destination file exists.
 * @param[out] file_size     Destination file size in bytes.
 * @return true on success; false on overflow/error.
 */
static bool compute_save_window(text_viewer_ctx_t *ctx, size_t *window_start, size_t *window_end,
                                size_t *prefix_size, size_t *suffix_start, size_t *suffix_size,
                                bool *have_existing, size_t *file_size);

/**
 * @brief Build temp path for save (same dir) and clear stale file.
 *
 * @param dest_path Destination file path.
 * @param tmp_path  Output temp path buffer.
 * @param tmp_size  Size of temp buffer.
 * @param ctx       Viewer context (for status).
 * @return true on success; false on overflow.
 */
static bool build_temp_path_for_save(const char *dest_path, char *tmp_path, size_t tmp_size, text_viewer_ctx_t *ctx);

/**
 * @brief Open source (if exists) and temp streams for save.
 *
 * @param dest_path Destination path.
 * @param tmp_path  Temp path.
 * @param have_existing True if destination exists.
 * @param[out] src_out  Opened source FILE* (nullable).
 * @param[out] tmp_out  Opened temp FILE*.
 * @param ctx       Viewer context (for status/retry).
 * @return true on success; false otherwise.
 */
static bool open_save_streams(const char *dest_path, const char *tmp_path, bool have_existing,
                              FILE **src_out, FILE **tmp_out, text_viewer_ctx_t *ctx);

/**
 * @brief Copy prefix bytes from src to temp.
 *
 * @param src       Source FILE* (nullable when prefix_size==0).
 * @param tmp       Temp FILE*.
 * @param prefix_size Bytes to copy.
 * @param dest_path Destination path (for logging).
 * @param tmp_path  Temp path (for logging).
 * @param ctx       Viewer context.
 * @return true on success; false on IO error.
 */
static bool copy_prefix(FILE *src, FILE *tmp, size_t prefix_size, const char *dest_path, const char *tmp_path, text_viewer_ctx_t *ctx);

/**
 * @brief Write textarea body to temp file.
 *
 * @param tmp      Temp FILE*.
 * @param text     Text content.
 * @param tmp_path Temp path (for logging).
 * @param ctx      Viewer context.
 * @return true on success; false on IO error.
 */
static bool write_body(FILE *tmp, const char *text, const char *tmp_path, text_viewer_ctx_t *ctx);

/**
 * @brief Copy suffix bytes from src to temp.
 *
 * @param src        Source FILE*.
 * @param tmp        Temp FILE*.
 * @param suffix_start Offset to seek.
 * @param suffix_size  Bytes to copy.
 * @param dest_path   Destination path (for logging).
 * @param tmp_path    Temp path (for logging).
 * @param ctx         Viewer context.
 * @return true on success; false on IO error.
 */
static bool copy_suffix(FILE *src, FILE *tmp, size_t suffix_start, size_t suffix_size,
                        const char *dest_path, const char *tmp_path, text_viewer_ctx_t *ctx);

/**
 * @brief Rename temp file over destination with error handling.
 *
 * @param tmp_path  Temp path.
 * @param dest_path Destination path.
 * @param ctx       Viewer context.
 * @return true on success; false on failure.
 */
static bool finalize_rename(const char *tmp_path, const char *dest_path, text_viewer_ctx_t *ctx);

/**
 * @brief Update in-memory state after successful save.
 *
 * @param ctx         Viewer context.
 * @param text        Saved text.
 * @param prefix_size Prefix bytes length.
 * @param suffix_size Suffix bytes length.
 */
static void finalize_save_state(text_viewer_ctx_t *ctx, const char *text, size_t prefix_size, size_t suffix_size);

/**
 * @brief Close FILE*s and optionally remove temp path.
 *
 * @param src      Source FILE* (nullable).
 * @param tmp      Temp FILE* (nullable).
 * @param tmp_path Temp path to remove (nullable).
 */
static void cleanup_save_files(FILE *src, FILE *tmp, const char *tmp_path);

/**
 * @brief Handle slider press/drag/release to jump between chunk windows.
 *
 * Tracks the target step while dragging and applies the chunk load on release; no-ops
 * if blocked or if the knob returns to the current step.
 *
 * Events:
 * - LV_EVENT_PRESSED: record drag start unless blocked.
 * - LV_EVENT_VALUE_CHANGED: update pending step while dragging (if not blocked).
 * - LV_EVENT_RELEASED/PRESS_LOST: trigger chunk load or reset if blocked.
 *
 * @param e LVGL slider event with user data = text_viewer_ctx_t*.
 */
static void on_slider_value_changed(lv_event_t *e);

/**
 * @brief Route slider events (press/drag/release) to update pending step or load chunks.
 *
 * Delegates to helper for release; tracks drag state and pending step on press/value change.
 *
 * @param e             LVGL slider event.
 * @param ctx           Viewer context.
 * @param window_size   Chunks per window.
 * @param step          Chunks per step.
 * @param total_chunks  Total chunks available.
 * @param max_start     Maximum starting chunk index.
 * @param max_step_index Maximum slider step index.
 */
static void handle_slider_event(lv_event_t *e, text_viewer_ctx_t *ctx, size_t window_size, size_t step, size_t total_chunks, size_t max_start, size_t max_step_index);

/**
 * @brief Load two consecutive chunks into the textarea and position the cursor at the boundary.
 *
 * @param ctx             Viewer context.
 * @param first_offset_kb Offset (KB) of the first chunk.
 * @param second_offset_kb Offset (KB) of the second chunk.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t load_window(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb);

/**
 * @brief Enable/disable the Save button based on @c editable and @c dirty.
 *
 * @param ctx Viewer context.
 */
static void update_buttons(text_viewer_ctx_t *ctx);

/*********************************************************************************************/

/******************************* Keyboard & interaction helpers ******************************/

/**
 * @brief Explicitly show the keyboard in edit mode.
 *
 * @param ctx    Viewer context.
 * @param target Text area to focus (falls back to main text area if NULL).
 */
static void show_keyboard(text_viewer_ctx_t *ctx, lv_obj_t *target);

/**
 * @brief Explicitly hide the keyboard and detach it from the textarea.
 *
 * @param ctx Viewer context.
 */
static void hide_keyboard(text_viewer_ctx_t *ctx);

/**
 * @brief Show the keyboard when the text area is tapped in edit mode.
 *
 * @param e LVGL event.
 */
static void on_text_area_clicked(lv_event_t *e);
/**
 * @brief Immediately jump the text area's scroll to the cursor position.
 *
 * Skips LVGL's default smooth scroll animation and forces the text area
 * to scroll instantly to the final cursor location. Useful when loading
 * a new text chunk and repositioning the cursor manually.
 *
 * @param ctx Pointer to the text viewer context. Must not be NULL.
 */
static void skip_cursor_animation(text_viewer_ctx_t *ctx);

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
static void on_text_scrolled(lv_event_t *e);

/**
 * @brief Hide the keyboard when its cancel/close button is pressed.
 *
 * @param e LVGL event.
 */
static void on_keyboard_cancel(lv_event_t *e);

/**
 * @brief Show the keyboard when the filename dialog textarea is tapped.
 *
 * @param e LVGL event.
 */
static void on_name_textarea_clicked(lv_event_t *e);

/**
 * @brief Trigger Save when the keyboard's OK button is pressed.
 *
 * @param e LVGL event.
 */
static void on_keyboard_ready(lv_event_t *e);

/**
 * @brief Hide the keyboard when tapping outside editable widgets.
 *
 * @param e LVGL event.
 */
static void on_screen_clicked(lv_event_t *e);

/*********************************************************************************************/

/************************************** Editing workflow *************************************/

/**
 * @brief Text change handler: updates @c dirty, Save button, and status.
 *
 * Ignored when not editable or when @c suppress_events is true.
 *
 * @param e LVGL event.
 */
static void on_text_changed(lv_event_t *e);

/**
 * @brief Save the currently loaded text chunk back to the underlying file.
 *
 * This function writes the contents of the LVGL textarea in @p ctx->graphics.text_area
 * into the backing file at @p ctx->path, only within the byte window
 * corresponding to the currently loaded chunks (defined by
 * ctx->last_file_offset_kb and ctx->current_file_offset_kb).
 *
 * Save strategy:
 * - If @p ctx is NULL, the function returns immediately.
 * - If this is a new file with no name yet, a name dialog is shown and
 *   the function returns without writing.
 * - If the file name is still missing, a "Missing file name" status is set.
 * - Computes a byte window [window_start, window_end) for the loaded text
 *   (based on chunk offsets and FS_TEXT_READ_CHUNK_SIZE_B), with overflow checks.
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
 *      - Sets a human-readable status string via set_status().
 *      - Logs an error via ESP_LOGE().
 *      - Calls sd_card_schedule_retry() to trigger SD-card recovery logic.
 *      - Cleans up open FILE handles and removes the temporary file.
 *
 * On success:
 * - Updates the "original" text snapshot via set_original().
 * - Clears @p ctx->flags.dirty (sets it to false).
 * - Sets status to "Saved".
 *
 * @param ctx Pointer to the text viewer context. May be NULL, in which case
 *            the function returns immediately without side effects.
 */
static void handle_save(text_viewer_ctx_t *ctx);

/**
 * @brief "Save" button event handler.
 *
 * @param e LVGL event.
 */
static void on_save(lv_event_t *e);

/**
 * @brief "Back" button handler: closes the screen or prompts to save/discard.
 *
 * @param e LVGL event.
 */
static void on_back(lv_event_t *e);

/**
 * @brief Show prompt before changing chunk when dirty.
 *
 * @param ctx Viewer context.
 */
static void show_chunk_prompt(text_viewer_ctx_t *ctx);

/**
 * @brief Close chunk-change prompt if present.
 *
 * @param ctx Viewer context.
 */
static void close_chunk_prompt(text_viewer_ctx_t *ctx);

/**
 * @brief Clear pending-chunk flags and slider state.
 *
 * Resets edge flags, clears pending flag, and refreshes slider.
 *
 * @param ctx Viewer context.
 */
static void clear_pending_chunk_state(text_viewer_ctx_t *ctx);

/**
 * @brief Handle the "Save" choice from the chunk prompt.
 *
 * Triggers save, applies pending chunk if clean, or clears pending state when appropriate.
 *
 * @param ctx Viewer context.
 */
static void handle_chunk_prompt_save(text_viewer_ctx_t *ctx);

/**
 * @brief Handle the "Discard" choice from the chunk prompt.
 *
 * Clears dirty flag, updates buttons, and applies the pending chunk.
 *
 * @param ctx Viewer context.
 */
static void handle_chunk_prompt_discard(text_viewer_ctx_t *ctx);

/**
 * @brief Handle the "Cancel" choice from the chunk prompt.
 *
 * Cancels pending chunk and resets edge/slider state.
 *
 * @param ctx Viewer context.
 */
static void handle_chunk_prompt_cancel(text_viewer_ctx_t *ctx);

/**
 * @brief Check whether a pending chunk can be applied now.
 *
 * @param ctx Viewer context.
 * @return true if a pending chunk exists and SD is available; false otherwise.
 */
static bool should_apply_pending_chunk(const text_viewer_ctx_t *ctx);

/**
 * @brief Position cursor and skip animation after loading a pending chunk.
 *
 * @param ctx        Viewer context.
 * @param content_h  Current textarea content height.
 */
static void update_cursor_after_chunk(text_viewer_ctx_t *ctx, lv_coord_t content_h);

/**
 * @brief Finalize state after successfully applying a pending chunk.
 *
 * Updates offsets, clears edge flags, refreshes slider, and clears the pending flag.
 *
 * @param ctx Viewer context.
 */
static void finalize_pending_chunk_success(text_viewer_ctx_t *ctx);

/**
 * @brief Handle failure when applying a pending chunk.
 *
 * Logs the error, schedules SD retry, and clears edge flags.
 *
 * @param ctx Viewer context.
 * @param err Error code returned by load_window.
 */
static void handle_pending_chunk_failure(text_viewer_ctx_t *ctx, esp_err_t err);

/**
 * @brief Handle chunk-change prompt buttons.
 *
 * @param e LVGL event.
 */
static void on_chunk_prompt(lv_event_t *e);

/**
 * @brief Apply a pending chunk load after save/discard decision.
 *
 * @param ctx Viewer context.
 */
static void apply_pending_chunk(text_viewer_ctx_t *ctx);

/**
 * @brief Schedule loading a new chunk window (with optional prompt if dirty).
 *
 * @param ctx Viewer context.
 * @param first_offset_kb First chunk offset to load.
 * @param second_offset_kb Second chunk offset to load.
 * @param from_top True if triggered from top edge scroll.
 */
static void request_chunk_load(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb, bool from_top);

/**
 * @brief Poll SD reconnection and retry pending actions.
 *
 * @param timer LVGL timer.
 */
static void on_sd_retry_timer(lv_timer_t *timer);

/**
 * @brief Schedule SD reconnection prompt and retry logic.
 *
 * @param ctx Viewer context.
 * @param action Action to retry after reconnection.
 */
static void schedule_sd_retry(text_viewer_ctx_t *ctx, text_viewer_sd_action_t action);

/**
 * @brief Check whether the SD reconnection semaphore is ready.
 *
 * @param ctx Viewer context.
 * @return true if reconnection was confirmed; false otherwise.
 */
static bool sd_reconnect_ready(text_viewer_ctx_t *ctx);

/**
 * @brief Perform the pending SD action after reconnection.
 *
 * @param ctx    Viewer context.
 * @param action Pending action to resume.
 */
static void perform_sd_retry_action(text_viewer_ctx_t *ctx, text_viewer_sd_action_t action);

/*********************************************************************************************/

/**
 * @brief Validate candidate filename (must end with .txt and contain safe chars).
 *
 * @param name Null-terminated candidate name.
 * @return true if the name is non-empty, contains no invalid characters, and ends with ".txt".
 */
static bool validate_name(const char *name);

/**
 * @brief Whether a dot pointer references an existing non-empty ".txt" extension.
 *
 * @param dot Pointer to the last '.' in the string (may be NULL).
 * @return true if @p dot points to a ".txt" suffix with at least one character after the dot.
 */
static bool has_txt_extension(const char *dot);

/**
 * @brief Whether a dot pointer references a non-empty extension different from ".txt".
 *
 * @param dot Pointer to the last '.' in the string (may be NULL).
 * @return true if @p dot points to a non-empty extension that is not ".txt".
 */
static bool has_other_extension(const char *dot);

/**
 * @brief Check if there is enough buffer space to append ".txt".
 *
 * @param current_len Current string length (excluding terminator).
 * @param buf_len     Total buffer capacity (including terminator).
 * @return true if ".txt" can be appended without overflow.
 */
static bool can_append_txt(size_t current_len, size_t buf_len);

/**
 * @brief Ensure \".txt\" suffix (adds or fixes trailing dot cases).
 *
 * @param name Buffer containing the filename to update (in-place).
 * @param len  Total size of @p name buffer.
 */
static void ensure_txt_extension(char *name, size_t len);

/**
 * @brief Compose absolute path for a new file.
 *
 * @param ctx     Viewer context (requires valid directory).
 * @param name    Filename to append.
 * @param out     Destination buffer for resulting path.
 * @param out_len Size of @p out in bytes.
 * @return ESP_OK on success or ESP_ERR_* on failure.
 */
static esp_err_t compose_new_path(text_viewer_ctx_t *ctx, const char *name, char *out, size_t out_len);

/**
 * @brief Check if a filesystem path already exists.
 *
 * @param path Absolute path to test.
 * @return true if the path exists, false otherwise.
 */
static bool path_exists(const char *path);

/**
 * @brief Show the overwrite/cancel dialog when the chosen new filename already exists.
 *
 * Caches the conflicting name/path and presents a modal dialog that lets the user
 * either overwrite the existing file or cancel and keep editing the name dialog.
 *
 * @param ctx  Viewer context.
 * @param path Full destination path that already exists.
 * @param name Filename (without path) that conflicts.
 */
static void show_save_conflict(text_viewer_ctx_t *ctx, const char *path, const char *name);

/**
 * @brief Close and clear any active save conflict dialog/state.
 *
 * Destroys the modal conflict message box (if present) and clears the cached
 * conflicting path/name so stale data is not reused.
 *
 * @param ctx Viewer context.
 */
static void close_save_conflict(text_viewer_ctx_t *ctx);

/**
 * @brief Event handler for the save conflict dialog buttons.
 *
 * Handles Overwrite or Cancel: overwrite applies the cached path/name and triggers
 * save; cancel leaves the name dialog open and refocuses the keyboard for renaming.
 *
 * @param e LVGL event carrying the button user data choice.
 */
static void on_save_conflict(lv_event_t *e);

/**
 * @brief Apply a validated new file name/path to the viewer state.
 *
 * Stores the chosen name/path, exits "new file" mode, updates the displayed path,
 * and closes the name dialog prior to saving.
 *
 * @param ctx       Viewer context.
 * @param name      Filename selected by the user.
 * @param full_path Absolute path composed from directory + name.
 */
static void apply_new_file_path(text_viewer_ctx_t *ctx, const char *name, const char *full_path);

/**
 * @brief Show the filename dialog used when saving a new file.
 *
 * @param ctx Viewer context.
 */
static void show_name_dialog(text_viewer_ctx_t *ctx);

/**
 * @brief Close the filename dialog (if present) and restore edit state.
 *
 * @param ctx Viewer context.
 */
static void close_name_dialog(text_viewer_ctx_t *ctx);

/**
 * @brief Confirm the filename dialog as if pressing Save.
 *
 * Validates, composes the path, and triggers the save flow.
 *
 * @param ctx Viewer context.
 * @return true on success, false if validation failed.
 */
static bool confirm_name_dialog(text_viewer_ctx_t *ctx);

/**
 * @brief Filename dialog button handler.
 *
 * @param e LVGL event.
 */
static void on_name_dialog(lv_event_t *e);

/*********************************************************************************************/

/************************************ Confirmation dialog ************************************/

/**
 * @brief Show the save/discard/cancel confirmation dialog.
 *
 * No-op if a dialog is already open.
 *
 * @param ctx Viewer context.
 */
static void show_confirm(text_viewer_ctx_t *ctx);

/**
 * @brief Check if a target object is inside (or is) a given parent object.
 *
 * @param parent  The LVGL object considered as the potential ancestor.
 * @param target  The LVGL object whose ancestry is checked.
 */
static bool target_in(lv_obj_t *parent, lv_obj_t *target);

/**
 * @brief Close and clear the confirmation dialog if present.
 *
 * @param ctx Viewer context.
 */
static void close_confirm(text_viewer_ctx_t *ctx);

/**
 * @brief Confirmation dialog button handler (Save / Discard / Cancel).
 *
 * @param e LVGL event.
 */
static void on_confirm(lv_event_t *e);

/**
 * @brief Close the viewer, unload the screen, and invoke the close callback.
 *
 * Resets mode and frees resources (keyboard target, original snapshot).
 *
 * @param ctx     Viewer context.
 * @param changed True if file content was saved/changed (passed to callback).
 */
static void close_ctx(text_viewer_ctx_t *ctx, bool changed);

/**
 * @brief Validate open options and determine whether this is a new file.
 *
 * @param opts         Open options (must not be NULL).
 * @param out_new_file Output flag set to true when creating a new file.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on validation failure.
 */
static esp_err_t validate_open_opts(const text_viewer_open_opts_t *opts, bool *out_new_file);

/**
 * @brief Load initial content and chunk offsets for the viewer.
 *
 * For new files, allocates an empty string. For existing files, reads the first
 * and (optionally) second chunk and concatenates them.
 *
 * @param opts             Open options.
 * @param new_file         Whether this is a new file.
 * @param[out] content_out Allocated text buffer (caller frees).
 * @param[out] file_size_kb File size in KB.
 * @param[out] first_offset_kb  First chunk offset.
 * @param[out] second_offset_kb Second chunk offset (may equal first).
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t load_initial_content(const text_viewer_open_opts_t *opts, bool new_file, char **content_out,
                                      size_t *file_size_kb, size_t *first_offset_kb, size_t *second_offset_kb);

/**
 * @brief Initialize empty content buffer for a new file.
 *
 * Allocates an empty string and assigns it to @p content_out.
 *
 * @param[out] content_out Allocated empty string (caller frees).
 * @return ESP_OK on success; ESP_ERR_NO_MEM on allocation failure.
 */
static esp_err_t init_empty_content(char **content_out);

/**
 * @brief Compute file size metadata and second chunk offset.
 *
 * Stat the file to derive size in KB and set second chunk offset (1 KB when size>0).
 *
 * @param opts Open options containing the path.
 * @param[out] file_size_kb   File size in KB.
 * @param[out] second_offset_kb Second chunk offset (0 or 1).
 */
static void compute_file_metadata(const text_viewer_open_opts_t *opts, size_t *file_size_kb, size_t *second_offset_kb);

/**
 * @brief Read the first (and optional second) text chunks from file.
 *
 * Reads chunk 0 and, if @p second_offset_kb != 0, reads that chunk too.
 *
 * @param path             File path.
 * @param second_offset_kb Second chunk offset (may be 0).
 * @param[out] chunk_a     First chunk buffer.
 * @param[out] len_a       Length of first chunk.
 * @param[out] chunk_b     Second chunk buffer (optional).
 * @param[out] len_b       Length of second chunk.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t read_initial_chunks(const char *path, size_t second_offset_kb, char **chunk_a, size_t *len_a,
                                     char **chunk_b, size_t *len_b);

/**
 * @brief Concatenate up to two chunks into a single buffer.
 *
 * Allocates a new buffer of len_a+len_b+1, copies both chunks, and null-terminates.
 *
 * @param chunk_a     First chunk (nullable if len_a==0).
 * @param len_a       Length of first chunk.
 * @param chunk_b     Second chunk (nullable if len_b==0).
 * @param len_b       Length of second chunk.
 * @param[out] content_out Output concatenated buffer (caller frees).
 * @return ESP_OK on success; ESP_ERR_NO_MEM on allocation failure.
 */
static esp_err_t join_chunks(char *chunk_a, size_t len_a, char *chunk_b, size_t len_b, char **content_out);

/**
 * @brief Initialize viewer context state before showing the screen.
 *
 * Resets flags, assigns callbacks, offsets, and path/directory labels.
 *
 * @param ctx              Viewer context.
 * @param opts             Open options.
 * @param new_file         Whether this is a new file.
 * @param first_offset_kb  First chunk offset.
 * @param second_offset_kb Second chunk offset.
 * @param file_size_kb     File size in KB.
 */
static void init_viewer_context(text_viewer_ctx_t *ctx, const text_viewer_open_opts_t *opts, bool new_file,
                                size_t first_offset_kb, size_t second_offset_kb, size_t file_size_kb);

/*********************************************************************************************/

esp_err_t text_viewer_open(const text_viewer_open_opts_t *opts)
{
    bool new_file = false;
    esp_err_t err = validate_open_opts(opts, &new_file);
    if (err != ESP_OK) return err;

    char *content = NULL;
    size_t file_size_kb = 0;
    size_t first_offset_kb = 0;
    size_t second_offset_kb = 0;
    err = load_initial_content(opts, new_file, &content, &file_size_kb, &first_offset_kb, &second_offset_kb);
    if (err != ESP_OK) return err;

    text_viewer_ctx_t *ctx = &s_viewer;
    if (!ctx->graphics.screen)
    {
        build_screen(ctx);
    }

    init_viewer_context(ctx, opts, new_file, first_offset_kb, second_offset_kb, file_size_kb);

    lv_textarea_set_text(ctx->graphics.text_area, content);
    set_original(ctx, content);
    free(content);
    ctx->flags.suppress_events = false;
    if (ctx->flags.new_file)
    {
        set_status(ctx, "New TXT");
    }
    else
    {
        set_status(ctx, ctx->flags.editable ? "Edit mode" : "View mode");
    }
    apply_mode(ctx);
    update_slider(ctx);
    lv_screen_load(ctx->graphics.screen);
    if (ctx->flags.new_file)
    {
        lv_textarea_set_cursor_pos(ctx->graphics.text_area, 0);
        lv_obj_add_state(ctx->graphics.text_area, LV_STATE_FOCUSED);
        show_keyboard(ctx, ctx->graphics.text_area);
    }
    return ESP_OK;
}

static esp_err_t validate_open_opts(const text_viewer_open_opts_t *opts, bool *out_new_file)
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

    if (out_new_file)
    {
        *out_new_file = new_file;
    }
    return ESP_OK;
}

static esp_err_t load_initial_content(const text_viewer_open_opts_t *opts, bool new_file, char **content_out,
                                      size_t *file_size_kb, size_t *first_offset_kb, size_t *second_offset_kb)
{
    if (!content_out || !file_size_kb || !first_offset_kb || !second_offset_kb)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *first_offset_kb = 0;
    *second_offset_kb = 0;
    *file_size_kb = 0;

    if (new_file)
    {
        return init_empty_content(content_out);
    }

    char *chunk_a = NULL;
    char *chunk_b = NULL;
    size_t len_a = 0;
    size_t len_b = 0;
    compute_file_metadata(opts, file_size_kb, second_offset_kb);

    esp_err_t err = read_initial_chunks(opts->path, *second_offset_kb, &chunk_a, &len_a, &chunk_b, &len_b);
    if (err != ESP_OK)
    {
        free(chunk_a);
        free(chunk_b);
        return err;
    }

    err = join_chunks(chunk_a, len_a, chunk_b, len_b, content_out);

    free(chunk_a);
    free(chunk_b);
    return err;
}

static esp_err_t init_empty_content(char **content_out)
{
    *content_out = strdup("");
    return *content_out ? ESP_OK : ESP_ERR_NO_MEM;
}

static void compute_file_metadata(const text_viewer_open_opts_t *opts, size_t *file_size_kb, size_t *second_offset_kb)
{
    struct stat st = {0};
    if (stat(opts->path, &st) == 0 && S_ISREG(st.st_mode))
    {
        *file_size_kb = (st.st_size > 0) ? ((size_t)st.st_size - 1u) / 1024u : 0;
    }
    else
    {
        *file_size_kb = 0;
    }
    *second_offset_kb = (*file_size_kb > 0) ? 1 : 0;
}

static esp_err_t read_initial_chunks(const char *path, size_t second_offset_kb, char **chunk_a, size_t *len_a,
                                     char **chunk_b, size_t *len_b)
{
    esp_err_t err = fs_text_read_range(path, 0, chunk_a, len_a);
    if (err != ESP_OK)
    {
        return err;
    }

    if (second_offset_kb != 0)
    {
        err = fs_text_read_range(path, second_offset_kb, chunk_b, len_b);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t join_chunks(char *chunk_a, size_t len_a, char *chunk_b, size_t len_b, char **content_out)
{
    size_t total = len_a + len_b;
    char *content = (char *)malloc(total + 1);
    if (!content)
    {
        return ESP_ERR_NO_MEM;
    }

    if (len_a && chunk_a)
    {
        memcpy(content, chunk_a, len_a);
    }
    if (len_b && chunk_b)
    {
        memcpy(content + len_a, chunk_b, len_b);
    }
    content[total] = '\0';

    *content_out = content;
    return ESP_OK;
}

static void init_viewer_context(text_viewer_ctx_t *ctx, const text_viewer_open_opts_t *opts, bool new_file,
                                size_t first_offset_kb, size_t second_offset_kb, size_t file_size_kb)
{
    close_confirm(ctx);
    ctx->flags.active = true;
    ctx->flags.editable = new_file ? true : opts->editable;
    ctx->flags.new_file = new_file;
    ctx->flags.dirty = new_file ? true : false;
    ctx->flags.suppress_events = true;
    ctx->graphics.return_screen = opts->return_screen;
    ctx->close_cb = opts->on_close;
    ctx->close_ctx = opts->user_ctx;

    ctx->current_file_offset_kb = second_offset_kb;
    ctx->last_file_offset_kb = first_offset_kb;
    ctx->max_file_offset_kb = file_size_kb;

    ctx->graphics.name_dialog = NULL;
    ctx->graphics.name_textarea = NULL;
    ctx->graphics.chunk_mbox = NULL;
    ctx->graphics.save_conflict_mbox = NULL;
    ctx->graphics.sd_retry_timer = NULL;
    ctx->flags.at_top_edge = false;
    ctx->flags.at_bottom_edge = false;
    ctx->flags.pending_chunk = false;
    ctx->pending_first_offset_kb = 0;
    ctx->pending_second_offset_kb = 0;
    ctx->flags.pending_scroll_up = false;
    ctx->flags.waiting_sd = false;
    ctx->sd_retry_action = TEXT_VIEWER_SD_NONE;
    ctx->flags.content_changed = false;
    ctx->flags.slider_suppress_change = false;
    ctx->flags.slider_drag_active = false;
    ctx->slider_pending_step = SIZE_MAX;
    ctx->conflict_path[0] = '\0';
    ctx->conflict_name[0] = '\0';

    if (new_file)
    {
        ctx->path[0] = '\0';
        strlcpy(ctx->directory, opts->directory, sizeof(ctx->directory));
        strlcpy(ctx->pending_name, ".txt", sizeof(ctx->pending_name));
        set_path_label(ctx, ctx->directory);
    }
    else
    {
        ctx->directory[0] = '\0';
        ctx->pending_name[0] = '\0';
        strlcpy(ctx->path, opts->path, sizeof(ctx->path));
        set_path_label(ctx, ctx->path);
    }
}

static void build_screen(text_viewer_ctx_t *ctx)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 2, 0);
    lv_obj_set_style_pad_gap(scr, 5, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, on_screen_clicked, LV_EVENT_CLICKED, ctx);
    ctx->graphics.screen = scr;

    lv_obj_t *toolbar = lv_obj_create(scr);
    lv_obj_remove_style_all(toolbar);
    lv_obj_set_size(toolbar, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(toolbar, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(toolbar, 3, 0);
    lv_obj_set_flex_align(toolbar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    styles_set_card_color(toolbar, 0);
    lv_obj_set_style_bg_opa(toolbar, LV_OPA_COVER, 0);
    ctx->graphics.toolbar = toolbar;

    lv_obj_t *back_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(back_btn, 6, 0);
    lv_obj_set_style_pad_all(back_btn, 6, 0);   
    styles_set_button(back_btn);  
    lv_obj_add_event_cb(back_btn, on_back, LV_EVENT_CLICKED, ctx);
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_center(back_lbl);

    ctx->graphics.save_btn = lv_button_create(toolbar);
    lv_obj_set_style_radius(ctx->graphics.save_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->graphics.save_btn, 6, 0);     
    styles_set_button(ctx->graphics.save_btn);   
    lv_obj_add_event_cb(ctx->graphics.save_btn, on_save, LV_EVENT_CLICKED, ctx);
    lv_obj_t *save_lbl = lv_label_create(ctx->graphics.save_btn);
    lv_label_set_text(save_lbl, LV_SYMBOL_SAVE " Save");
    lv_obj_center(save_lbl);

    lv_obj_t *status_spacer_left = lv_obj_create(toolbar);
    lv_obj_remove_style_all(status_spacer_left);
    lv_obj_set_flex_grow(status_spacer_left, 1);
    lv_obj_set_height(status_spacer_left, 1);

    ctx->graphics.status_label = lv_label_create(toolbar);
    lv_label_set_text(ctx->graphics.status_label, "");
    lv_label_set_long_mode(ctx->graphics.status_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(ctx->graphics.status_label, LV_TEXT_ALIGN_CENTER, 0);
    styles_set_text_color(ctx->graphics.status_label, 0);
    lv_obj_set_style_text_font(ctx->graphics.status_label, &Domine_16, 0);
    const lv_font_t *status_font = lv_obj_get_style_text_font(ctx->graphics.status_label, LV_PART_MAIN);
    lv_coord_t status_height = status_font ? status_font->line_height : 18;
    lv_obj_set_style_min_height(ctx->graphics.status_label, status_height, 0);
    lv_obj_set_style_max_height(ctx->graphics.status_label, status_height, 0);

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

    ctx->graphics.path_label = lv_label_create(path_row);
    lv_label_set_long_mode(ctx->graphics.path_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_flex_grow(ctx->graphics.path_label, 1);
    lv_obj_set_width(ctx->graphics.path_label, LV_PCT(100));
    lv_obj_set_style_text_align(ctx->graphics.path_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(ctx->graphics.path_label, "");

    lv_coord_t slider_gap = 6;

    lv_obj_t *text_row = lv_obj_create(scr);
    lv_obj_remove_style_all(text_row);
    lv_obj_set_size(text_row, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(text_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(text_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(text_row, slider_gap, 0);
    lv_obj_set_style_pad_right(text_row, slider_gap, 0);
    lv_obj_set_flex_grow(text_row, 1);    

    ctx->graphics.text_area = lv_textarea_create(text_row);
    lv_obj_set_flex_grow(ctx->graphics.text_area, 1);
    lv_obj_set_height(ctx->graphics.text_area, LV_PCT(100));
    lv_obj_set_style_pad_all(ctx->graphics.text_area, 0, 0);
    styles_set_textarea(ctx->graphics.text_area);
    lv_textarea_set_cursor_click_pos(ctx->graphics.text_area, false);
    lv_obj_set_scrollbar_mode(ctx->graphics.text_area, LV_SCROLLBAR_MODE_AUTO);
    styles_set_textarea(ctx->graphics.text_area);
    lv_obj_add_event_cb(ctx->graphics.text_area, on_text_changed, LV_EVENT_VALUE_CHANGED, ctx);
    lv_obj_add_event_cb(ctx->graphics.text_area, on_text_area_clicked, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.text_area, on_text_scrolled, LV_EVENT_SCROLL, ctx);
    
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
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_PRESSED, ctx);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_VALUE_CHANGED, ctx);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_RELEASED, ctx);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_PRESS_LOST, ctx);
    lv_obj_clear_flag(list_slider, LV_OBJ_FLAG_SCROLL_CHAIN);
    ctx->graphics.chunk_slider = list_slider;
    
    ctx->graphics.keyboard = lv_keyboard_create(scr);
    styles_set_keyboard(ctx->graphics.keyboard);
    lv_keyboard_set_textarea(ctx->graphics.keyboard, ctx->graphics.text_area);
    lv_obj_add_flag(ctx->graphics.keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(ctx->graphics.keyboard, on_keyboard_cancel, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->graphics.keyboard, on_keyboard_ready, LV_EVENT_READY, ctx);
}

static void apply_mode(text_viewer_ctx_t *ctx)
{
    if (ctx->flags.editable)
    {
        lv_obj_clear_state(ctx->graphics.text_area, LV_STATE_DISABLED);
        lv_textarea_set_cursor_click_pos(ctx->graphics.text_area, true);
        lv_obj_add_flag(ctx->graphics.text_area, LV_OBJ_FLAG_CLICK_FOCUSABLE);
        hide_keyboard(ctx);
        lv_obj_clear_flag(ctx->graphics.save_btn, LV_OBJ_FLAG_HIDDEN);
        lv_textarea_set_cursor_pos(ctx->graphics.text_area, 0);
    }
    else
    {
        lv_textarea_set_cursor_click_pos(ctx->graphics.text_area, false);
        lv_obj_clear_flag(ctx->graphics.text_area, LV_OBJ_FLAG_CLICK_FOCUSABLE);
        hide_keyboard(ctx);
        lv_obj_add_flag(ctx->graphics.save_btn, LV_OBJ_FLAG_HIDDEN);
        lv_textarea_clear_selection(ctx->graphics.text_area);
        lv_textarea_set_cursor_pos(ctx->graphics.text_area, 0);
    }
    lv_obj_scroll_to_y(ctx->graphics.text_area, 0, LV_ANIM_OFF);
    update_buttons(ctx);
}

static void set_status(text_viewer_ctx_t *ctx, const char *msg)
{
    if (ctx->graphics.status_label && msg)
    {
        lv_label_set_text(ctx->graphics.status_label, msg);
    }
}

static void set_path_label(text_viewer_ctx_t *ctx, const char *path)
{
    if (!ctx || !ctx->graphics.path_label)
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

    lv_label_set_text(ctx->graphics.path_label, display);
    restart_path_scroll(ctx);
}

static void path_scroll_timer_cb(lv_timer_t *timer)
{
    text_viewer_ctx_t *ctx = (text_viewer_ctx_t *)lv_timer_get_user_data(timer);
    if (ctx) {
        ctx->graphics.path_scroll_timer = NULL;
        if (ctx->graphics.path_label && lv_obj_is_valid(ctx->graphics.path_label)) {
            lv_label_set_long_mode(ctx->graphics.path_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        }
    }
    lv_timer_del(timer);
}

static void restart_path_scroll(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.path_label) {
        return;
    }

    if (ctx->graphics.path_scroll_timer) {
        lv_timer_del(ctx->graphics.path_scroll_timer);
        ctx->graphics.path_scroll_timer = NULL;
    }

    /* Start clipped, then enable scroll after a short delay. */
    lv_label_set_long_mode(ctx->graphics.path_label, LV_LABEL_LONG_CLIP);
    ctx->graphics.path_scroll_timer = lv_timer_create(path_scroll_timer_cb, TEXT_VIEWER_PATH_SCROLL_DELAY_MS, ctx);
    if (ctx->graphics.path_scroll_timer) {
        lv_timer_set_repeat_count(ctx->graphics.path_scroll_timer, 1);
    }
}

static void set_original(text_viewer_ctx_t *ctx, const char *text)
{
    free(ctx->original_text);
    ctx->original_text = text ? strdup(text) : NULL;
}

static void get_slider_params(text_viewer_ctx_t *ctx, size_t *window_size, size_t *step)
{
    (void)ctx;
    if (!window_size || !step) {
        return;
    }
    *window_size = 2; /* two adjacent chunks = one window */
    *step = 1;        /* one chunk per step */
}

static void update_slider(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.chunk_slider) {
        return;
    }

    size_t window_size = 1;
    size_t step = 1;
    get_slider_params(ctx, &window_size, &step);

    size_t total_chunks = ctx->max_file_offset_kb + 1; /* offsets are in KB chunks */
    if (total_chunks == 0) {
        total_chunks = 1;
    }

    if (total_chunks <= window_size) {
        update_slider_disabled(ctx);
        return;
    }

    update_slider_range(ctx, window_size, step, total_chunks);
}

static void update_slider_disabled(text_viewer_ctx_t *ctx)
{
    bool prev = ctx->flags.slider_suppress_change;
    ctx->flags.slider_suppress_change = true;
    lv_slider_set_range(ctx->graphics.chunk_slider, 0, 0);
    lv_slider_set_value(ctx->graphics.chunk_slider, 0, LV_ANIM_OFF);
    ctx->flags.slider_suppress_change = prev;
    ctx->slider_pending_step = 0;
    ctx->flags.slider_drag_active = false;
    lv_obj_add_state(ctx->graphics.chunk_slider, LV_STATE_DISABLED);
}

static size_t clamp_current_step(const text_viewer_ctx_t *ctx, size_t step, size_t max_start, size_t max_step_index)
{
    size_t current_start = ctx->last_file_offset_kb;
    if (current_start > max_start) {
        current_start = max_start;
    }
    size_t current_step = step ? (current_start / step) : 0;
    if (current_step > max_step_index) {
        current_step = max_step_index;
    }
    return current_step;
}

static void update_slider_range(text_viewer_ctx_t *ctx, size_t window_size, size_t step, size_t total_chunks)
{
    size_t max_start = total_chunks - window_size;
    size_t max_step_index = step ? ((max_start + step - 1) / step) : 0;
    int32_t max_val = (int32_t)max_step_index;

    size_t current_step = clamp_current_step(ctx, step, max_start, max_step_index);

    bool prev = ctx->flags.slider_suppress_change;
    ctx->flags.slider_suppress_change = true;
    lv_slider_set_range(ctx->graphics.chunk_slider, max_val, 0); /* min at top, max at bottom */
    lv_slider_set_value(ctx->graphics.chunk_slider, (int32_t)current_step, LV_ANIM_OFF);
    ctx->flags.slider_suppress_change = prev;
    ctx->slider_pending_step = current_step;
    lv_obj_remove_state(ctx->graphics.chunk_slider, LV_STATE_DISABLED);
}

static void on_slider_value_changed(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || ctx->flags.slider_suppress_change) {
        return;
    }

    size_t window_size = 1;
    size_t step = 1;
    size_t total_chunks = 0;
    size_t max_start = 0;
    size_t max_step_index = 0;
    if (!get_slider_bounds(ctx, &window_size, &step, &total_chunks, &max_start, &max_step_index)) {
        return; /* Nothing to scroll */
    }

    handle_slider_event(e, ctx, window_size, step, total_chunks, max_start, max_step_index);
}

static void handle_slider_event(lv_event_t *e, text_viewer_ctx_t *ctx, size_t window_size, size_t step, size_t total_chunks, size_t max_start, size_t max_step_index)
{
    lv_event_code_t code = lv_event_get_code(e);
    bool blocked = slider_is_blocked(ctx);
    size_t clamped_step = clamp_slider_value(e, max_step_index);

    if (code == LV_EVENT_PRESSED) {
        if (blocked) {
            return;
        }
        ctx->flags.slider_drag_active = true;
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
        handle_slider_release(ctx, blocked, clamped_step, max_step_index, max_start, window_size, step);
    }
}

static bool slider_is_blocked(const text_viewer_ctx_t *ctx)
{
    return ctx->flags.waiting_sd || ctx->graphics.chunk_mbox || ctx->flags.pending_chunk;
}

static bool get_slider_bounds(text_viewer_ctx_t *ctx, size_t *window_size, size_t *step, size_t *total_chunks,
                              size_t *max_start, size_t *max_step_index)
{
    get_slider_params(ctx, window_size, step);
    *total_chunks = ctx->max_file_offset_kb + 1;
    if (*total_chunks == 0) {
        *total_chunks = 1;
    }
    if (*total_chunks <= *window_size) {
        return false;
    }

    *max_start = *total_chunks - *window_size;
    *max_step_index = *step ? ((*max_start + *step - 1) / *step) : 0;
    return true;
}

static size_t clamp_slider_value(lv_event_t *e, size_t max_step_index)
{
    int32_t slider_val = lv_slider_get_value(lv_event_get_target(e));
    if (slider_val < 0) {
        slider_val = 0;
    }

    size_t clamped_step = (size_t)slider_val;
    if (clamped_step > max_step_index) {
        clamped_step = max_step_index;
    }
    return clamped_step;
}

static void handle_slider_release(text_viewer_ctx_t *ctx, bool blocked, size_t clamped_step,
                                  size_t max_step_index, size_t max_start, size_t window_size, size_t step)
{
    if (blocked) {
        ctx->slider_pending_step = SIZE_MAX;
        ctx->flags.slider_drag_active = false;
        update_slider(ctx);
        return;
    }

    size_t target_step = slider_target_step(ctx, clamped_step, max_step_index);

    size_t current_step = clamp_current_step(ctx, step, max_start, max_step_index);

    if (target_step == current_step) {
        ctx->slider_pending_step = SIZE_MAX;
        ctx->flags.slider_drag_active = false;
        return;
    }

    size_t first_offset = 0;
    size_t second_offset = 0;
    compute_window_offsets(target_step, max_step_index, max_start, step, window_size,
                           ctx->max_file_offset_kb, &first_offset, &second_offset);

    bool from_top = target_step < current_step;
    ctx->slider_pending_step = SIZE_MAX;
    ctx->flags.slider_drag_active = false;
    request_chunk_load(ctx, first_offset, second_offset, from_top);
}

static size_t slider_target_step(const text_viewer_ctx_t *ctx, size_t clamped_step, size_t max_step_index)
{
    size_t target_step = (ctx->slider_pending_step != SIZE_MAX) ? ctx->slider_pending_step : clamped_step;
    if (target_step > max_step_index) {
        target_step = max_step_index;
    }
    return target_step;
}

static void compute_window_offsets(size_t target_step, size_t max_step_index, size_t max_start, size_t step,
                                   size_t window_size, size_t max_file_offset_kb,
                                   size_t *first_offset, size_t *second_offset)
{
    size_t new_start = (target_step >= max_step_index) ? max_start : (target_step * step);
    if (new_start > max_start) {
        new_start = max_start;
    }

    size_t first = new_start;
    size_t second = first + (window_size > 1 ? (window_size - 1) : 0);
    if (second > max_file_offset_kb) {
        second = max_file_offset_kb;
    }
    if (window_size > 1 && second == first && first > 0) {
        first -= 1;
    }

    if (first_offset) *first_offset = first;
    if (second_offset) *second_offset = second;
}

static esp_err_t read_window_chunks(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb,
                                    char **chunk_a, size_t *len_a, char **chunk_b, size_t *len_b)
{
    esp_err_t err = fs_text_read_range(ctx->path, first_offset_kb, chunk_a, len_a);
    if (err != ESP_OK) {
        return err;
    }

    if (second_offset_kb != first_offset_kb) {
        err = fs_text_read_range(ctx->path, second_offset_kb, chunk_b, len_b);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t join_window_chunks(char *chunk_a, size_t len_a, char *chunk_b, size_t len_b, char **joined_out)
{
    size_t total = len_a + len_b;
    char *joined = (char *)malloc(total + 1);
    if (!joined) {
        return ESP_ERR_NO_MEM;
    }

    if (len_a && chunk_a) {
        memcpy(joined, chunk_a, len_a);
    }
    if (len_b && chunk_b) {
        memcpy(joined + len_a, chunk_b, len_b);
    }
    joined[total] = '\0';

    *joined_out = joined;
    return ESP_OK;
}

static void apply_joined_window(text_viewer_ctx_t *ctx, const char *joined)
{
    bool prev_suppress = ctx->flags.suppress_events;
    ctx->flags.suppress_events = true;
    lv_textarea_set_text(ctx->graphics.text_area, joined);
    set_original(ctx, joined);
    ctx->flags.dirty = false;
    update_buttons(ctx);
    ctx->flags.suppress_events = prev_suppress;
}

static esp_err_t load_window(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb)
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

    esp_err_t err = read_window_chunks(ctx, first_offset_kb, second_offset_kb, &chunk_a, &len_a, &chunk_b, &len_b);
    if (err != ESP_OK)
    {
        goto cleanup;
    }

    err = join_window_chunks(chunk_a, len_a, chunk_b, len_b, &joined);
    if (err != ESP_OK)
    {
        goto cleanup;
    }

    apply_joined_window(ctx, joined);

cleanup:
    free(joined);
    free(chunk_a);
    free(chunk_b);
    return err;
}

static void update_buttons(text_viewer_ctx_t *ctx)
{
    if (!ctx->flags.editable)
    {
        return;
    }
    if (ctx->flags.dirty)
    {
        lv_obj_clear_state(ctx->graphics.save_btn, LV_STATE_DISABLED);
    }
    else
    {
        lv_obj_add_state(ctx->graphics.save_btn, LV_STATE_DISABLED);
    }
}

static void show_keyboard(text_viewer_ctx_t *ctx, lv_obj_t *target)
{
    if (!ctx || !ctx->flags.editable)
    {
        return;
    }
    if (target)
    {
        lv_keyboard_set_textarea(ctx->graphics.keyboard, target);
    }
    else if (!lv_keyboard_get_textarea(ctx->graphics.keyboard))
    {
        lv_keyboard_set_textarea(ctx->graphics.keyboard, ctx->graphics.text_area);
    }
    lv_obj_clear_flag(ctx->graphics.keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void hide_keyboard(text_viewer_ctx_t *ctx)
{
    if (!ctx)
    {
        return;
    }
    if (!lv_obj_has_flag(ctx->graphics.keyboard, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_add_flag(ctx->graphics.keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if (lv_keyboard_get_textarea(ctx->graphics.keyboard))
    {
        lv_keyboard_set_textarea(ctx->graphics.keyboard, NULL);
    }
}

static void on_text_area_clicked(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->flags.editable)
    {
        return;
    }
    show_keyboard(ctx, ctx->graphics.text_area);
}

static void skip_cursor_animation(text_viewer_ctx_t *ctx)
{
    // Jump to the new cursor position immediately (skip the default scroll animation)
    lv_point_t target_scroll = {0};
    lv_obj_get_scroll_end(ctx->graphics.text_area, &target_scroll);
    lv_obj_scroll_to(ctx->graphics.text_area, target_scroll.x, target_scroll.y, LV_ANIM_OFF);    
}

static void on_text_scrolled(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    if (text_scroll_should_ignore(ctx))
    {
        return;
    }

    bool at_top = lv_obj_get_scroll_top(ctx->graphics.text_area) <= 0;
    bool at_bottom = lv_obj_get_scroll_bottom(ctx->graphics.text_area) <= 0;

    handle_scroll_top_edge(ctx, at_top);
    handle_scroll_bottom_edge(ctx, at_bottom);
}

static bool text_scroll_should_ignore(const text_viewer_ctx_t *ctx)
{
    return ctx->flags.waiting_sd || ctx->graphics.chunk_mbox || ctx->flags.pending_chunk;
}

static void handle_scroll_top_edge(text_viewer_ctx_t *ctx, bool at_top)
{
    if (at_top && !ctx->flags.at_top_edge)
    {
        ctx->flags.at_top_edge = true;

        if (!ctx->flags.new_file && ctx->last_file_offset_kb > 0)
        {
            size_t new_first = ctx->last_file_offset_kb - 1;
            size_t new_second = ctx->last_file_offset_kb;
            request_chunk_load(ctx, new_first, new_second, true);
        }
    }
    else if (!at_top)
    {
        ctx->flags.at_top_edge = false;
    }
}

static void handle_scroll_bottom_edge(text_viewer_ctx_t *ctx, bool at_bottom)
{
    if (at_bottom && !ctx->flags.at_bottom_edge)
    {
        ctx->flags.at_bottom_edge = true;

        if (!ctx->flags.new_file && ctx->current_file_offset_kb < ctx->max_file_offset_kb)
        {
            size_t next_offset = ctx->current_file_offset_kb + 1;
            size_t first_offset = ctx->current_file_offset_kb;
            request_chunk_load(ctx, first_offset, next_offset, false);
        }
    }
    else if (!at_bottom)
    {
        ctx->flags.at_bottom_edge = false;
    }
}

static void on_keyboard_cancel(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    hide_keyboard(ctx);
}

static void on_name_textarea_clicked(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.name_textarea)
    {
        return;
    }
    show_keyboard(ctx, ctx->graphics.name_textarea);
}

static void on_keyboard_ready(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->flags.editable)
    {
        return;
    }

    lv_obj_t *target = ctx->graphics.keyboard ? lv_keyboard_get_textarea(ctx->graphics.keyboard) : NULL;
    if (ctx->graphics.name_dialog && target && target == ctx->graphics.name_textarea)
    {
        confirm_name_dialog(ctx);
        return;
    }

    handle_save(ctx);
}

static void on_screen_clicked(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->flags.editable)
    {
        return;
    }
    if (ctx->graphics.name_dialog)
    {
        return;
    }
    if (lv_obj_has_flag(ctx->graphics.keyboard, LV_OBJ_FLAG_HIDDEN))
    {
        return;
    }
    lv_obj_t *target = lv_event_get_target(e);
    if (target_in(ctx->graphics.text_area, target) ||
        target_in(ctx->graphics.keyboard, target))
    {
        return;
    }
    hide_keyboard(ctx);
}

static void on_text_changed(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->flags.editable || ctx->flags.suppress_events)
    {
        return;
    }
    const char *text = lv_textarea_get_text(ctx->graphics.text_area);
    const char *orig = ctx->original_text ? ctx->original_text : "";
    bool dirty = strcmp(text, orig) != 0;
    if (dirty != ctx->flags.dirty)
    {
        ctx->flags.dirty = dirty;
        update_buttons(ctx);
        set_status(ctx, dirty ? "Modified" : "Saved");
    }
}

static void handle_save(text_viewer_ctx_t *ctx)
{
    const char *text = NULL;
    if (!save_prechecks(ctx, &text))
    {
        return;
    }

    size_t window_start = 0, window_end = 0;
    size_t prefix_size = 0, suffix_start = 0, suffix_size = 0;
    size_t file_size = 0;
    bool have_existing = false;
    if (!compute_save_window(ctx, &window_start, &window_end, &prefix_size, &suffix_start, &suffix_size,
                             &have_existing, &file_size))
    {
        return;
    }
    (void)file_size;

    char tmp_path[FS_TEXT_MAX_PATH];
    if (!build_temp_path_for_save(ctx->path, tmp_path, sizeof(tmp_path), ctx))
    {
        return;
    }

    if (!file_save_success(ctx, text, tmp_path, have_existing, window_start, window_end,
                           prefix_size, suffix_start, suffix_size)) 
    {
        return;
    }

    finalize_save_state(ctx, text, prefix_size, suffix_size);
}

static bool file_save_success(text_viewer_ctx_t *ctx, const char *text, const char *tmp_path, bool have_existing,
                              size_t window_start, size_t window_end,
                              size_t prefix_size, size_t suffix_start, size_t suffix_size)
{
    FILE *src = NULL;
    FILE *tmp = NULL;
    ESP_LOGD(TAG, "Saving range [%zu, %zu) with prefix=%zu, suffix_start=%zu, suffix_size=%zu",
             window_start, window_end, prefix_size, suffix_start, suffix_size);

    if (!open_save_streams(ctx->path, tmp_path, have_existing, &src, &tmp, ctx))
        goto cleanup;

    if (!copy_prefix(src, tmp, prefix_size, ctx->path, tmp_path, ctx))
        goto cleanup;

    if (!write_body(tmp, text, tmp_path, ctx))
        goto cleanup;

    if (!copy_suffix(src, tmp, suffix_start, suffix_size, ctx->path, tmp_path, ctx))
        goto cleanup;   

    cleanup_save_files(src, tmp, NULL); /* closes only */

    if (!finalize_rename(tmp_path, ctx->path, ctx))
        return false;

    return true;

cleanup:
    cleanup_save_files(src, tmp, tmp_path);
    return false;
}

static bool save_prechecks(text_viewer_ctx_t *ctx, const char **text_out)
{
    if (!ctx) {
        return false;
    }
    if (ctx->flags.waiting_sd) {
        set_status(ctx, "Reconnect SD");
        return false;
    }
    if (ctx->flags.new_file && ctx->path[0] == '\0') {
        show_name_dialog(ctx);
        return false;
    }

    const char *text = lv_textarea_get_text(ctx->graphics.text_area);
    if (!text) {
        text = "";
    }
    if (ctx->path[0] == '\0') {
        set_status(ctx, "Missing file name");
        return false;
    }
    if (text_out) {
        *text_out = text;
    }
    return true;
}

static bool compute_save_window(text_viewer_ctx_t *ctx, size_t *window_start, size_t *window_end,
                                size_t *prefix_size, size_t *suffix_start, size_t *suffix_size,
                                bool *have_existing, size_t *file_size)
{
    size_t first_kb = ctx->last_file_offset_kb;
    size_t second_kb = ctx->current_file_offset_kb;
    size_t chunk_count = (second_kb > first_kb) ? (second_kb - first_kb + 1u) : 1u;

    if (first_kb > SIZE_MAX / 1024u || chunk_count > SIZE_MAX / FS_TEXT_READ_CHUNK_SIZE_B) {
        set_status(ctx, "Range overflow");
        return false;
    }

    size_t start = first_kb * 1024u;
    size_t span = chunk_count * FS_TEXT_READ_CHUNK_SIZE_B;
    size_t end = start + span;
    if (end < start) {
        set_status(ctx, "Range overflow");
        return false;
    }

    struct stat st = {0};
    bool existing = (stat(ctx->path, &st) == 0 && S_ISREG(st.st_mode));
    size_t size = existing ? (size_t)st.st_size : 0u;

    if (start > size) start = size;
    if (end > size) end = size;

    if (window_start) *window_start = start;
    if (window_end) *window_end = end;
    if (prefix_size) *prefix_size = start;
    if (suffix_start) *suffix_start = end;
    if (suffix_size) *suffix_size = (end < size) ? (size - end) : 0u;
    if (have_existing) *have_existing = existing;
    if (file_size) *file_size = size;
    return true;
}

static bool build_temp_path_for_save(const char *dest_path, char *tmp_path, size_t tmp_size, text_viewer_ctx_t *ctx)
{
    char dir[FS_TEXT_MAX_PATH];
    const char *slash = strrchr(dest_path, '/');
    if (slash) {
        size_t dir_len = (size_t)(slash - dest_path);
        if (dir_len == 0) {
            if (sizeof(dir) < 2) {
                set_status(ctx, "Path too long");
                return false;
            }
            dir[0] = '/';
            dir[1] = '\0';
        } else if (dir_len < sizeof(dir)) {
            memcpy(dir, dest_path, dir_len);
            dir[dir_len] = '\0';
        } else {
            set_status(ctx, "Path too long");
            return false;
        }
    } else {
        strlcpy(dir, ".", sizeof(dir));
    }

    int needed = snprintf(tmp_path, tmp_size, "%s/tmpwrt.tmp", dir);
    if (needed < 0 || needed >= (int)tmp_size) {
        set_status(ctx, "Path too long");
        return false;
    }
    remove(tmp_path);
    return true;
}

static bool open_save_streams(const char *dest_path, const char *tmp_path, bool have_existing,
                              FILE **src_out, FILE **tmp_out, text_viewer_ctx_t *ctx)
{
    FILE *src = NULL;
    if (have_existing) {
        src = fopen(dest_path, "rb");
        if (!src) {
            set_status(ctx, "Open failed");
            ESP_LOGE(TAG, "Failed to open %s for patching", dest_path);
            schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return false;
        }
    }

    FILE *tmp = fopen(tmp_path, "wb");
    if (!tmp) {
        if (src) fclose(src);
        set_status(ctx, "Temp open failed");
        ESP_LOGE(TAG, "Failed to open %s", tmp_path);
        schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
        return false;
    }

    if (src_out) *src_out = src;
    if (tmp_out) *tmp_out = tmp;
    return true;
}

static bool copy_prefix(FILE *src, FILE *tmp, size_t prefix_size, const char *dest_path, const char *tmp_path, text_viewer_ctx_t *ctx)
{
    if (prefix_size == 0) {
        return true;
    }
    char buf[FS_TEXT_READ_CHUNK_SIZE_B];
    size_t remaining = prefix_size;
    while (remaining > 0) {
        size_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
        if (!src || fread(buf, 1, chunk, src) != chunk) {
            set_status(ctx, "Read failed");
            ESP_LOGE(TAG, "Failed to read prefix from %s", dest_path);
            schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return false;
        }
        if (fwrite(buf, 1, chunk, tmp) != chunk) {
            set_status(ctx, "Write failed");
            ESP_LOGE(TAG, "Failed to write prefix to %s", tmp_path);
            schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return false;
        }
        remaining -= chunk;
    }
    return true;
}

static bool write_body(FILE *tmp, const char *text, const char *tmp_path, text_viewer_ctx_t *ctx)
{
    size_t text_len = text ? strlen(text) : 0;
    if (text_len == 0) {
        return true;
    }
    if (fwrite(text, 1, text_len, tmp) != text_len) {
        set_status(ctx, "Write failed");
        ESP_LOGE(TAG, "Failed to write textarea to %s", tmp_path);
        schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
        return false;
    }
    return true;
}

static bool copy_suffix(FILE *src, FILE *tmp, size_t suffix_start, size_t suffix_size,
                        const char *dest_path, const char *tmp_path, text_viewer_ctx_t *ctx)
{
    if (suffix_size == 0 || !src) {
        return true;
    }

    if (fseek(src, (long)suffix_start, SEEK_SET) != 0) {
        set_status(ctx, "Seek failed");
        ESP_LOGE(TAG, "Failed to seek %s to %zu", dest_path, suffix_start);
        schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
        return false;
    }

    char buf[FS_TEXT_READ_CHUNK_SIZE_B];
    size_t remaining = suffix_size;
    while (remaining > 0) {
        size_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
        size_t got = fread(buf, 1, chunk, src);
        if (got != chunk) {
            set_status(ctx, "Read failed");
            ESP_LOGE(TAG, "Failed to read suffix from %s", dest_path);
            schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return false;
        }
        if (fwrite(buf, 1, chunk, tmp) != chunk) {
            set_status(ctx, "Write failed");
            ESP_LOGE(TAG, "Failed to write suffix to %s", tmp_path);
            schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
            return false;
        }
        remaining -= chunk;
    }
    return true;
}

static bool finalize_rename(const char *tmp_path, const char *dest_path, text_viewer_ctx_t *ctx)
{
    if (rename(tmp_path, dest_path) != 0) {
        if (errno == EEXIST && remove(dest_path) == 0 && rename(tmp_path, dest_path) == 0) {
            return true;
        }
        set_status(ctx, "Rename failed");
        ESP_LOGE(TAG, "rename(%s -> %s) failed (errno=%d)", tmp_path, dest_path, errno);
        remove(tmp_path);
        schedule_sd_retry(ctx, TEXT_VIEWER_SD_SAVE);
        return false;
    }
    return true;
}

static void finalize_save_state(text_viewer_ctx_t *ctx, const char *text, size_t prefix_size, size_t suffix_size)
{
    size_t text_len = text ? strlen(text) : 0;
    size_t new_size = prefix_size + text_len + suffix_size;
    ctx->max_file_offset_kb = (new_size > 0) ? ((new_size - 1u) / 1024u) : 0u;
    if (ctx->last_file_offset_kb > ctx->max_file_offset_kb) {
        ctx->last_file_offset_kb = ctx->max_file_offset_kb;
    }
    if (ctx->current_file_offset_kb > ctx->max_file_offset_kb) {
        ctx->current_file_offset_kb = ctx->max_file_offset_kb;
    }
    ctx->flags.at_top_edge = false;
    ctx->flags.at_bottom_edge = false;

    set_original(ctx, text);
    ctx->flags.dirty = false;
    ctx->flags.content_changed = true;
    set_status(ctx, "Saved");
    update_slider(ctx);
}

static void cleanup_save_files(FILE *src, FILE *tmp, const char *tmp_path)
{
    if (src) {
        fclose(src);
    }
    if (tmp) {
        fclose(tmp);
    }
    if (tmp_path) {
        remove(tmp_path);
    }
}

static void on_save(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    handle_save(ctx);
}

static void on_back(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    if (ctx->flags.editable && ctx->flags.dirty)
    {
        show_confirm(ctx);
        return;
    }
    close_ctx(ctx, false);
}

static void show_chunk_prompt(text_viewer_ctx_t *ctx)
{
    if (!ctx || ctx->graphics.chunk_mbox || !ctx->flags.pending_chunk)
    {
        return;
    }
    lv_obj_t *mbox = lv_msgbox_create(ctx->graphics.screen);
    styles_set_msgbox(mbox);
    lv_obj_add_flag(mbox, LV_OBJ_FLAG_FLOATING);
    ctx->graphics.chunk_mbox = mbox;
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
    lv_obj_add_event_cb(save_btn, on_chunk_prompt, LV_EVENT_CLICKED, ctx);

    lv_obj_t *discard_btn = lv_msgbox_add_footer_button(mbox, "Discard");
    lv_obj_set_user_data(discard_btn, (void *)TEXT_VIEWER_CHUNK_DISCARD);
    lv_obj_set_flex_grow(discard_btn, 1);
    lv_obj_add_event_cb(discard_btn, on_chunk_prompt, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, NULL);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_add_event_cb(cancel_btn, on_chunk_prompt, LV_EVENT_CLICKED, ctx);
}

static void close_chunk_prompt(text_viewer_ctx_t *ctx)
{
    if (ctx && ctx->graphics.chunk_mbox)
    {
        lv_msgbox_close(ctx->graphics.chunk_mbox);
        ctx->graphics.chunk_mbox = NULL;
    }
}

static void clear_pending_chunk_state(text_viewer_ctx_t *ctx)
{
    if (!ctx)
    {
        return;
    }
    ctx->flags.pending_chunk = false;
    ctx->flags.at_top_edge = false;
    ctx->flags.at_bottom_edge = false;
    update_slider(ctx);
}

static void handle_chunk_prompt_save(text_viewer_ctx_t *ctx)
{
    if (!ctx)
    {
        return;
    }
    handle_save(ctx);
    if (!ctx->flags.dirty)
    {
        apply_pending_chunk(ctx);
    }
    else if (!ctx->flags.waiting_sd)
    {
        clear_pending_chunk_state(ctx);
    }
}

static void handle_chunk_prompt_discard(text_viewer_ctx_t *ctx)
{
    if (!ctx)
    {
        return;
    }
    ctx->flags.dirty = false;
    update_buttons(ctx);
    apply_pending_chunk(ctx);
}

static void handle_chunk_prompt_cancel(text_viewer_ctx_t *ctx)
{
    clear_pending_chunk_state(ctx);
}

static bool should_apply_pending_chunk(const text_viewer_ctx_t *ctx)
{
    return ctx && ctx->flags.pending_chunk && !ctx->flags.waiting_sd;
}

static void update_cursor_after_chunk(text_viewer_ctx_t *ctx, lv_coord_t content_h)
{
    if (ctx->flags.pending_scroll_up)
    {
        lv_textarea_set_cursor_pos(ctx->graphics.text_area, (int32_t)FS_TEXT_READ_CHUNK_SIZE_B + content_h);
    }
    else
    {
        lv_textarea_set_cursor_pos(ctx->graphics.text_area, (int32_t)FS_TEXT_READ_CHUNK_SIZE_B - content_h);
    }
    skip_cursor_animation(ctx);
}

static void finalize_pending_chunk_success(text_viewer_ctx_t *ctx)
{
    ctx->last_file_offset_kb = ctx->pending_first_offset_kb;
    ctx->current_file_offset_kb = ctx->pending_second_offset_kb;
    ctx->flags.at_top_edge = false;
    ctx->flags.at_bottom_edge = false;
    update_slider(ctx);
    ctx->flags.pending_chunk = false;
}

static void handle_pending_chunk_failure(text_viewer_ctx_t *ctx, esp_err_t err)
{
    ESP_LOGE(TAG, "Failed to load chunk: %s", esp_err_to_name(err));
    schedule_sd_retry(ctx, TEXT_VIEWER_SD_CHUNK);
    ctx->flags.at_top_edge = false;
    ctx->flags.at_bottom_edge = false;
}

static void apply_pending_chunk(text_viewer_ctx_t *ctx)
{
    if (!should_apply_pending_chunk(ctx))
    {
        return;
    }

    esp_err_t err = load_window(ctx, ctx->pending_first_offset_kb, ctx->pending_second_offset_kb);
    if (err != ESP_OK)
    {
        handle_pending_chunk_failure(ctx, err);
        return;
    }

    lv_coord_t content_h = lv_obj_get_content_height(ctx->graphics.text_area);
    update_cursor_after_chunk(ctx, content_h);
    finalize_pending_chunk_success(ctx);
}

static void on_chunk_prompt(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    void *ud = lv_obj_get_user_data(lv_event_get_target(e));
    close_chunk_prompt(ctx);

    if (ud == (void *)TEXT_VIEWER_CHUNK_SAVE)
    {
        handle_chunk_prompt_save(ctx);
    }
    else if (ud == (void *)TEXT_VIEWER_CHUNK_DISCARD)
    {
        handle_chunk_prompt_discard(ctx);
    }
    else
    {
        handle_chunk_prompt_cancel(ctx);
    }
}

static void request_chunk_load(text_viewer_ctx_t *ctx, size_t first_offset_kb, size_t second_offset_kb, bool from_top)
{
    if (!ctx || ctx->graphics.chunk_mbox)
    {
        return;
    }
    if (ctx->flags.waiting_sd)
    {
        ctx->pending_first_offset_kb = first_offset_kb;
        ctx->pending_second_offset_kb = second_offset_kb;
        ctx->flags.pending_scroll_up = from_top;
        ctx->flags.pending_chunk = true;
        return;
    }

    ctx->pending_first_offset_kb = first_offset_kb;
    ctx->pending_second_offset_kb = second_offset_kb;
    ctx->flags.pending_scroll_up = from_top;
    ctx->flags.pending_chunk = true;

    if (ctx->flags.dirty)
    {
        show_chunk_prompt(ctx);
    }
    else
    {
        apply_pending_chunk(ctx);
    }
}

static bool sd_reconnect_ready(text_viewer_ctx_t *ctx)
{
    if (!reconnection_success)
    {
        set_status(ctx, "Reconnect SD");
        return false;
    }
    if (xSemaphoreTake(reconnection_success, 0) != pdTRUE)
    {
        set_status(ctx, "Reconnect SD");
        return false;
    }
    return true;
}

static void perform_sd_retry_action(text_viewer_ctx_t *ctx, text_viewer_sd_action_t action)
{
    if (action == TEXT_VIEWER_SD_SAVE)
    {
        handle_save(ctx);
        if (ctx->flags.pending_chunk && !ctx->flags.dirty && !ctx->flags.waiting_sd)
        {
            apply_pending_chunk(ctx);
        }
    }
    else if (action == TEXT_VIEWER_SD_CHUNK)
    {
        apply_pending_chunk(ctx);
    }
}

static void on_sd_retry_timer(lv_timer_t *timer)
{
    text_viewer_ctx_t *ctx = lv_timer_get_user_data(timer);
    if (!ctx || !ctx->flags.waiting_sd)
    {
        return;
    }
    if (!sd_reconnect_ready(ctx))
    {
        return;
    }

    ctx->flags.waiting_sd = false;
    text_viewer_sd_action_t action = ctx->sd_retry_action;
    ctx->sd_retry_action = TEXT_VIEWER_SD_NONE;
    set_status(ctx, "SD reconnected");

    perform_sd_retry_action(ctx, action);
}

static void schedule_sd_retry(text_viewer_ctx_t *ctx, text_viewer_sd_action_t action)
{
    if (!ctx)
    {
        return;
    }
    if (ctx->flags.waiting_sd)
    {
        ctx->sd_retry_action = action;
        return;
    }
    ctx->flags.waiting_sd = true;
    ctx->sd_retry_action = action;
    set_status(ctx, "Reconnect SD");
    sd_card_schedule_retry();

    if (!ctx->graphics.sd_retry_timer)
    {
        ctx->graphics.sd_retry_timer = lv_timer_create(on_sd_retry_timer, 250, ctx);
    }
}


/************************************* New-file utilities *************************************/

static bool validate_name(const char *name)
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

static bool has_txt_extension(const char *dot)
{
    return dot && dot[1] != '\0' && strcasecmp(dot, ".txt") == 0;
}

static bool has_other_extension(const char *dot)
{
    return dot && dot[1] != '\0' && strcasecmp(dot, ".txt") != 0;
}

static bool can_append_txt(size_t current_len, size_t buf_len)
{
    return current_len + 4 < buf_len;
}

static void ensure_txt_extension(char *name, size_t len)
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
    if (has_txt_extension(dot) || has_other_extension(dot))
    {
        return;
    }
    if (!can_append_txt(n, len))
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

static esp_err_t compose_new_path(text_viewer_ctx_t *ctx, const char *name, char *out, size_t out_len)
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

static bool path_exists(const char *path)
{
    if (!path || path[0] == '\0')
    {
        return false;
    }
    struct stat st = {0};
    return stat(path, &st) == 0;
}

static void close_save_conflict(text_viewer_ctx_t *ctx)
{
    if (ctx && ctx->graphics.save_conflict_mbox)
    {
        lv_msgbox_close(ctx->graphics.save_conflict_mbox);
        ctx->graphics.save_conflict_mbox = NULL;
    }
    if (ctx)
    {
        ctx->conflict_path[0] = '\0';
        ctx->conflict_name[0] = '\0';
    }
}

static void show_save_conflict(text_viewer_ctx_t *ctx, const char *path, const char *name)
{
    if (!ctx || !path || !name || !ctx->graphics.name_dialog)
    {
        return;
    }
    close_save_conflict(ctx);
    strlcpy(ctx->conflict_path, path, sizeof(ctx->conflict_path));
    strlcpy(ctx->conflict_name, name, sizeof(ctx->conflict_name));

    lv_obj_t *mbox = lv_msgbox_create(ctx->graphics.screen);
    styles_set_msgbox(mbox);
    lv_obj_add_flag(mbox, LV_OBJ_FLAG_FLOATING);
    ctx->graphics.save_conflict_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_set_width(mbox, LV_PCT(80));
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text_fmt(label, "\"%s\" already exists. Overwrite?", ctx->conflict_name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *replace_btn = lv_msgbox_add_footer_button(mbox, "Overwrite");
    lv_obj_set_user_data(replace_btn, (void *)1);
    styles_set_button(replace_btn);
    lv_obj_add_event_cb(replace_btn, on_save_conflict, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, on_save_conflict, LV_EVENT_CLICKED, ctx);

    set_status(ctx, "File already exists");
}

static void apply_new_file_path(text_viewer_ctx_t *ctx, const char *name, const char *full_path)
{
    if (!ctx || !name || !full_path || full_path[0] == '\0')
    {
        return;
    }
    strlcpy(ctx->path, full_path, sizeof(ctx->path));
    ctx->directory[0] = '\0';
    ctx->flags.new_file = false;
    close_name_dialog(ctx);
    strlcpy(ctx->pending_name, name, sizeof(ctx->pending_name));
    set_path_label(ctx, ctx->path);
}

static void on_save_conflict(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    char conflict_path[FS_TEXT_MAX_PATH];
    char conflict_name[FS_NAV_MAX_NAME];
    strlcpy(conflict_path, ctx->conflict_path, sizeof(conflict_path));
    strlcpy(conflict_name, ctx->conflict_name, sizeof(conflict_name));
    int choice = (int)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));

    close_save_conflict(ctx);

    if (conflict_path[0] == '\0' || conflict_name[0] == '\0')
    {
        return;
    }

    if (choice == 1)
    {
        apply_new_file_path(ctx, conflict_name, conflict_path);
        handle_save(ctx);
    }
    else
    {
        /* Keep dialog open for editing; refocus name field and keyboard. */
        if (ctx->graphics.name_textarea)
        {
            lv_obj_add_state(ctx->graphics.name_textarea, LV_STATE_FOCUSED);
            lv_textarea_set_cursor_pos(ctx->graphics.name_textarea, LV_TEXTAREA_CURSOR_LAST);
            show_keyboard(ctx, ctx->graphics.name_textarea);
        }
        set_status(ctx, "Choose another name");
    }
}

static void show_name_dialog(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->flags.new_file || !ctx->flags.editable || ctx->graphics.name_dialog)
    {
        return;
    }
    lv_obj_t *dlg = lv_msgbox_create(ctx->graphics.screen);
    styles_set_msgbox(dlg);
    ctx->graphics.name_dialog = dlg;
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_max_width(dlg, LV_PCT(65), 0);
    lv_obj_set_width(dlg, LV_PCT(65));

    lv_obj_t *content = lv_msgbox_get_content(dlg);
    lv_obj_t *label = lv_label_create(content);
    lv_label_set_text(label, "File name");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);

    ctx->graphics.name_textarea = lv_textarea_create(content);
    lv_textarea_set_one_line(ctx->graphics.name_textarea, true);
    lv_textarea_set_max_length(ctx->graphics.name_textarea, FS_NAV_MAX_NAME - 1);
    const char *initial = ctx->pending_name[0] ? ctx->pending_name : ".txt";
    lv_textarea_set_text(ctx->graphics.name_textarea, initial);
    lv_textarea_set_cursor_pos(ctx->graphics.name_textarea, 0);
    lv_obj_add_state(ctx->graphics.name_textarea, LV_STATE_FOCUSED);
    lv_obj_clear_state(ctx->graphics.text_area, LV_STATE_FOCUSED);
    lv_obj_add_state(ctx->graphics.text_area, LV_STATE_DISABLED);
    styles_set_textarea(ctx->graphics.name_textarea);
    lv_obj_set_width(ctx->graphics.name_textarea, LV_PCT(100));
    lv_textarea_set_cursor_click_pos(ctx->graphics.text_area, false);

    lv_obj_t *save_btn = lv_msgbox_add_footer_button(dlg, "Save");
    lv_obj_set_user_data(save_btn, (void *)1);
    styles_set_button(save_btn);
    lv_obj_add_event_cb(save_btn, on_name_dialog, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(dlg, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, on_name_dialog, LV_EVENT_CLICKED, ctx);

    show_keyboard(ctx, ctx->graphics.name_textarea);
    lv_obj_add_event_cb(ctx->graphics.name_textarea, on_name_textarea_clicked, LV_EVENT_CLICKED, ctx);

    lv_obj_update_layout(ctx->graphics.keyboard);
    lv_obj_update_layout(dlg);
    lv_coord_t keyboard_top = lv_obj_get_y(ctx->graphics.keyboard);
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

static void close_name_dialog(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.name_dialog)
    {
        return;
    }
    close_save_conflict(ctx);
    if (ctx->graphics.name_textarea)
    {
        const char *current = lv_textarea_get_text(ctx->graphics.name_textarea);
        if (current)
        {
            strlcpy(ctx->pending_name, current, sizeof(ctx->pending_name));
        }
    }
    lv_msgbox_close(ctx->graphics.name_dialog);
    ctx->graphics.name_dialog = NULL;
    ctx->graphics.name_textarea = NULL;
    lv_obj_clear_state(ctx->graphics.text_area, LV_STATE_DISABLED);
    lv_textarea_set_cursor_click_pos(ctx->graphics.text_area, true);
    hide_keyboard(ctx);
}

static bool confirm_name_dialog(text_viewer_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.name_dialog)
    {
        return false;
    }
    close_save_conflict(ctx);

    const char *raw = ctx->graphics.name_textarea ? lv_textarea_get_text(ctx->graphics.name_textarea) : "";
    char name_buf[FS_NAV_MAX_NAME];
    strlcpy(name_buf, raw ? raw : "", sizeof(name_buf));
    ensure_txt_extension(name_buf, sizeof(name_buf));
    if (!validate_name(name_buf))
    {
        set_status(ctx, "Invalid .txt name");
        return false;
    }
    char new_path[FS_TEXT_MAX_PATH];
    esp_err_t compose_err = compose_new_path(ctx, name_buf, new_path, sizeof(new_path));
    if (compose_err != ESP_OK)
    {
        set_status(ctx, "Path too long");
        return false;
    }
    if (path_exists(new_path))
    {
        show_save_conflict(ctx, new_path, name_buf);
        return false;
    }

    apply_new_file_path(ctx, name_buf, new_path);
    handle_save(ctx);
    return true;
}

static void on_name_dialog(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.name_dialog)
    {
        return;
    }
    bool confirm = (bool)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    if (!confirm)
    {
        close_name_dialog(ctx);
        return;
    }

    confirm_name_dialog(ctx);
}

static void show_confirm(text_viewer_ctx_t *ctx)
{
    if (ctx->graphics.confirm_mbox)
    {
        return;
    }
    lv_obj_t *mbox = lv_msgbox_create(ctx->graphics.screen);
    styles_set_msgbox(mbox);
    lv_obj_add_flag(mbox, LV_OBJ_FLAG_FLOATING);
    ctx->graphics.confirm_mbox = mbox;
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
    lv_obj_add_event_cb(save_btn, on_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *discard_btn = lv_msgbox_add_footer_button(mbox, "Discard");
    lv_obj_set_user_data(discard_btn, (void *)TEXT_VIEWER_CONFIRM_DISCARD);
    lv_obj_set_flex_grow(discard_btn, 1);
    styles_set_button(discard_btn);
    lv_obj_add_event_cb(discard_btn, on_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, NULL);
    lv_obj_set_flex_grow(cancel_btn, 1);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, on_confirm, LV_EVENT_CLICKED, ctx);
}

static bool target_in(lv_obj_t *parent, lv_obj_t *target)
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

static void close_confirm(text_viewer_ctx_t *ctx)
{
    if (ctx->graphics.confirm_mbox)
    {
        lv_msgbox_close(ctx->graphics.confirm_mbox);
        ctx->graphics.confirm_mbox = NULL;
    }
}

static void on_confirm(lv_event_t *e)
{
    text_viewer_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx)
    {
        return;
    }
    void *ud = lv_obj_get_user_data(lv_event_get_target(e));
    close_confirm(ctx);
    if (ud == (void *)TEXT_VIEWER_CONFIRM_SAVE)
    {
        handle_save(ctx);
    }
    else if (ud == (void *)TEXT_VIEWER_CONFIRM_DISCARD)
    {
        close_ctx(ctx, false);
    }
}

static void close_ctx(text_viewer_ctx_t *ctx, bool changed)
{
    close_confirm(ctx);
    close_chunk_prompt(ctx);
    close_save_conflict(ctx);
    close_name_dialog(ctx);
    if (ctx->graphics.sd_retry_timer)
    {
        lv_timer_del(ctx->graphics.sd_retry_timer);
        ctx->graphics.sd_retry_timer = NULL;
    }
    ctx->flags.active = false;
    ctx->flags.editable = false;
    ctx->flags.dirty = false;
    ctx->flags.suppress_events = false;
    ctx->flags.new_file = false;
    ctx->directory[0] = '\0';
    ctx->pending_name[0] = '\0';
    ctx->flags.pending_chunk = false;
    ctx->flags.waiting_sd = false;
    ctx->sd_retry_action = TEXT_VIEWER_SD_NONE;
    ctx->flags.content_changed = false;
    ctx->conflict_path[0] = '\0';
    ctx->conflict_name[0] = '\0';
    lv_keyboard_set_textarea(ctx->graphics.keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.keyboard, LV_OBJ_FLAG_HIDDEN);
    /* Drop heavy UI tree (text area buffer) so large files release heap after close. */
    if (ctx->graphics.screen) {
        lv_obj_del(ctx->graphics.screen);
        ctx->graphics.screen = NULL;
        ctx->graphics.toolbar = NULL;
        ctx->graphics.path_label = NULL;
        ctx->graphics.status_label = NULL;
        ctx->graphics.save_btn = NULL;
        ctx->graphics.text_area = NULL;
        ctx->graphics.keyboard = NULL;
        ctx->graphics.chunk_slider = NULL;
    }
    free(ctx->original_text);
    ctx->original_text = NULL;
    if (ctx->graphics.return_screen)
    {
        lv_screen_load(ctx->graphics.return_screen);
    }
    if (ctx->close_cb)
    {
        ctx->close_cb(true, ctx->close_ctx); // Force refresh in caller (e.g., file browser list)
    }
}
