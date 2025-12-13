#include "file_manager.h"

#include <sys/stat.h>
#include <stdbool.h>
#include <stdint.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "lvgl.h"

#include "text_viewer_screen.h"
#include "fs_navigator.h"
#include "fs_text_ops.h"
#include "Domine_16.h"
#include "settings.h"
#include "styles.h"
#include "jpg.h"

#define TAG "file_manager"

#define FILE_BROWSER_MAX_SORTABLE_ITEMS_DEFAULT     64      /* May cause OOM errors if larger */
#define FILE_BROWSER_LIST_WINDOW_SIZE_DEFAULT       32      /* May cause OOM errors if larger */
#define FILE_BROWSER_LIST_WINDOW_STEP_DEFAULT       16
#define FILE_BROWSER_PATH_SCROLL_DELAY_MS_DEFAULT   2000
#define FILE_BROWSER_ENTRY_SCROLL_DELAY_MS_DEFAULT  FILE_BROWSER_PATH_SCROLL_DELAY_MS_DEFAULT
#define FILE_BROWSER_SLIDER_GAP_DEFAULT             8
#define FILE_BROWSER_WAIT_STACK_SIZE_B_DEFAULT      (6 * 1024)
#define FILE_BROWSER_WAIT_PRIO_DEFAULT              4

_Static_assert(FILE_BROWSER_LIST_WINDOW_SIZE_DEFAULT <= FILE_BROWSER_MAX_SORTABLE_ITEMS_DEFAULT,
               "List window size cannot exceed max sortable items");
_Static_assert(FILE_BROWSER_LIST_WINDOW_SIZE_DEFAULT > 0,
               "List window size must be non-zero");
_Static_assert(FILE_BROWSER_LIST_WINDOW_STEP_DEFAULT <= (FILE_BROWSER_LIST_WINDOW_SIZE_DEFAULT / 2),
               "List window step cannot exceed window size");
_Static_assert(FILE_BROWSER_LIST_WINDOW_STEP_DEFAULT > 0,
               "List window step must be non-zero");
_Static_assert(FILE_BROWSER_MAX_SORTABLE_ITEMS_DEFAULT > 0,
               "Max sortable items must be non-zero");

typedef enum {
    FILE_BROWSER_ACTION_EDIT = 1,
    FILE_BROWSER_ACTION_DELETE = 2,
    FILE_BROWSER_ACTION_CANCEL = 3,
    FILE_BROWSER_ACTION_RENAME = 4,
    FILE_BROWSER_ACTION_COPY = 5,
    FILE_BROWSER_ACTION_CUT = 6,
} file_manager_action_type_t;

typedef struct {
    bool active;
    bool is_dir;
    bool is_txt;
    char name[FS_NAV_MAX_NAME];
    char directory[FS_NAV_MAX_PATH];
} file_manager_action_item_t;

typedef struct {
    bool has_item;
    bool cut; /* true = cut (move), false = copy */
    bool is_dir;
    char name[FS_NAV_MAX_NAME];
    char src_path[FS_NAV_MAX_PATH];
} file_manager_clipboard_t;

typedef struct{
    lv_obj_t *screen;
    lv_obj_t *path_label;
    lv_obj_t *settings_btn;
    lv_obj_t *tools_dd;
    lv_obj_t *datetime_btn;
    lv_obj_t *datetime_label;
    lv_obj_t *sort_overlay;
    lv_obj_t *date_time_overlay;
    lv_obj_t *sort_criteria_dd;
    lv_obj_t *sort_direction_dd;
    lv_obj_t *second_header;
    lv_obj_t *parent_btn;
    lv_obj_t *list;
    lv_obj_t *list_slider;
    lv_obj_t *folder_dialog;
    lv_obj_t *folder_textarea;
    lv_obj_t *folder_keyboard;
    lv_obj_t *paste_btn;
    lv_obj_t *paste_label;
    lv_obj_t *cancel_paste_btn;
    lv_obj_t *cancel_paste_label;
    lv_obj_t *action_mbox;
    lv_obj_t *confirm_mbox;
    lv_obj_t *paste_conflict_mbox;
    lv_obj_t *copy_confirm_mbox;
    lv_obj_t *loading_dialog;
    lv_obj_t *date_time_dialog;
    lv_obj_t *rename_dialog;
    lv_obj_t *rename_textarea;
    lv_obj_t *rename_keyboard;
    lv_timer_t *path_scroll_timer;
    lv_timer_t *list_scroll_timer;   
}file_manager_graphics_t;

typedef struct{
    bool initialized;
    bool clock_user_set;
    bool clock_timer_running;
    bool suppress_click;
    bool pending_go_parent;
    bool paste_target_valid;
    bool list_has_paged;
    bool list_at_top_edge;
    bool list_at_bottom_edge;
    bool list_suppress_scroll;
    bool slider_drag_active;
    bool slider_suppress_change;
    bool preserve_window_on_reload;
}file_manager_flags_t;

typedef struct {
    char paste_conflict_path[FS_NAV_MAX_PATH];
    char paste_conflict_name[FS_NAV_MAX_NAME];
    char paste_target_path[FS_NAV_MAX_PATH];    
    file_manager_action_item_t action_item;
    file_manager_clipboard_t clipboard;
    file_manager_graphics_t graphics;
    file_manager_flags_t flags;
    esp_timer_handle_t clock_timer;
    size_t slider_pending_step;
    size_t reload_anchor_index;    
    size_t list_window_start;
    size_t list_window_size;
    fs_nav_t nav;
} file_manager_ctx_t;

static file_manager_ctx_t s_file_manager;                /* Singleton UI context */
static TaskHandle_t file_manager_wait_task = NULL;  /* Task used to wait for sdspi reconnection after a failure */

/***************************************** Image Helpers *****************************************/
/**
 * @brief Returns true if filename has a known image extension.
 *
 * Current formats: PNG/JPG/JPEG/BMP/GIF (case-insensitive).
 */
static bool is_file_image(const char *name);

/**
 * @brief Returns true if filename ends in .jpg or .jpeg (case-insensitive).
 */
static bool is_file_jpeg(const char *name);

/**
 * @brief Item click handler for JPEG files (path composition + view stub).
 *
 * @param ctx   Active browser context.
 * @param item  Navigator item selected from the list.
 */
static void handle_jpeg(file_manager_ctx_t *ctx, const fs_nav_item_t *item);

/**
 * @brief Compose the full filesystem path for a JPEG item.
 *
 * @param ctx Browser context.
 * @param item Navigator item.
 * @param[out] path Output buffer.
 * @param path_len Output buffer size.
 * @return true on success, false on failure.
 */
static bool handle_jpeg_compose_path(file_manager_ctx_t *ctx, const fs_nav_item_t *item, char *path, size_t path_len);

/**
 * @brief Convert absolute path to LVGL storage path ("S:" + relative).
 *
 * @param relative Relative path (after mount point).
 * @param item_name Item name for logging.
 * @param[out] lv_path Output buffer.
 * @param lv_path_len Output buffer size.
 * @return true on success, false on failure.
 */
static bool handle_jpeg_build_lv_path(const char *relative, const char *item_name, char *lv_path, size_t lv_path_len);

/**
 * @brief Open JPEG viewer and handle error prompts.
 *
 * @param ctx Browser context.
 * @param lv_path LVGL path to open.
 * @param full_path Absolute path (for logging).
 */
static void handle_jpeg_open_with_prompts(file_manager_ctx_t *ctx, const char *lv_path, const char *full_path);

/************************************ UI & Data Refresh Helpers ***********************************/

/**
 * @brief Launch a helper task that waits for SD reconnection.
 *
 * Creates @c wait_for_reconnection_task if it is not already
 * running. The helper blocks on the @ref reconnection_success semaphore and,
 * once the SD retry flow signals recovery, refreshes the browser view.
 */
static void schedule_wait_for_reconnection(void);

/**
 * @brief Worker that blocks until SD reconnection completes, then reloads UI.
 *
 * Waits indefinitely on @ref reconnection_success. Once the semaphore is given
 * (meaning @ref sd_card_retry_init succeeded) it calls @ref refresh_current_dir.
 * If the reload fails the device restarts to recover from the fatal state.
 *
 * @param arg Unused.
 */
static void wait_for_reconnection_task(void* arg);

/**
 * @brief Wait on the reconnection semaphore and set retry flag on failure.
 *
 * @param schedule_retry Output flag set true when waiting fails.
 * @return true if semaphore was taken, false otherwise.
 */
static bool wait_task_take_reconnection_sem(bool *schedule_retry);

/**
 * @brief Handle reconnection flow when the browser is initialized.
 *
 * Processes pending parent navigation and refreshes the current directory unless
 * an earlier step requested a retry.
 *
 * @param ctx Browser context.
 * @param schedule_retry In/out flag to track retry necessity.
 */
static void wait_task_handle_initialized(file_manager_ctx_t *ctx, bool *schedule_retry);

/**
 * @brief Attempt pending parent navigation after reconnection.
 *
 * Clears pending_go_parent and tries fs_nav_go_parent, scheduling retry on error.
 *
 * @param ctx Browser context.
 * @param schedule_retry In/out flag to track retry necessity.
 */
static void wait_task_handle_pending_parent(file_manager_ctx_t *ctx, bool *schedule_retry);

/**
 * @brief Refresh the current directory after reconnection if no retry is pending.
 *
 * @param ctx Browser context.
 * @param schedule_retry In/out flag to track retry necessity.
 */
static void wait_task_refresh_after_reconnect(file_manager_ctx_t *ctx, bool *schedule_retry);

/**
 * @brief Restart the file manager when it was not initialized at reconnection time.
 *
 * @param schedule_retry In/out flag to track retry necessity.
 */
static void wait_task_restart_if_needed(bool *schedule_retry);

/**
 * @brief Schedule SD card retry when requested.
 *
 * @param schedule_retry Retry flag.
 */
static void wait_task_schedule_retry(bool schedule_retry);

/**
 * @brief Clear the wait task handle and delete the current task.
 */
static void wait_task_cleanup_and_delete(void);

/**
 * @brief Stop and delete a previous clock timer if it existed.
 *
 * @param timer Clock timer handle.
 * @param was_running True if it was running.
 */
static void cleanup_previous_clock_timer(esp_timer_handle_t timer, bool was_running);

/**
 * @brief Delete previous LVGL timers and screen after locking the display.
 *
 * @param old_screen Previous screen object (nullable).
 * @param old_path_timer Previous path scroll timer (nullable).
 * @param old_list_timer Previous list scroll timer (nullable).
 */
static void cleanup_previous_ui(lv_obj_t *old_screen, lv_timer_t *old_path_timer, lv_timer_t *old_list_timer);

/**
 * @brief Build the LVGL screen hierarchy (main header + path + secondary header + list).
 *
 * Creates the root screen and child widgets:
 * - Main header with settings button and tools dropdown.
 * - Path label of the current absolute path.
 * - Secondary header row with parent button on the left and paste/cancel pinned to the right.
 * - Item list (file/folder items).
 *
 * @param[in,out] ctx Browser context (must be non-NULL).
 * @internal UI construction only; does not query filesystem.
 */
static void build_file_manager_screen(file_manager_ctx_t *ctx);

/**
 * @brief Click handler for the header "Set Date&Time" button.
 *
 * Delegates to the shared settings dialog to pick a date&time.
 *
 * @param e LVGL event (CLICKED) with user data = file_manager_ctx_t*.
 */
static void on_datetime_click(lv_event_t *e);

/**
 * @brief Build the date&time dialog overlay for the file manager.
 *
 * Creates an overlay with Wi-Fi & SNTP and manual set.
 *
 * @param ctx Active file manager context.
 */
static void build_date_time_dialog(file_manager_ctx_t *ctx);

/**
 * @brief Close and destroy the date&time dialog overlay.
 *
 * @param e LVGL event (CLICKED) with user data = file_manager_ctx_t*.
 */
static void close_date_time_dialog(lv_event_t *e);

/**
 * @brief Open the manual date&time picker from the settings module.
 *
 * @param e LVGL event (CLICKED) with user data = file_manager_ctx_t*.
 */
static void manual_date_time(lv_event_t *e);

/**
 * @brief Trigger SNTP refresh via the settings dialog.
 *
 * @param e LVGL event (CLICKED) with user data = file_manager_ctx_t*.
 */
static void sntp_date_time(lv_event_t *e);

/**
 * @brief Start the periodic clock timer (esp_timer) to refresh the header clock label.
 *
 * Creates the timer on first call, then starts it if not already running.
 *
 * @param ctx Active file browser context.
 */
static void start_clock_timer(file_manager_ctx_t *ctx);

/**
 * @brief esp_timer callback fired every second to request a clock label refresh.
 *
 * Posts an async call into the LVGL context to update the label.
 *
 * @param arg Unused.
 */
static void clock_timer_cb(void *arg);

/**
 * @brief LVGL-context callback to update the clock label with current time/date.
 *
 * Formats HH:MM - MM/DD/YY and toggles visibility between the label and the
 * placeholder button once a valid time is set.
 *
 * @param arg Unused.
 */
static void clock_update_async(void *arg);

/**
 * @brief Check if datetime label exists before updating clock.
 *
 * @param ctx Browser context.
 * @return true if label exists, false otherwise.
 */
static bool has_datetime_label(const file_manager_ctx_t *ctx);

/**
 * @brief Handle clock UI when no user-set time is available.
 *
 * Shows the button and hides the label.
 *
 * @param ctx Browser context.
 */
static void clock_update_handle_not_set(file_manager_ctx_t *ctx);

/**
 * @brief Format current time/date into buffer.
 *
 * @param buf Output buffer.
 * @param buf_len Buffer length.
 */
static void clock_update_format(char *buf, size_t buf_len);

/**
 * @brief Apply formatted time to label and toggle visibility.
 *
 * @param ctx Browser context.
 * @param buf Formatted time string.
 */
static void clock_update_apply(file_manager_ctx_t *ctx, const char *buf);

/**
 * @brief Restarts the delayed scrolling animation for the path label.
 *
 * This function cancels any existing scroll-start timer, forces the path label into
 * clipped mode, and creates a new one-shot timer that will re-enable circular
 * scrolling after FILE_BROWSER_PATH_SCROLL_DELAY_MS_DEFAULT milliseconds.
 *
 * It is typically used whenever the displayed path changes, ensuring the scroll
 * animation restarts cleanly and does not begin immediately.
 *
 * @param ctx Pointer to the file browser UI context. Must contain a valid path_label.
 */
static void restart_path_scroll(file_manager_ctx_t *ctx);

/**
 * @brief Timer callback used to enable scrolling for the file browser path label.
 *
 * This function is invoked after a short delay to switch the path label's long mode
 * from clipped (LV_LABEL_LONG_CLIP) to circular scrolling (LV_LABEL_LONG_SCROLL_CIRCULAR).
 * The delay prevents immediate scrolling and makes the UI feel smoother when paths change.
 *
 * @param timer Pointer to the LVGL timer that triggered the callback.
 *              Its user_data must contain a valid file_manager_ctx_t*.
 */
static void path_scroll_timer_cb(lv_timer_t *timer);

/**
 * @brief Restart delayed scrolling for list item labels.
 *
 * Forces all visible item labels into clipped mode, then schedules a one-shot
 * timer to re-enable circular scrolling after FILE_BROWSER_ENTRY_SCROLL_DELAY_MS_DEFAULT.
 *
 * @param ctx Browser context containing the list widget.
 */
static void restart_entry_scroll(file_manager_ctx_t *ctx);

/**
 * @brief Return the label child inside a list button, if any.
 *
 * @param btn List button object created via lv_list_add_btn.
 * @return Pointer to the label child or NULL if not found.
 */
static lv_obj_t *get_list_btn_label(lv_obj_t *btn);

/**
 * @brief Timer callback that enables scrolling for list item labels.
 *
 * @param timer LVGL timer (user_data = file_manager_ctx_t*).
 */
static void entry_scroll_timer_cb(lv_timer_t *timer);

/**
 * @brief Reset the virtual list window to the first page.
 *
 * @param[in,out] ctx Browser context.
 */
 static void reset_window(file_manager_ctx_t *ctx);

/**
 * @brief Rebuild the visible list window and reposition scroll/anchor.
 *
 * @param[in,out] ctx Browser context.
 * @param start_index Global item index to start the window from.
 * @param anchor_index Global item index to keep visible/centered (SIZE_MAX to skip).
 * @param center_anchor True to center the anchor item, false to align it near top.
 * @param scroll_to_top Fallback scroll when no anchor: true = top, false = bottom.
 */
static void build_item_list(file_manager_ctx_t *ctx, size_t start_index, size_t anchor_index, bool center_anchor, bool scroll_to_top);

/**
 * @brief Check if build_item_list has a valid context and list to operate on.
 *
 * @param ctx Browser context.
 * @return true if ctx and ctx->graphics.list are valid, false otherwise.
 */
static bool is_item_list_valid(const file_manager_ctx_t *ctx);

/**
 * @brief Set the navigator window and reset edge flags.
 *
 * Logs and schedules retries on failure.
 *
 * @param ctx Browser context.
 * @param start_index Starting index for the window.
 * @return ESP_OK on success or error from fs_nav_set_window.
 */
static esp_err_t is_item_set_nav(file_manager_ctx_t *ctx, size_t start_index);

/**
 * @brief Refresh list contents and related UI while suppressing scroll events.
 *
 * Sets list_suppress_scroll, shows loading, repopulates list, updates layout and slider.
 *
 * @param ctx Browser context.
 */
static void refresh_item_list(file_manager_ctx_t *ctx);

/**
 * @brief Find the anchor LVGL object corresponding to an item index.
 *
 * @param ctx Browser context.
 * @param anchor_index Global item index to anchor (SIZE_MAX to skip).
 * @return LVGL object for the anchor or NULL if not found/in range.
 */
static lv_obj_t *item_list_find_anchor(file_manager_ctx_t *ctx, size_t anchor_index);

/**
 * @brief Perform scroll positioning based on anchor and paging state.
 *
 * @param ctx Browser context.
 * @param anchor_obj Anchor object (nullable).
 * @param center_anchor True to center the anchor item.
 * @param scroll_to_top Fallback scroll when no anchor: true = top, false = bottom.
 */
static void item_list_scroll_to_anchor(file_manager_ctx_t *ctx, lv_obj_t *anchor_obj, bool center_anchor, bool scroll_to_top);

/**
 * @brief Restore list scroll suppression flag to its previous value.
 *
 * @param ctx Browser context.
 * @param prev_suppress Previous flag value to restore.
 */
static void item_window_restore_scroll_flag(file_manager_ctx_t *ctx, bool prev_suppress);

/**
 * @brief Helper to set a sensible reload anchor when none is provided.
 *
 * Uses the middle of the current window (clamped later) so reloads return near
 * the current view instead of the top of the window.
 *
 * @param[in,out] ctx Browser context.
 */
static void set_reload_anchor_current(file_manager_ctx_t *ctx);

/**
 * @brief Synchronize all UI elements with the current navigation state.
 *
 * Updates path, sort badges, and repopulates the list with current items.
 *
 * @param[in,out] ctx Browser context.
 */
static void sync_view(file_manager_ctx_t *ctx);

/**
 * @brief Validate presence of second-header widgets (parent/paste/cancel).
 *
 * @param[in,out] ctx Browser context.
 * 
 * @return true if all required controls exist; false otherwise.
 */
static bool check_second_header(file_manager_ctx_t *ctx);

/**
 * @brief Refresh visibility/state of the second header (parent + paste/cancel).
 *
 * Updates parent/paste/cancel controls and hides the row when neither parent
 * navigation nor paste actions are available.
 * 
 * @param[in,out] ctx Browser context.
 */
static void update_second_header(file_manager_ctx_t *ctx);

/**
 * @brief Show/hide the parent navigation button depending on hierarchy depth.
 *
 * @param[in,out] ctx Browser context.
 */
static void update_parent_button(file_manager_ctx_t *ctx);

/**
 * @brief Update the path label from the current navigator path.
 *
 * @param[in,out] ctx Browser context.
 */
 static void update_path_label(file_manager_ctx_t *ctx);

/**
 * @brief Update the sort mode and direction badges.
 *
 * @param[in,out] ctx Browser context.
 */
 static void update_sort_badges(file_manager_ctx_t *ctx);

/**
 * @brief Rebuild the item list from current directory contents.
 *
 * Renders a window of items starting at @c ctx->list_window_start for
 * @c ctx->list_window_size items (clamped to available items). For files, a
 * formatted size is shown; for directories, the number of immediate children
 * is shown. The parent item is rendered separately above the list (when
 * available).
 *
 * @param[in,out] ctx Browser context.
 */
 static void populate_list(file_manager_ctx_t *ctx);

/**
 * @brief Stop and clear any existing entry scroll timer.
 *
 * @param ctx Browser context.
 */
static void populate_list_clear_scroll_timer(file_manager_ctx_t *ctx);

/**
 * @brief Remove all children from the list widget.
 *
 * @param ctx Browser context.
 */
static void populate_list_clear_list(file_manager_ctx_t *ctx);

/**
 * @brief Retrieve navigator items and their count.
 *
 * @param ctx Browser context.
 * @param[out] count Number of items returned.
 * @return Pointer to items array or NULL.
 */
static const fs_nav_item_t *populate_list_get_items(file_manager_ctx_t *ctx, size_t *count);

/**
 * @brief Handle empty folder case by showing placeholder text.
 *
 * @param ctx Browser context.
 * @param items Item array pointer.
 * @param count Item count.
 * @return true if handled (empty), false otherwise.
 */
static bool populate_list_handle_empty(file_manager_ctx_t *ctx, const fs_nav_item_t *items, size_t count);

/**
 * @brief Get current window start offset.
 *
 * @param ctx Browser context.
 * @return Window start index.
 */
static size_t populate_list_window_start(file_manager_ctx_t *ctx);

/**
 * @brief Format list entry text for a file item.
 *
 * @param item Item descriptor.
 * @param display_index 1-based display index.
 * @param[out] out Output buffer.
 * @param out_len Output buffer length.
 */
static void populate_list_format_file_text(const fs_nav_item_t *item, size_t display_index, char *out, size_t out_len);

/**
 * @brief Format list entry text for a directory item (with child count).
 *
 * @param ctx Browser context.
 * @param item Item descriptor.
 * @param display_index 1-based display index.
 * @param[out] out Output buffer.
 * @param out_len Output buffer length.
 */
static void populate_list_format_dir_text(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t display_index, char *out, size_t out_len);

/**
 * @brief Get the icon symbol for an item.
 *
 * @param item Item descriptor.
 * @return Icon string.
 */
static const char *populate_list_icon_for_item(const fs_nav_item_t *item);

/**
 * @brief Create and configure a list button for an item.
 *
 * @param ctx Browser context.
 * @param item Item descriptor.
 * @param rel_index Relative index inside window.
 * @param icon Icon symbol.
 * @param text Button label text.
 */
static void populate_list_create_button(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t rel_index, const char *icon, const char *text);

/**
 * @brief Count the number of items inside a directory.
 *
 * This function checks whether the given item represents a directory,
 * builds its full path, opens it, and counts all items inside it except
 * the special items "." and "..".
 *
 * @param[in]  ctx        File browser context. Must not be NULL.
 * @param[in]  item      Directory item to inspect. Must represent a directory.
 * @param[out] out_count  Output pointer where the number of items will be stored.
 *
 * @return true on success, false on invalid parameters, path composition failure,
 *         directory open failure, or any other error.
 */
 static bool count_dir_items(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t *out_count);

/**
 * @brief Format a byte size into a short human-friendly string.
 *
 * Uses B/KB/MB/GB up to 1 decimal place for KB or larger.
 *
 * @param bytes   Size in bytes.
 * @param[out] out Output buffer for the formatted text.
 * @param out_len Length of @p out.
 */
 static void format_size(size_t bytes, char *out, size_t out_len);

/**
 * @brief Refresh the current directory view and redraw the list.
 *
 * Re-reads directory items via @c fs_nav_refresh and repopulates the LVGL list.
 * If @c preserve_window_on_reload is true, keeps the current virtual window/anchor
 * (clamped to the new totals); otherwise resets to the first window.
 *
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_STATE if the browser was not started
 * - Error from @c fs_nav_refresh
 * - ESP_ERR_TIMEOUT if display lock cannot be acquired
 */
 static esp_err_t refresh_current_dir(void);

/**
 * @brief Check that the file manager is initialized.
 *
 * @param ctx Browser context.
 * @return true if initialized, false otherwise.
 */
static bool refresh_dir_is_initialized(const file_manager_ctx_t *ctx);

/**
 * @brief Refresh navigator items.
 *
 * @param ctx Browser context.
 * @return ESP_OK or error from fs_nav_refresh.
 */
static esp_err_t refresh_dir_nav_refresh(file_manager_ctx_t *ctx);

/**
 * @brief Apply preserved window positioning and anchor selection.
 *
 * @param ctx Browser context.
 * @param saved_start Previous list window start.
 */
static void refresh_dir_preserve_window(file_manager_ctx_t *ctx, size_t saved_start);

/**
 * @brief Decide whether to preserve or reset the window based on flags.
 *
 * @param ctx Browser context.
 * @param saved_start Previous list window start.
 */
static void refresh_dir_handle_window(file_manager_ctx_t *ctx, size_t saved_start);

/**
 * @brief Try to lock the display for LVGL updates.
 *
 * @return true if lock acquired, false otherwise.
 */
static bool refresh_dir_lock_display(void);

/**
 * @brief Update UI after refresh while the display is locked.
 *
 * @param ctx Browser context.
 */
static void refresh_dir_apply_ui_updates(file_manager_ctx_t *ctx);

/**************************************************************************************************/


/***************************** List Interactions & Text Editor Bridge *****************************/

/**
 * @brief Item click handler: enter directories, open viewers or show prompt.
 *
 * If the clicked item is a directory, enters it and refreshes the view.
 * If it is a supported file, opens an appropriate viewer. Otherwise, shows
 * an informational prompt.
 *
 * @param e LVGL event (CLICKED) with user data = @c file_manager_ctx_t*.
 */
 static void on_item_click(lv_event_t *e);

/**
 * @brief Scroll handler for the item list (virtual window paging).
 *
 * @param e LVGL event (LV_EVENT_SCROLL) with user data = @c file_manager_ctx_t*.
 */
static void on_list_scrolled(lv_event_t *e);

/**
 * @brief Handle slider press/drag/release to jump between list windows.
 *
 * Tracks the target step while dragging and applies the list jump only on release.
 * If the knob returns to the current step, no reload is triggered.
 *
 * @param e LVGL slider event (pressed/value changed/released) with user data = file_manager_ctx_t*.
 */
static void on_slider_value_changed(lv_event_t *e);

/**
 * @brief Clear suppressed click flag if set.
 *
 * @param ctx Browser context.
 * @return true if click was suppressed and handled, false otherwise.
 */
static bool item_click_handle_suppress(file_manager_ctx_t *ctx);

/**
 * @brief Fetch the clicked item and index from the event.
 *
 * @param ctx Browser context.
 * @param e   LVGL event.
 * @param[out] out_item Returned item pointer.
 * @param[out] out_index Returned item index.
 * @return true on success, false on invalid state.
 */
static bool item_click_get_item(file_manager_ctx_t *ctx, lv_event_t *e, const fs_nav_item_t **out_item, size_t *out_index);

/**
 * @brief Handle directory click (enter and refresh) if applicable.
 *
 * @param ctx Browser context.
 * @param item Clicked item.
 * @param index Item index.
 * @return true if handled as directory, false otherwise.
 */
static bool item_click_handle_dir(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t index);

/**
 * @brief Handle TXT click by opening text viewer.
 *
 * @param ctx Browser context.
 * @param item Clicked item.
 * @param index Item index.
 * @return true if handled as TXT, false otherwise.
 */
static bool item_click_handle_txt(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t index);

/**
 * @brief Handle JPEG click.
 *
 * @param ctx Browser context.
 * @param item Clicked item.
 * @return true if handled as JPEG, false otherwise.
 */
static bool item_click_handle_jpeg(file_manager_ctx_t *ctx, const fs_nav_item_t *item);

/**
 * @brief Evaluate scroll position and update paging state at top/bottom edges.
 *
 * @param ctx Browser context.
 */
static void list_scroll_update_edges(file_manager_ctx_t *ctx);

/**
 * @brief Attempt to page down when near bottom edge.
 *
 * @param ctx Browser context.
 */
static void list_scroll_handle_bottom(file_manager_ctx_t *ctx);

/**
 * @brief Attempt to page up when near top edge.
 *
 * @param ctx Browser context.
 */
static void list_scroll_handle_top(file_manager_ctx_t *ctx);

/**
 * @brief Guard for slider event handling (ctx and suppress flag).
 *
 * @param ctx Browser context.
 * @return true if processing is allowed, false otherwise.
 */
static bool slider_can_handle_event(const file_manager_ctx_t *ctx);

/**
 * @brief Return true if all items fit in one window.
 *
 * @param total Total items.
 * @param window_size Window size.
 * @return true if no paging needed.
 */
static bool slider_total_fits_window(size_t total, size_t window_size);

/**
 * @brief Compute maximum step index.
 *
 * @param max_start Maximum start index.
 * @param step Step size.
 * @return Max step index.
 */
static size_t slider_max_step_index(size_t max_start, size_t step);

/**
 * @brief Clamp slider value to valid step range.
 *
 * @param max_step_index Max step index.
 * @param slider_val Raw slider value.
 * @return Clamped step value.
 */
static size_t slider_clamp_value(size_t max_step_index, int32_t slider_val);

/**
 * @brief Handle pressed event: start tracking drag.
 *
 * @param ctx Browser context.
 * @param clamped_step Current step.
 * @param code Event code.
 * @return true if handled.
 */
static bool slider_handle_pressed(file_manager_ctx_t *ctx, size_t clamped_step, lv_event_code_t code);

/**
 * @brief Handle value changed event during drag.
 *
 * @param ctx Browser context.
 * @param clamped_step Current step.
 * @param code Event code.
 * @return true if handled.
 */
static bool slider_handle_value_changed(file_manager_ctx_t *ctx, size_t clamped_step, lv_event_code_t code);

/**
 * @brief Resolve target step on release.
 *
 * @param ctx Browser context.
 * @param clamped_step Clamped current step.
 * @param max_step_index Max step index.
 * @return Target step.
 */
static size_t slider_target_step(file_manager_ctx_t *ctx, size_t clamped_step, size_t max_step_index);

/**
 * @brief Compute current step from window start.
 *
 * @param ctx Browser context.
 * @param step Step size.
 * @param max_start Max start index.
 * @param max_step_index Max step index.
 * @return Current step.
 */
static size_t slider_current_step(const file_manager_ctx_t *ctx, size_t step, size_t max_start, size_t max_step_index);

/**
 * @brief Check if target step matches current, clearing drag state.
 *
 * @param ctx Browser context.
 * @param target_step Target step.
 * @param current_step Current step.
 * @return true if no-op.
 */
static bool slider_is_noop(file_manager_ctx_t *ctx, size_t target_step, size_t current_step);

/**
 * @brief Compute new window start from step.
 *
 * @param target_step Target step.
 * @param max_step_index Max step index.
 * @param step Step size.
 * @param max_start Max start index.
 * @return New start index.
 */
static size_t slider_compute_new_start(size_t target_step, size_t max_step_index, size_t step, size_t max_start);

/**
 * @brief Finalize paging after slider release.
 *
 * @param ctx Browser context.
 * @param new_start New start index.
 */
static void slider_finalize_paging(file_manager_ctx_t *ctx, size_t new_start);

/**
 * @brief Sync the slider range/value to the current list window.
 *
 * Recomputes slider bounds from total items and step size, clamps the knob to
 * the active step (including the last window), disables the slider when only
 * one window exists, and tracks the pending step for drag handling.
 *
 * @param[in,out] ctx Browser context with list and slider state.
 */
static void update_slider(file_manager_ctx_t *ctx);

/**
 * @brief Validate that slider-related objects exist.
 *
 * @param ctx Browser context.
 * @return true if ctx and list slider are valid, false otherwise.
 */
static bool slider_is_valid(const file_manager_ctx_t *ctx);

/**
 * @brief Get the list row container for padding adjustments.
 *
 * @param ctx Browser context.
 * @return Parent object of the list or NULL.
 */
static lv_obj_t *slider_get_list_row(const file_manager_ctx_t *ctx);

/**
 * @brief Disable slider when all items fit in one window.
 *
 * Preserves suppress flag, resets value, disables drag state and applies padding.
 *
 * @param ctx Browser context.
 * @param list_row Parent row containing the list (nullable).
 */
static void slider_disable_for_single_window(file_manager_ctx_t *ctx, lv_obj_t *list_row);

/**
 * @brief Compute the maximum starting index for the slider.
 *
 * @param total Total items.
 * @param window_size Items per window.
 * @return Maximum start index.
 */
static size_t slider_max_start(size_t total, size_t window_size);

/**
 * @brief Compute the current slider step for the active window.
 *
 * @param ctx Browser context.
 * @param max_start Maximum start index.
 * @param max_step_index Maximum step index.
 * @param step Step size.
 * @param max_val Clamping value.
 * @return Current step index.
 */
static size_t slider_compute_current_step(const file_manager_ctx_t *ctx, size_t max_start, size_t max_step_index, size_t step, int32_t max_val);

/**
 * @brief Apply slider range/value while suppressing change callbacks.
 *
 * @param ctx Browser context.
 * @param max_val Maximum slider value.
 * @param current_step Current step to set.
 */
static void slider_apply_range_and_value(file_manager_ctx_t *ctx, int32_t max_val, size_t current_step);

/**
 * @brief Enable slider and adjust padding when multiple windows exist.
 *
 * @param ctx Browser context.
 * @param list_row Parent row containing the list (nullable).
 */
static void enable_slider(file_manager_ctx_t *ctx, lv_obj_t *list_row);

/**
 * @brief Resolve window size and step with safe defaults.
 *
 * Uses context/config to produce non-zero values for both the window size and the step.
 *
 * @param[in]  ctx          Browser context.
 * @param[out] window_size  Effective items-per-window (>=1).
 * @param[out] step         Effective step size (>=1).
 */
static void get_window_params(file_manager_ctx_t *ctx, size_t *window_size, size_t *step);

/**
 * @brief Show an informational prompt for unsupported file formats.
 */
static void show_unsupported_prompt(void);

/**
 * @brief Show an informational prompt for too big image resolution.
 */
static void show_image_resolution_too_large_to_display_prompt(void);

/**
 * @brief Show an informational prompt for not enough memory or image too large.
 */
static void show_not_enough_memory_prompt(void);

/**
 * @brief Show an informational prompt for unsupported jpeg formats.
 */
static void show_jpeg_unsupported_prompt(void);

/**
 * @brief Close handler for the unsupported-format prompt.
 *
 * @param e LVGL event (CLICKED) with user data = message box to close.
 */
static void close_unsupported_msgbox(lv_event_t *e);

/**
 * @brief Long-press handler for a list item to open the action menu.
 *
 * Marks the click as suppressed (to avoid triggering the normal click handler),
 * resolves the pressed item index, prepares the action item and shows
 * the action menu.
 *
 * @param e LVGL event (LV_EVENT_LONG_PRESSED) with user data = @c file_manager_ctx_t*.
 */
 static void on_item_long_press(lv_event_t *e);

/**
 * @brief Parent button handler: go up one level (if possible).
 *
 * @param e LVGL event (CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_parent_click(lv_event_t *e);

/**
 * @brief Open the settings screen when the toolbar settings button is clicked.
 *
 * Retrieves the browser context from event user data, guards null pointers,
 * and delegates to @ref settings_open_settings. Logs an error on failure.
 */
static void on_settings_click(lv_event_t *e);

/**
 * @brief Tools dropdown handler (New Folder / New TXT / Sort).
 *
 * @param e LVGL event (VALUE_CHANGED) with user data = @c file_manager_ctx_t*.
 */
static void on_tools_changed(lv_event_t *e);

/**
 * @brief Sort criteria dropdown handler.
 *
 * Triggered when the user changes the sorting field (e.g., name, size, date).
 * Only retrieves the context; actual application happens on "Apply".
 *
 * @param e LVGL event (e.g., LV_EVENT_VALUE_CHANGED) with user data = @c file_manager_ctx_t*.
 */
static void on_sort_criteria_changed(lv_event_t *e);

/**
 * @brief Sort direction dropdown handler.
 *
 * Triggered when the user switches between ascending/descending sorting.
 * Only retrieves the context; actual application happens on "Apply".
 *
 * @param e LVGL event (e.g., LV_EVENT_VALUE_CHANGED) with user data = @c file_manager_ctx_t*.
 */
static void on_sort_direction_changed(lv_event_t *e);

/**
 * @brief Apply the selected sorting mode to the file browser.
 *
 * Updates the navigator sorting mode and direction, refreshes sort badges,
 * resets the list window, and repopulates the visible file list.
 *
 * @param ctx File browser context owning sorting state and UI elements.
 * @param mode Sorting mode to apply (name/date/size).
 * @param ascending True for ascending order, false for descending.
 */
static void apply_sort(file_manager_ctx_t *ctx, fs_nav_sort_mode_t mode, bool ascending);

/**
 * @brief Display the sorting dialog overlay.
 *
 * Creates a modal dialog containing sort criteria and direction dropdowns,
 * along with "Apply" and "Cancel" actions. Automatically closes any existing
 * sort panel before creating a new one.
 *
 * @param ctx File browser context used to populate and manage the dialog.
 */
static void show_sort_dialog(file_manager_ctx_t *ctx);

/**
 * @brief Close and destroy the sorting dialog.
 *
 * Removes the overlay dialog from screen, clears dialog-related pointers,
 * and resets internal dialog state.
 *
 * @param ctx File browser context that owns the dialog instance.
 */
static void close_sort_dialog(file_manager_ctx_t *ctx);

/**
 * @brief "Apply" button handler for the sort dialog.
 *
 * Reads the selected sort criteria and direction from dropdowns,
 * updates the navigator sorting state, refreshes the file list,
 * and closes the sort dialog.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_sort_apply(lv_event_t *e);

/**
 * @brief "Cancel" button handler for the sort dialog.
 *
 * Closes the sort dialog without applying any changes
 * and updates the sort badges in the toolbar.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_sort_cancel(lv_event_t *e);

/**
 * @brief Callback invoked when the text editor/viewer screen is closed.
 *
 * If the content was changed, triggers a browser reload to reflect updates
 * (file size, timestamp, new file, etc.).
 *
 * @param changed  True if the editor modified the file.
 * @param user_ctx User context, expected to be @c file_manager_ctx_t*.
 */
static void editor_closed(bool changed, void *user_ctx);

/**
 * @brief Start the "New TXT" creation flow by opening the text editor.
 *
 * Creates a new editable text document inside the current navigator
 * directory. A default filename ("new_file.txt") is suggested to the
 * editor. When the editor is closed, the file browser is notified
 * through @c editor_closed().
 *
 * On failure to open the editor, an error is logged and an SD-card
 * retry is scheduled to handle potential transient I/O issues.
 *
 * @param ctx File browser context providing navigation state and UI targets.
 */
static void start_new_txt(file_manager_ctx_t *ctx);

/**
 * @brief Start the "New Folder" flow by opening the folder creation dialog.
 *
 * Opens the folder creation dialog for the current navigator path.
 *
 * @param ctx File browser context used to dispatch the dialog.
 */
static void start_new_folder(file_manager_ctx_t *ctx);

/**************************************************************************************************/


/************************************* Folder Creation Dialog *************************************/

/**
 * @brief Show the "Create folder" dialog overlay.
 *
 * Creates a semi-transparent overlay with a card containing title, status label,
 * folder-name text area, action buttons and an on-screen keyboard. The dialog is
 * stored in @c ctx->graphics.folder_dialog and related pointers.
 *
 * @param[in,out] ctx Browser context that owns the dialog.
 */
 static void show_folder_dialog(file_manager_ctx_t *ctx);

/**
 * @brief Close and destroy the "Create folder" dialog overlay.
 *
 * Deletes the overlay object and clears all folder dialog-related pointers
 * in the context.
 *
 * @param[in,out] ctx Browser context that owns the dialog.
 */
 static void close_folder_dialog(file_manager_ctx_t *ctx);

/**
 * @brief Handle the folder creation action from the dialog.
 *
 * Triggered either by the "Create" button or keyboard READY event.
 * Validates the folder name, attempts to create it and updates status text
 * on error. On success, closes the dialog and reloads the browser view.
 *
 * @param e LVGL event with user data = @c file_manager_ctx_t*.
 */
static void on_folder_create(lv_event_t *e);

/**
 * @brief Cancel handler for the "Create folder" dialog.
 *
 * Simply closes the dialog without creating a folder.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_folder_cancel(lv_event_t *e);

/**
 * @brief Set status message and color in the "Create folder" dialog.
 *
 * Updates the folder status label text and chooses an error or neutral color
 * depending on the @p error flag.
 *
 * @param[in,out] ctx Browser context owning the folder status label.
 * @param msg         Message text to display (must be non-NULL).
 * @param error       True to use an error color, false for neutral/info color.
 */
static void set_folder_status(file_manager_ctx_t *ctx, const char *msg, bool error);

/**
 * @brief Create a folder in the current directory with the given name.
 *
 * Uses @c fs_nav_compose_path() to generate an absolute path and calls @c mkdir().
 *
 * @param[in,out] ctx Browser context providing the current path.
 * @param name        Folder name (already validated).
 * @return ESP_OK on success,
 *         ESP_ERR_INVALID_STATE if the folder already exists,
 *         ESP_FAIL on generic failure or errno-based errors.
 */
static esp_err_t create_folder(file_manager_ctx_t *ctx, const char *name);

/**
 * @brief Handles the cancel action from the folder creation keyboard.
 *
 * This callback is triggered when the user cancels or closes the keyboard
 * during folder creation. The function detaches the keyboard from the
 * textarea and hides the keyboard widget.
 *
 * @param e Pointer to the LVGL event descriptor.
 */
static void on_folder_keyboard_cancel(lv_event_t *e);

/**
 * @brief Shows the keyboard when the folder creation textarea is clicked.
 *
 * This callback is triggered when the user taps the folder name textarea.
 * It associates the on-screen keyboard with the textarea and makes the
 * keyboard visible for text input.
 *
 * @param e Pointer to the LVGL event descriptor.
 */
static void on_folder_textarea_clicked(lv_event_t *e);

/**************************************************************************************************/


/*********************************** Filesystem Utility Helpers ***********************************/

/**
 * @brief Check if a given name is a valid filesystem item name.
 *
 * Rejects empty strings and names containing '\', '/', ':', '*', '?', '"', '<', '>' or '|'.
 *
 * @param name Candidate name string.
 * @return true if the name is valid, false otherwise.
 */
 static bool is_valid_name(const char *name);

/**
 * @brief Trim leading and trailing whitespace characters from a string in-place.
 *
 * Whitespace considered: space, tab, newline and carriage return.
 *
 * @param[in,out] name String buffer to trim; may be shifted in memory.
 */
 static void trim_whitespace(char *name);

/**
 * @brief Recursively delete a path, which may be a file or directory tree.
 *
 * Uses @c stat() to determine whether the path is a directory. If so, iterates
 * over items with @c opendir()/readdir(), recursively deletes children and
 * finally removes the directory. If it is a file, calls @c remove().
 *
 * Missing paths (ENOENT) are treated as success.
 *
 * @param path Path to delete (must be non-empty string).
 * @return ESP_OK on success,
 *         ESP_ERR_INVALID_ARG for invalid input,
 *         ESP_ERR_INVALID_SIZE if child path buffer would overflow,
 *         ESP_FAIL on other filesystem/errno-based errors.
 */
 static esp_err_t delete_path(const char *path);

/**
 * @brief Validate input path for deletion.
 *
 * @param path Path string.
 * @return true if invalid, false otherwise.
 */
static bool delete_path_invalid_arg(const char *path);

/**
 * @brief stat() wrapper for delete_path with ENOENT mapped to ESP_ERR_NOT_FOUND.
 *
 * @param path Path to stat.
 * @param[out] st Filled stat structure on success.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if missing, ESP_FAIL otherwise.
 */
static esp_err_t delete_path_stat(const char *path, struct stat *st);

/**
 * @brief Check stat result for directory type.
 *
 * @param st Stat structure.
 * @return true if directory.
 */
static bool delete_path_is_dir(const struct stat *st);

/**
 * @brief Compose child path under a parent directory.
 *
 * @param parent Parent path.
 * @param name Child name.
 * @param[out] out Output buffer.
 * @param out_len Buffer length.
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE on overflow.
 */
static esp_err_t delete_path_compose_child(const char *parent, const char *name, char *out, size_t out_len);

/**
 * @brief Recursively delete a directory and its contents.
 *
 * @param path Directory path.
 * @return ESP_OK on success or error code.
 */
static esp_err_t delete_path_delete_dir(const char *path);

/**
 * @brief Delete a single file.
 *
 * @param path File path.
 * @return ESP_OK on success or ESP_FAIL on error.
 */
static esp_err_t delete_path_delete_file(const char *path);

/**
 * @brief Recursively accumulate byte size for a file or directory tree.
 *
 * @param path  Absolute path to a file or directory.
 * @param bytes In/out accumulator; on success, increased by the size found.
 * @return ESP_OK on success,
 *         ESP_ERR_INVALID_ARG for invalid input,
 *         ESP_ERR_INVALID_SIZE if a composed child path would overflow,
 *         ESP_FAIL on stat/opendir errors.
 */
static esp_err_t compute_total_size(const char *path, uint64_t *bytes);

/**************************************************************************************************/

/*************************************** Clipboard & Paste Helpers ********************************/

/**
 * @brief Update visibility and state of "Paste" and "Cancel Paste" buttons.
 *
 * When the clipboard is empty, both buttons are hidden and disabled.
 * When a clipboard item exists (copy or cut), both buttons are shown
 * and enabled, allowing the user to complete or cancel the paste action.
 *
 * @param ctx File browser context that owns the paste and cancel buttons.
 */
static void update_paste_button(file_manager_ctx_t *ctx);

/**
 * @brief "Paste" button handler (dispatches copy/cut flow).
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_paste_click(lv_event_t *e);

/**
 * @brief "Cancel Paste" button handler — clears clipboard and resets paste state.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 *
 * This function is triggered when the user clicks the "Cancel Paste" button.
 * It clears the current clipboard contents and updates the paste button state
 * to reflect that no copy/move operation is in progress.
 */
static void on_cancel_paste_click(lv_event_t *e);

/**
 * @brief Show overwrite/rename prompt when paste destination already exists.
 *
 * @param ctx       Browser context.
 * @param dest_path Absolute destination path that already exists.
 */
static void show_paste_conflict(file_manager_ctx_t *ctx, const char *dest_path);

/**
 * @brief Close the paste conflict dialog if present.
 *
 * @param ctx Browser context.
 */
static void close_paste_conflict(file_manager_ctx_t *ctx);

/**
 * @brief Handle overwrite/rename/cancel selection from paste conflict dialog.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_paste_conflict(lv_event_t *e);

/**
 * @brief Show copy confirmation prompt with total size (used on Paste for copy).
 *
 * @param ctx   Browser context (requires clipboard + target set).
 * @param bytes Total bytes to be copied.
 */
static void show_copy_confirm(file_manager_ctx_t *ctx, uint64_t bytes);

/**
 * @brief Close copy confirmation prompt if present.
 *
 * @param ctx Browser context.
 */
static void close_copy_confirm(file_manager_ctx_t *ctx);

/**
 * @brief Handle copy confirmation buttons (OK/Cancel).
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_copy_confirm(lv_event_t *e);

/**
 * @brief Show a loading overlay during long copy/cut operations.
 *
 * @param ctx Browser context.
 */
static void show_loading(file_manager_ctx_t *ctx);

/**
 * @brief Hide the loading overlay if present.
 *
 * @param ctx Browser context.
 */
static void hide_loading(file_manager_ctx_t *ctx);

/**
 * @brief Execute copy or cut into destination path.
 *
 * @param ctx Browser context with an active clipboard.
 * @param dest_path Destination absolute path.
 * @param allow_overwrite True to delete an existing destination before writing.
 */
static esp_err_t perform_paste(file_manager_ctx_t *ctx, const char *dest_path, bool allow_overwrite);

/**
 * @brief Recursive copy (file or directory).
 *
 * @param src  Absolute source path.
 * @param dest Absolute destination path.
 * @return ESP_OK on success or an error from @c copy_file/dir.
 */
static esp_err_t copy_item(const char *src, const char *dest);

/**
 * @brief Copy a single file from src to dest using buffered I/O.
 *
 * @param src  Absolute source file path.
 * @param dest Absolute destination file path (created/overwritten).
 * @return ESP_OK on success; ESP_FAIL on fopen/fread/fwrite errors.
 */
static esp_err_t copy_file(const char *src, const char *dest);

/**
 * @brief Recursively copy a directory tree.
 *
 * Creates the destination directory, then copies children recursively
 * via @c copy_item().
 *
 * @param src  Absolute source directory path.
 * @param dest Absolute destination directory path (created).
 * @return ESP_OK on success; ESP_FAIL/ESP_ERR_INVALID_SIZE on errors.
 */
static esp_err_t copy_dir(const char *src, const char *dest);

/**
 * @brief Check if a path is a subpath of another (prefix + separator).
 *
 * @param parent Potential parent path.
 * @param child  Path to test.
 * @return true if child starts with parent and is below it.
 */
static bool is_subpath(const char *parent, const char *child);

/**
 * @brief Lightweight existence check using stat().
 *
 * @param path Absolute path to test.
 * @return true if stat() succeeds, false otherwise.
 */
static bool path_exists(const char *path);

/**
 * @brief Generate a unique "<name>_copy" (or numbered) within a directory.
 *
 * @param directory Destination directory path.
 * @param name      Base item name.
 * @param out       Output buffer for new name.
 * @param out_len   Size of @p out.
 * @return ESP_OK if a free name was produced; ESP_ERR_NOT_FOUND if none within attempts;
 *         ESP_ERR_INVALID_ARG/SIZE on bad inputs.
 */
static esp_err_t generate_copy_name(const char *directory, const char *name, char *out, size_t out_len);

/**
 * @brief Validate inputs for generate_copy_name.
 *
 * @param directory Target directory.
 * @param name Item name.
 * @param out Output buffer.
 * @param out_len Buffer length.
 * @return true if inputs are invalid.
 */
static bool copy_name_invalid_args(const char *directory, const char *name, char *out, size_t out_len);

/**
 * @brief Split a name into base and extension.
 *
 * @param name Input name.
 * @param[out] base Base buffer.
 * @param[out] ext Extension buffer.
 */
static void copy_name_split(const char *name, char *base, char *ext);

/**
 * @brief Compute max base length leaving room for suffix and extension.
 *
 * @param ext_len Extension length.
 * @return Max base length (0 if impossible).
 */
static size_t copy_name_max_base_len(size_t ext_len);

/**
 * @brief Clamp base length to the allowed maximum.
 *
 * @param base Base string buffer.
 * @param max_base_len Allowed length.
 * @return ESP_OK or ESP_ERR_INVALID_SIZE if max_base_len is zero.
 */
static esp_err_t copy_name_clamp_base(char *base, size_t max_base_len);

/**
 * @brief Build candidate copy name for a given index.
 *
 * @param base Base name.
 * @param ext Extension.
 * @param index Copy index (0 for first copy).
 * @param[out] candidate Output buffer.
 * @param cand_len Buffer length.
 * @return true if written successfully.
 */
static bool copy_name_build_candidate(const char *base, const char *ext, int index, char *candidate, size_t cand_len);

/**
 * @brief Compose full path for candidate name.
 *
 * @param directory Target directory.
 * @param candidate Candidate name.
 * @param[out] full Output buffer.
 * @param full_len Buffer length.
 * @return true if written successfully.
 */
static bool copy_name_compose_full(const char *directory, const char *candidate, char *full, size_t full_len);

/**
 * @brief Reset clipboard state to empty.
 *
 * @param ctx Browser context.
 */
static void clear_clipboard(file_manager_ctx_t *ctx);

/**
 * @brief Show a simple OK message box with provided text.
 *
 * @param msg Null-terminated message to display.
 */
static void show_message(const char *msg);

/**
 * @brief Format a 64-bit byte count into a short human-readable string.
 *
 * @param bytes   Number of bytes.
 * @param out     Output buffer.
 * @param out_len Buffer length.
 */
static void format_size64(uint64_t bytes, char *out, size_t out_len);

/**************************************************************************************************/


/************************************** Action Menu Workflow **************************************/

/**
 * @brief Populate @c action_item from a selected navigator item.
 *
 * Copies flags, name and current directory into the context action item
 * and marks it active.
 *
 * @param[in,out] ctx Browser context.
 * @param item       Navigator item to copy from.
 */
 static void prepare_action_item(file_manager_ctx_t *ctx, const fs_nav_item_t *item);

/**
 * @brief Show the action menu (Rename/Delete/Edit/Cancel) for current item.
 *
 * Creates a message box containing the item name and one or two button rows
 * depending on whether the item is editable text or not.
 *
 * @param[in,out] ctx Browser context with an active @c action_item.
 */
 static void show_action_menu(file_manager_ctx_t *ctx);

/**
 * @brief Close and clear the currently open action menu message box.
 *
 * If an action message box is present, closes it and nulls the pointer.
 *
 * @param[in,out] ctx Browser context.
 */
 static void close_action_menu(file_manager_ctx_t *ctx);

/**
 * @brief Handler for action menu buttons (Edit/Rename/Delete/Cancel).
 *
 * Reads the @c file_manager_action_type_t from button user data and performs
 * the corresponding action (open editor, show rename dialog, show delete
 * confirm, or cancel).
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
 static void on_action_button(lv_event_t *e);

/**
 * @brief Show a Yes/No confirmation dialog for deleting the selected item.
 *
 * Creates a message box with the item name in the prompt and two footer
 * buttons: "Yes" and "No".
 *
 * @param[in,out] ctx Browser context with an active @c action_item.
 */
 static void file_manager_show_delete_confirm(file_manager_ctx_t *ctx);

/**
 * @brief Close and clear the delete confirmation message box.
 *
 * If a confirmation message box is present, closes it and nulls the pointer.
 *
 * @param[in,out] ctx Browser context.
 */
 static void close_delete_confirm(file_manager_ctx_t *ctx);

/**
 * @brief Handler for delete confirmation buttons ("Yes"/"No").
 *
 * If confirmed, attempts to delete the selected item. Otherwise, clears
 * the action state.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
 static void delete_confirm(lv_event_t *e);

/**
 * @brief Delete the currently selected action item and reload the browser.
 *
 * Composes the full path, recursively deletes the target (if directory) and
 * reloads the file browser view on success.
 *
 * @param[in,out] ctx Browser context with an active @c action_item.
 * @return ESP_OK on success or appropriate error code.
 */
 static esp_err_t selected_item(file_manager_ctx_t *ctx);

/**************************************************************************************************/


/************************************* Action State Utilities *************************************/

/**
 * @brief Compose a full filesystem path from @c action_item.directory and name.
 *
 * Assembles "<directory>/<name>" into the provided output buffer.
 *
 * @param ctx      Browser context with an active @c action_item.
 * @param[out] out Output buffer for the composed path.
 * @param out_len  Size of @p out buffer in bytes.
 * @return ESP_OK on success,
 *         ESP_ERR_INVALID_STATE if state is invalid,
 *         ESP_ERR_INVALID_SIZE if the buffer is too small.
 */
 static esp_err_t action_compose_path(const file_manager_ctx_t *ctx, char *out, size_t out_len);

/**
 * @brief Clear all transient action-related state from the context.
 *
 * Closes action and confirm dialogs, closes rename dialog and resets
 * the @c action_item fields.
 *
 * @param[in,out] ctx Browser context.
 */
 static void clear_action_state(file_manager_ctx_t *ctx);

/**************************************************************************************************/


/************************************* Rename Dialog Workflow *************************************/

/**
 * @brief Set status text and color in the rename dialog using the title label.
 *
 * @param[in,out] ctx Browser context.
 * @param msg         Message text to display (must be non-NULL).
 * @param error       True to use error color, false for neutral/info color.
 */
 static void set_rename_status(file_manager_ctx_t *ctx, const char *msg, bool error);

/**
 * @brief Handle rename dialog acceptance (button or keyboard).
 *
 * @param e LVGL event with user data = file_manager_ctx_t*.
 */
static void on_rename_accept(lv_event_t *e);

/**
 * @brief Validate the rename text input and set status on failure.
 *
 * @param ctx Browser context.
 * @param text Input text.
 * @param[out] out_name Output buffer for trimmed name.
 * @return true if valid and trimmed, false otherwise.
 */
static bool rename_validate_input(file_manager_ctx_t *ctx, const char *text, char *out_name);

/**
 * @brief Handle no-op rename when the name is unchanged.
 *
 * @param ctx Browser context.
 * @param name New name.
 * @return true if handled (no-op), false otherwise.
 */
static bool rename_handle_noop(file_manager_ctx_t *ctx, const char *name);

/**
 * @brief Apply rename and report errors in the dialog.
 *
 * @param ctx Browser context.
 * @param name New name.
 * @return ESP_OK on success or error code.
 */
static esp_err_t rename_apply(file_manager_ctx_t *ctx, const char *name);

/**
 * @brief Finalize UI and refresh after successful rename.
 *
 * @param ctx Browser context.
 */
static void rename_finalize_success(file_manager_ctx_t *ctx);

/**
 * @brief Show the rename dialog for the currently selected item.
 *
 * Builds an overlay with a card containing the current item name, a status label,
 * a text area prefilled with the existing name and a "Save"/"Cancel" button row,
 * plus an on-screen keyboard. Any existing rename dialog is closed first.
 *
 * @param[in,out] ctx Browser context with a valid @c action_item.
 */
 static void show_rename_dialog(file_manager_ctx_t *ctx);

/**
 * @brief Close and destroy the rename dialog overlay.
 *
 * Deletes the dialog overlay and clears all rename dialog-related pointers
 * in the context.
 *
 * @param[in,out] ctx Browser context that owns the dialog.
 */
static void close_rename_dialog(file_manager_ctx_t *ctx);

/**
 * @brief Accept handler for the rename dialog (button or keyboard).
 *
 * Validates the new name, checks for no-op, attempts rename via
 * @c perform_rename(), displays any errors in the dialog and,
 * on success, closes the dialog and reloads the browser.
 *
 * @param e LVGL event (LV_EVENT_CLICKED or LV_EVENT_READY) with user data = @c file_manager_ctx_t*.
 */
static void on_rename_accept(lv_event_t *e);

/**
 * @brief Cancel handler for the rename dialog.
 *
 * Closes the dialog and clears action state without renaming.
 *
 * @param e LVGL event (LV_EVENT_CLICKED) with user data = @c file_manager_ctx_t*.
 */
static void on_rename_cancel(lv_event_t *e);

/**
 * @brief Perform the actual filesystem rename for the current action item.
 *
 * Builds the old and new paths and calls @c rename(). If the destination
 * already exists, returns ESP_ERR_INVALID_STATE.
 *
 * @param[in,out] ctx   Browser context with an active @c action_item.
 * @param new_name      New item name (validated, non-empty).
 * @return ESP_OK on success or an appropriate ESP_ERR_* code on failure.
 */
static esp_err_t perform_rename(file_manager_ctx_t *ctx, const char *new_name);

/**
 * @brief Handles the cancel action from the rename keyboard.
 *
 * This event callback is triggered when the user closes or cancels the
 * on-screen keyboard during the rename operation. It detaches the textarea
 * from the keyboard and hides the keyboard widget.
 *
 * @param e Pointer to the LVGL event descriptor.
 */
static void on_rename_keyboard_cancel(lv_event_t *e);

/**
 * @brief Displays the rename keyboard when the rename textarea is clicked.
 *
 * This event callback is triggered when the user taps the rename textarea.
 * It attaches the textarea to the on-screen keyboard and ensures the keyboard
 * becomes visible for text input.
 *
 * @param e Pointer to the LVGL event descriptor.
 */
static void on_rename_textarea_clicked(lv_event_t *e);

/**************************************************************************************************/

/************************************ File Manager Start Helpers ***********************************/

/**
 * @brief Delete previous screen and timers, if any.
 *
 * @param cfg Previously initialized configuration.
 */
static void file_manager_cleanup(file_manager_ctx_t *ctx);

/**
 * @brief Build a default file manager config using compile-time constants.
 *
 * @return Config with root path from CONFIG_SDSPI_MOUNT_POINT and default max items.
 */
static file_manager_config_t build_default_file_manager_config(void);

/**
 * @brief Validate presence of a root path in the config.
 *
 * Logs an error if missing.
 *
 * @param cfg Config to validate.
 * @param tag Logging tag to use.
 * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise.
 */
static esp_err_t validate_root_path(const file_manager_config_t *cfg, const char *tag);

/**
 * @brief Reset and prepare the file manager context.
 *
 * Clears state, resets window, and hooks time callbacks.
 *
 * @param ctx Context to initialize.
 */
static void initialize_file_manager_context(file_manager_ctx_t *ctx);

/**
 * @brief Build navigator config from the browser config.
 *
 * @param browser_cfg Source browser config.
 * @return Navigator config with resolved max_items.
 */
static fs_nav_config_t build_nav_config(const file_manager_config_t *browser_cfg);

/**
 * @brief Initialize the file navigator and handle failure side effects.
 *
 * Schedules SD retry helpers on failure.
 *
 * @param ctx File manager context.
 * @param nav_cfg Navigator configuration.
 * @param tag Logging tag to use.
 * @return ESP_OK on success or error from fs_nav_init.
 */
static esp_err_t init_file_navigator(file_manager_ctx_t *ctx, const fs_nav_config_t *nav_cfg, const char *tag);

/**
 * @brief Acquire the display lock or clean up navigator on failure.
 *
 * @param ctx File manager context.
 * @param tag Logging tag to use.
 * @return ESP_OK if lock acquired, ESP_ERR_TIMEOUT otherwise.
 */
static esp_err_t lock_display_or_cleanup(file_manager_ctx_t *ctx, const char *tag);

/**************************************************************************************************/

esp_err_t file_manager_start(void)
{
    const char* TAG_FILE_BROWSER_START = "file_manager_start";

    file_manager_ctx_t *ctx = &s_file_manager;
    file_manager_cleanup(ctx);
    initialize_file_manager_context(ctx);

    file_manager_config_t browser_cfg = build_default_file_manager_config();
    esp_err_t config_err = validate_root_path(&browser_cfg, TAG_FILE_BROWSER_START);
    if (config_err != ESP_OK) {
        return config_err;
    }

    fs_nav_config_t nav_cfg = build_nav_config(&browser_cfg);
    esp_err_t nav_err = init_file_navigator(ctx, &nav_cfg, TAG_FILE_BROWSER_START);
    if (nav_err != ESP_OK) {
        return nav_err;
    }

    esp_err_t lock_err = lock_display_or_cleanup(ctx, TAG_FILE_BROWSER_START);
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    build_file_manager_screen(ctx);
    sync_view(ctx);
    lv_screen_load(ctx->graphics.screen);
    bsp_display_unlock();
    
    return ESP_OK;
}

void file_manager_reset_clock_display(void)
{
    file_manager_ctx_t *ctx = &s_file_manager;
    ctx->flags.clock_user_set = false;

    if (ctx->graphics.datetime_label) {
        lv_label_set_text(ctx->graphics.datetime_label, "00:00 - 01/01/70");
        lv_obj_add_flag(ctx->graphics.datetime_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (ctx->graphics.datetime_btn) {
        lv_obj_clear_flag(ctx->graphics.datetime_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

void file_manager_on_time_set(void)
{
    file_manager_ctx_t *ctx = &s_file_manager;
    ctx->flags.clock_user_set = true;
    clock_update_async(NULL);
}

static void file_manager_cleanup(file_manager_ctx_t *ctx)
{
    lv_obj_t *old_screen = ctx->graphics.screen;
    lv_timer_t *old_path_timer = ctx->graphics.path_scroll_timer;
    lv_timer_t *old_list_timer = ctx->graphics.list_scroll_timer;
    cleanup_previous_ui(old_screen, old_path_timer, old_list_timer);
    
    esp_timer_handle_t old_clock_timer = ctx->clock_timer;
    bool old_clock_running = ctx->flags.clock_timer_running;
    cleanup_previous_clock_timer(old_clock_timer, old_clock_running);
}

static file_manager_config_t build_default_file_manager_config(void)
{
    file_manager_config_t cfg = {
        .root_path = CONFIG_SDSPI_MOUNT_POINT,
        .max_items = FILE_BROWSER_MAX_SORTABLE_ITEMS_DEFAULT,
    };
    return cfg;
}

static esp_err_t validate_root_path(const file_manager_config_t *cfg, const char *tag)
{
    if (!cfg->root_path) {
        ESP_LOGE(tag, "Failed to find a root path: (%s)", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static void initialize_file_manager_context(file_manager_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    clear_action_state(ctx);
    reset_window(ctx);
    settings_register_time_callbacks(file_manager_on_time_set, file_manager_reset_clock_display);
}

static void cleanup_previous_clock_timer(esp_timer_handle_t timer, bool was_running)
{
    if (!timer) {
        return;
    }
    if (was_running) {
        esp_timer_stop(timer);
    }
    esp_timer_delete(timer);
}

static void cleanup_previous_ui(lv_obj_t *old_screen, lv_timer_t *old_path_timer, lv_timer_t *old_list_timer)
{
    if (old_path_timer) {
        lv_timer_del(old_path_timer);
    }
    if (old_list_timer) {
        lv_timer_del(old_list_timer);
    }
    if (old_screen) {
        lv_obj_del(old_screen);
    }
}

static fs_nav_config_t build_nav_config(const file_manager_config_t *browser_cfg)
{
    fs_nav_config_t nav_cfg = {
        .root_path = browser_cfg->root_path,
        .max_items = browser_cfg->max_items ? browser_cfg->max_items : FILE_BROWSER_MAX_SORTABLE_ITEMS_DEFAULT,
    };
    return nav_cfg;
}

static esp_err_t init_file_navigator(file_manager_ctx_t *ctx, const fs_nav_config_t *nav_cfg, const char *tag)
{
    esp_err_t nav_err = fs_nav_init(&ctx->nav, nav_cfg);
    if (nav_err != ESP_OK) {
        ESP_LOGE(tag, "Failed to initialize the file system navigator: (%s)", esp_err_to_name(nav_err));
        sd_card_schedule_retry();
        schedule_wait_for_reconnection();
        return nav_err;
    }
    ctx->flags.initialized = true;
    return ESP_OK;
}

static esp_err_t lock_display_or_cleanup(file_manager_ctx_t *ctx, const char *tag)
{
    if (!bsp_display_lock(0)) {
        fs_nav_deinit(&ctx->nav);
        ctx->flags.initialized = false;
        ESP_LOGE(tag, "LVGL display lock cannot be acquired: (%s)", esp_err_to_name(ESP_ERR_TIMEOUT));
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void build_file_manager_screen(file_manager_ctx_t *ctx)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    styles_set_screen(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 2, 0);
    lv_obj_set_style_pad_gap(scr, 5, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    ctx->graphics.screen = scr;

    lv_obj_t *main_header = lv_obj_create(scr);
    lv_obj_remove_style_all(main_header);
    lv_obj_set_size(main_header, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(main_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(main_header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(main_header, 3, 0);
    styles_set_card_color(main_header, 0);
    lv_obj_set_style_bg_opa(main_header, LV_OPA_COVER, 0);

    ctx->graphics.settings_btn = lv_button_create(main_header);
    lv_obj_set_style_radius(ctx->graphics.settings_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->graphics.settings_btn, 6, 0);
    styles_set_button(ctx->graphics.settings_btn);
    lv_obj_t *settings_lbl = lv_label_create(ctx->graphics.settings_btn);
    lv_label_set_text(settings_lbl, LV_SYMBOL_SETTINGS " Settings");
    lv_obj_add_event_cb(ctx->graphics.settings_btn, on_settings_click, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_text_align(settings_lbl, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *tools_dd = lv_dropdown_create(main_header);
    lv_dropdown_set_options_static(tools_dd, "New Folder\nNew TXT\nSort");
    lv_dropdown_set_selected(tools_dd, 0);
    lv_dropdown_set_text(tools_dd, "Tools");
    lv_obj_set_width(tools_dd, 70);
    lv_obj_set_style_pad_all(tools_dd, 4, 0);
    lv_obj_set_style_pad_top(tools_dd, 6, 0);
    lv_obj_set_style_pad_bottom(tools_dd, 6, 0);
    lv_obj_set_style_border_width(tools_dd, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(tools_dd, 6, LV_PART_MAIN);
    styles_set_button(tools_dd);
    lv_obj_add_event_cb(tools_dd, on_tools_changed, LV_EVENT_VALUE_CHANGED, ctx);
    ctx->graphics.tools_dd = tools_dd;

    lv_obj_t *tools_list = lv_dropdown_get_list(ctx->graphics.tools_dd);
    styles_set_dropdown(tools_list);

    /* Spacer to consume remaining header width before centering the clock label/button area. */
    lv_obj_t *header_spacer_left = lv_obj_create(main_header);
    lv_obj_remove_style_all(header_spacer_left);
    lv_obj_set_flex_grow(header_spacer_left, 1);
    lv_obj_set_height(header_spacer_left, 1);

    /* Date&Time placeholder button (visible by default). */
    ctx->graphics.datetime_btn = lv_button_create(main_header);
    lv_obj_set_style_radius(ctx->graphics.datetime_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->graphics.datetime_btn, 6, 0);
    styles_set_button(ctx->graphics.datetime_btn);
    lv_obj_t *datetime_btn_lbl = lv_label_create(ctx->graphics.datetime_btn);
    lv_label_set_text(datetime_btn_lbl, "Set Date&Time");
    lv_obj_center(datetime_btn_lbl);
    lv_obj_add_event_cb(ctx->graphics.datetime_btn, on_datetime_click, LV_EVENT_CLICKED, ctx);

    /* Date&Time label (hidden until a time is set). */
    ctx->graphics.datetime_label = lv_label_create(main_header);
    lv_label_set_text(ctx->graphics.datetime_label, "00:00 - 01/01/70");
    lv_obj_set_style_text_align(ctx->graphics.datetime_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(ctx->graphics.datetime_label, &Domine_16, 0);
    styles_set_text_color(ctx->graphics.datetime_label, 0);
    lv_obj_add_flag(ctx->graphics.datetime_label, LV_OBJ_FLAG_HIDDEN);

    /* Spacer to balance layout so the button stays centered in the remaining space. */
    lv_obj_t *header_spacer_right = lv_obj_create(main_header);
    lv_obj_remove_style_all(header_spacer_right);
    lv_obj_set_flex_grow(header_spacer_right, 1);
    lv_obj_set_height(header_spacer_right, 1);

    lv_obj_t *path_row = lv_obj_create(scr);
    lv_obj_remove_style_all(path_row);
    lv_obj_set_size(path_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(path_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(path_row, 4, 0);

    start_clock_timer(ctx);

    lv_obj_t *path_prefix = lv_label_create(path_row);
    lv_label_set_text(path_prefix, "Path: ");
    lv_obj_set_style_text_align(path_prefix, LV_TEXT_ALIGN_LEFT, 0);
    styles_set_text_color(path_prefix, 0);

    ctx->graphics.path_label = lv_label_create(path_row);
    lv_label_set_long_mode(ctx->graphics.path_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_flex_grow(ctx->graphics.path_label, 1);
    lv_obj_set_width(ctx->graphics.path_label, LV_PCT(100));
    lv_obj_set_style_text_align(ctx->graphics.path_label, LV_TEXT_ALIGN_LEFT, 0);
    styles_set_text_color(ctx->graphics.path_label, 0);
    lv_label_set_text(ctx->graphics.path_label, "/");

    ctx->graphics.second_header = lv_obj_create(scr);
    lv_obj_remove_style_all(ctx->graphics.second_header);
    lv_obj_set_size(ctx->graphics.second_header, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(ctx->graphics.second_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctx->graphics.second_header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(ctx->graphics.second_header, 3, 0);

    ctx->graphics.parent_btn = lv_button_create(ctx->graphics.second_header);
    lv_obj_set_size(ctx->graphics.parent_btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(ctx->graphics.parent_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->graphics.parent_btn, 5, 0);
    styles_set_button(ctx->graphics.parent_btn);
    lv_obj_add_event_cb(ctx->graphics.parent_btn, on_parent_click, LV_EVENT_CLICKED, ctx);
    lv_obj_t *parent_lbl = lv_label_create(ctx->graphics.parent_btn);
    lv_label_set_text(parent_lbl, LV_SYMBOL_UP " Parent Folder");
    lv_obj_set_style_text_align(parent_lbl, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_add_flag(ctx->graphics.parent_btn, LV_OBJ_FLAG_HIDDEN);

    /* Spacer grows to push paste/cancel to the right edge. */
    lv_obj_t *header_spacer = lv_obj_create(ctx->graphics.second_header);
    lv_obj_remove_style_all(header_spacer);
    lv_obj_set_flex_grow(header_spacer, 1);
    lv_obj_set_height(header_spacer, 1);

    ctx->graphics.paste_btn = lv_button_create(ctx->graphics.second_header);
    lv_obj_set_style_radius(ctx->graphics.paste_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->graphics.paste_btn, 5, 0);
    styles_set_button(ctx->graphics.paste_btn);
    lv_obj_add_event_cb(ctx->graphics.paste_btn, on_paste_click, LV_EVENT_CLICKED, ctx);
    ctx->graphics.paste_label = lv_label_create(ctx->graphics.paste_btn);
    lv_label_set_text(ctx->graphics.paste_label, "Paste");
    lv_obj_set_style_text_align(ctx->graphics.paste_label, LV_TEXT_ALIGN_CENTER, 0);

    ctx->graphics.cancel_paste_btn = lv_button_create(ctx->graphics.second_header);
    lv_obj_set_style_radius(ctx->graphics.cancel_paste_btn, 6, 0);
    lv_obj_set_style_pad_all(ctx->graphics.cancel_paste_btn, 5, 0);
    styles_set_button(ctx->graphics.cancel_paste_btn);
    lv_obj_add_event_cb(ctx->graphics.cancel_paste_btn, on_cancel_paste_click, LV_EVENT_CLICKED, ctx);
    ctx->graphics.cancel_paste_label = lv_label_create(ctx->graphics.cancel_paste_btn);
    lv_label_set_text(ctx->graphics.cancel_paste_label, "Cancel");
    lv_obj_set_style_text_align(ctx->graphics.cancel_paste_label, LV_TEXT_ALIGN_CENTER, 0);
    update_second_header(ctx);

    lv_obj_t *list_row = lv_obj_create(scr);
    lv_obj_remove_style_all(list_row);
    lv_obj_set_size(list_row, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(list_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(list_row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(list_row, FILE_BROWSER_SLIDER_GAP_DEFAULT, 0);
    lv_obj_set_style_pad_right(list_row, FILE_BROWSER_SLIDER_GAP_DEFAULT, 0);
    lv_obj_set_flex_grow(list_row, 1);

    ctx->graphics.list = lv_list_create(list_row);
    lv_obj_set_flex_grow(ctx->graphics.list, 1);
    lv_obj_set_width(ctx->graphics.list, LV_PCT(100));
    lv_obj_set_style_min_width(ctx->graphics.list, 0, 0);
    lv_obj_set_height(ctx->graphics.list, LV_PCT(100));
    lv_obj_set_style_pad_left(ctx->graphics.list, 1, 0);
    lv_obj_set_style_pad_right(ctx->graphics.list, 1, 0);
    lv_obj_set_style_pad_bottom(ctx->graphics.list, 1, 0);
    styles_set_card_color(ctx->graphics.list, 0);
    styles_set_border_color(ctx->graphics.list, 0);
    lv_obj_set_style_bg_opa(ctx->graphics.list, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ctx->graphics.list, 1, 0);
    lv_obj_add_event_cb(ctx->graphics.list, on_list_scrolled, LV_EVENT_SCROLL, ctx);

    lv_obj_t *list_slider = lv_slider_create(list_row);
    lv_slider_set_orientation(list_slider, LV_SLIDER_ORIENTATION_VERTICAL);
    lv_slider_set_range(list_slider, 100, 0); /* Min at top, max at bottom */
    lv_slider_set_value(list_slider, 0, LV_ANIM_OFF);
    lv_obj_set_width(list_slider, 14);
    lv_obj_set_height(list_slider, LV_PCT(82));
    lv_obj_set_style_translate_y(list_slider, 1, 0);
    styles_set_slider(list_slider);
    lv_obj_set_style_bg_opa(list_slider, LV_OPA_60, 0);
    lv_obj_set_style_radius(list_slider, 6, 0);
    lv_obj_set_style_bg_opa(list_slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(list_slider, 6, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(list_slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_border_width(list_slider, 1, LV_PART_KNOB);
    lv_obj_set_style_radius(list_slider, 5, LV_PART_KNOB);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_PRESSED, ctx);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_VALUE_CHANGED, ctx);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_RELEASED, ctx);
    lv_obj_add_event_cb(list_slider, on_slider_value_changed, LV_EVENT_PRESS_LOST, ctx);
    lv_obj_clear_flag(list_slider, LV_OBJ_FLAG_SCROLL_CHAIN); /* Keep list from scrolling when dragging slider */
    ctx->graphics.list_slider = list_slider;
}

static void reset_window(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    ctx->list_window_start = 0;
    ctx->list_window_size = FILE_BROWSER_LIST_WINDOW_SIZE_DEFAULT;
    ctx->flags.list_at_top_edge = false;
    ctx->flags.list_at_bottom_edge = false;
    ctx->flags.list_suppress_scroll = false;
    ctx->flags.list_has_paged = false;
    ctx->flags.slider_drag_active = false;
    ctx->slider_pending_step = SIZE_MAX;
    ctx->flags.preserve_window_on_reload = false;
    ctx->reload_anchor_index = SIZE_MAX;
}

static void set_reload_anchor_current(file_manager_ctx_t *ctx)
{
    if (!ctx || ctx->reload_anchor_index != SIZE_MAX) {
        return;
    }
    size_t mid = ctx->list_window_start;
    if (ctx->list_window_size > 0) {
        mid += ctx->list_window_size / 2;
    }
    ctx->reload_anchor_index = mid;
}

static void get_window_params(file_manager_ctx_t *ctx, size_t *window_size, size_t *step)
{
    if (!ctx || !window_size || !step) {
        return;
    }

    size_t ws = ctx->list_window_size ? ctx->list_window_size : FILE_BROWSER_LIST_WINDOW_SIZE_DEFAULT;
    if (ws == 0) {
        ws = 1;
    }

    size_t st = FILE_BROWSER_LIST_WINDOW_STEP_DEFAULT ? FILE_BROWSER_LIST_WINDOW_STEP_DEFAULT : (ws / 2);
    if (st == 0) {
        st = 1;
    }

    *window_size = ws;
    *step = st;
}

static bool is_item_list_valid(const file_manager_ctx_t *ctx)
{
    return (ctx && ctx->graphics.list);
}

static esp_err_t is_item_set_nav(file_manager_ctx_t *ctx, size_t start_index)
{
    esp_err_t werr = fs_nav_set_window(&ctx->nav, start_index, ctx->list_window_size);
    if (werr != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set window: %s", esp_err_to_name(werr));
        sd_card_schedule_retry();
        return werr;
    }

    ctx->list_window_start = fs_nav_window_start(&ctx->nav);
    ctx->flags.list_at_top_edge = false;
    ctx->flags.list_at_bottom_edge = false;
    return ESP_OK;
}

static void refresh_item_list(file_manager_ctx_t *ctx)
{
    ctx->flags.list_suppress_scroll = true;
    show_loading(ctx);
    populate_list(ctx);
    hide_loading(ctx);
    lv_obj_update_layout(ctx->graphics.list);
    update_slider(ctx);
}

static lv_obj_t *item_list_find_anchor(file_manager_ctx_t *ctx, size_t anchor_index)
{
    if (anchor_index == SIZE_MAX) {
        return NULL;
    }

    size_t count = 0;
    fs_nav_items(&ctx->nav, &count);
    if (anchor_index < ctx->list_window_start || anchor_index >= ctx->list_window_start + count) {
        return NULL;
    }

    size_t rel = anchor_index - ctx->list_window_start;
    uint32_t child_cnt = lv_obj_get_child_count(ctx->graphics.list);
    for (uint32_t i = 0; i < child_cnt; i++) {
        lv_obj_t *child = lv_obj_get_child(ctx->graphics.list, i);
        if ((size_t)(uintptr_t)lv_obj_get_user_data(child) == rel) {
            return child;
        }
    }
    return NULL;
}

static void item_list_scroll_to_anchor(file_manager_ctx_t *ctx, lv_obj_t *anchor_obj, bool center_anchor, bool scroll_to_top)
{
    if (anchor_obj) {
        if (center_anchor) {
            lv_obj_scroll_to_view(anchor_obj, LV_ANIM_OFF);
            lv_coord_t mid = lv_obj_get_y(anchor_obj) + lv_obj_get_height(anchor_obj) / 2;
            lv_coord_t list_h = lv_obj_get_height(ctx->graphics.list);
            lv_coord_t target = mid - list_h / 2;
            lv_obj_scroll_to_y(ctx->graphics.list, target, LV_ANIM_OFF);
        } else {
            lv_obj_scroll_to_view(anchor_obj, LV_ANIM_OFF);
        }
        return;
    }

    if (ctx->flags.list_has_paged) {
        /* Center only after the first paging has occurred. */
        lv_obj_scroll_to_y(ctx->graphics.list, lv_obj_get_scroll_bottom(ctx->graphics.list) / 2, LV_ANIM_OFF);
    } else if (scroll_to_top) {
        lv_obj_scroll_to_y(ctx->graphics.list, 0, LV_ANIM_OFF);
    } else {
        lv_obj_scroll_to_y(ctx->graphics.list, lv_obj_get_scroll_bottom(ctx->graphics.list), LV_ANIM_OFF);
    }
}

static void item_window_restore_scroll_flag(file_manager_ctx_t *ctx, bool prev_suppress)
{
    ctx->flags.list_suppress_scroll = prev_suppress;
}

static void build_item_list(file_manager_ctx_t *ctx, size_t start_index, size_t anchor_index, bool center_anchor, bool scroll_to_top)
{
    if (!is_item_list_valid(ctx)) {
        return;
    }

    esp_err_t werr = is_item_set_nav(ctx, start_index);
    if (werr != ESP_OK) {
        return;
    }

    bool prev_suppress = ctx->flags.list_suppress_scroll;
    refresh_item_list(ctx);

    lv_obj_t *anchor_obj = item_list_find_anchor(ctx, anchor_index);

    item_list_scroll_to_anchor(ctx, anchor_obj, center_anchor, scroll_to_top);

    item_window_restore_scroll_flag(ctx, prev_suppress);
}

static bool slider_is_valid(const file_manager_ctx_t *ctx)
{
    return (ctx && ctx->graphics.list_slider);
}

static lv_obj_t *slider_get_list_row(const file_manager_ctx_t *ctx)
{
    return (ctx && ctx->graphics.list) ? lv_obj_get_parent(ctx->graphics.list) : NULL;
}

static void slider_disable_for_single_window(file_manager_ctx_t *ctx, lv_obj_t *list_row)
{
    bool prev_suppress = ctx->flags.slider_suppress_change;
    ctx->flags.slider_suppress_change = true;
    lv_slider_set_range(ctx->graphics.list_slider, 0, 0);
    lv_slider_set_value(ctx->graphics.list_slider, 0, LV_ANIM_OFF);
    ctx->flags.slider_suppress_change = prev_suppress;
    ctx->slider_pending_step = 0;
    ctx->flags.slider_drag_active = false;
    lv_obj_add_state(ctx->graphics.list_slider, LV_STATE_DISABLED);
    if (list_row) {
        lv_obj_set_style_pad_right(list_row, FILE_BROWSER_SLIDER_GAP_DEFAULT, 0);
    }
}

static size_t slider_max_start(size_t total, size_t window_size)
{
    return (total > window_size) ? (total - window_size) : 0;
}

static size_t slider_compute_current_step(const file_manager_ctx_t *ctx, size_t max_start, size_t max_step_index, size_t step, int32_t max_val)
{
    size_t current_step = 0;
    if (ctx->list_window_start >= max_start) {
        current_step = max_step_index; /* force knob to bottom when at last window */
    } else {
        current_step = step ? (ctx->list_window_start / step) : 0;
        if (current_step > (size_t)max_val) {
            current_step = (size_t)max_val;
        }
    }
    return current_step;
}

static void slider_apply_range_and_value(file_manager_ctx_t *ctx, int32_t max_val, size_t current_step)
{
    bool prev_suppress = ctx->flags.slider_suppress_change;
    ctx->flags.slider_suppress_change = true;
    lv_slider_set_range(ctx->graphics.list_slider, max_val, 0); /* min at top, max at bottom */
    lv_slider_set_value(ctx->graphics.list_slider, (int32_t)current_step, LV_ANIM_OFF);
    ctx->flags.slider_suppress_change = prev_suppress;
    ctx->slider_pending_step = current_step;
}

static void enable_slider(file_manager_ctx_t *ctx, lv_obj_t *list_row)
{
    lv_obj_remove_state(ctx->graphics.list_slider, LV_STATE_DISABLED);
    if (list_row) {
        lv_obj_set_style_pad_right(list_row, FILE_BROWSER_SLIDER_GAP_DEFAULT, 0);
    }
}

static void update_slider(file_manager_ctx_t *ctx)
{
    if (!slider_is_valid(ctx)) {
        return;
    }

    size_t window_size = 1;
    size_t step = 1;
    get_window_params(ctx, &window_size, &step);
    size_t total = fs_nav_total_items(&ctx->nav);

    lv_obj_t *list_row = slider_get_list_row(ctx);

    /* If everything fits in one window, lock the slider at start. */
    if (total <= window_size) {
        slider_disable_for_single_window(ctx, list_row);
        return;
    }

    size_t max_start = slider_max_start(total, window_size);
    size_t max_step_index = slider_max_step_index(max_start, step);
    int32_t max_val = (int32_t)max_step_index;

    size_t current_step = slider_compute_current_step(ctx, max_start, max_step_index, step, max_val);

    slider_apply_range_and_value(ctx, max_val, current_step);

    enable_slider(ctx, list_row);
}

static void schedule_wait_for_reconnection(void)
{
    if (file_manager_wait_task){
        return;
    }
    
    BaseType_t res = xTaskCreatePinnedToCore(wait_for_reconnection_task,
                                             "file_manager_wait_task",
                                             FILE_BROWSER_WAIT_STACK_SIZE_B_DEFAULT,
                                             NULL,
                                             FILE_BROWSER_WAIT_PRIO_DEFAULT,
                                             &file_manager_wait_task,
                                             tskNO_AFFINITY);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create file browser wait task");
        file_manager_wait_task = NULL;
    }                                             
}

static bool wait_task_take_reconnection_sem(bool *schedule_retry)
{
    if (xSemaphoreTake(reconnection_success, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to wait for SD reconnection, scheduling retry...");
        if (schedule_retry) {
            *schedule_retry = true;
        }
        return false;
    }
    return true;
}

static void wait_task_handle_pending_parent(file_manager_ctx_t *ctx, bool *schedule_retry)
{
    if (!ctx->flags.pending_go_parent) {
        return;
    }
    ctx->flags.pending_go_parent = false;
    esp_err_t nav_err = fs_nav_go_parent(&ctx->nav);
    if (nav_err != ESP_OK){
        ESP_LOGE(TAG, "fs_nav_go_parent() failed after reconnection (%s), scheduling retry...", esp_err_to_name(nav_err));
        if (schedule_retry) {
            *schedule_retry = true;
        }
    }
}

static void wait_task_refresh_after_reconnect(file_manager_ctx_t *ctx, bool *schedule_retry)
{
    if (schedule_retry && *schedule_retry) {
        return;
    }
    esp_err_t err = refresh_current_dir();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "refresh_current_dir() failed while trying to refresh the screen after an SD card reconnection, scheduling retry...\n");
        if (schedule_retry) {
            *schedule_retry = true;
        }
    }
}

static void wait_task_handle_initialized(file_manager_ctx_t *ctx, bool *schedule_retry)
{
    wait_task_handle_pending_parent(ctx, schedule_retry);
    wait_task_refresh_after_reconnect(ctx, schedule_retry);
}

static void wait_task_restart_if_needed(bool *schedule_retry)
{
    if (schedule_retry && *schedule_retry) {
        return;
    }
    esp_err_t err = file_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "file_manager_start() failed after SD reconnection (%s), scheduling retry...", esp_err_to_name(err));
        if (schedule_retry) {
            *schedule_retry = true;
        }
    }
}

static void wait_task_schedule_retry(bool schedule_retry)
{
    if (schedule_retry) {
        sd_card_schedule_retry();
    }
}

static void wait_task_cleanup_and_delete(void)
{
    file_manager_wait_task = NULL;
    vTaskDelete(NULL);
}

static void wait_for_reconnection_task(void* arg)
{
    file_manager_ctx_t *ctx = &s_file_manager;
    bool schedule_retry = false;

    bool took_sem = wait_task_take_reconnection_sem(&schedule_retry);
    if (took_sem) {
        if (ctx->flags.initialized) {
            wait_task_handle_initialized(ctx, &schedule_retry);
        } else {
            wait_task_restart_if_needed(&schedule_retry);
        }
    }
    
    wait_task_schedule_retry(schedule_retry);

    wait_task_cleanup_and_delete();
}

static void sync_view(file_manager_ctx_t *ctx)
{
    if (!ctx->graphics.screen) {
        return;
    }
    bool preserve = ctx->flags.preserve_window_on_reload;
    ctx->flags.preserve_window_on_reload = false;
    if (preserve && ctx->reload_anchor_index == SIZE_MAX) {
        set_reload_anchor_current(ctx);
    }
    size_t anchor = ctx->reload_anchor_index;
    ctx->reload_anchor_index = SIZE_MAX;
    if (!preserve) {
        reset_window(ctx);
    } else {
        ctx->flags.list_at_top_edge = false;
        ctx->flags.list_at_bottom_edge = false;
        ctx->flags.list_suppress_scroll = false;
        ctx->flags.list_has_paged = false;
    }
    update_path_label(ctx);
    update_sort_badges(ctx);
    update_second_header(ctx);
    build_item_list(ctx, ctx->list_window_start, anchor, true, true);
}

static bool check_second_header(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.second_header){
        return false;
    }    

    if(!ctx->graphics.parent_btn || !ctx->graphics.paste_btn || !ctx->graphics.cancel_paste_btn) {
        return false;
    }

    return true;
}

static void update_second_header(file_manager_ctx_t *ctx)
{
    if (!check_second_header(ctx)){
        return;
    }

    update_parent_button(ctx);
    update_paste_button(ctx);

    if (!fs_nav_can_go_parent(&ctx->nav) && !ctx->clipboard.has_item){
        lv_obj_add_flag(ctx->graphics.second_header, LV_OBJ_FLAG_HIDDEN);
    }else{
        lv_obj_clear_flag(ctx->graphics.second_header, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_parent_button(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.parent_btn) {
        return;
    }

    if (fs_nav_can_go_parent(&ctx->nav)) {
        lv_obj_clear_flag(ctx->graphics.parent_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(ctx->graphics.parent_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void path_scroll_timer_cb(lv_timer_t *timer)
{
    file_manager_ctx_t *ctx = (file_manager_ctx_t *)lv_timer_get_user_data(timer);
    if (ctx) {
        ctx->graphics.path_scroll_timer = NULL;
        if (ctx->graphics.path_label && lv_obj_is_valid(ctx->graphics.path_label)) {
            lv_label_set_long_mode(ctx->graphics.path_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        }
    }
    lv_timer_del(timer);
}

static void restart_path_scroll(file_manager_ctx_t *ctx)
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
    ctx->graphics.path_scroll_timer = lv_timer_create(path_scroll_timer_cb, FILE_BROWSER_PATH_SCROLL_DELAY_MS_DEFAULT, ctx);
    if (ctx->graphics.path_scroll_timer) {
        lv_timer_set_repeat_count(ctx->graphics.path_scroll_timer, 1);
    }
}

static void update_path_label(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.path_label) {
        return;
    }
    const char *path = fs_nav_current_path(&ctx->nav);
    const char *mount = CONFIG_SDSPI_MOUNT_POINT;
    char display[FS_NAV_MAX_PATH + 8];

    if (path && mount && strncmp(path, mount, strlen(mount)) == 0) {
        const char *rest = path + strlen(mount);
        if (*rest == '/') {
            rest++;
        }
        if (*rest == '\0') {
            strlcpy(display, "/", sizeof(display));
        } else {
            snprintf(display, sizeof(display), "/%s", rest);
        }
    } else {
        snprintf(display, sizeof(display), "%s", path ? path : "-");
    }

    lv_label_set_text(ctx->graphics.path_label, display);
    restart_path_scroll(ctx);
}

static void update_sort_badges(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    bool sort_enabled = fs_nav_is_sort_enabled(&ctx->nav);
    fs_nav_sort_mode_t mode = fs_nav_get_sort(&ctx->nav);
    bool asc = fs_nav_is_sort_ascending(&ctx->nav);

    if (ctx->graphics.sort_criteria_dd) {
        lv_dropdown_set_selected(ctx->graphics.sort_criteria_dd, (uint16_t)mode);
        if (sort_enabled) {
            lv_obj_clear_state(ctx->graphics.sort_criteria_dd, LV_STATE_DISABLED);
        } else {
            lv_obj_add_state(ctx->graphics.sort_criteria_dd, LV_STATE_DISABLED);
        }
    }

    if (ctx->graphics.sort_direction_dd) {
        lv_dropdown_set_selected(ctx->graphics.sort_direction_dd, asc ? 0 : 1);
        if (sort_enabled) {
            lv_obj_clear_state(ctx->graphics.sort_direction_dd, LV_STATE_DISABLED);
        } else {
            lv_obj_add_state(ctx->graphics.sort_direction_dd, LV_STATE_DISABLED);
        }
    }
}

static lv_obj_t *get_list_btn_label(lv_obj_t *btn)
{
    if (!btn) {
        return NULL;
    }

    uint32_t child_cnt = lv_obj_get_child_count(btn);
    for (uint32_t i = 0; i < child_cnt; ++i) {
        lv_obj_t *child = lv_obj_get_child(btn, i);
        if (child && lv_obj_check_type(child, &lv_label_class)) {
            return child;
        }
    }

    return NULL;
}

static void entry_scroll_timer_cb(lv_timer_t *timer)
{
    file_manager_ctx_t *ctx = (file_manager_ctx_t *)lv_timer_get_user_data(timer);
    if (ctx) {
        ctx->graphics.list_scroll_timer = NULL;
        if (ctx->graphics.list) {
            uint32_t child_cnt = lv_obj_get_child_count(ctx->graphics.list);
            for (uint32_t i = 0; i < child_cnt; ++i) {
                lv_obj_t *label = get_list_btn_label(lv_obj_get_child(ctx->graphics.list, i));
                if (label) {
                    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
                }
            }
        }
    }
    lv_timer_del(timer);
}

static void restart_entry_scroll(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.list) {
        return;
    }

    if (ctx->graphics.list_scroll_timer) {
        lv_timer_del(ctx->graphics.list_scroll_timer);
        ctx->graphics.list_scroll_timer = NULL;
    }

    uint32_t child_cnt = lv_obj_get_child_count(ctx->graphics.list);
    bool has_labels = false;
    for (uint32_t i = 0; i < child_cnt; ++i) {
        lv_obj_t *label = get_list_btn_label(lv_obj_get_child(ctx->graphics.list, i));
        if (label) {
            lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
            has_labels = true;
        }
    }

    if (!has_labels) {
        return;
    }

    ctx->graphics.list_scroll_timer = lv_timer_create(entry_scroll_timer_cb, FILE_BROWSER_ENTRY_SCROLL_DELAY_MS_DEFAULT, ctx);
    if (ctx->graphics.list_scroll_timer) {
        lv_timer_set_repeat_count(ctx->graphics.list_scroll_timer, 1);
    }
}

static void populate_list_clear_scroll_timer(file_manager_ctx_t *ctx)
{
    if (ctx->graphics.list_scroll_timer) {
        lv_timer_del(ctx->graphics.list_scroll_timer);
        ctx->graphics.list_scroll_timer = NULL;
    }
}

static void populate_list_clear_list(file_manager_ctx_t *ctx)
{
    lv_obj_clean(ctx->graphics.list);
}

static const fs_nav_item_t *populate_list_get_items(file_manager_ctx_t *ctx, size_t *count)
{
    return fs_nav_items(&ctx->nav, count);
}

static bool populate_list_handle_empty(file_manager_ctx_t *ctx, const fs_nav_item_t *items, size_t count)
{
    if (items && count > 0) {
        return false;
    }
    lv_obj_t *lbl = lv_label_create(ctx->graphics.list);
    styles_set_text_color(lbl, 0);
    lv_label_set_text(lbl, "Empty folder");
    lv_obj_center(lbl);
    lv_obj_set_style_text_opa(lbl, LV_OPA_60, 0);
    return true;
}

static size_t populate_list_window_start(file_manager_ctx_t *ctx)
{
    return fs_nav_window_start(&ctx->nav);
}

static void populate_list_format_file_text(const fs_nav_item_t *item, size_t display_index, char *out, size_t out_len)
{
    char meta[32];
    format_size(item->size_bytes, meta, sizeof(meta));
    snprintf(out, out_len, "%s\nItem: %zu | Size: %s", item->name, display_index, meta);
}

static void populate_list_format_dir_text(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t display_index, char *out, size_t out_len)
{
    size_t child_count = 0;
    char meta[32];
    const char *count_label = "Unknown";
    if (count_dir_items(ctx, item, &child_count)) {
        snprintf(meta, sizeof(meta), "%u", (unsigned int)child_count);
        count_label = meta;
    }
    snprintf(out, out_len, "%s\nItem: %zu | Sub-Items: %s", item->name, display_index, count_label);
}

static const char *populate_list_icon_for_item(const fs_nav_item_t *item)
{
    return item->is_dir ? LV_SYMBOL_DIRECTORY : (is_file_image(item->name) ? LV_SYMBOL_IMAGE : LV_SYMBOL_FILE);
}

static void populate_list_create_button(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t rel_index, const char *icon, const char *text)
{
    lv_obj_t *btn = lv_list_add_btn(ctx->graphics.list, icon, text);
    lv_obj_set_style_pad_all(btn, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    styles_set_list_button(btn);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_user_data(btn, (void *)(uintptr_t)rel_index);
    lv_obj_add_event_cb(btn, on_item_click, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(btn, on_item_long_press, LV_EVENT_LONG_PRESSED, ctx);
}

static void populate_list(file_manager_ctx_t *ctx)
{
    populate_list_clear_scroll_timer(ctx);

    populate_list_clear_list(ctx);

    size_t count = 0;
    const fs_nav_item_t *items = populate_list_get_items(ctx, &count);
    if (populate_list_handle_empty(ctx, items, count)) {
        return;
    }

    /* Window start gives the absolute offset of the first visible item. */
    size_t window_start = populate_list_window_start(ctx);

    for (size_t i = 0; i < count; ++i) {
        fs_nav_ensure_meta(&ctx->nav, i);
        const fs_nav_item_t *item = &items[i];
        size_t display_index = window_start + i + 1; /* 1-based absolute index */

        char text[FS_NAV_MAX_NAME + 64];
        if (!item->is_dir) {
            populate_list_format_file_text(item, display_index, text, sizeof(text));
        } else {
            populate_list_format_dir_text(ctx, item, display_index, text, sizeof(text));
        }

        const char *icon = populate_list_icon_for_item(item);

        populate_list_create_button(ctx, item, i, icon, text);
    }

    restart_entry_scroll(ctx);
}

static bool count_dir_items(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t *out_count)
{
    if (!ctx || !item || !out_count || !item->is_dir) {
        return false;
    }

    char path[FS_NAV_MAX_PATH];
    if (fs_nav_compose_path(&ctx->nav, item->name, path, sizeof(path)) != ESP_OK) {
        return false;
    }

    DIR *dir = opendir(path);
    if (!dir) {
        return false;
    }

    size_t count = 0;
    struct dirent *dent = NULL;
    while ((dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        count++;
    }
    closedir(dir);

    *out_count = count;
    return true;
}

static void format_size(size_t bytes, char *out, size_t out_len)
{
    static const char *suffixes[] = {"B", "KB", "MB", "GB"};
    double value = (double)bytes;
    size_t idx = 0;
    while (value >= 1024.0 && idx < 3) {
        value /= 1024.0;
        idx++;
    }
    if (idx == 0) {
        snprintf(out, out_len, "%u %s", (unsigned int)bytes, suffixes[idx]);
    } else {
        snprintf(out, out_len, "%.1f %s", value, suffixes[idx]);
    }
}

static void format_size64(uint64_t bytes, char *out, size_t out_len)
{
    static const char *suffixes[] = {"B", "KB", "MB", "GB", "TB"};
    double value = (double)bytes;
    size_t idx = 0;
    while (value >= 1024.0 && idx < 4) {
        value /= 1024.0;
        idx++;
    }
    if (idx == 0) {
        snprintf(out, out_len, "%llu %s", (unsigned long long)bytes, suffixes[idx]);
    } else {
        snprintf(out, out_len, "%.1f %s", value, suffixes[idx]);
    }
}

/**
 * @brief Basic image-type detection for choosing an icon.
 */
static bool is_file_image(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) {
        return false;
    }

    return strcasecmp(dot, ".png") == 0 ||
           strcasecmp(dot, ".jpg") == 0 ||
           strcasecmp(dot, ".jpeg") == 0 ||
           strcasecmp(dot, ".bmp") == 0 ||
           strcasecmp(dot, ".gif") == 0;
}

static bool is_file_jpeg(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) {
        return false;
    }
    return strcasecmp(dot, ".jpg") == 0 || strcasecmp(dot, ".jpeg") == 0;
}

static bool handle_jpeg_compose_path(file_manager_ctx_t *ctx, const fs_nav_item_t *item, char *path, size_t path_len)
{
    if (fs_nav_compose_path(&ctx->nav, item->name, path, path_len) != ESP_OK) {
        ESP_LOGE(TAG, "Path too long for \"%s\"", item->name);
        return false;
    }
    return true;
}

static bool handle_jpeg_build_lv_path(const char *relative, const char *item_name, char *lv_path, size_t lv_path_len)
{
    int needed = snprintf(lv_path, lv_path_len, "S:%s", relative);
    if (needed < 0 || needed >= (int)lv_path_len) {
        ESP_LOGE(TAG, "LVGL path too long for \"%s\"", item_name);
        return false;
    }
    return true;
}

static void handle_jpeg_open_with_prompts(file_manager_ctx_t *ctx, const char *lv_path, const char *full_path)
{
    jpg_viewer_open_opts_t opts = {
        .path = lv_path,
        .return_screen = ctx->graphics.screen
    };

    esp_err_t err = jpg_viewer_open(&opts);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGE(TAG, "The image is corrupted or this specific JPG type is not supported by the system.");
            show_jpeg_unsupported_prompt();
        } else if (err == ESP_ERR_NO_MEM){
            ESP_LOGE(TAG, "The image is too large or there is no more internal memory to open it.");
            show_not_enough_memory_prompt();
        }else if (err == ESP_ERR_INVALID_SIZE){
            ESP_LOGE(TAG, "The image resolution is too large do display.");
            show_image_resolution_too_large_to_display_prompt();
        }else{
            ESP_LOGE(TAG, "Failed to open JPEG \"%s\": %s", full_path, esp_err_to_name(err));
            sd_card_schedule_retry();
        }
    }
}

static void handle_jpeg(file_manager_ctx_t *ctx, const fs_nav_item_t *item)
{
    if (!ctx || !item) {
        return;
    }

    char path[FS_NAV_MAX_PATH];
    if (!handle_jpeg_compose_path(ctx, item, path, sizeof(path))) {
        return;
    }

    const char *root = CONFIG_SDSPI_MOUNT_POINT;
    size_t root_len = strlen(root);
    const char *relative = path;
    if (strncmp(path, root, root_len) == 0) {
        relative = path + root_len; /* keep leading slash after mountpoint */
    }

    char lv_path[FS_NAV_MAX_PATH + 4];
    if (!handle_jpeg_build_lv_path(relative, item->name, lv_path, sizeof(lv_path))) {
        return;
    }

    handle_jpeg_open_with_prompts(ctx, lv_path, path);
}

static bool refresh_dir_is_initialized(const file_manager_ctx_t *ctx)
{
    return (ctx && ctx->flags.initialized);
}

static esp_err_t refresh_dir_nav_refresh(file_manager_ctx_t *ctx)
{
    return fs_nav_refresh(&ctx->nav);
}

static void refresh_dir_preserve_window(file_manager_ctx_t *ctx, size_t saved_start)
{
    size_t window_size = 1;
    size_t step = 1;
    get_window_params(ctx, &window_size, &step);
    size_t total = fs_nav_total_items(&ctx->nav);
    if (window_size == 0) {
        window_size = 1;
    }
    if (total > 0 && total > window_size) {
        size_t max_start = total - window_size;
        if (saved_start > max_start) {
            saved_start = max_start;
        }
    } else {
        saved_start = 0;
    }
    ctx->list_window_start = saved_start;
    if (ctx->reload_anchor_index == SIZE_MAX) {
        size_t anchor = saved_start;
        if (window_size > 1) {
            anchor += window_size / 2;
        }
        if (total > 0 && anchor >= total) {
            anchor = total - 1;
        }
        ctx->reload_anchor_index = anchor;
    }
}

static void refresh_dir_handle_window(file_manager_ctx_t *ctx, size_t saved_start)
{
    bool preserve_window = ctx->flags.preserve_window_on_reload;
    ctx->flags.preserve_window_on_reload = preserve_window;

    if (preserve_window) {
        refresh_dir_preserve_window(ctx, saved_start);
    } else {
        reset_window(ctx);
    }
}

static bool refresh_dir_lock_display(void)
{
    return bsp_display_lock(0);
}

static void refresh_dir_apply_ui_updates(file_manager_ctx_t *ctx)
{
    sync_view(ctx);
    clear_action_state(ctx);
    close_paste_conflict(ctx);
    hide_loading(ctx);
}

static esp_err_t refresh_current_dir(void)
{
    file_manager_ctx_t *ctx = &s_file_manager;
    if (!refresh_dir_is_initialized(ctx)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = refresh_dir_nav_refresh(ctx);
    if (err != ESP_OK) {
        return err;
    }

    size_t saved_start = ctx->list_window_start;

    refresh_dir_handle_window(ctx, saved_start);

    if (!refresh_dir_lock_display()) {
        return ESP_ERR_TIMEOUT;
    }
    refresh_dir_apply_ui_updates(ctx);
    bsp_display_unlock();
    return ESP_OK;
}

static void show_unsupported_prompt(void)
{
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "This file format is not supported.");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, close_unsupported_msgbox, LV_EVENT_CLICKED, mbox);
}

static void show_image_resolution_too_large_to_display_prompt(void)
{
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "The image resolution is too large do display.");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, close_unsupported_msgbox, LV_EVENT_CLICKED, mbox);
}

static void show_not_enough_memory_prompt(void)
{
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "The image is too large or there is no more internal memory to open it.");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, close_unsupported_msgbox, LV_EVENT_CLICKED, mbox);
}

static void show_jpeg_unsupported_prompt(void)
{
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "The image is corrupted or this specific JPG type is not supported by the system.");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, close_unsupported_msgbox, LV_EVENT_CLICKED, mbox);
}

static void close_unsupported_msgbox(lv_event_t *e)
{
    lv_obj_t *mbox = lv_event_get_user_data(e);
    if (mbox) {
        lv_msgbox_close(mbox);
    }
}

static void on_datetime_click(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.screen){
        return;
    }
    
    build_date_time_dialog(ctx);
}

static void build_date_time_dialog(file_manager_ctx_t *ctx)
{
    if (ctx->graphics.date_time_overlay){
        lv_obj_delete(ctx->graphics.date_time_overlay);
        ctx->graphics.date_time_overlay = NULL;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    styles_set_bg_color(overlay, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    ctx->graphics.date_time_overlay = overlay;

    lv_obj_t *dlg = lv_obj_create(overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 6, 0);
    lv_obj_set_style_pad_gap(dlg, 4, 0);
    lv_obj_set_size(dlg, lv_pct(85), lv_pct(80));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(dlg, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dlg, LV_SCROLLBAR_MODE_AUTO);
    styles_set_dialog(dlg);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_center(dlg);
    ctx->graphics.date_time_dialog = dlg;

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Set Date&Time");
    lv_obj_set_style_text_font(title, &Domine_16, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_add_flag(title, LV_OBJ_FLAG_EVENT_BUBBLE);    

    /* SNTP row */
    lv_obj_t *row_sntp = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_sntp);
    lv_obj_set_flex_flow(row_sntp, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_sntp, 6, 0);
    lv_obj_set_style_pad_all(row_sntp, 0, 0);
    lv_obj_set_width(row_sntp, LV_PCT(90));
    lv_obj_set_height(row_sntp, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_sntp, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_sntp, 10, 0);
    lv_obj_add_flag(row_sntp, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *sntp_button = lv_button_create(row_sntp);
    lv_obj_set_flex_grow(sntp_button, 1);
    lv_obj_set_style_radius(sntp_button, 8, 0);
    lv_obj_set_style_pad_all(sntp_button, 6, 0); 
    styles_set_button(sntp_button);
    lv_obj_add_event_cb(sntp_button, sntp_date_time, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(sntp_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *sntp_lbl = lv_label_create(sntp_button);
    lv_label_set_text(sntp_lbl, "Wi-Fi & SNTP");
    lv_obj_center(sntp_lbl);      

    /* Manual time row */
    lv_obj_t *row_manual = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_manual);
    lv_obj_set_flex_flow(row_manual, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_manual, 6, 0);
    lv_obj_set_style_pad_all(row_manual, 0, 0);
    lv_obj_set_width(row_manual, LV_PCT(90));
    lv_obj_set_height(row_manual, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_manual, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_manual, 10, 0);
    lv_obj_add_flag(row_manual, LV_OBJ_FLAG_EVENT_BUBBLE);        

    lv_obj_t *manual_button = lv_button_create(row_manual);
    lv_obj_set_flex_grow(manual_button, 1);
    lv_obj_set_style_radius(manual_button, 8, 0);
    lv_obj_set_style_pad_all(manual_button, 6, 0); 
    styles_set_button(manual_button);
    lv_obj_add_event_cb(manual_button, manual_date_time, LV_EVENT_CLICKED, ctx);
    lv_obj_set_style_align(manual_button, LV_ALIGN_CENTER, 0);
    lv_obj_t *manual_lbl = lv_label_create(manual_button);
    lv_label_set_text(manual_lbl, "Manual Date&Time");
    lv_obj_center(manual_lbl);

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
    lv_obj_set_style_margin_top(close_btn, 15, 0);
    lv_obj_add_event_cb(close_btn, close_date_time_dialog, LV_EVENT_CLICKED, ctx);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "Close");
    lv_obj_center(close_lbl);
    lv_obj_add_flag(close_lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    
}

static void close_date_time_dialog(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e); 
    if (ctx && ctx->graphics.date_time_overlay) {
        lv_obj_del(ctx->graphics.date_time_overlay);
        ctx->graphics.date_time_dialog = NULL;
        ctx->graphics.date_time_overlay = NULL;
    }    
}

static void manual_date_time(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.screen){
        return;
    }
    
    settings_show_date_time_dialog(ctx ? ctx->graphics.screen : NULL);
}

static void sntp_date_time(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.screen){
        return;
    }
    
    settings_show_sntp_dialog(ctx ? ctx->graphics.screen : NULL);
}

static void start_clock_timer(file_manager_ctx_t *ctx)
{
    if (!ctx || ctx->flags.clock_timer_running) {
        return;
    }

    if (!ctx->clock_timer) {
        esp_timer_create_args_t args = {
            .callback = clock_timer_cb,
            .arg = NULL,
            .name = "fb_clock"
        };
        esp_err_t err = esp_timer_create(&args, &ctx->clock_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create clock timer: %s", esp_err_to_name(err));
            return;
        }
    }

    esp_err_t err = esp_timer_start_periodic(ctx->clock_timer, 1000000); /* 1s */
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to start clock timer: %s", esp_err_to_name(err));
        return;
    }
    ctx->flags.clock_timer_running = true;
}

static void clock_timer_cb(void *arg)
{
    /* Run UI update in LVGL context */
    lv_async_call(clock_update_async, NULL);
}

static void clock_update_async(void *arg)
{
    file_manager_ctx_t *ctx = &s_file_manager;
    if (!has_datetime_label(ctx)) {
        return;
    }

    if (!ctx->flags.clock_user_set) {
        clock_update_handle_not_set(ctx);
        return;
    }

    char buf[32];
    clock_update_format(buf, sizeof(buf));

    clock_update_apply(ctx, buf);
}

static bool has_datetime_label(const file_manager_ctx_t *ctx)
{
    return (ctx && ctx->graphics.datetime_label);
}

static void clock_update_handle_not_set(file_manager_ctx_t *ctx)
{
    if (ctx->graphics.datetime_btn) {
        lv_obj_clear_flag(ctx->graphics.datetime_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (ctx->graphics.datetime_label) {
        lv_obj_add_flag(ctx->graphics.datetime_label, LV_OBJ_FLAG_HIDDEN);
    }
}

static void clock_update_format(char *buf, size_t buf_len)
{
    time_t now = time(NULL);
    struct tm tm_info;
    localtime_r(&now, &tm_info);

    snprintf(buf, buf_len, "%02d:%02d - %02d/%02d/%02d",
             tm_info.tm_hour,
             tm_info.tm_min,
             tm_info.tm_mon + 1,
             tm_info.tm_mday,
             (tm_info.tm_year + 1900) % 100);
}

static void clock_update_apply(file_manager_ctx_t *ctx, const char *buf)
{
    lv_label_set_text(ctx->graphics.datetime_label, buf);

    /* Show the label and hide the button */
    if (ctx->graphics.datetime_btn) {
        lv_obj_add_flag(ctx->graphics.datetime_btn, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_flag(ctx->graphics.datetime_label, LV_OBJ_FLAG_HIDDEN);
}

static void on_item_click(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    if (item_click_handle_suppress(ctx)) {
        return;
    }

    const fs_nav_item_t *item = NULL;
    size_t index = 0;
    if (!item_click_get_item(ctx, e, &item, &index)) {
        return;
    }

    if (item_click_handle_dir(ctx, item, index)) {
        return;
    }

    if (item_click_handle_txt(ctx, item, index)) {
        return;
    }

    if (item_click_handle_jpeg(ctx, item)) {
        return;
    }

    show_unsupported_prompt();
}

static bool item_click_handle_suppress(file_manager_ctx_t *ctx)
{
    if (!ctx->flags.suppress_click) {
        return false;
    }
    ctx->flags.suppress_click = false;
    return true;
}

static bool item_click_get_item(file_manager_ctx_t *ctx, lv_event_t *e, const fs_nav_item_t **out_item, size_t *out_index)
{
    lv_obj_t *btn = lv_event_get_target(e);
    size_t index = (size_t)(uintptr_t)lv_obj_get_user_data(btn);

    size_t count = 0;
    const fs_nav_item_t *items = fs_nav_items(&ctx->nav, &count);
    if (!items || index >= count) {
        return false;
    }

    fs_nav_ensure_meta(&ctx->nav, index);
    *out_item = &items[index];
    *out_index = index;
    return true;
}

static bool item_click_handle_dir(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t index)
{
    if (!item->is_dir) {
        return false;
    }

    show_loading(ctx);
    esp_err_t err = fs_nav_enter(&ctx->nav, index);
    hide_loading(ctx);
    if (err == ESP_OK) {
        sync_view(ctx);
    } else {
        const char *item_name = (item && item->name) ? item->name : "<item>";
        ESP_LOGE(TAG, "Failed to enter \"%s\": %s", item_name, esp_err_to_name(err));
        sd_card_schedule_retry();
        schedule_wait_for_reconnection();
    }
    return true;
}

static bool item_click_handle_txt(file_manager_ctx_t *ctx, const fs_nav_item_t *item, size_t index)
{
    if (!fs_text_is_txt(item->name)) {
        return false;
    }

    ctx->reload_anchor_index = ctx->list_window_start + index;
    char path[FS_NAV_MAX_PATH];
    if (fs_nav_compose_path(&ctx->nav, item->name, path, sizeof(path)) == ESP_OK) {
        text_viewer_open_opts_t opts = {
            .path = path,
            .return_screen = ctx->graphics.screen,
            .editable = false,
        };
        show_loading(ctx);
        esp_err_t err = text_viewer_open(&opts);
        hide_loading(ctx);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to view \"%s\": %s", item->name, esp_err_to_name(err));
            sd_card_schedule_retry();
        }
    } else {
        ESP_LOGE(TAG, "Path too long for \"%s\"", item->name);
    }
    return true;
}

static bool item_click_handle_jpeg(file_manager_ctx_t *ctx, const fs_nav_item_t *item)
{
    if (!is_file_jpeg(item->name)) {
        return false;
    }
    handle_jpeg(ctx, item);
    return true;
}

static void on_list_scrolled(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || ctx->flags.list_suppress_scroll) {
        return;
    }

    list_scroll_update_edges(ctx);
}

static void list_scroll_update_edges(file_manager_ctx_t *ctx)
{
    bool at_top = lv_obj_get_scroll_top(ctx->graphics.list) <= 0;
    bool at_bottom = lv_obj_get_scroll_bottom(ctx->graphics.list) <= 0;

    if (at_bottom) {
        list_scroll_handle_bottom(ctx);
    } else {
        ctx->flags.list_at_bottom_edge = false;
    }

    if (at_top) {
        list_scroll_handle_top(ctx);
    } else {
        ctx->flags.list_at_top_edge = false;
    }
}

static void list_scroll_handle_bottom(file_manager_ctx_t *ctx)
{
    if (ctx->flags.list_at_bottom_edge) {
        return;
    }
    ctx->flags.list_at_bottom_edge = true;

    size_t total = fs_nav_total_items(&ctx->nav);

    size_t window_size = 1;
    size_t step = 1;
    get_window_params(ctx, &window_size, &step);

    size_t current_count = 0;
    fs_nav_items(&ctx->nav, &current_count);
    size_t available_end = ctx->list_window_start + current_count;
    if (total <= window_size || available_end >= total) {
        return;
    }

    size_t max_start = (total > window_size) ? (total - window_size) : 0;
    size_t new_start = ctx->list_window_start + step;
    if (new_start > max_start) new_start = max_start;
    size_t anchor_global = new_start + (step ? (step - 1) : 0); /* last overlapping item */
    if (anchor_global >= total) anchor_global = total ? (total - 1) : 0;
    ctx->flags.list_has_paged = true;
    build_item_list(ctx, new_start, anchor_global, true, false);
}

static void list_scroll_handle_top(file_manager_ctx_t *ctx)
{
    if (ctx->flags.list_at_top_edge) {
        return;
    }
    ctx->flags.list_at_top_edge = true;

    size_t total = fs_nav_total_items(&ctx->nav);

    size_t window_size = 1;
    size_t step = 1;
    get_window_params(ctx, &window_size, &step);

    if (total <= window_size || ctx->list_window_start == 0) {
        return;
    }

    size_t new_start = (ctx->list_window_start > step) ? (ctx->list_window_start - step) : 0;
    size_t anchor_global = new_start + step; /* first overlapping item from previous window */
    if (anchor_global >= total) anchor_global = total ? (total - 1) : 0;
    ctx->flags.list_has_paged = true;
    build_item_list(ctx, new_start, anchor_global, true, false);
}

static bool slider_can_handle_event(const file_manager_ctx_t *ctx)
{
    return (ctx && !ctx->flags.slider_suppress_change);
}

static bool slider_total_fits_window(size_t total, size_t window_size)
{
    return (total <= window_size);
}

static size_t slider_max_step_index(size_t max_start, size_t step)
{
    return step ? ((max_start + step - 1) / step) : 0;
}

static size_t slider_clamp_value(size_t max_step_index, int32_t slider_val)
{
    if (slider_val < 0) {
        slider_val = 0;
    }
    size_t clamped_step = (size_t)slider_val;
    if (clamped_step > max_step_index) {
        clamped_step = max_step_index;
    }
    return clamped_step;
}

static bool slider_handle_pressed(file_manager_ctx_t *ctx, size_t clamped_step, lv_event_code_t code)
{
    if (code != LV_EVENT_PRESSED) {
        return false;
    }
    ctx->flags.slider_drag_active = true;
    ctx->slider_pending_step = clamped_step;
    return true;
}

static bool slider_handle_value_changed(file_manager_ctx_t *ctx, size_t clamped_step, lv_event_code_t code)
{
    if (code != LV_EVENT_VALUE_CHANGED) {
        return false;
    }
    ctx->slider_pending_step = clamped_step;
    return true;
}

static size_t slider_target_step(file_manager_ctx_t *ctx, size_t clamped_step, size_t max_step_index)
{
    size_t target_step = (ctx->slider_pending_step == SIZE_MAX) ? clamped_step : ctx->slider_pending_step;
    if (target_step > max_step_index) {
        target_step = max_step_index;
    }
    return target_step;
}

static size_t slider_current_step(const file_manager_ctx_t *ctx, size_t step, size_t max_start, size_t max_step_index)
{
    size_t current_step = step ? (ctx->list_window_start / step) : 0;
    if (ctx->list_window_start >= max_start) {
        current_step = max_step_index;
    }
    return current_step;
}

static bool slider_is_noop(file_manager_ctx_t *ctx, size_t target_step, size_t current_step)
{
    if (target_step != current_step) {
        return false;
    }
    ctx->slider_pending_step = SIZE_MAX;
    ctx->flags.slider_drag_active = false;
    return true;
}

static size_t slider_compute_new_start(size_t target_step, size_t max_step_index, size_t step, size_t max_start)
{
    size_t new_start = (target_step >= max_step_index) ? max_start : (target_step * step);
    if (new_start > max_start) {
        new_start = max_start;
    }
    return new_start;
}

static void slider_finalize_paging(file_manager_ctx_t *ctx, size_t new_start)
{
    ctx->slider_pending_step = SIZE_MAX;
    ctx->flags.slider_drag_active = false;
    ctx->flags.list_has_paged = true;
    build_item_list(ctx, new_start, SIZE_MAX, true, true);
}

static void on_slider_value_changed(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!slider_can_handle_event(ctx)) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(e);

    size_t window_size = 1;
    size_t step = 1;
    get_window_params(ctx, &window_size, &step);
    size_t total = fs_nav_total_items(&ctx->nav);
    if (slider_total_fits_window(total, window_size)) {
        return; /* Nothing to scroll */
    }

    size_t max_start = total - window_size;
    size_t max_step_index = slider_max_step_index(max_start, step);

    int32_t slider_val = lv_slider_get_value(lv_event_get_target(e));
    size_t clamped_step = slider_clamp_value(max_step_index, slider_val);

    if (slider_handle_pressed(ctx, clamped_step, code)) {
        return;
    }

    if (slider_handle_value_changed(ctx, clamped_step, code)) {
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        size_t target_step = slider_target_step(ctx, clamped_step, max_step_index);

        size_t current_step = slider_current_step(ctx, step, max_start, max_step_index);
        if (slider_is_noop(ctx, target_step, current_step)) {
            return;
        }

        size_t new_start = slider_compute_new_start(target_step, max_step_index, step, max_start);

        slider_finalize_paging(ctx, new_start);
    }
}

static void on_item_long_press(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    ctx->flags.suppress_click = true;

    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_remove_state(btn, LV_STATE_PRESSED | LV_STATE_FOCUSED);
    size_t index = (size_t)(uintptr_t)lv_obj_get_user_data(btn);
    ctx->reload_anchor_index = ctx->list_window_start + index;

    size_t count = 0;
    const fs_nav_item_t *items = fs_nav_items(&ctx->nav, &count);
    if (!items || index >= count) {
        return;
    }

    fs_nav_ensure_meta(&ctx->nav, index);
    items = fs_nav_items(&ctx->nav, &count);
    const fs_nav_item_t *item = &items[index];
    prepare_action_item(ctx, item);
    show_action_menu(ctx);
}

static void on_parent_click(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    show_loading(ctx);
    esp_err_t err = fs_nav_go_parent(&ctx->nav);
    if (err == ESP_OK) {
        sync_view(ctx);
    } else {
        ESP_LOGE(TAG, "Failed to go parent: %s", esp_err_to_name(err));
        ctx->flags.pending_go_parent = true;
        sd_card_schedule_retry();
        schedule_wait_for_reconnection();
    }
    hide_loading(ctx);
}

static void on_settings_click(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.screen || !ctx->graphics.settings_btn){
        return;
    }

    esp_err_t err = settings_open_settings(ctx->graphics.screen);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Failed to open settings: (%s)", esp_err_to_name(err));
    }
}

static void on_tools_changed(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    static bool s_updating = false;
    if (s_updating) {
        return;
    }

    lv_obj_t *dd = lv_event_get_target(e);
    uint16_t sel = lv_dropdown_get_selected(dd);

    switch (sel) {
        case 0: start_new_folder(ctx); break;
        case 1: start_new_txt(ctx);    break;
        case 2: show_sort_dialog(ctx); break;
        default: break;
    }

    if (sel != 0) {
        s_updating = true;
        lv_dropdown_set_selected(dd, 0);
        lv_dropdown_set_text(dd, "Tools");
        s_updating = false;
    }
    else {
        lv_dropdown_set_text(dd, "Tools");
    }
}

static void apply_sort(file_manager_ctx_t *ctx, fs_nav_sort_mode_t mode, bool ascending)
{
    if (!ctx) {
        return;
    }

    if (fs_nav_set_sort(&ctx->nav, mode, ascending) == ESP_OK) {
        update_sort_badges(ctx);
        reset_window(ctx);
        build_item_list(ctx, ctx->list_window_start, SIZE_MAX, true, true);
    }
}

static void close_sort_dialog(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.sort_overlay) {
        return;
    }
    lv_obj_del(ctx->graphics.sort_overlay);
    ctx->graphics.sort_overlay = NULL;
    ctx->graphics.sort_criteria_dd = NULL;
    ctx->graphics.sort_direction_dd = NULL;
}

static void show_sort_dialog(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    close_sort_dialog(ctx);

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    ctx->graphics.sort_overlay = overlay;

    lv_obj_t *dlg = lv_obj_create(overlay);
    lv_obj_set_style_radius(dlg, 12, 0);
    lv_obj_set_style_pad_all(dlg, 6, 0);
    lv_obj_set_style_pad_gap(dlg, 4, 0);
    lv_obj_set_size(dlg, lv_pct(82), lv_pct(70));
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dlg, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    styles_set_bg_color(dlg, 0);
    lv_obj_set_style_bg_opa(dlg, LV_OPA_COVER, 0);
    styles_set_border_color(dlg, 0);
    lv_obj_set_style_border_width(dlg, 2, 0);
    styles_set_text_color(dlg, 0);
    lv_obj_center(dlg);

    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Sort");
    styles_set_text_color(title, 0);
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_font(title, &Domine_16, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *row_dir = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_dir);
    lv_obj_set_flex_flow(row_dir, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_dir, 6, 0);
    lv_obj_set_width(row_dir, LV_SIZE_CONTENT);
    lv_obj_set_height(row_dir, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_dir, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_dir, 6, 0);
    lv_obj_t *dir_lbl = lv_label_create(row_dir);
    styles_set_text_color(dir_lbl, 0);
    lv_label_set_text(dir_lbl, "Direction:");
    
    ctx->graphics.sort_direction_dd = lv_dropdown_create(row_dir);
    lv_dropdown_set_options_static(ctx->graphics.sort_direction_dd, "Ascending\nDescending");
    lv_obj_set_width(ctx->graphics.sort_direction_dd, 120);
    styles_set_button(ctx->graphics.sort_direction_dd);
    lv_obj_add_event_cb(ctx->graphics.sort_direction_dd, on_sort_direction_changed, LV_EVENT_VALUE_CHANGED, ctx);

    lv_obj_t *direction_list = lv_dropdown_get_list(ctx->graphics.sort_direction_dd);
    styles_set_dropdown(direction_list);

    lv_obj_update_layout(row_dir);
    lv_coord_t row_dir_w = lv_obj_get_width(row_dir);
    if (row_dir_w <= 0) {
        row_dir_w = LV_SIZE_CONTENT;
    }

    lv_obj_t *row_crit = lv_obj_create(dlg);
    lv_obj_remove_style_all(row_crit);
    lv_obj_set_flex_flow(row_crit, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row_crit, 6, 0);
    lv_obj_set_width(row_crit, row_dir_w);
    lv_obj_set_height(row_crit, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(row_crit, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_margin_top(row_crit, 3, 0);
    lv_obj_t *crit_lbl = lv_label_create(row_crit);
    styles_set_text_color(crit_lbl, 0);
    lv_label_set_text(crit_lbl, "Criteria:");

    ctx->graphics.sort_criteria_dd = lv_dropdown_create(row_crit);
    lv_dropdown_set_options_static(ctx->graphics.sort_criteria_dd, "Name\nDate\nSize");
    lv_obj_set_width(ctx->graphics.sort_criteria_dd, 120);
    lv_obj_add_event_cb(ctx->graphics.sort_criteria_dd, on_sort_criteria_changed, LV_EVENT_VALUE_CHANGED, ctx);
    styles_set_button(ctx->graphics.sort_criteria_dd);

    lv_obj_t *sort_list = lv_dropdown_get_list(ctx->graphics.sort_criteria_dd);
    styles_set_dropdown(sort_list);

    /* Place Criteria row above Direction while keeping aligned widths. */
    lv_obj_move_to_index(row_crit, lv_obj_get_index(row_dir));

    lv_obj_t *actions = lv_obj_create(dlg);
    lv_obj_remove_style_all(actions);
    lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(actions, 8, 0);
    lv_obj_set_width(actions, LV_PCT(100));
    lv_obj_set_height(actions, LV_SIZE_CONTENT);
    lv_obj_set_style_margin_top(actions, 10, 0);
    lv_obj_set_flex_align(actions, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *apply_btn = lv_button_create(actions);
    lv_obj_set_flex_grow(apply_btn, 1);
    styles_set_button(apply_btn);
    lv_obj_t *apply_lbl = lv_label_create(apply_btn);
    lv_label_set_text(apply_lbl, "Apply");
    styles_set_text_color(apply_lbl, 0);
    lv_obj_center(apply_lbl);
    lv_obj_add_event_cb(apply_btn, on_sort_apply, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_button_create(actions);
    lv_obj_set_flex_grow(cancel_btn, 1);
    styles_set_button(cancel_btn);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    styles_set_text_color(cancel_lbl, 0);
    lv_obj_center(cancel_lbl);
    lv_obj_add_event_cb(cancel_btn, on_sort_cancel, LV_EVENT_CLICKED, ctx);

    update_sort_badges(ctx);
}

static void start_new_txt(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    const char *dir = fs_nav_current_path(&ctx->nav);
    if (!dir) {
        return;
    }

    text_viewer_open_opts_t opts = {
        .directory = dir,
        .suggested_name = "new_file.txt",
        .return_screen = ctx->graphics.screen,
        .editable = true,
        .on_close = editor_closed,
        .user_ctx = ctx,
    };
    esp_err_t err = text_viewer_open(&opts);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start new file editor: %s", esp_err_to_name(err));
        sd_card_schedule_retry();
    }
}

static void start_new_folder(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    show_folder_dialog(ctx);
}

static void on_sort_criteria_changed(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
}

static void on_sort_direction_changed(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
}

static void on_sort_apply(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    fs_nav_sort_mode_t mode = fs_nav_get_sort(&ctx->nav);
    bool ascending = fs_nav_is_sort_ascending(&ctx->nav);

    if (ctx->graphics.sort_criteria_dd) {
        uint16_t sel = lv_dropdown_get_selected(ctx->graphics.sort_criteria_dd);
        if (sel < FS_NAV_SORT_COUNT) {
            mode = (fs_nav_sort_mode_t)sel;
        }
    }
    if (ctx->graphics.sort_direction_dd) {
        uint16_t sel = lv_dropdown_get_selected(ctx->graphics.sort_direction_dd);
        ascending = (sel == 0);
    }

    apply_sort(ctx, mode, ascending);
    close_sort_dialog(ctx);
}

static void on_sort_cancel(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    close_sort_dialog(ctx);
    update_sort_badges(ctx);
}

static void editor_closed(bool changed, void *user_ctx)
{
    file_manager_ctx_t *ctx = (file_manager_ctx_t *)user_ctx;
    if (!ctx || !changed) {
        return;
    }

    ctx->flags.preserve_window_on_reload = true;
    esp_err_t err = refresh_current_dir();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reload after editor: %s", esp_err_to_name(err));
        sd_card_schedule_retry();
    }
}

static void show_folder_dialog(file_manager_ctx_t *ctx)
{
    if (ctx->graphics.folder_dialog) {
        return;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    ctx->graphics.folder_dialog = overlay;

    lv_obj_t *dlg = lv_msgbox_create(overlay);
    styles_set_msgbox(dlg);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_max_width(dlg, LV_PCT(65), 0);
    lv_obj_set_width(dlg, LV_PCT(65));

    lv_obj_t *content = lv_msgbox_get_content(dlg);
    lv_obj_clear_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_left(content, 8, 0);
    lv_obj_set_style_pad_right(content, 8, 0);

    lv_obj_t *label = lv_label_create(content);
    lv_label_set_text(label, "Folder name");
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);

    ctx->graphics.folder_textarea = lv_textarea_create(content);
    lv_textarea_set_one_line(ctx->graphics.folder_textarea, true);
    lv_textarea_set_max_length(ctx->graphics.folder_textarea, FS_NAV_MAX_NAME - 1);
    lv_textarea_set_text(ctx->graphics.folder_textarea, "");
    styles_set_textarea(ctx->graphics.folder_textarea );
    lv_textarea_set_cursor_pos(ctx->graphics.folder_textarea, 0);
    lv_obj_set_width(ctx->graphics.folder_textarea, LV_PCT(100));

    ctx->graphics.folder_keyboard = lv_keyboard_create(overlay);
    styles_set_keyboard(ctx->graphics.folder_keyboard);
    lv_keyboard_set_textarea(ctx->graphics.folder_keyboard, ctx->graphics.folder_textarea);
    lv_obj_clear_flag(ctx->graphics.folder_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_state(ctx->graphics.folder_textarea, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(ctx->graphics.folder_keyboard, on_folder_keyboard_cancel, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->graphics.folder_textarea, on_folder_textarea_clicked, LV_EVENT_CLICKED, ctx);
    lv_obj_add_flag(ctx->graphics.folder_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_align(ctx->graphics.folder_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t *save_btn = lv_msgbox_add_footer_button(dlg, "Save");
    lv_obj_set_user_data(save_btn, (void *)1);
    lv_obj_set_flex_grow(save_btn, 1);
    lv_obj_set_style_pad_top(save_btn, 4, 0);
    lv_obj_set_style_pad_bottom(save_btn, 4, 0);
    lv_obj_set_style_min_height(save_btn, 32, 0);
    styles_set_button(save_btn);
    lv_obj_add_event_cb(save_btn, on_folder_create, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(dlg, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_pad_top(cancel_btn, 4, 0);
    lv_obj_set_style_pad_bottom(cancel_btn, 4, 0);
    lv_obj_set_style_min_height(cancel_btn, 32, 0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, on_folder_cancel, LV_EVENT_CLICKED, ctx);

    lv_obj_add_event_cb(ctx->graphics.folder_textarea, on_folder_create, LV_EVENT_READY, ctx);

    lv_obj_update_layout(ctx->graphics.folder_keyboard);
    lv_obj_update_layout(dlg);
    lv_coord_t keyboard_top = lv_obj_get_y(ctx->graphics.folder_keyboard);
    lv_coord_t dialog_h = lv_obj_get_height(dlg);
    lv_coord_t margin = 10;
    if (keyboard_top > dialog_h) {
        lv_coord_t candidate = (keyboard_top - dialog_h) / 2;
        if (candidate > 10) {
            margin = candidate;
        }
    }
    lv_obj_align(dlg, LV_ALIGN_TOP_MID, 0, margin);
}

static void close_folder_dialog(file_manager_ctx_t *ctx)
{
    if (!ctx->graphics.folder_dialog) {
        return;
    }
    lv_obj_del(ctx->graphics.folder_dialog);
    ctx->graphics.folder_dialog = NULL;
    ctx->graphics.folder_textarea = NULL;
    ctx->graphics.folder_keyboard = NULL;
}

static void on_folder_create(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    const char *text = ctx->graphics.folder_textarea ? lv_textarea_get_text(ctx->graphics.folder_textarea) : NULL;
    if (!text) {
        set_folder_status(ctx, "Invalid name", true);
        return;
    }

    char name[FS_NAV_MAX_NAME];
    strlcpy(name, text, sizeof(name));
    trim_whitespace(name);
    if (!is_valid_name(name)) {
        set_folder_status(ctx, "Invalid folder name", true);
        return;
    }

    esp_err_t err = create_folder(ctx, name);
    if (err != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            set_folder_status(ctx,
                                           "Name already exists (WARNING: FAT is case-insensitive)",
                                           true);
        } else {
            set_folder_status(ctx, esp_err_to_name(err), true);
            sd_card_schedule_retry();
        }
        return;
    }

    close_folder_dialog(ctx);
    esp_err_t reload = refresh_current_dir();
    if (reload != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh after folder create: %s", esp_err_to_name(reload));
        sd_card_schedule_retry();
    }
}

static void on_folder_cancel(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    close_folder_dialog(ctx);
}

static void set_folder_status(file_manager_ctx_t *ctx, const char *msg, bool error)
{
    if (!ctx || !ctx->graphics.folder_dialog || !msg) {
        return;
    }
    lv_obj_t *dlg = lv_obj_get_child(ctx->graphics.folder_dialog, 0);
    if (!dlg) {
        return;
    }
    lv_obj_t *content = lv_msgbox_get_content(dlg);
    if (!content) {
        return;
    }
    lv_obj_t *title = lv_obj_get_child(content, 0);
    if (!title) {
        return;
    }
    error ? lv_color_hex(0xff6b6b) : styles_set_text_color(title, 0);
    lv_label_set_text(title, msg);
}

static esp_err_t create_folder(file_manager_ctx_t *ctx, const char *name)
{
    char path[FS_NAV_MAX_PATH];
    esp_err_t err = fs_nav_compose_path(&ctx->nav, name, path, sizeof(path));
    if (err != ESP_OK) {
        return err;
    }

    if (mkdir(path, 0775) != 0) {
        if (errno == EEXIST) {
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGE(TAG, "mkdir(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void on_folder_keyboard_cancel(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.folder_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.folder_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.folder_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void on_folder_textarea_clicked(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.folder_keyboard || !ctx->graphics.folder_textarea) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.folder_keyboard, ctx->graphics.folder_textarea);
    lv_obj_clear_flag(ctx->graphics.folder_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static bool is_valid_name(const char *name)
{
    if (!name || name[0] == '\0') {
        return false;
    }
    for (const char *p = name; *p; ++p) {
        if (
                *p == '\\' || *p == '/' || *p == ':' ||
                *p == '*'  || *p == '?' || *p == '"' ||
                *p == '<'  || *p == '>' || *p == '|'
            ) 
        {
            return false;
        }
    }
    return true;
}

static void trim_whitespace(char *name)
{
    if (!name) {
        return;
    }
    char *start = name;
    while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r') {
        start++;
    }
    char *end = start + strlen(start);
    while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\n' || end[-1] == '\r')) {
        *--end = '\0';
    }
    if (start != name) {
        memmove(name, start, (size_t)(end - start) + 1);
    }
}

static bool delete_path_invalid_arg(const char *path)
{
    return (!path || path[0] == '\0');
}

static esp_err_t delete_path_stat(const char *path, struct stat *st)
{
    if (stat(path, st) != 0) {
        if (errno == ENOENT) {
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "stat(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static bool delete_path_is_dir(const struct stat *st)
{
    return S_ISDIR(st->st_mode);
}

static esp_err_t delete_path_compose_child(const char *parent, const char *name, char *out, size_t out_len)
{
    int needed = snprintf(out, out_len, "%s/%s", parent, name);
    if (needed < 0 || needed >= (int)out_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static esp_err_t delete_path_delete_file(const char *path)
{
    if (remove(path) != 0) {
        ESP_LOGE(TAG, "remove(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t delete_path_delete_dir(const char *path)
{
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    struct dirent *dent = NULL;
    while ((dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        char child[FS_NAV_MAX_PATH];
        esp_err_t compose = delete_path_compose_child(path, dent->d_name, child, sizeof(child));
        if (compose != ESP_OK) {
            closedir(dir);
            return compose;
        }
        esp_err_t err = delete_path(child);
        if (err != ESP_OK) {
            closedir(dir);
            return err;
        }
    }
    closedir(dir);
    if (rmdir(path) != 0) {
        ESP_LOGE(TAG, "rmdir(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t delete_path(const char *path)
{
    if (delete_path_invalid_arg(path)) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    esp_err_t st_err = delete_path_stat(path, &st);
    if (st_err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (st_err != ESP_OK) {
        return st_err;
    }

    if (delete_path_is_dir(&st)) {
        return delete_path_delete_dir(path);
    }

    return delete_path_delete_file(path);
}

static esp_err_t compute_total_size(const char *path, uint64_t *bytes)
{
    if (!path || !bytes || path[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    struct stat st;
    if (stat(path, &st) != 0) {
        ESP_LOGE(TAG, "stat(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    if (!S_ISDIR(st.st_mode)) {
        *bytes += (uint64_t)st.st_size;
        return ESP_OK;
    }

    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    struct dirent *dent = NULL;
    while ((dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        char child[FS_NAV_MAX_PATH];
        int needed = snprintf(child, sizeof(child), "%s/%s", path, dent->d_name);
        if (needed < 0 || needed >= (int)sizeof(child)) {
            closedir(dir);
            return ESP_ERR_INVALID_SIZE;
        }
        esp_err_t err = compute_total_size(child, bytes);
        if (err != ESP_OK) {
            closedir(dir);
            return err;
        }
    }
    closedir(dir);
    return ESP_OK;
}

static void show_message(const char *msg)
{
    if (!msg) {
        return;
    }
    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, msg);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, close_unsupported_msgbox, LV_EVENT_CLICKED, mbox);
}

static void clear_clipboard(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    memset(&ctx->clipboard, 0, sizeof(ctx->clipboard));
}

static void update_paste_button(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.paste_btn || !ctx->graphics.paste_label || !ctx->graphics.cancel_paste_btn) {
        return;
    }

    if (!ctx->clipboard.has_item) {
        lv_obj_add_flag(ctx->graphics.paste_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_state(ctx->graphics.paste_btn, LV_STATE_DISABLED);
        lv_obj_add_flag(ctx->graphics.cancel_paste_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_state(ctx->graphics.cancel_paste_btn, LV_STATE_DISABLED);
    } else {
        lv_obj_clear_flag(ctx->graphics.paste_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_state(ctx->graphics.paste_btn, LV_STATE_DISABLED);
        lv_obj_clear_flag(ctx->graphics.cancel_paste_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_state(ctx->graphics.cancel_paste_btn, LV_STATE_DISABLED);
    }
}

static bool path_exists(const char *path)
{
    struct stat st;
    return path && path[0] != '\0' && (stat(path, &st) == 0);
}

static bool is_subpath(const char *parent, const char *child)
{
    if (!parent || !child) {
        return false;
    }
    size_t parent_len = strlen(parent);
    size_t child_len = strlen(child);
    if (parent_len == 0 || child_len <= parent_len) {
        return false;
    }
    if (strncmp(parent, child, parent_len) != 0) {
        return false;
    }
    if (parent[parent_len - 1] == '/') {
        return true;
    }
    return child[parent_len] == '/';
}

static esp_err_t copy_file(const char *src, const char *dest)
{
    FILE *in = fopen(src, "rb");
    if (!in) {
        ESP_LOGE(TAG, "fopen(%s) failed (errno=%d)", src, errno);
        return ESP_FAIL;
    }
    FILE *out = fopen(dest, "wb");
    if (!out) {
        ESP_LOGE(TAG, "fopen(%s) failed (errno=%d)", dest, errno);
        fclose(in);
        return ESP_FAIL;
    }

    uint8_t buf[4096];
    size_t r = 0;
    esp_err_t err = ESP_OK;
    while ((r = fread(buf, 1, sizeof(buf), in)) > 0) {
        size_t w = fwrite(buf, 1, r, out);
        if (w != r) {
            ESP_LOGE(TAG, "fwrite(%s) failed (errno=%d)", dest, errno);
            err = ESP_FAIL;
            break;
        }
    }

    if (ferror(in)) {
        ESP_LOGE(TAG, "fread(%s) failed (errno=%d)", src, errno);
        err = ESP_FAIL;
    }

    fclose(out);
    fclose(in);
    if (err != ESP_OK) {
        remove(dest);
    }
    return err;
}

static esp_err_t copy_dir(const char *src, const char *dest)
{
    if (mkdir(dest, 0775) != 0) {
        ESP_LOGE(TAG, "mkdir(%s) failed (errno=%d)", dest, errno);
        return ESP_FAIL;
    }

    DIR *dir = opendir(src);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed (errno=%d)", src, errno);
        rmdir(dest);
        return ESP_FAIL;
    }

    struct dirent *dent = NULL;
    while ((dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        char child_src[FS_NAV_MAX_PATH];
        char child_dest[FS_NAV_MAX_PATH];
        int ns = snprintf(child_src, sizeof(child_src), "%s/%s", src, dent->d_name);
        int nd = snprintf(child_dest, sizeof(child_dest), "%s/%s", dest, dent->d_name);
        if (ns < 0 || ns >= (int)sizeof(child_src) || nd < 0 || nd >= (int)sizeof(child_dest)) {
            closedir(dir);
            delete_path(dest);
            return ESP_ERR_INVALID_SIZE;
        }
        esp_err_t err = copy_item(child_src, child_dest);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to copy item: (%s)", esp_err_to_name(err));
            closedir(dir);
            delete_path(dest);
            return err;
        }
    }
    closedir(dir);
    return ESP_OK;
}

static esp_err_t copy_item(const char *src, const char *dest)
{
    if (!src || !dest) {
        return ESP_ERR_INVALID_ARG;
    }
    struct stat st;
    if (stat(src, &st) != 0) {
        ESP_LOGE(TAG, "stat(%s) failed (errno=%d)", src, errno);
        return ESP_FAIL;
    }

    if (S_ISDIR(st.st_mode)) {
        return copy_dir(src, dest);
    }
    return copy_file(src, dest);
}

static bool copy_name_invalid_args(const char *directory, const char *name, char *out, size_t out_len)
{
    return (!directory || !name || !out || out_len == 0);
}

static void copy_name_split(const char *name, char *base, char *ext)
{
    const char *dot = strrchr(name, '.');
    if (dot && dot != name && dot[1] != '\0') {
        size_t base_len = (size_t)(dot - name);
        if (base_len >= FS_NAV_MAX_NAME) {
            base_len = FS_NAV_MAX_NAME - 1;
        }
        memcpy(base, name, base_len);
        base[base_len] = '\0';
        strlcpy(ext, dot, FS_NAV_MAX_NAME);
    } else {
        strlcpy(base, name, FS_NAV_MAX_NAME);
        ext[0] = '\0';
    }
}

static size_t copy_name_max_base_len(size_t ext_len)
{
    size_t max_suffix_len = 12; /* longest suffix: "_copy (100)" (11 chars) + cushion */
    size_t max_base_len = FS_NAV_MAX_NAME - 1;
    if (max_base_len > ext_len + max_suffix_len) {
        max_base_len -= (ext_len + max_suffix_len);
    } else {
        max_base_len = 0;
    }
    return max_base_len;
}

static esp_err_t copy_name_clamp_base(char *base, size_t max_base_len)
{
    if (max_base_len == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (strlen(base) > max_base_len) {
        base[max_base_len] = '\0';
    }
    return ESP_OK;
}

static bool copy_name_build_candidate(const char *base, const char *ext, int index, char *candidate, size_t cand_len)
{
    int written = 0;
    if (index == 0) {
        written = snprintf(candidate, cand_len, "%s_copy%s", base, ext);
    } else {
        written = snprintf(candidate, cand_len, "%s_copy (%d)%s", base, index + 1, ext);
    }
    return (written >= 0 && written < (int)cand_len);
}

static bool copy_name_compose_full(const char *directory, const char *candidate, char *full, size_t full_len)
{
    int needed = snprintf(full, full_len, "%s/%s", directory, candidate);
    return (needed >= 0 && needed < (int)full_len);
}

static esp_err_t generate_copy_name(const char *directory, const char *name, char *out, size_t out_len)
{
    if (copy_name_invalid_args(directory, name, out, out_len)) {
        return ESP_ERR_INVALID_ARG;
    }

    char base[FS_NAV_MAX_NAME];
    char ext[FS_NAV_MAX_NAME];
    copy_name_split(name, base, ext);

    char candidate[FS_NAV_MAX_NAME];
    size_t ext_len = strlen(ext);
    size_t max_base_len = copy_name_max_base_len(ext_len);
    esp_err_t clamp = copy_name_clamp_base(base, max_base_len);
    if (clamp != ESP_OK) {
        return clamp;
    }

    for (int i = 0; i < 100; ++i) {
        if (!copy_name_build_candidate(base, ext, i, candidate, sizeof(candidate))) {
            continue;
        }

        char full[FS_NAV_MAX_PATH];
        if (!copy_name_compose_full(directory, candidate, full, sizeof(full))) {
            continue;
        }
        if (!path_exists(full)) {
            strlcpy(out, candidate, out_len);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static void close_paste_conflict(file_manager_ctx_t *ctx)
{
    if (ctx && ctx->graphics.paste_conflict_mbox) {
        lv_msgbox_close(ctx->graphics.paste_conflict_mbox);
        ctx->graphics.paste_conflict_mbox = NULL;
        ctx->paste_conflict_path[0] = '\0';
        ctx->paste_conflict_name[0] = '\0';
    }
}

static void show_paste_conflict(file_manager_ctx_t *ctx, const char *dest_path)
{
    if (!ctx || !ctx->clipboard.has_item || !dest_path) {
        return;
    }
    close_paste_conflict(ctx);
    strlcpy(ctx->paste_conflict_path, dest_path, sizeof(ctx->paste_conflict_path));
    strlcpy(ctx->paste_conflict_name, ctx->clipboard.name, sizeof(ctx->paste_conflict_name));

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.paste_conflict_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text_fmt(label, "\"%s\" already exists. Replace or keep both?", ctx->paste_conflict_name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *replace_btn = lv_msgbox_add_footer_button(mbox, "Replace");
    lv_obj_set_user_data(replace_btn, (void *)1);
    lv_obj_add_event_cb(replace_btn, on_paste_conflict, LV_EVENT_CLICKED, ctx);

    lv_obj_t *rename_btn = lv_msgbox_add_footer_button(mbox, "Keep both");
    lv_obj_set_user_data(rename_btn, (void *)2);
    lv_obj_add_event_cb(rename_btn, on_paste_conflict, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    lv_obj_add_event_cb(cancel_btn, on_paste_conflict, LV_EVENT_CLICKED, ctx);
}

static esp_err_t perform_paste(file_manager_ctx_t *ctx, const char *dest_path, bool allow_overwrite)
{
    if (!ctx || !ctx->clipboard.has_item || !dest_path) {
        return ESP_ERR_INVALID_STATE;
    }

    if (ctx->clipboard.is_dir && is_subpath(ctx->clipboard.src_path, dest_path)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!allow_overwrite && path_exists(dest_path)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (allow_overwrite && path_exists(dest_path)) {
        esp_err_t del = delete_path(dest_path);
        if (del != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete destination before overwrite: %s", esp_err_to_name(del));
            return del;
        }
    }

    esp_err_t err = ESP_OK;
    if (ctx->clipboard.cut) {
        if (rename(ctx->clipboard.src_path, dest_path) != 0) {
            if (errno != EXDEV) {
                ESP_LOGW(TAG, "rename(%s -> %s) failed (errno=%d), falling back to copy+delete", ctx->clipboard.src_path, dest_path, errno);
            }
            err = copy_item(ctx->clipboard.src_path, dest_path);
            if (err == ESP_OK) {
                err = delete_path(ctx->clipboard.src_path);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to remove source after cut: %s", esp_err_to_name(err));
                }
            }
        }
        if (err == ESP_OK) {
            clear_clipboard(ctx);
            update_second_header(ctx);
        }
        return err;
    }

    err = copy_item(ctx->clipboard.src_path, dest_path);
    if (err == ESP_OK) {
        clear_clipboard(ctx);
        update_second_header(ctx);
    }else{
        ESP_LOGE(TAG, "Failed to copy item: (%s)", esp_err_to_name(err));
    }
    return err;
}

static void on_paste_click(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->clipboard.has_item) {
        return;
    }

    char dest_path[FS_NAV_MAX_PATH];
    esp_err_t err = fs_nav_compose_path(&ctx->nav, ctx->clipboard.name, dest_path, sizeof(dest_path));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to compose paste path: %s", esp_err_to_name(err));
        show_message("Destination path too long.");
        return;
    }

    if (strcmp(dest_path, ctx->clipboard.src_path) == 0) {
        show_message("Already in this folder.");
        return;
    }

    if (ctx->clipboard.is_dir && is_subpath(ctx->clipboard.src_path, dest_path)) {
        show_message("Cannot paste a folder inside itself.");
        return;
    }

    if (!ctx->clipboard.cut) {
        uint64_t total = 0;
        esp_err_t size_err = compute_total_size(ctx->clipboard.src_path, &total);
        if (size_err != ESP_OK) {
            sd_card_schedule_retry();
            return;
        }
        strlcpy(ctx->paste_target_path, dest_path, sizeof(ctx->paste_target_path));
        ctx->flags.paste_target_valid = true;
        show_copy_confirm(ctx, total);
        return;
    }

    if (path_exists(dest_path)) {
        show_paste_conflict(ctx, dest_path);
        return;
    }
    show_loading(ctx);
    err = perform_paste(ctx, dest_path, false);
    hide_loading(ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Paste failed: (%s)", esp_err_to_name(err));
        sd_card_schedule_retry();
        return;
    }

    ctx->flags.preserve_window_on_reload = true;
    set_reload_anchor_current(ctx);
    err = refresh_current_dir();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh after paste: %s", esp_err_to_name(err));
        sd_card_schedule_retry();
    }
}

static void on_paste_conflict(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    char conflict_path[FS_NAV_MAX_PATH];
    char conflict_name[FS_NAV_MAX_NAME];
    strlcpy(conflict_path, ctx->paste_conflict_path, sizeof(conflict_path));
    strlcpy(conflict_name, ctx->paste_conflict_name, sizeof(conflict_name));
    int action = (int)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    close_paste_conflict(ctx);

    if (!ctx->clipboard.has_item || conflict_path[0] == '\0') {
        return;
    }

    esp_err_t err = ESP_OK;
    show_loading(ctx);
    if (action == 1) {
        err = perform_paste(ctx, conflict_path, true);
    } else if (action == 2) {
        const char *last = strrchr(conflict_path, '/');
        if (!last) {
            hide_loading(ctx);
            show_message("Invalid destination path.");
            return;
        }
        char directory[FS_NAV_MAX_PATH];
        if (last == conflict_path) {
            /* Conflict path at root, treat directory as "/" */
            strlcpy(directory, "/", sizeof(directory));
        } else {
            size_t dir_len = (size_t)(last - conflict_path);
            if (dir_len >= sizeof(directory)) {
                hide_loading(ctx);
                show_message("Path too long.");
                return;
            }
            memcpy(directory, conflict_path, dir_len);
            directory[dir_len] = '\0';
        }

        char new_name[FS_NAV_MAX_NAME];
        err = generate_copy_name(directory, conflict_name, new_name, sizeof(new_name));
        if (err != ESP_OK) {
            hide_loading(ctx);
            show_message("Could not generate a new name.");
            return;
        }

        char dest_path[FS_NAV_MAX_PATH];
        int needed = snprintf(dest_path, sizeof(dest_path), "%s/%s", directory, new_name);
        if (needed < 0 || needed >= (int)sizeof(dest_path)) {
            hide_loading(ctx);
            show_message("Path too long.");
            return;
        }
        err = perform_paste(ctx, dest_path, false);
    } else {
        hide_loading(ctx);
        return;
    }
    hide_loading(ctx);

    if (err != ESP_OK) {
        show_message(esp_err_to_name(err));
        sd_card_schedule_retry();
        return;
    }

    ctx->flags.preserve_window_on_reload = true;
    set_reload_anchor_current(ctx);
    err = refresh_current_dir();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh after paste: %s", esp_err_to_name(err));
        sd_card_schedule_retry();
    }
}

static void on_cancel_paste_click(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);

    if (!ctx || !ctx->graphics.cancel_paste_btn || !ctx->graphics.cancel_paste_label){
        return;
    }

    clear_clipboard(ctx);
    update_second_header(ctx);
}

static void close_copy_confirm(file_manager_ctx_t *ctx)
{
    if (ctx && ctx->graphics.copy_confirm_mbox) {
        lv_msgbox_close(ctx->graphics.copy_confirm_mbox);
        ctx->graphics.copy_confirm_mbox = NULL;
    }
}

static void show_copy_confirm(file_manager_ctx_t *ctx, uint64_t bytes)
{
    if (!ctx || !ctx->clipboard.has_item || !ctx->flags.paste_target_valid) {
        return;
    }
    close_copy_confirm(ctx);

    char size_str[32];
    format_size64(bytes, size_str, sizeof(size_str));

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.copy_confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text_fmt(label, "Copy %s?", size_str);
    styles_set_text_color(label, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *ok_btn = lv_msgbox_add_footer_button(mbox, "OK");
    lv_obj_set_user_data(ok_btn, (void *)1);
    styles_set_button(ok_btn);
    lv_obj_add_event_cb(ok_btn, on_copy_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(mbox, "Cancel");
    lv_obj_set_user_data(cancel_btn, (void *)0);
    styles_set_button(cancel_btn);
    lv_obj_add_event_cb(cancel_btn, on_copy_confirm, LV_EVENT_CLICKED, ctx);
}

static void on_copy_confirm(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    bool confirm = (bool)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    close_copy_confirm(ctx);

    if (!confirm || !ctx->flags.paste_target_valid) {
        ctx->flags.paste_target_valid = false;
        ctx->paste_target_path[0] = '\0';
        return;
    }

    char dest_path[FS_NAV_MAX_PATH];
    strlcpy(dest_path, ctx->paste_target_path, sizeof(dest_path));
    ctx->flags.paste_target_valid = false;
    ctx->paste_target_path[0] = '\0';

    if (path_exists(dest_path)) {
        show_paste_conflict(ctx, dest_path);
        return;
    }

    show_loading(ctx);
    esp_err_t err = perform_paste(ctx, dest_path, false);
    hide_loading(ctx);
    if (err != ESP_OK) {
        show_message(esp_err_to_name(err));
        sd_card_schedule_retry();
        return;
    }

    ctx->flags.preserve_window_on_reload = true;
    set_reload_anchor_current(ctx);
    err = refresh_current_dir();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh after paste: %s", esp_err_to_name(err));
        sd_card_schedule_retry();
    }
}

static void prepare_action_item(file_manager_ctx_t *ctx, const fs_nav_item_t *item)
{
    if (!ctx || !item) {
        return;
    }
    ctx->action_item.active = true;
    ctx->action_item.is_dir = item->is_dir;
    ctx->action_item.is_txt = !item->is_dir && fs_text_is_txt(item->name);
    strlcpy(ctx->action_item.name, item->name, sizeof(ctx->action_item.name));
    const char *dir = fs_nav_current_path(&ctx->nav);
    if (!dir) {
        dir = "";
    }
    strlcpy(ctx->action_item.directory, dir, sizeof(ctx->action_item.directory));
}

static void show_action_menu(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->action_item.active) {
        return;
    }
    close_action_menu(ctx);

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    lv_coord_t mbox_pad = lv_obj_get_style_pad_left(mbox, 0);
    lv_obj_set_style_pad_all(mbox, mbox_pad + 8, 0);
    ctx->graphics.action_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, ctx->action_item.name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_coord_t label_pad_bottom = lv_obj_get_style_pad_bottom(label, 0);
    lv_obj_set_style_pad_bottom(label, label_pad_bottom + 10, 0);

    lv_obj_t *footer = lv_obj_create(mbox);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(footer, 4, 0);

    lv_obj_t *row1 = lv_obj_create(footer);
    lv_obj_remove_style_all(row1);
    lv_obj_set_size(row1, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row1, 4, 0);

    lv_obj_t *rename_btn = lv_button_create(row1);
    lv_obj_set_flex_grow(rename_btn, 1);
    styles_set_button(rename_btn);
    lv_obj_t *rename_lbl = lv_label_create(rename_btn);
    lv_label_set_text(rename_lbl, "Rename");
    styles_set_text_color(rename_lbl, 0);
    lv_obj_center(rename_lbl);
    lv_obj_set_user_data(rename_btn, (void *)FILE_BROWSER_ACTION_RENAME);
    lv_obj_add_event_cb(rename_btn, on_action_button, LV_EVENT_CLICKED, ctx);

    lv_obj_t *del_btn = lv_button_create(row1);
    lv_obj_set_flex_grow(del_btn, 1);
    styles_set_button(del_btn);
    lv_obj_t *del_lbl = lv_label_create(del_btn);
    lv_label_set_text(del_lbl, "Delete");
    styles_set_text_color(del_lbl, 0);
    lv_obj_center(del_lbl);
    lv_obj_set_user_data(del_btn, (void *)FILE_BROWSER_ACTION_DELETE);
    lv_obj_add_event_cb(del_btn, on_action_button, LV_EVENT_CLICKED, ctx);

    lv_obj_t *row2 = lv_obj_create(footer);
    lv_obj_remove_style_all(row2);
    lv_obj_set_size(row2, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row2, 4, 0);

    lv_obj_t *copy_btn = lv_button_create(row2);
    lv_obj_set_flex_grow(copy_btn, 1);
    styles_set_button(copy_btn);
    lv_obj_t *copy_lbl = lv_label_create(copy_btn);
    lv_label_set_text(copy_lbl, "Copy");
    styles_set_text_color(del_lbl, 0);
    lv_obj_center(copy_lbl);
    lv_obj_set_user_data(copy_btn, (void *)FILE_BROWSER_ACTION_COPY);
    lv_obj_add_event_cb(copy_btn, on_action_button, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cut_btn = lv_button_create(row2);
    lv_obj_set_flex_grow(cut_btn, 1);
    styles_set_button(cut_btn);
    lv_obj_t *cut_lbl = lv_label_create(cut_btn);
    lv_label_set_text(cut_lbl, "Cut");
    styles_set_text_color(cut_lbl, 0);
    lv_obj_center(cut_lbl);
    lv_obj_set_user_data(cut_btn, (void *)FILE_BROWSER_ACTION_CUT);
    lv_obj_add_event_cb(cut_btn, on_action_button, LV_EVENT_CLICKED, ctx);

    lv_obj_t *row3 = lv_obj_create(footer);
    lv_obj_remove_style_all(row3);
    lv_obj_set_size(row3, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row3, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row3, 4, 0);

    bool has_edit = (!ctx->action_item.is_dir && ctx->action_item.is_txt);
    if (has_edit) {
        lv_obj_t *edit_btn = lv_button_create(row3);
        lv_obj_set_flex_grow(edit_btn, 1);
        styles_set_button(edit_btn);
        lv_obj_t *edit_lbl = lv_label_create(edit_btn);
        lv_label_set_text(edit_lbl, "Edit");
        styles_set_text_color(edit_lbl, 0);
        lv_obj_center(edit_lbl);
        lv_obj_set_user_data(edit_btn, (void *)FILE_BROWSER_ACTION_EDIT);
        lv_obj_add_event_cb(edit_btn, on_action_button, LV_EVENT_CLICKED, ctx);

        lv_obj_t *cancel_btn = lv_button_create(row3);
        lv_obj_set_flex_grow(cancel_btn, 1);
        styles_set_button(cancel_btn);
        lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
        lv_label_set_text(cancel_lbl, "Cancel");
        styles_set_text_color(cancel_lbl, 0);
        lv_obj_center(cancel_lbl);
        lv_obj_set_user_data(cancel_btn, (void *)FILE_BROWSER_ACTION_CANCEL);
        lv_obj_add_event_cb(cancel_btn, on_action_button, LV_EVENT_CLICKED, ctx);
    } else {
        lv_obj_set_flex_align(row3, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_t *cancel_btn = lv_button_create(row3);
        lv_obj_set_flex_grow(cancel_btn, 0);
        lv_obj_set_width(cancel_btn, LV_PCT(60));
        styles_set_button(cancel_btn);
        lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
        lv_label_set_text(cancel_lbl, "Cancel");
        styles_set_text_color(cancel_lbl, 0);
        lv_obj_center(cancel_lbl);
        lv_obj_set_user_data(cancel_btn, (void *)FILE_BROWSER_ACTION_CANCEL);
        lv_obj_add_event_cb(cancel_btn, on_action_button, LV_EVENT_CLICKED, ctx);
    }
}

static void close_action_menu(file_manager_ctx_t *ctx)
{
    if (ctx && ctx->graphics.action_mbox) {
        lv_msgbox_close(ctx->graphics.action_mbox);
        ctx->graphics.action_mbox = NULL;
    }
}

static void on_action_button(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    file_manager_action_type_t action = (file_manager_action_type_t)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));

    close_action_menu(ctx);

    switch (action) {
        case FILE_BROWSER_ACTION_EDIT: {
            if (!ctx->action_item.active || ctx->action_item.is_dir || !ctx->action_item.is_txt) {
                return;
            }
            char path[FS_NAV_MAX_PATH];
            if (action_compose_path(ctx, path, sizeof(path)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to compose path for edit");
                return;
            }
            text_viewer_open_opts_t opts = {
                .path = path,
                .return_screen = ctx->graphics.screen,
                .editable = true,
                .on_close = editor_closed,
                .user_ctx = ctx,
            };
            esp_err_t err = text_viewer_open(&opts);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to edit \"%s\": %s", ctx->action_item.name, esp_err_to_name(err));
                sd_card_schedule_retry();
            } else {
                clear_action_state(ctx);
            }
            break;
        }
        case FILE_BROWSER_ACTION_RENAME:
            show_rename_dialog(ctx);
            break;
        case FILE_BROWSER_ACTION_DELETE:
            file_manager_show_delete_confirm(ctx);
            break;
        case FILE_BROWSER_ACTION_COPY:
        case FILE_BROWSER_ACTION_CUT: {
            if (!ctx->action_item.active) {
                return;
            }
            char src_path[FS_NAV_MAX_PATH];
            if (action_compose_path(ctx, src_path, sizeof(src_path)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to compose path for clipboard");
                return;
            }
            memset(&ctx->clipboard, 0, sizeof(ctx->clipboard));
            ctx->clipboard.has_item = true;
            ctx->clipboard.cut = (action == FILE_BROWSER_ACTION_CUT);
            ctx->clipboard.is_dir = ctx->action_item.is_dir;
            strlcpy(ctx->clipboard.name, ctx->action_item.name, sizeof(ctx->clipboard.name));
            strlcpy(ctx->clipboard.src_path, src_path, sizeof(ctx->clipboard.src_path));
            update_second_header(ctx);
            clear_action_state(ctx);
            break;
        }
        case FILE_BROWSER_ACTION_CANCEL:
        default:
            clear_action_state(ctx);
            break;
    }
}

static void file_manager_show_delete_confirm(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->action_item.active) {
        return;
    }
    close_delete_confirm(ctx);

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.confirm_mbox = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text_fmt(label, "Delete \"%s\"?", ctx->action_item.name);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *yes_btn = lv_msgbox_add_footer_button(mbox, "Yes");
    styles_set_button(yes_btn);
    lv_obj_set_user_data(yes_btn, (void *)1);
    lv_obj_add_event_cb(yes_btn, delete_confirm, LV_EVENT_CLICKED, ctx);

    lv_obj_t *no_btn = lv_msgbox_add_footer_button(mbox, "No");
    styles_set_button(no_btn);
    lv_obj_set_user_data(no_btn, (void *)0);
    lv_obj_add_event_cb(no_btn, delete_confirm, LV_EVENT_CLICKED, ctx);
}

static void close_delete_confirm(file_manager_ctx_t *ctx)
{
    if (ctx && ctx->graphics.confirm_mbox) {
        lv_msgbox_close(ctx->graphics.confirm_mbox);
        ctx->graphics.confirm_mbox = NULL;
    }
}

static void hide_loading(file_manager_ctx_t *ctx)
{
    if (ctx && ctx->graphics.loading_dialog) {
        lv_msgbox_close(ctx->graphics.loading_dialog);
        ctx->graphics.loading_dialog = NULL;
    }
}

static void show_loading(file_manager_ctx_t *ctx)
{
    if (!ctx || ctx->graphics.loading_dialog) {
        return;
    }

    lv_obj_t *mbox = lv_msgbox_create(NULL);
    styles_set_msgbox(mbox);
    ctx->graphics.loading_dialog = mbox;
    lv_obj_set_style_max_width(mbox, LV_PCT(80), 0);
    lv_obj_center(mbox);

    lv_obj_t *label = lv_label_create(mbox);
    lv_label_set_text(label, "Loading");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    styles_set_text_color(label, 0);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    /* Force an immediate refresh so the mbox appears before heavy work. */
    lv_obj_invalidate(mbox);
    lv_refr_now(NULL);
}

static void delete_confirm(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    bool confirm = (bool)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    close_delete_confirm(ctx);

    if (!confirm) {
        clear_action_state(ctx);
        return;
    }

    show_loading(ctx);
    esp_err_t err = selected_item(ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Delete failed: %s", esp_err_to_name(err));
        sd_card_schedule_retry();
    }
    hide_loading(ctx);
}

static esp_err_t selected_item(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->action_item.active) {
        return ESP_ERR_INVALID_STATE;
    }

    char path[FS_NAV_MAX_PATH];
    esp_err_t err = action_compose_path(ctx, path, sizeof(path));
    if (err != ESP_OK) {
        return err;
    }

    err = delete_path(path);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete %s: %s", path, esp_err_to_name(err));
        return err;
    }

    clear_action_state(ctx);
    ctx->flags.preserve_window_on_reload = true;
    set_reload_anchor_current(ctx);
    return refresh_current_dir();
}

static esp_err_t action_compose_path(const file_manager_ctx_t *ctx, char *out, size_t out_len)
{
    if (!ctx || !ctx->action_item.active || !out || out_len == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ctx->action_item.directory[0] == '\0' || ctx->action_item.name[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }
    int needed = snprintf(out, out_len, "%s/%s", ctx->action_item.directory, ctx->action_item.name);
    if (needed < 0 || needed >= (int)out_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static void clear_action_state(file_manager_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }
    close_action_menu(ctx);
    close_delete_confirm(ctx);
    close_copy_confirm(ctx);
    close_rename_dialog(ctx);
    ctx->action_item.active = false;
    ctx->action_item.is_dir = false;
    ctx->action_item.is_txt = false;
    ctx->action_item.name[0] = '\0';
    ctx->action_item.directory[0] = '\0';
    ctx->flags.paste_target_valid = false;
    ctx->paste_target_path[0] = '\0';
}

static void set_rename_status(file_manager_ctx_t *ctx, const char *msg, bool error)
{
    if (!ctx || !ctx->graphics.rename_dialog || !msg) {
        return;
    }
    lv_obj_t *dlg = lv_obj_get_child(ctx->graphics.rename_dialog, 0);
    if (!dlg) {
        return;
    }
    lv_obj_t *content = lv_msgbox_get_content(dlg);
    if (!content) {
        return;
    }
    lv_obj_t *title = lv_obj_get_child(content, 0);
    if (!title) {
        return;
    }
    error ? lv_color_hex(0xff6b6b) : styles_set_text_color(title, 0);
    lv_label_set_text(title, msg);
}

static void show_rename_dialog(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->action_item.active) {
        return;
    }
    close_rename_dialog(ctx);

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_30, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_FLOATING | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_CLICK_FOCUSABLE);
    ctx->graphics.rename_dialog = overlay;

    lv_obj_t *dlg = lv_msgbox_create(overlay);
    styles_set_msgbox(dlg);
    lv_obj_add_flag(dlg, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_max_width(dlg, LV_PCT(65), 0);
    lv_obj_set_width(dlg, LV_PCT(65));

    lv_obj_t *content = lv_msgbox_get_content(dlg);
    lv_obj_clear_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *label = lv_label_create(content);
    styles_set_text_color(label, 0);
    lv_label_set_text(label, ctx->action_item.is_dir ? "Folder name" : "File name");
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_pad_left(content, 8, 0);
    lv_obj_set_style_pad_right(content, 8, 0);

    ctx->graphics.rename_textarea = lv_textarea_create(content);
    lv_textarea_set_one_line(ctx->graphics.rename_textarea, true);
    lv_textarea_set_max_length(ctx->graphics.rename_textarea, FS_NAV_MAX_NAME - 1);
    lv_textarea_set_text(ctx->graphics.rename_textarea, ctx->action_item.name);
    lv_textarea_set_cursor_pos(ctx->graphics.rename_textarea, LV_TEXTAREA_CURSOR_LAST);
    lv_obj_set_width(ctx->graphics.rename_textarea, LV_PCT(100));
    styles_set_textarea(ctx->graphics.rename_textarea);

    ctx->graphics.rename_keyboard = lv_keyboard_create(overlay);
    styles_set_keyboard(ctx->graphics.rename_keyboard);
    lv_keyboard_set_textarea(ctx->graphics.rename_keyboard, ctx->graphics.rename_textarea);
    lv_obj_clear_flag(ctx->graphics.rename_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_state(ctx->graphics.rename_textarea, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(ctx->graphics.rename_keyboard, on_rename_keyboard_cancel, LV_EVENT_CANCEL, ctx);
    lv_obj_add_event_cb(ctx->graphics.rename_textarea, on_rename_textarea_clicked, LV_EVENT_CLICKED, ctx);
    lv_obj_add_event_cb(ctx->graphics.rename_textarea, on_rename_accept, LV_EVENT_READY, ctx);
    lv_obj_update_layout(ctx->graphics.rename_keyboard);
    lv_obj_add_flag(ctx->graphics.rename_keyboard, LV_OBJ_FLAG_FLOATING);
    lv_obj_align(ctx->graphics.rename_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t *save_btn = lv_msgbox_add_footer_button(dlg, "Save");
    styles_set_button(save_btn);
    lv_obj_set_user_data(save_btn, (void *)1);
    lv_obj_set_flex_grow(save_btn, 1);
    lv_obj_set_style_pad_top(save_btn, 4, 0);
    lv_obj_set_style_pad_bottom(save_btn, 4, 0);
    lv_obj_set_style_min_height(save_btn, 32, 0);
    lv_obj_add_event_cb(save_btn, on_rename_accept, LV_EVENT_CLICKED, ctx);

    lv_obj_t *cancel_btn = lv_msgbox_add_footer_button(dlg, "Cancel");
    styles_set_button(cancel_btn);
    lv_obj_set_user_data(cancel_btn, (void *)0);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_style_pad_top(cancel_btn, 4, 0);
    lv_obj_set_style_pad_bottom(cancel_btn, 4, 0);
    lv_obj_set_style_min_height(cancel_btn, 32, 0);
    lv_obj_add_event_cb(cancel_btn, on_rename_cancel, LV_EVENT_CLICKED, ctx);

    lv_obj_update_layout(dlg);
    lv_coord_t keyboard_top = lv_obj_get_y(ctx->graphics.rename_keyboard);
    lv_coord_t dialog_h = lv_obj_get_height(dlg);
    lv_coord_t margin = 10;
    if (keyboard_top > dialog_h) {
        lv_coord_t candidate = (keyboard_top - dialog_h) / 2;
        if (candidate > 10) {
            margin = candidate;
        }
    }
    lv_obj_align(dlg, LV_ALIGN_TOP_MID, 0, margin);
}

static void close_rename_dialog(file_manager_ctx_t *ctx)
{
    if (!ctx || !ctx->graphics.rename_dialog) {
        return;
    }
    lv_obj_del(ctx->graphics.rename_dialog);
    ctx->graphics.rename_dialog = NULL;
    ctx->graphics.rename_textarea = NULL;
    ctx->graphics.rename_keyboard = NULL;
}

static void on_rename_accept(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.rename_textarea) {
        return;
    }

    char name[FS_NAV_MAX_NAME];
    const char *text = lv_textarea_get_text(ctx->graphics.rename_textarea);
    if (!rename_validate_input(ctx, text, name)) {
        return;
    }

    if (strcmp(name, ctx->action_item.name) == 0) {
        if (rename_handle_noop(ctx, name)) {
            return;
        }
    }

    esp_err_t err = rename_apply(ctx, name);
    if (err != ESP_OK) {
        return;
    }

    rename_finalize_success(ctx);
}

static void on_rename_cancel(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }
    close_rename_dialog(ctx);
    clear_action_state(ctx);
}

static esp_err_t perform_rename(file_manager_ctx_t *ctx, const char *new_name)
{
    if (!ctx || !ctx->action_item.active || !new_name || new_name[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }

    char old_path[FS_NAV_MAX_PATH];
    esp_err_t err = action_compose_path(ctx, old_path, sizeof(old_path));
    if (err != ESP_OK) {
        return err;
    }

    char new_path[FS_NAV_MAX_PATH];
    int needed = snprintf(new_path, sizeof(new_path), "%s/%s", ctx->action_item.directory, new_name);
    if (needed < 0 || needed >= (int)sizeof(new_path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (rename(old_path, new_path) != 0) {
        if (errno == EEXIST) {
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGE(TAG, "rename(%s -> %s) failed (errno=%d)", old_path, new_path, errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static bool rename_validate_input(file_manager_ctx_t *ctx, const char *text, char *out_name)
{
    if (!text) {
        set_rename_status(ctx, "Invalid name", true);
        return false;
    }
    strlcpy(out_name, text, FS_NAV_MAX_NAME);
    trim_whitespace(out_name);
    if (!is_valid_name(out_name)) {
        set_rename_status(ctx, "Invalid name", true);
        return false;
    }
    return true;
}

static bool rename_handle_noop(file_manager_ctx_t *ctx, const char *name)
{
    if (strcmp(name, ctx->action_item.name) != 0) {
        return false;
    }
    close_rename_dialog(ctx);
    clear_action_state(ctx);
    return true;
}

static esp_err_t rename_apply(file_manager_ctx_t *ctx, const char *name)
{
    esp_err_t err = perform_rename(ctx, name);
    if (err != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            set_rename_status(ctx,
                              "Name already exists (WARNING: FAT is case-insensitive)",
                              true);
        } else {
            set_rename_status(ctx, esp_err_to_name(err), true);
            sd_card_schedule_retry();
        }
    }
    return err;
}

static void rename_finalize_success(file_manager_ctx_t *ctx)
{
    close_rename_dialog(ctx);
    clear_action_state(ctx);
    ctx->flags.preserve_window_on_reload = true;
    esp_err_t reload = refresh_current_dir();
    if (reload != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh after rename: %s", esp_err_to_name(reload));
        sd_card_schedule_retry();
    }
}

static void on_rename_keyboard_cancel(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.rename_keyboard) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.rename_keyboard, NULL);
    lv_obj_add_flag(ctx->graphics.rename_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void on_rename_textarea_clicked(lv_event_t *e)
{
    file_manager_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->graphics.rename_keyboard || !ctx->graphics.rename_textarea) {
        return;
    }
    lv_keyboard_set_textarea(ctx->graphics.rename_keyboard, ctx->graphics.rename_textarea);
    lv_obj_clear_flag(ctx->graphics.rename_keyboard, LV_OBJ_FLAG_HIDDEN);
}
