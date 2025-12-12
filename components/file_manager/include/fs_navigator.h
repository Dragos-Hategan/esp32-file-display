#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <time.h>

#include "esp_err.h"

#define FS_NAV_MAX_PATH 256
#define FS_NAV_MAX_NAME 96

typedef enum {
    FS_NAV_SORT_NAME = 0,
    FS_NAV_SORT_DATE = 1,
    FS_NAV_SORT_SIZE = 2,
    FS_NAV_SORT_COUNT
} fs_nav_sort_mode_t;

typedef struct {
    char *name;
    bool is_dir;
    bool needs_stat;
    size_t size_bytes;
    time_t modified;
} fs_nav_item_t;

typedef struct fs_nav {
    char root[FS_NAV_MAX_PATH];
    char current[FS_NAV_MAX_PATH];
    char relative[FS_NAV_MAX_PATH];
    fs_nav_item_t *items;
    size_t item_count;      /* number of items currently loaded in buffer */
    size_t capacity;         /* allocated capacity of buffer */
    size_t max_items;      /* threshold for enabling sort (0 = no threshold) */
    size_t total_items;    /* full count in current directory */
    size_t window_start;     /* current window offset */
    size_t window_size;      /* desired window size */
    fs_nav_sort_mode_t sort_mode;
    bool ascending;
    bool sort_enabled;
} fs_nav_t;

typedef struct {
    const char *root_path;
    size_t max_items;
} fs_nav_config_t;

/**
 * @brief Initialize a navigator rooted at @p cfg->root_path and load persisted state if present.
 *
 * Trims trailing slashes, validates root is an absolute directory, restores last relative path,
 * sort mode and direction from NVS (best effort), then performs an initial directory scan.
 *
 * @param[out] nav Navigator instance to initialize.
 * @param[in]  cfg Configuration (root path and item cap).
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG on null args or invalid root path
 * - ESP_ERR_NOT_FOUND if root is not accessible
 * - Errors from fs_nav_refresh on initial scan failure
 */
esp_err_t fs_nav_init(fs_nav_t *nav, const fs_nav_config_t *cfg);

/**
 * @brief Release resources held by the navigator (directory items).
 *
 * @param[in,out] nav Navigator to deinitialize (safe to pass NULL).
 */
void fs_nav_deinit(fs_nav_t *nav);

/**
 * @brief Rescan the current directory and refresh navigator state.
 *
 * Computes @c total_items. If @c total_items <= @c max_items (or @c max_items==0),
 * sorting stays enabled and all items are loaded/sorted. Otherwise, sorting is disabled and
 * only the first window is loaded; additional windows must be fetched via @c fs_nav_set_window().
 *
 * @param[in,out] nav Navigator.
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if @p nav is NULL
 * - ESP_FAIL if directory cannot be opened
 * - ESP_ERR_NO_MEM if allocation fails
 */
esp_err_t fs_nav_refresh(fs_nav_t *nav);

/**
 * @brief Get a pointer to the currently loaded items window.
 *
 * When sorting is enabled, returns a slice starting at @c window_start of length
 * up to @c window_size. When sorting is disabled, returns the last window loaded
 * via @c fs_nav_set_window().
 *
 * @param[in]  nav   Navigator.
 * @param[out] count Optional; set to number of valid items in the window.
 * @return Pointer to internal array (NULL if unavailable).
 * @warning The pointer becomes invalid after @c fs_nav_refresh, @c fs_nav_set_window or sort changes.
 */
const fs_nav_item_t *fs_nav_items(const fs_nav_t *nav, size_t *count);

/**
 * @brief Get absolute current path (root + relative).
 * @param[in] nav Navigator.
 * @return NUL-terminated path or NULL.
 */
const char *fs_nav_current_path(const fs_nav_t *nav);

/**
 * @brief Get relative path from root ('' means root).
 * @param[in] nav Navigator.
 * @return NUL-terminated relative path or NULL.
 */
const char *fs_nav_relative_path(const fs_nav_t *nav);

/**
 * @brief Whether a parent directory exists (i.e., we are not at root).
 * @param[in] nav Navigator.
 * @return true if @p nav is valid and relative path is non-empty.
 */
bool fs_nav_can_go_parent(const fs_nav_t *nav);

/**
 * @brief Enter the directory at @p index in the current listing and refresh.
 *
 * @param[in,out] nav   Navigator.
 * @param[in]     index Index into @c fs_nav_items.
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG for bad args/index
 * - ESP_ERR_INVALID_STATE if selected item is not a directory
 * - Errors from internal path set/refresh/store
 */
esp_err_t fs_nav_enter(fs_nav_t *nav, size_t index);

/**
 * @brief Go to parent directory (if any) and refresh.
 *
 * @param[in,out] nav Navigator.
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_STATE if already at root
 * - Errors from internal path set/refresh/store
 */
esp_err_t fs_nav_go_parent(fs_nav_t *nav);

/**
 * @brief Set sort mode and direction, then sort current items and persist state.
 *
 * @param[in,out] nav       Navigator.
 * @param[in]     mode      Sort mode (Name/Date/Size).
 * @param[in]     ascending true for ascending, false for descending.
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if @p nav is NULL or mode is invalid
 * - Errors from state store
 */
esp_err_t fs_nav_set_sort(fs_nav_t *nav, fs_nav_sort_mode_t mode, bool ascending);

/**
 * @brief Get current sort mode.
 * @param[in] nav Navigator.
 * @return Sort mode (Name by default if @p nav is NULL).
 */
fs_nav_sort_mode_t fs_nav_get_sort(const fs_nav_t *nav);

/**
 * @brief Check if sort direction is ascending.
 * @param[in] nav Navigator.
 * @return true if ascending (default true when @p nav is NULL).
 */
bool fs_nav_is_sort_ascending(const fs_nav_t *nav);

/**
 * @brief Check if sorting is currently enabled (total_items <= max_items or max_items==0).
 */
bool fs_nav_is_sort_enabled(const fs_nav_t *nav);

/**
 * @brief Set the current window (offset + size) to load for the directory listing.
 *
 * When sorting is enabled (item count <= max_items), only the view window is adjusted.
 * When sorting is disabled (item count > max_items or max_items==0), this will reload
 * just the requested window from the filesystem without holding all items.
 *
 * @param nav   Navigator.
 * @param start Zero-based offset into the directory items.
 * @param size  Number of items to load (must be >0).
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG on bad params; ESP_FAIL on I/O errors;
 *         ESP_ERR_NO_MEM on allocation failure.
 */
esp_err_t fs_nav_set_window(fs_nav_t *nav, size_t start, size_t size);

/**
 * @brief Get total number of items in current directory.
 */
size_t fs_nav_total_items(const fs_nav_t *nav);

/**
 * @brief Get current window start offset.
 */
size_t fs_nav_window_start(const fs_nav_t *nav);

/**
 * @brief Compose an absolute path by appending @p item_name to the current directory.
 *
 * @param[in]  nav         Navigator (must be initialized).
 * @param[in]  item_name  Child item name (e.g., from fs_nav_items()).
 * @param[out] out         Buffer for the resulting absolute path.
 * @param[in]  out_len     Size of @p out.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on bad inputs, ESP_ERR_INVALID_SIZE if buffer is too small.
 */
esp_err_t fs_nav_compose_path(const fs_nav_t *nav, const char *item_name, char *out, size_t out_len);

/**
 * @brief Ensure metadata (stat) for an item if pending.
 *
 * @param nav Navigator.
 * @param index Item index.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG on bad inputs; ESP_FAIL on stat errors.
 */
esp_err_t fs_nav_ensure_meta(fs_nav_t *nav, size_t index);

#ifdef __cplusplus
}
#endif
