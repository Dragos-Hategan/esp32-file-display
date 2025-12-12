#include "fs_navigator.h"

#include <dirent.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include "esp_heap_caps.h"

#include "esp_crc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"

#define TAG "fs_nav"

#define FS_NAV_STATE_MAGIC 0x464E4156u
#define FS_NAV_NVS_NAMESPACE "fsnav"
#define FS_NAV_NVS_KEY "state_v1"
#define FS_NAV_STATE_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    char relative[FS_NAV_MAX_PATH];
    uint32_t sort_mode;
    uint8_t ascending;
    uint8_t reserved[3];
    uint32_t crc32;
} fs_nav_state_blob_t;

static fs_nav_sort_mode_t s_cmp_mode = FS_NAV_SORT_NAME;
static bool s_cmp_ascending = true;
/**
 * @brief Validate a relative path (no leading '/', no '.' or '..' segments).
 *
 * Empty string is allowed (root). Slash-separated segments must be non-empty and
 * not equal to "." or "..".
 *
 * @param[in] relative Candidate relative path.
 * @return true if valid.
 */
static bool is_valid_relative(const char *relative);

/**
 * @brief Free capacity buffer, item names and reset item_count.
 *
 * @param nav Navigator.
 */
static void clear_items(fs_nav_t *nav);

/**
 * @brief Recompute absolute current path from root + relative.
 *
 * @param[in,out] nav Navigator.
 */
static void update_current_path(fs_nav_t *nav);

/**
 * @brief Verify that the SD mount (root/current) is still accessible.
 *
 * @param nav Navigator descriptor.
 * @return ESP_OK if both root and current paths are directories; error otherwise.
 */
static esp_err_t check_storage_ready(const fs_nav_t *nav);

/**
 * @brief Set @c nav->relative (validated & cleaned) and rebuild @c nav->current.
 *
 * @param[in,out] nav      Navigator.
 * @param[in]     relative New relative path (may be NULL or empty for root).
 * @return
 * - ESP_OK on success
 * - ESP_ERR_INVALID_ARG if @p relative is invalid
 * - ESP_ERR_INVALID_SIZE if paths exceed buffers
 */
static esp_err_t set_relative(fs_nav_t *nav, const char *relative);

/**
 * @brief Persist current relative path and sort settings to NVS.
 *
 * Computes CRC32 over blob fields and commits.
 *
 * @param[in] nav Navigator.
 * @return
 * - ESP_OK on success
 * - Errors from NVS open/set/commit
 */
static esp_err_t store_state(const fs_nav_t *nav);

/**
 * @brief Load persisted state (relative path and sort) from NVS and validate it.
 *
 * Validates blob size, magic, version, CRC, then applies relative path and sort.
 * If the restored path doesn't exist anymore, resets to root with @c ESP_ERR_NOT_FOUND.
 *
 * @param[in,out] nav Navigator to modify.
 * @return
 * - ESP_OK on success
 * - ESP_ERR_NVS_* / ESP_ERR_INVALID_* on decode/validation failures
 * - ESP_ERR_NOT_FOUND if restored path no longer exists
 */
static esp_err_t load_state(fs_nav_t *nav);

/**
 * @brief Sort the current items array with current mode and direction.
 *
 * Directories are kept together and sorted by name; files follow the chosen mode.
 *
 * @param[in,out] nav Navigator (no-op for <2 items or null array).
 */
static void sort_items(fs_nav_t *nav);

/**
 * @brief qsort comparator for @c fs_nav_item_t honoring directories-first and sort settings.
 *
 * Directories are always grouped before files. Within directories, sorting is by name to keep
 * navigation intuitive. Files are sorted by current mode (Name/Date/Size). Ties fall back to name.
 *
 * @param lhs Pointer to @c fs_nav_item_t (left).
 * @param rhs Pointer to @c fs_nav_item_t (right).
 * @return Negative/zero/positive per strcmp-style semantics; reversed if descending.
 */
static int item_compare(const void *lhs, const void *rhs);

/**
 * @brief Initialize navigator defaults from configuration.
 *
 * @param nav Navigator context to initialize (cleared on success).
 * @param cfg Configuration containing root path and limits.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG on null args or missing root.
 */
static esp_err_t init_nav_defaults(fs_nav_t *nav, const fs_nav_config_t *cfg);

/**
 * @brief Copy and sanitize the root path (strip trailing slashes, require absolute).
 *
 * @param nav       Navigator context.
 * @param root_path Root path to copy.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG on empty/overflow or non-absolute paths.
 */
static esp_err_t sanitize_root(fs_nav_t *nav, const char *root_path);

/**
 * @brief Verify that the current root path exists and is a directory.
 *
 * @param nav Navigator context (expects current path set).
 * @return ESP_OK if accessible; ESP_ERR_NOT_FOUND otherwise.
 */
static esp_err_t verify_root_accessible(fs_nav_t *nav);

/**
 * @brief Log a warning if loading persisted state failed.
 *
 * @param nav       Navigator context.
 * @param state_err Result returned by load_state().
 */
static void apply_loaded_state_or_log(fs_nav_t *nav, esp_err_t state_err);

/**
 * @brief Trigger initial refresh and log on failure.
 *
 * @param nav Navigator context.
 * @return ESP_OK on success; error from fs_nav_refresh otherwise.
 */
static esp_err_t perform_initial_refresh(fs_nav_t *nav);

/**
 * @brief Clear items and reset counters prior to a refresh.
 *
 * @param nav Navigator context.
 */
static void reset_refresh_state(fs_nav_t *nav);

/**
 * @brief Count directory entries excluding "." and "..".
 *
 * @param nav       Navigator context.
 * @param total_out Out: total entries found.
 * @return ESP_OK on success; ESP_FAIL on errors.
 */
static esp_err_t count_directory_entries(fs_nav_t *nav, size_t *total_out);

/**
 * @brief Ensure a non-zero window size (defaults to 32).
 *
 * @param nav Navigator context.
 */
static void ensure_window_size_default(fs_nav_t *nav);

/**
 * @brief Refresh navigator in sorted mode (load all, sort, set window).
 *
 * @param nav   Navigator context.
 * @param total Total items to load.
 * @return ESP_OK on success; error codes from allocation/loading otherwise.
 */
static esp_err_t refresh_sorted(fs_nav_t *nav, size_t total);

/**
 * @brief Ensure item array capacity for target count.
 *
 * @param nav    Navigator context.
 * @param target Desired capacity.
 * @return ESP_OK on success; ESP_ERR_NO_MEM on allocation failure.
 */
static esp_err_t allocate_items(fs_nav_t *nav, size_t target);

/**
 * @brief Create a navigator item from a dirent.
 *
 * @param dent Directory entry.
 * @param dest Destination item to populate.
 * @return ESP_OK on success; ESP_ERR_NO_MEM on allocation failure.
 */
static esp_err_t create_nav_item_from_dirent(const struct dirent *dent, fs_nav_item_t *dest);

/**
 * @brief Process a single directory entry during load.
 *
 * Skips "." and "..", creates item, increments index, and propagates error code.
 *
 * @param dent       Current dirent.
 * @param nav        Navigator context.
 * @param idx        In/out index of next slot.
 * @param load_errno Out errno-like error to set on allocation failure.
 * @return ESP_OK on success; ESP_FAIL/ESP_ERR_NO_MEM otherwise.
 */
static esp_err_t load_directory_entry(struct dirent *dent, fs_nav_t *nav, size_t *idx, int *load_errno);

/**
 * @brief Load all directory entries into the item array (sorted mode).
 *
 * @param nav Navigator context (expects capacity reserved).
 * @return ESP_OK on success; ESP_FAIL/ESP_ERR_NO_MEM on errors.
 */
static esp_err_t load_directory_items(fs_nav_t *nav);

/**
 * @brief Get current window slice for sorted mode.
 *
 * @param nav   Navigator context.
 * @param count Out: number of items returned (may be NULL).
 * @return Pointer to first item in window, or NULL if out of range.
 */
static const fs_nav_item_t *items_for_sorted(const fs_nav_t *nav, size_t *count);

/**
 * @brief Get current items pointer/count for unsorted mode.
 *
 * @param nav   Navigator context.
 * @param count Out: number of items (may be NULL).
 * @return Pointer to items array (may be NULL if empty).
 */
static const fs_nav_item_t *items_for_unsorted(const fs_nav_t *nav, size_t *count);

esp_err_t fs_nav_init(fs_nav_t *nav, const fs_nav_config_t *cfg)
{
    esp_err_t err = init_nav_defaults(nav, cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = sanitize_root(nav, cfg->root_path);
    if (err != ESP_OK) {
        return err;
    }

    err = set_relative(nav, "");
    if (err != ESP_OK) {
        return err;
    }

    err = verify_root_accessible(nav);
    if (err != ESP_OK) {
        return err;
    }

    apply_loaded_state_or_log(nav, load_state(nav));
    return perform_initial_refresh(nav);
}

void fs_nav_deinit(fs_nav_t *nav)
{
    if (!nav) {
        return;
    }
    clear_items(nav);
}

esp_err_t fs_nav_refresh(fs_nav_t *nav)
{
    if (!nav) {
        return ESP_ERR_INVALID_ARG;
    }

    reset_refresh_state(nav);

    esp_err_t storage_err = check_storage_ready(nav);
    if (storage_err != ESP_OK) {
        nav->item_count = 0;
        return storage_err;
    }

    size_t total = 0;
    esp_err_t err = count_directory_entries(nav, &total);
    if (err != ESP_OK) {
        return err;
    }

    if (total == 0) {
        nav->total_items = 0;
        nav->item_count = 0;
        return ESP_OK;
    }

    nav->total_items = total;
    nav->sort_enabled = (nav->max_items == 0) ? true : (total <= nav->max_items);
    ensure_window_size_default(nav);

    if (nav->sort_enabled) {
        return refresh_sorted(nav, total);
    }

    nav->window_start = 0;
    return fs_nav_set_window(nav, 0, nav->window_size);
}

const fs_nav_item_t *fs_nav_items(const fs_nav_t *nav, size_t *count)
{
    if (!nav || !nav->items) {
        if (count) {
            *count = 0;
        }
        return NULL;
    }

    return nav->sort_enabled ? items_for_sorted(nav, count) : items_for_unsorted(nav, count);
}

const char *fs_nav_current_path(const fs_nav_t *nav)
{
    return nav ? nav->current : NULL;
}

const char *fs_nav_relative_path(const fs_nav_t *nav)
{
    return nav ? nav->relative : NULL;
}

bool fs_nav_can_go_parent(const fs_nav_t *nav)
{
    return nav && nav->relative[0] != '\0';
}

esp_err_t fs_nav_enter(fs_nav_t *nav, size_t index)
{
    if (!nav || index >= nav->item_count) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t actual_index = nav->sort_enabled ? (nav->window_start + index) : index;
    if (actual_index >= nav->item_count) {
        return ESP_ERR_INVALID_ARG;
    }

    const fs_nav_item_t *item = &nav->items[actual_index];
    if (!item->is_dir) {
        return ESP_ERR_INVALID_STATE;
    }

    char prev_relative[FS_NAV_MAX_PATH];
    strlcpy(prev_relative, nav->relative, sizeof(prev_relative));

    char next_relative[FS_NAV_MAX_PATH];
    if (prev_relative[0] == '\0') {
        strlcpy(next_relative, item->name, sizeof(next_relative));
    } else {
        int written = snprintf(next_relative, sizeof(next_relative), "%s/%s", prev_relative, item->name);
        if (written <= 0 || (size_t)written >= sizeof(next_relative)) {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    esp_err_t err = set_relative(nav, next_relative);
    if (err != ESP_OK) {
        return err;
    }

    err = fs_nav_refresh(nav);
    if (err == ESP_OK) {
        store_state(nav);
    } else {
        // best-effort restore previous location if refresh failed
        set_relative(nav, prev_relative);
    }
    return err;
}

esp_err_t fs_nav_go_parent(fs_nav_t *nav)
{
    if (!fs_nav_can_go_parent(nav)) {
        return ESP_ERR_INVALID_STATE;
    }

    char prev_relative[FS_NAV_MAX_PATH];
    strlcpy(prev_relative, nav->relative, sizeof(prev_relative));

    char new_relative[FS_NAV_MAX_PATH];
    strlcpy(new_relative, prev_relative, sizeof(new_relative));

    char *slash = strrchr(new_relative, '/');
    if (slash) {
        *slash = '\0';
    } else {
        new_relative[0] = '\0';
    }

    esp_err_t err = set_relative(nav, new_relative);
    if (err != ESP_OK) {
        return err;
    }

    err = fs_nav_refresh(nav);
    if (err == ESP_OK) {
        store_state(nav);
    } else {
        // Restore previous relative path so navigation state doesn't drift
        set_relative(nav, prev_relative);
    }
    return err;
}

esp_err_t fs_nav_set_sort(fs_nav_t *nav, fs_nav_sort_mode_t mode, bool ascending)
{
    if (!nav || mode >= FS_NAV_SORT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    nav->sort_mode = mode;
    nav->ascending = ascending;
    if (nav->sort_enabled) {
        sort_items(nav);
    }
    return store_state(nav);
}

fs_nav_sort_mode_t fs_nav_get_sort(const fs_nav_t *nav)
{
    return nav ? nav->sort_mode : FS_NAV_SORT_NAME;
}

bool fs_nav_is_sort_ascending(const fs_nav_t *nav)
{
    return nav ? nav->ascending : true;
}

bool fs_nav_is_sort_enabled(const fs_nav_t *nav)
{
    return nav ? nav->sort_enabled : true;
}

esp_err_t fs_nav_set_window(fs_nav_t *nav, size_t start, size_t size)
{
    if (!nav || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (nav->total_items == 0) {
        clear_items(nav);
        nav->window_start = 0;
        nav->window_size = size;
        return ESP_OK;
    }

    if (start >= nav->total_items) {
        start = nav->total_items ? (nav->total_items - 1) : 0;
    }

    nav->window_start = start;
    nav->window_size = size;

    if (nav->sort_enabled) {
        /* All items are already loaded; slicing is handled by fs_nav_items */
        return ESP_OK;
    }

    clear_items(nav);

    if (nav->capacity < size) {
        fs_nav_item_t *new_items = heap_caps_realloc(nav->items,
                                                        size * sizeof(fs_nav_item_t),
                                                        MALLOC_CAP_8BIT);
        if (!new_items) {
            ESP_LOGE(TAG, "Out of memory while allocating window of %zu items for \"%s\"", size, nav->current);
            return ESP_ERR_NO_MEM;
        }
        nav->items = new_items;
        nav->capacity = size;
    }

    DIR *dir = opendir(nav->current);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed while setting window: errno=%d", nav->current, errno);
        nav->item_count = 0;
        return ESP_FAIL;
    }

    struct dirent *dent = NULL;
    errno = 0;
    /* Skip to start */
    size_t skipped = 0;
    while (skipped < start && (dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        skipped++;
    }

    size_t idx = 0;
    errno = 0;
    while ((dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        if (idx >= size) {
            break;
        }

        fs_nav_item_t *dest = &nav->items[idx];
        memset(dest, 0, sizeof(*dest));

        size_t name_len = strnlen(dent->d_name, FS_NAV_MAX_NAME - 1);
        dest->name = (char *)heap_caps_malloc(name_len + 1, MALLOC_CAP_8BIT);
        if (!dest->name) {
            ESP_LOGE(TAG, "Out of memory duplicating item name");
            break;
        }
        memcpy(dest->name, dent->d_name, name_len);
        dest->name[name_len] = '\0';

        dest->needs_stat = true;
        dest->is_dir = (dent->d_type == DT_DIR);
        dest->size_bytes = 0;
        dest->modified = 0;

        idx++;
    }
    int load_errno = errno;
    closedir(dir);

    nav->item_count = idx;

    if (load_errno != 0) {
        clear_items(nav);
        ESP_LOGE(TAG, "readdir(%s) failed while loading window: errno=%d", nav->current, load_errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

size_t fs_nav_total_items(const fs_nav_t *nav)
{
    return nav ? nav->total_items : 0;
}

size_t fs_nav_window_start(const fs_nav_t *nav)
{
    return nav ? nav->window_start : 0;
}

esp_err_t fs_nav_compose_path(const fs_nav_t *nav, const char *item_name, char *out, size_t out_len)
{
    if (!nav || !item_name || !out || out_len == 0 || item_name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    int needed = snprintf(out, out_len, "%s/%s", nav->current, item_name);
    if (needed < 0 || needed >= (int)out_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t fs_nav_ensure_meta(fs_nav_t *nav, size_t index)
{
    if (!nav || !nav->items) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t actual_index = nav->sort_enabled ? (nav->window_start + index) : index;
    if (actual_index >= nav->item_count) {
        return ESP_ERR_INVALID_ARG;
    }

    fs_nav_item_t *e = &nav->items[actual_index];
    if (!e->needs_stat) {
        return ESP_OK;
    }

    char path[FS_NAV_MAX_PATH * 2];
    int written = snprintf(path, sizeof(path), "%s/%s", nav->current, e->name ? e->name : "");
    if (written <= 0 || (size_t)written >= sizeof(path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    struct stat st = {0};
    if (stat(path, &st) != 0) {
        ESP_LOGE(TAG, "stat(%s) failed: errno=%d", path, errno);
        return ESP_FAIL;
    }

    e->is_dir = S_ISDIR(st.st_mode);
    e->size_bytes = st.st_size;
    e->modified = st.st_mtime;
    e->needs_stat = false;
    return ESP_OK;
}

static bool is_valid_relative(const char *relative)
{
    if (!relative || relative[0] == '\0') {
        return true;
    }

    const char *p = relative;
    while (*p == '/') {
        p++;
    }

    while (*p) {
        const char *segment_start = p;
        while (*p && *p != '/') {
            p++;
        }
        size_t len = p - segment_start;
        if (len == 0) {
            return false;
        }
        if ((len == 1 && segment_start[0] == '.') ||
            (len == 2 && segment_start[0] == '.' && segment_start[1] == '.')) {
            return false;
        }
        if (*p == '/') {
            p++;
        }
    }
    return true;
}

static void clear_items(fs_nav_t *nav)
{
    if (!nav || !nav->items) {
        return;
    }
    for (size_t i = 0; i < nav->item_count; ++i) {
        if (nav->items[i].name) {
            heap_caps_free(nav->items[i].name);
            nav->items[i].name = NULL;
        }
    }
    heap_caps_free(nav->items);
    nav->items = NULL;
    nav->capacity = 0;
    nav->item_count = 0;
}

static void update_current_path(fs_nav_t *nav)
{
    if (nav->relative[0] == '\0') {
        strlcpy(nav->current, nav->root, sizeof(nav->current));
    } else {
        strlcpy(nav->current, nav->root, sizeof(nav->current));
        strlcat(nav->current, "/", sizeof(nav->current));
        strlcat(nav->current, nav->relative, sizeof(nav->current));
    }
}

static esp_err_t check_storage_ready(const fs_nav_t *nav)
{
    if (!nav) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    if (stat(nav->root, &st) != 0 || !S_ISDIR(st.st_mode)) {
        int err = errno;
        ESP_LOGE(TAG, "Storage root \"%s\" unavailable (errno=%d)", nav->root, err);
        return ESP_ERR_NOT_FOUND;
    }
    if (stat(nav->current, &st) != 0 || !S_ISDIR(st.st_mode)) {
        int err = errno;
        ESP_LOGE(TAG, "Directory \"%s\" unavailable (errno=%d)", nav->current, err);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static esp_err_t set_relative(fs_nav_t *nav, const char *relative)
{
    const char *clean = relative ? relative : "";
    while (*clean == '/') {
        clean++;
    }

    if (!is_valid_relative(clean)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t rel_len = strlen(clean);
    if (rel_len >= sizeof(nav->relative)) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t root_len = strlen(nav->root);
    size_t needed = root_len + (rel_len ? 1 : 0) + rel_len;
    if (needed >= sizeof(nav->current)) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (rel_len == 0) {
        nav->relative[0] = '\0';
    } else {
        strlcpy(nav->relative, clean, sizeof(nav->relative));
    }

    update_current_path(nav);
    return ESP_OK;
}

static esp_err_t store_state(const fs_nav_t *nav)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FS_NAV_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    fs_nav_state_blob_t blob = {
        .magic = FS_NAV_STATE_MAGIC,
        .version = FS_NAV_STATE_VERSION,
        .sort_mode = nav->sort_mode,
        .ascending = nav->ascending ? 1 : 0,
    };
    strlcpy(blob.relative, nav->relative, sizeof(blob.relative));
    blob.crc32 = esp_crc32_le(0, (const uint8_t *)&blob, sizeof(blob) - sizeof(blob.crc32));

    err = nvs_set_blob(handle, FS_NAV_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t load_state(fs_nav_t *nav)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FS_NAV_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    fs_nav_state_blob_t blob = {0};
    size_t blob_size = sizeof(blob);
    err = nvs_get_blob(handle, FS_NAV_NVS_KEY, &blob, &blob_size);
    nvs_close(handle);
    if (err != ESP_OK) {
        return err;
    }
    if (blob_size != sizeof(blob)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (blob.magic != FS_NAV_STATE_MAGIC || blob.version != FS_NAV_STATE_VERSION) {
        return ESP_ERR_INVALID_VERSION;
    }

    uint32_t crc = esp_crc32_le(0, (const uint8_t *)&blob, sizeof(blob) - sizeof(blob.crc32));
    if (crc != blob.crc32) {
        return ESP_ERR_INVALID_CRC;
    }

    blob.relative[sizeof(blob.relative) - 1] = '\0';

    if (is_valid_relative(blob.relative)) {
        if (set_relative(nav, blob.relative) != ESP_OK) {
            set_relative(nav, "");
        }
    } else {
        set_relative(nav, "");
        return ESP_ERR_INVALID_ARG;
    }

    if (blob.sort_mode < FS_NAV_SORT_COUNT) {
        nav->sort_mode = blob.sort_mode;
    }
    nav->ascending = blob.ascending != 0;

    struct stat st = {0};
    if (stat(nav->current, &st) != 0 || !S_ISDIR(st.st_mode)) {
        set_relative(nav, "");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static void sort_items(fs_nav_t *nav)
{
    if (!nav || nav->item_count < 2 || !nav->items || !nav->sort_enabled) {
        return;
    }
    s_cmp_mode = nav->sort_mode;
    s_cmp_ascending = nav->ascending;
    qsort(nav->items, nav->item_count, sizeof(fs_nav_item_t), item_compare);
}

static int item_compare(const void *lhs, const void *rhs)
{
    const fs_nav_item_t *a = lhs;
    const fs_nav_item_t *b = rhs;

    if (a->is_dir != b->is_dir) {
        return a->is_dir ? -1 : 1;
    }

    int cmp = 0;
    fs_nav_sort_mode_t mode = a->is_dir ? FS_NAV_SORT_NAME : s_cmp_mode;
    switch (mode) {
        case FS_NAV_SORT_DATE:
            if (a->modified == b->modified) {
                cmp = 0;
            } else {
                cmp = (a->modified < b->modified) ? -1 : 1;
            }
            break;
        case FS_NAV_SORT_SIZE:
            if (a->size_bytes == b->size_bytes) {
                cmp = 0;
            } else {
                cmp = (a->size_bytes < b->size_bytes) ? -1 : 1;
            }
            break;
        case FS_NAV_SORT_NAME:
        default:
            cmp = strcasecmp(a->name, b->name);
            break;
    }

    if (cmp == 0) {
        cmp = strcasecmp(a->name, b->name);
    }
    return s_cmp_ascending ? cmp : -cmp;
}

static esp_err_t init_nav_defaults(fs_nav_t *nav, const fs_nav_config_t *cfg)
{
    if (!nav || !cfg || !cfg->root_path) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(nav, 0, sizeof(*nav));
    nav->max_items = cfg->max_items;
    nav->sort_mode = FS_NAV_SORT_NAME;
    nav->ascending = true;
    nav->sort_enabled = true;
    nav->window_size = 32; /* default; UI may override via fs_nav_set_window */
    return ESP_OK;
}

static esp_err_t sanitize_root(fs_nav_t *nav, const char *root_path)
{
    size_t root_len = strnlen(root_path, FS_NAV_MAX_PATH);
    if (root_len == 0 || root_len >= FS_NAV_MAX_PATH) {
        return ESP_ERR_INVALID_ARG;
    }

    strlcpy(nav->root, root_path, sizeof(nav->root));
    size_t len = strlen(nav->root);
    while (len > 1 && nav->root[len - 1] == '/') {
        nav->root[len - 1] = '\0';
        len--;
    }
    if (nav->root[0] != '/') {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t verify_root_accessible(fs_nav_t *nav)
{
    struct stat st = {0};
    if (stat(nav->current, &st) != 0 || !S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG, "Root path \"%s\" not accessible (errno=%d)", nav->current, errno);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static void apply_loaded_state_or_log(fs_nav_t *nav, esp_err_t state_err)
{
    (void)nav;
    if (state_err != ESP_OK) {
        ESP_LOGW(TAG, "Using default navigator state (%s)", esp_err_to_name(state_err));
    }
}

static esp_err_t perform_initial_refresh(fs_nav_t *nav)
{
    esp_err_t err = fs_nav_refresh(nav);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initial refresh failed (%s)", esp_err_to_name(err));
    }
    return err;
}

static void reset_refresh_state(fs_nav_t *nav)
{
    clear_items(nav);
    nav->total_items = 0;
    nav->window_start = 0;
}

static esp_err_t count_directory_entries(fs_nav_t *nav, size_t *total_out)
{
    DIR *dir = opendir(nav->current);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed: errno=%d", nav->current, errno);
        nav->item_count = 0;
        return ESP_FAIL;
    }

    size_t total = 0;
    struct dirent *dent = NULL;
    errno = 0;
    while ((dent = readdir(dir)) != NULL) {
        if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
            continue;
        }
        total++;
    }
    int count_errno = errno;
    closedir(dir);
    if (count_errno != 0) {
        ESP_LOGE(TAG, "readdir(%s) failed while counting: errno=%d", nav->current, count_errno);
        nav->item_count = 0;
        return ESP_FAIL;
    }

    if (total_out) {
        *total_out = total;
    }
    return ESP_OK;
}

static void ensure_window_size_default(fs_nav_t *nav)
{
    if (nav->window_size == 0) {
        nav->window_size = 32;
    }
}

static esp_err_t allocate_items(fs_nav_t *nav, size_t target)
{
    if (nav->capacity >= target) {
        return ESP_OK;
    }
    fs_nav_item_t *new_items = heap_caps_realloc(nav->items,
                                                 target * sizeof(fs_nav_item_t),
                                                 MALLOC_CAP_8BIT);
    if (!new_items) {
        ESP_LOGE(TAG, "Out of memory while allocating %zu items for \"%s\"", target, nav->current);
        nav->item_count = 0;
        return ESP_ERR_NO_MEM;
    }
    nav->items = new_items;
    nav->capacity = target;
    return ESP_OK;
}

static esp_err_t create_nav_item_from_dirent(const struct dirent *dent, fs_nav_item_t *dest)
{
    memset(dest, 0, sizeof(*dest));

    size_t name_len = strnlen(dent->d_name, FS_NAV_MAX_NAME - 1);
    dest->name = (char *)heap_caps_malloc(name_len + 1, MALLOC_CAP_8BIT);
    if (!dest->name) {
        ESP_LOGE(TAG, "Out of memory duplicating item name");
        return ESP_ERR_NO_MEM;
    }
    memcpy(dest->name, dent->d_name, name_len);
    dest->name[name_len] = '\0';

    dest->needs_stat = true;
    dest->is_dir = (dent->d_type == DT_DIR);
    dest->size_bytes = 0;
    dest->modified = 0;
    return ESP_OK;
}

static esp_err_t load_directory_entry(struct dirent *dent, fs_nav_t *nav, size_t *idx, int *load_errno)
{
    if (strcmp(dent->d_name, ".") == 0 || strcmp(dent->d_name, "..") == 0) {
        return ESP_OK;
    }
    fs_nav_item_t *dest = &nav->items[*idx];
    esp_err_t err = create_nav_item_from_dirent(dent, dest);
    if (err != ESP_OK) {
        *load_errno = ENOMEM;
        return err;
    }
    (*idx)++;
    return ESP_OK;
}

static esp_err_t load_directory_items(fs_nav_t *nav)
{
    DIR *dir = opendir(nav->current);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed on second pass: errno=%d", nav->current, errno);
        nav->item_count = 0;
        return ESP_FAIL;
    }

    size_t idx = 0;
    int load_errno = 0;
    struct dirent *dent = NULL;
    errno = 0;
    while ((dent = readdir(dir)) != NULL) {
        if (load_directory_entry(dent, nav, &idx, &load_errno) != ESP_OK) {
            break;
        }
    }
    load_errno = errno;
    closedir(dir);

    if (load_errno != 0) {
        clear_items(nav);
        ESP_LOGE(TAG, "readdir(%s) failed while loading: errno=%d", nav->current, load_errno);
        nav->item_count = 0;
        return ESP_FAIL;
    }

    nav->item_count = idx;
    nav->window_start = 0;
    sort_items(nav);
    return ESP_OK;
}

static esp_err_t refresh_sorted(fs_nav_t *nav, size_t total)
{
    size_t target = total;
    esp_err_t err = allocate_items(nav, target);
    if (err != ESP_OK) {
        return err;
    }
    return load_directory_items(nav);
}

static const fs_nav_item_t *items_for_sorted(const fs_nav_t *nav, size_t *count)
{
    size_t start = nav->window_start;
    if (start >= nav->item_count) {
        if (count) {
            *count = 0;
        }
        return NULL;
    }
    size_t end = start + nav->window_size;
    if (end > nav->item_count) {
        end = nav->item_count;
    }
    size_t ret = end - start;
    if (count) {
        *count = ret;
    }
    return nav->items + start;
}

static const fs_nav_item_t *items_for_unsorted(const fs_nav_t *nav, size_t *count)
{
    if (count) {
        *count = nav->item_count;
    }
    return nav->items;
}
