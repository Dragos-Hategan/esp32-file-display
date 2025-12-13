#include "fs_text_ops.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "esp_log.h"

static const char *TAG = "fs_text";

/**
 * @brief Write data atomically by using a temporary file and rename().
 *
 * The function writes @p data to a temporary file in the same directory as
 * @p path, flushes and closes it, then renames the temporary file over the
 * destination. This minimizes the risk of ending up with a partially written
 * file if a failure occurs during the write.
 *
 * Steps:
 *  1) Derive the directory of @p path (or "." if none).
 *  2) Build "<dir>/tmpwrt.tmp" and remove any stale temp file.
 *  3) Write @p len bytes to the temp file and fflush()/fclose().
 *  4) rename(temp, path). If EEXIST, attempt remove(path) then rename again.
 *
 * @param path Destination file path to replace atomically.
 * @param data Buffer with data to write.
 * @param len  Number of bytes in @p data.
 *
 * @retval ESP_OK               On success (destination is replaced).
 * @retval ESP_ERR_INVALID_SIZE Directory or temp-path would overflow buffers.
 * @retval ESP_FAIL             fopen/fwrite/rename or cleanup failed.
 *
 * @note Uses a fixed temp name "tmpwrt.tmp" in the target directory to keep
 *       the rename on the same filesystem. This avoids cross-FS renames.
 * @warning Atomicity semantics depend on the underlying VFS/filesystem. On
 *          POSIX systems, rename() within the same directory is atomic; on
 *          embedded filesystems (e.g., FAT/SD, SPIFFS, LittleFS) behavior
 *          may vary, especially across power loss.
 */
/**
 * @brief Write data atomically by using a temporary file and rename().
 */
static esp_err_t write_atomic(const char *path, const char *data, size_t len);

/**
 * @brief Validate a candidate text-file path against module constraints.
 *
 * Checks that @p path is non-NULL, passes @ref fs_text_is_txt (e.g., extension),
 * and its length is within @c FS_TEXT_MAX_PATH.
 *
 * @param path Path string to validate.
 * @return true  if the path is acceptable for text operations.
 * @return false if the path is NULL, not a .txt (per policy), or too long.
 */
static bool check_path(const char *path);

/**
 * @brief Derive directory portion from a full path.
 *
 * Copies the directory component of @p path into @p dir, ensuring space and null termination.
 *
 * @param path     Destination path.
 * @param dir      Output buffer for directory.
 * @param dir_size Size of @p dir buffer.
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE on overflow.
 */
static esp_err_t get_dir_from_path(const char *path, char *dir, size_t dir_size);

/**
 * @brief Build temporary file path in the same directory and remove stale file.
 *
 * @param dir      Directory path.
 * @param tmp_path Output buffer for "<dir>/tmpwrt.tmp".
 * @param tmp_size Size of @p tmp_path buffer.
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if truncated.
 */
static esp_err_t build_tmp_path(const char *dir, char *tmp_path, size_t tmp_size);

/**
 * @brief Write data to a temporary file path.
 *
 * @param tmp_path Temporary file path.
 * @param data     Data buffer.
 * @param len      Length in bytes.
 * @return ESP_OK on success; ESP_FAIL on I/O errors.
 */
static esp_err_t write_temp_file(const char *tmp_path, const char *data, size_t len);

/**
 * @brief Finalize atomic write by renaming temp file over destination (with overwrite support).
 *
 * @param tmp_path Temporary file to promote.
 * @param dest_path Destination path to overwrite.
 * @return ESP_OK on success; ESP_FAIL on rename/remove failures.
 */
static esp_err_t finalize_atomic_write(const char *tmp_path, const char *dest_path);

/**
 * @brief Read a chunk into a malloc'd buffer and null-terminate it.
 *
 * @param f        Open FILE* positioned at the desired offset.
 * @param to_read  Bytes to read.
 * @param[out] out_buf Allocated buffer (caller frees).
 * @param[out] out_len Bytes actually read (optional).
 * @return ESP_OK on success; ESP_ERR_NO_MEM on alloc failure; ESP_FAIL on fread error.
 */
static esp_err_t read_chunk(FILE *f, size_t to_read, char **out_buf, size_t *out_len);

/**
 * @brief Compute file read offset and byte count for a chunked read.
 *
 * Validates file existence, clamps offset to EOF, applies FS_TEXT_READ_CHUNK_SIZE_B
 * and optional FS_TEXT_MAX_BYTES limit.
 *
 * @param path          File path.
 * @param offset_kb     Requested offset in KB.
 * @param[out] offset_bytes Effective byte offset (clamped).
 * @param[out] to_read      Bytes to read.
 * @return ESP_OK on success; error on invalid args/stat failures/size limits.
 */
static esp_err_t compute_read_params(const char *path, size_t offset_kb, size_t *offset_bytes, size_t *to_read);

/**
 * @brief stat() wrapper that validates the path is an existing regular file.
 *
 * @param path Path to inspect.
 * @param[out] st Populated stat buffer on success.
 * @return ESP_OK if the file exists and is regular; ESP_FAIL otherwise.
 */
static esp_err_t stat_regular_file(const char *path, struct stat *st);

/**
 * @brief Clamp KB offset to file size and return byte offset.
 *
 * @param st        Stat info for the file.
 * @param offset_kb Requested offset in kilobytes.
 * @param[out] offset_b Clamped byte offset.
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG on overflow.
 */
static esp_err_t clamp_read_offset(const struct stat *st, size_t offset_kb, size_t *offset_b);

/**
 * @brief Compute number of bytes to read for a chunk, respecting limits.
 *
 * @param file_size   Total file size in bytes.
 * @param offset_b    Starting byte offset.
 * @param[out] bytes_to_read Computed byte count to read.
 * @return ESP_OK on success; ESP_ERR_INVALID_SIZE if exceeding FS_TEXT_MAX_BYTES.
 */
static esp_err_t compute_bytes_to_read(size_t file_size, size_t offset_b, size_t *bytes_to_read);

/**
 * @brief Open a file for reading and seek to the given byte offset.
 *
 * @param path         File path.
 * @param offset_bytes Byte offset to seek to.
 * @param[out] out_file Opened FILE* (caller closes on success).
 * @return ESP_OK on success, ESP_FAIL on fopen/fseek errors.
 */
static esp_err_t open_and_seek(const char *path, size_t offset_bytes, FILE **out_file);

bool fs_text_is_txt(const char *name)
{
    if (!name) {
        return false;
    }
    const char *dot = strrchr(name, '.');
    return dot && strcasecmp(dot, ".txt") == 0;
}

esp_err_t fs_text_create(const char *path)
{
    if (!check_path(path)) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    if (stat(path, &st) == 0) {
        return ESP_ERR_INVALID_STATE; // Already exists
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "create fopen(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    fclose(f);
    return ESP_OK;
}

esp_err_t fs_text_read_range(const char *path, size_t offset_kb, char **out_buf, size_t *out_len)
{
    if (!out_buf || !check_path(path)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t offset_bytes = 0;
    size_t to_read = 0;
    esp_err_t err = compute_read_params(path, offset_kb, &offset_bytes, &to_read);
    if (err != ESP_OK) {
        return err;
    }

    FILE *f = NULL;
    err = open_and_seek(path, offset_bytes, &f);
    if (err != ESP_OK) {
        return err;
    }

    err = read_chunk(f, to_read, out_buf, out_len);
    fclose(f);
    return err;
}

esp_err_t fs_text_write(const char *path, const char *data, size_t len)
{
    if (!check_path(path) || (!data && len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!data) {
        len = 0;
    }
    return write_atomic(path, data, len);
}

esp_err_t fs_text_append(const char *path, const char *data, size_t len)
{
    if (!check_path(path) || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(path, "ab");
    if (!f) {
        /* Try to create the file if it doesn't exist */
        f = fopen(path, "wb");
        if (!f) {
            ESP_LOGE(TAG, "fopen(%s) failed (errno=%d)", path, errno);
            return ESP_FAIL;
        }
    }

    size_t written = fwrite(data, 1, len, f);
    if (written != len) {
        ESP_LOGE(TAG, "append fwrite(%s) failed (errno=%d)", path, errno);
        fclose(f);
        return ESP_FAIL;
    }
    fflush(f);
    fclose(f);
    return ESP_OK;
}

esp_err_t fs_text_delete(const char *path)
{
    if (!check_path(path)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (remove(path) != 0) {
        ESP_LOGE(TAG, "remove(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t compute_read_params(const char *path, size_t offset_kb, size_t *offset_bytes, size_t *to_read)
{
    struct stat st = {0};
    esp_err_t err = stat_regular_file(path, &st);
    if (err != ESP_OK) {
        return err;
    }

    size_t offset_b = 0;
    err = clamp_read_offset(&st, offset_kb, &offset_b);
    if (err != ESP_OK) {
        return err;
    }

    size_t bytes_to_read = 0;
    err = compute_bytes_to_read((size_t)st.st_size, offset_b, &bytes_to_read);
    if (err != ESP_OK) {
        return err;
    }

    *offset_bytes = offset_b;
    *to_read = bytes_to_read;
    return ESP_OK;
}

static esp_err_t open_and_seek(const char *path, size_t offset_bytes, FILE **out_file)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "fopen(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }

    if (fseek(f, (long)offset_bytes, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "fseek(%s, %zu) failed (errno=%d)", path, offset_bytes, errno);
        fclose(f);
        return ESP_FAIL;
    }

    *out_file = f;
    return ESP_OK;
}

static esp_err_t stat_regular_file(const char *path, struct stat *st)
{
    if (stat(path, st) != 0 || !S_ISREG(st->st_mode)) {
        ESP_LOGE(TAG, "stat(%s) failed (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t clamp_read_offset(const struct stat *st, size_t offset_kb, size_t *offset_b)
{
    if (offset_kb > SIZE_MAX / 1024) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t candidate = offset_kb * 1024u;
    size_t file_size = (size_t)st->st_size;
    size_t file_size_kb = file_size / 1024;
    if (candidate >= file_size) {
        candidate = file_size_kb * 1024;
    }
    *offset_b = candidate;
    return ESP_OK;
}

static esp_err_t compute_bytes_to_read(size_t file_size, size_t offset_b, size_t *bytes_to_read)
{
    size_t max_available = file_size - offset_b;
    size_t chunk = FS_TEXT_READ_CHUNK_SIZE_B;
    if (chunk > max_available) {
        chunk = max_available;
    }

#ifdef FS_TEXT_MAX_BYTES
    if (chunk > FS_TEXT_MAX_BYTES) {
        ESP_LOGE(TAG, "Requested range too large (%zu bytes)", chunk);
        return ESP_ERR_INVALID_SIZE;
    }
#endif

    *bytes_to_read = chunk;
    return ESP_OK;
}

static esp_err_t read_chunk(FILE *f, size_t to_read, char **out_buf, size_t *out_len)
{
    char *buf = (char *)malloc(to_read + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    size_t read = fread(buf, 1, to_read, f);
    if (read == 0 && ferror(f)) {
        ESP_LOGE(TAG, "fread failed (errno=%d)", errno);
        free(buf);
        return ESP_FAIL;
    }
    buf[read] = '\0';

    *out_buf = buf;
    if (out_len) {
        *out_len = read;
    }
    return ESP_OK;
}

static esp_err_t get_dir_from_path(const char *path, char *dir, size_t dir_size)
{
    const char *slash = strrchr(path, '/');
    if (slash) {
        size_t dir_len = (size_t)(slash - path);
        if (dir_len == 0) {
            if (dir_size < 2) {
                return ESP_ERR_INVALID_SIZE;
            }
            dir[0] = '/';
            dir[1] = '\0';
        } else if (dir_len < dir_size) {
            memcpy(dir, path, dir_len);
            dir[dir_len] = '\0';
        } else {
            return ESP_ERR_INVALID_SIZE;
        }
    } else {
        strlcpy(dir, ".", dir_size);
    }
    return ESP_OK;
}

static esp_err_t build_tmp_path(const char *dir, char *tmp_path, size_t tmp_size)
{
    int needed = snprintf(tmp_path, tmp_size, "%s/tmpwrt.tmp", dir);
    if (needed < 0 || needed >= (int)tmp_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    remove(tmp_path);
    return ESP_OK;
}

static esp_err_t write_temp_file(const char *tmp_path, const char *data, size_t len)
{
    FILE *f = fopen(tmp_path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "fopen(%s) failed (errno=%d)", tmp_path, errno);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, len, f);
    if (written != len) {
        ESP_LOGE(TAG, "fwrite(%s) failed (errno=%d)", tmp_path, errno);
        fclose(f);
        remove(tmp_path);
        return ESP_FAIL;
    }
    fflush(f);
    fclose(f);
    return ESP_OK;
}

static esp_err_t finalize_atomic_write(const char *tmp_path, const char *dest_path)
{
    if (rename(tmp_path, dest_path) == 0) {
        return ESP_OK;
    }
    if (errno == EEXIST) {
        if (remove(dest_path) == 0 && rename(tmp_path, dest_path) == 0) {
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "rename(%s -> %s) failed (errno=%d)", tmp_path, dest_path, errno);
    remove(tmp_path);
    return ESP_FAIL;
}

static esp_err_t write_atomic(const char *path, const char *data, size_t len)
{
    char dir[FS_TEXT_MAX_PATH];
    esp_err_t err = get_dir_from_path(path, dir, sizeof(dir));
    if (err != ESP_OK) {
        return err;
    }

    char tmp_path[FS_TEXT_MAX_PATH];
    err = build_tmp_path(dir, tmp_path, sizeof(tmp_path));
    if (err != ESP_OK) {
        return err;
    }

    err = write_temp_file(tmp_path, data, len);
    if (err != ESP_OK) {
        return err;
    }

    return finalize_atomic_write(tmp_path, path);
}

static bool check_path(const char *path)
{
    if (!path || !fs_text_is_txt(path)) {
        return false;
    }
    size_t len = strnlen(path, FS_TEXT_MAX_PATH + 1);
    return len > 0 && len < FS_TEXT_MAX_PATH;
}
