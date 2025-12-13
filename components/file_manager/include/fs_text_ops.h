#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#define FS_TEXT_READ_CHUNK_SIZE_B   (1 * 1024)
#define FS_TEXT_MAX_PATH    512

/**
 * @brief Check whether a file name/path uses a .txt extension (case-insensitive).
 *
 * @param name File name or path.
 * @return true if the string ends with ".txt", false otherwise.
 */
bool fs_text_is_txt(const char *name);

/**
 * @brief Create a new empty file.
 *
 * Attempts to create a new file at @p path. If the file already exists,
 * returns ESP_ERR_INVALID_STATE. The file is opened in binary write mode
 * and immediately closed to ensure it exists and is empty.
 *
 * @param path Absolute or relative path to the file to create.
 *
 * @retval ESP_OK               File created successfully.
 * @retval ESP_ERR_INVALID_ARG  Invalid or unsafe path.
 * @retval ESP_ERR_INVALID_STATE File already exists.
 * @retval ESP_FAIL             Creation failed (e.g. permission or I/O error).
 *
 * @note The created file will have zero length.
 */
esp_err_t fs_text_create(const char *path);

/**
 * @brief Read a fixed-size block of text bytes from a file starting at a given offset.
 *
 * This function reads a chunk of data (FS_TEXT_READ_CHUNK_SIZE_B bytes) from a file,
 * starting at an offset specified in kilobytes. It validates the file path,
 * ensures the offset does not exceed the file size, adjusts the read length
 * when near EOF, and allocates a null-terminated buffer for the output.
 *
 * The caller takes ownership of the allocated buffer and must free it.
 *
 * @param[in]  path        Absolute file path to read from.
 * @param[in]  offset_kb   Offset in kilobytes from the start of the file.
 *                         The actual byte offset becomes `offset_kb * 1024`.
 * @param[out] out_buf     Pointer to a char* that will receive the allocated buffer.
 *                         Must not be NULL.
 * @param[out] out_len     Optional pointer to receive the number of bytes actually read.
 *                         Can be NULL.
 *
 * @return
 *      - ESP_OK                   On success.
 *      - ESP_ERR_INVALID_ARG      If parameters are invalid or the path fails validation.
 *      - ESP_ERR_INVALID_SIZE     If the requested read size exceeds FS_TEXT_MAX_BYTES.
 *      - ESP_ERR_NO_MEM           If memory allocation fails.
 *      - ESP_FAIL                 If file operations (stat, fopen, fseek, fread) fail.
 *
 * @note
 * - The output buffer is always null-terminated, even when reading binary data.
 * - If the offset lands beyond EOF, a zero-length read is performed.
 * - FS_TEXT_READ_CHUNK_SIZE_B defines the number of bytes to read per call.
 * - The function only works with regular files.
 */
esp_err_t fs_text_read_range(const char *path, size_t offset_kb, char **out_buf, size_t *out_len);

/**
 * @brief Atomically replace (or create) a text file with the provided buffer.
 *
 * Writes the buffer to a temporary file in the same directory and renames it
 * over @p path. When running on case-insensitive filesystems (e.g., FAT),
 * an existing file is removed before the rename to ensure the case-only rename
 * succeeds.
 *
 * @param path Absolute path to the .txt file.
 * @param data Buffer to write (can be NULL if @p len == 0).
 * @param len  Number of bytes from @p data to write.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on bad params, ESP_ERR_INVALID_SIZE if path is too long,
 *         ESP_FAIL on filesystem errors.
 */
esp_err_t fs_text_write(const char *path, const char *data, size_t len);

/**
 * @brief Append binary/text data to an existing file (or create if missing).
 *
 * Opens @p path in append mode and writes exactly @p len bytes from @p data
 * to the end of the file. If the file does not exist, a new one is created.
 *
 * @param path Absolute or relative path to the target file.
 * @param data Pointer to the data buffer to append.
 * @param len  Number of bytes to write.
 *
 * @retval ESP_OK              Data appended successfully.
 * @retval ESP_ERR_INVALID_ARG Invalid path or NULL data pointer.
 * @retval ESP_FAIL            File could not be opened or write failed.
 *
 * @note The file is opened in binary mode ("ab" or "wb"), suitable for both
 *       text and raw data. Data is flushed and file closed before returning.
 * @warning If insufficient storage is available, partial writes may occur
 *          before failure is detected.
 */
esp_err_t fs_text_append(const char *path, const char *data, size_t len);

/**
 * @brief Delete a file from the filesystem.
 *
 * Removes the file located at @p path. If the file does not exist,
 * the function logs an error and returns ESP_FAIL.
 *
 * @param path Absolute or relative path to the file to delete.
 *
 * @retval ESP_OK              File successfully removed.
 * @retval ESP_ERR_INVALID_ARG Invalid or unsafe path.
 * @retval ESP_FAIL            File could not be removed (errno reported).
 *
 * @note Uses standard C `remove()` for portability.
 * @warning Fails silently if the path points to a directory instead of a file.
 */
esp_err_t fs_text_delete(const char *path);

#ifdef __cplusplus
}
#endif
