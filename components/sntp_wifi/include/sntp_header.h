#ifndef SNTP_H
#define SNTP_H

#include "esp_err.h"

#define TZ_EUROPE_BUCHAREST "EET-2EEST,M3.5.0/3,M10.5.0/4"
#define SNTP_DEFAULT_TIMEZONE TZ_EUROPE_BUCHAREST

/**
 * @brief Block until time is synced or a timeout elapses.
 *
 * Polls SNTP sync status in 1s steps up to @p timeout_ms and validates the
 * resulting time against a minimum year. Returns ESP_ERR_TIMEOUT on failure.
 *
 * @param timeout_ms Maximum wait in milliseconds.
 * @return ESP_OK if time is valid, ESP_ERR_TIMEOUT otherwise.
 */
esp_err_t sntp_wait_for_time_blocking(uint32_t timeout_ms);

/**
 * @brief Initialize SNTP, set timezone, and wait for synchronization.
 *
 * Starts SNTP, applies the default timezone from @c SNTP_DEFAULT_TIMEZONE,
 * and waits up to 10 seconds for time to become valid.
 *
 * @return ESP_OK on success or an esp_err_t from SNTP init/sync.
 */
esp_err_t sntp_init(void);

#endif // SNTP_H