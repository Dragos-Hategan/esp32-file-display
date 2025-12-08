#ifndef SNTP_H
#define SNTP_H

#define TZ_EUROPE_BUCHAREST "EET-2EEST,M3.5.0/3,M10.5.0/4"
#define SNTP_DEFAULT_TIMEZONE TZ_EUROPE_BUCHAREST

/**
 * @brief Block until time is synchronized or a timeout occurs.
 *
 * @details
 * Waits for SNTP to complete synchronization. Uses the official
 * esp_netif_sntp_sync_wait() API (ESP-IDF 5.x). If that fails, falls
 * back to a manual check of the system time to validate synchronization.
 *
 * @param timeout_ms Maximum time to wait in milliseconds.
 *
 * @note Logs a warning if synchronization fails within the timeout.
 * 
 * @return 
 *      - ESP_OK on succes
 *      - other error codes from esp_netif_sntp_sync_wait on failure
 */
esp_err_t wait_for_time_blocking(uint32_t timeout_ms);

/**
 * @brief Initialize SNTP and configure the local time zone.
 *
 * @details
 * This function performs the following steps:
 *   - Initializes NVS (required for Wi-Fi).
 *   - Connects to Wi-Fi in station mode.
 *   - Starts SNTP client to synchronize time with NTP servers.
 *   - Configures the timezone for Europe/Bucharest (with automatic DST).
 *   - Blocks until the system time is synchronized or the given timeout expires.
 *
 * @note This function should be called once during system startup,
 *       typically from app_main().
 *
 * @param None
 * 
 * @return        
 *      - ESP_OK on succes
 *      - other error codes from on failure
 */
esp_err_t init_sntp(void);

#endif // SNTP_H