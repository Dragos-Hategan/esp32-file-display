#include "sntp_header.h"

#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_log.h"

#include "wifi.h"

#define SNTP_RETRY_NUMBER 5
#define SNTP_MIN_VALID_YEAR (2016)
#define SNTP_POLL_STEP_MS   (1000)

static const char *TAG = "sntp";

/**
 * @brief Start the SNTP client with a static NTP server.
 *
 * Initializes and starts the SNTP service using esp_netif with a fixed
 * NTP server ("pool.ntp.org"). Alternative configuration options include:
 *   - Accepting NTP servers from DHCP.
 *   - Renewing the server list after a new DHCP lease.
 *   - Controlling whether SNTP auto-starts.
 *
 * @note This function should be called once after network initialization
 *       (after Wi-Fi or Ethernet is up).
 *
 * @return ESP_OK on success or an error from esp_netif_sntp_init.
 */
static esp_err_t start_sntp(void);

/**
 * @brief Validate that the current time is beyond a minimum year threshold.
 *
 * @return true if the current time looks valid, false otherwise.
 */
static bool is_time_valid(void);

esp_err_t sntp_wait_for_time_blocking(uint32_t timeout_ms)
{
    const TickType_t max_wait_ticks = pdMS_TO_TICKS(timeout_ms);
    const TickType_t poll_ticks = pdMS_TO_TICKS(SNTP_POLL_STEP_MS);
    const TickType_t start = xTaskGetTickCount();

    while (true) {
        esp_err_t err = esp_netif_sntp_sync_wait(poll_ticks);
        if (err == ESP_OK && is_time_valid()) {
            ESP_LOGI(TAG, "Time synced");
            return ESP_OK;
        }

        if ((xTaskGetTickCount() - start) >= max_wait_ticks) {
            break;
        }
    }

    /* Final fallback check in case sync completed just after timeout. */
    if (is_time_valid()) {
        ESP_LOGI(TAG, "Time looks valid (post-timeout check).");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Time not synced (timeout after %u ms).", timeout_ms);
    return ESP_ERR_TIMEOUT;
}

esp_err_t sntp_init(void)
{
    esp_err_t err = start_sntp();
    if (err != ESP_OK){
        ESP_LOGI(TAG, "start_sntp failed: (%s)", esp_err_to_name(err));
        return err;
    }

    setenv("TZ", SNTP_DEFAULT_TIMEZONE, 1);
    tzset();

    err = sntp_wait_for_time_blocking(10000);
    if (err != ESP_OK){
        ESP_LOGI(TAG, "sntp_wait_for_time_blocking failed: (%s)", esp_err_to_name(err));
    }   

    return err;
}

static esp_err_t start_sntp(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    
    esp_err_t err = esp_netif_sntp_init(&config);
    if (err == ESP_OK){
        ESP_LOGI(TAG, "SNTP started via esp_netif");
    }else{
        ESP_LOGE(TAG, "SNTP init failed: (%s)", esp_err_to_name(err));
    }
    return err;
}

static bool is_time_valid(void)
{
    time_t now = 0;
    struct tm ti = {0};
    time(&now);
    localtime_r(&now, &ti);
    return (ti.tm_year > (SNTP_MIN_VALID_YEAR - 1900));
}
