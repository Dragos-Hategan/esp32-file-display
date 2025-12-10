#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"

#include "sntp_header.h"
#include "wifi.h"

#define SNTP_RETRY_NUMBER 5

static const char *TAG = "sntp";

/**
 * @brief Start the SNTP client with a static NTP server.
 *
 * @details
 * Initializes and starts the SNTP service using esp_netif with a fixed
 * NTP server ("pool.ntp.org"). Alternative configuration options include:
 *   - Accepting NTP servers from DHCP.
 *   - Renewing the server list after a new DHCP lease.
 *   - Controlling whether SNTP auto-starts.
 *
 * @note This function should be called once after network initialization
 *       (after Wi-Fi or Ethernet is up).
 *
 * @return 
 *      - ESP_OK on succes
 */
static esp_err_t sntp_start(void);

esp_err_t wait_for_time_blocking(uint32_t timeout_ms)
{
    esp_err_t err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(timeout_ms));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Time synced");
        return err;
    }

    // Very simple fallback check
    for (int retry = 0; retry < SNTP_RETRY_NUMBER; ++retry) {
        time_t now = 0;
        struct tm ti = {0};
        time(&now);
        localtime_r(&now, &ti);
        if (ti.tm_year > (2016 - 1900)) {
            ESP_LOGI(TAG, "Time looks valid (fallback).");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGW(TAG, "Time not synced (timeout).");
    return ESP_FAIL;
}


esp_err_t init_sntp(void)
{
    esp_err_t err = sntp_start();
    if (err != ESP_OK){
        ESP_LOGI(TAG, "sntp_start failed: (%s)", esp_err_to_name(err));
        return err;
    }

    setenv("TZ", SNTP_DEFAULT_TIMEZONE, 1);
    tzset();

    err = wait_for_time_blocking(10000);
    if (err != ESP_OK){
        ESP_LOGI(TAG, "wait_for_time_blocking failed: (%s)", esp_err_to_name(err));
    }   

    return err;
}

static esp_err_t sntp_start(void)
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