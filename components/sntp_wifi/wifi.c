/**
 * @file wifi.c
 * @brief Wi-Fi initialization and connection handling for ESP-IDF.
 *
 * @details
 * This module configures the ESP32 Wi-Fi subsystem in station mode (STA),
 * connects to the configured SSID, and blocks execution until a valid
 * IP address is obtained. It also provides automatic reconnection if
 * the connection is lost.
 */

#include "wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "nvs_flash.h"

#define WIFI_RETRY_NUMBER 10

static const char *TAG = "WIFI_INIT";
static SemaphoreHandle_t s_ip_ready;
static uint8_t connection_retry_counter = -1;

/**
 * @brief General Wi-Fi and IP event handler.
 *
 * Handles the following cases:
 * - `WIFI_EVENT_STA_START`: Initiates a connection attempt.
 * - `WIFI_EVENT_STA_DISCONNECTED`: Logs a warning and retries connection.
 * - `IP_EVENT_STA_GOT_IP`: Signals that an IP address has been acquired.
 *
 * @param[in] arg        Unused user argument.
 * @param[in] event_base The event base type (Wi-Fi or IP).
 * @param[in] event_id   Specific event ID within the base.
 * @param[in] event_data Pointer to event-specific data (unused here).
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    connection_retry_counter++;
    if (connection_retry_counter >= WIFI_RETRY_NUMBER){
        ESP_LOGE(TAG, "Too many retries, Wi-Fi failed.");
        xSemaphoreGive(s_ip_ready);
        return;
    }

    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "Disconnected. Retrying...");
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP!");
        if (s_ip_ready) {
            xSemaphoreGive(s_ip_ready);
        }
    }
}

esp_err_t wifi_init_sta(void)
{
    /* 1) Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND || err == ESP_ERR_NVS_NOT_INITIALIZED) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init());
        if (err != ESP_OK){
            ESP_LOGE(TAG, "nvs_flash_init failed: (%s)", esp_err_to_name(err));
            return err;
        }
    }

    /* 2) Network stack + default event loop */
    err = esp_netif_init();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_netif_init failed: (%s)", esp_err_to_name(err));
        return err;
    }
    err = esp_event_loop_create_default();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: (%s)", esp_err_to_name(err));
        return err;
    }
    esp_netif_create_default_wifi_sta();   

    /* 3) Initialize Wi-Fi driver */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_init failed: (%s)", esp_err_to_name(err));
        return err;
    }       

    /* 4) Configure STA parameters */
    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;   /**< Protected Management Frames supported */
    wifi_config.sta.pmf_cfg.required = false; /**< PMF not mandatory */

    /* 5) Create semaphore + register event handlers */
    s_ip_ready = xSemaphoreCreateBinary();
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_event_handler_register(WIFI_EVENT) failed: (%s)", esp_err_to_name(err));
        return err;
    }       
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_event_handler_register(IP_EVENT) failed: (%s)", esp_err_to_name(err));
        return err;
    }   

    /* 6) Start Wi-Fi + set config */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: (%s)", esp_err_to_name(err));
        return err;
    }       
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_set_config failed: (%s)", esp_err_to_name(err));
        return err;
    }       
    err = esp_wifi_start();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_start failed: (%s)", esp_err_to_name(err));
        return err;
    }       

    /* 7) Block until IP is acquired or too many retries happen */
    ESP_LOGI(TAG, "Connecting to WiFi...");
    if (xSemaphoreTake(s_ip_ready, portMAX_DELAY) != pdTRUE){
        ESP_LOGE(TAG, "xSemaphoreTake failed: (%s)", esp_err_to_name(ESP_FAIL));
        return ESP_FAIL;
    }

    if (connection_retry_counter < WIFI_RETRY_NUMBER){
        ESP_LOGI(TAG, "WiFi connected, proceeding...");
        return ESP_OK;
    }

    return ESP_FAIL;
}
