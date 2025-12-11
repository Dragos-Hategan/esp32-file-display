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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "settings.h"

#define MAX_WIFI_RETRIES 5

static const char *TAG = "wifi_init";

static uint8_t connection_retry_counter = 0;
static SemaphoreHandle_t s_ip_ready;
static bool s_creds_valid = true;

/**
 * @brief Initialize NVS, erasing if pages are full or version mismatched.
 *
 * @return ESP_OK on success, otherwise error from NVS APIs.
 */
static esp_err_t init_nvs(void);

/**
 * @brief Initialize esp_netif library.
 *
 * @return ESP_OK on success, otherwise esp_netif_init error.
 */
static esp_err_t init_network(void);

/**
 * @brief Create default event loop and default STA netif with handlers.
 *
 * @return ESP_OK on success, otherwise error from event loop/netif setup.
 */
static esp_err_t init_event_loop(void);

/**
 * @brief Initialize the Wi-Fi driver with default config.
 *
 * @return ESP_OK on success, otherwise error from esp_wifi_init.
 */
static esp_err_t init_wifi_driver(void);

/**
 * @brief Fill STA configuration from stored settings and select auth mode.
 *
 * @param[out] wifi_config Wi-Fi STA config structure to populate.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if SSID missing, or other errors.
 */
static esp_err_t config_sta_params(wifi_config_t *wifi_config);

/**
 * @brief Register Wi-Fi and IP event handlers for the STA flow.
 *
 * @return ESP_OK on success, otherwise error from esp_event_handler_register.
 */
static esp_err_t register_event_handlers(void);

/**
 * @brief Apply STA config and set mode to the Wi-Fi driver.
 *
 * @param[in] wifi_config Prepared STA configuration.
 * @return ESP_OK on success, otherwise errors from esp_wifi_* calls.
 */
static esp_err_t set_wifi(wifi_config_t *wifi_config);

/**
 * @brief Start Wi-Fi driver.
 *
 * @return ESP_OK on success, otherwise errors from esp_wifi_start call.
 */
static esp_err_t start_wifi(void);

/**
 * @brief Block until IP is obtained or retry limit is exceeded.
 *
 * @param[in] s_ip_ready Semaphore signaled by event handler.
 * @return ESP_OK on success, ESP_ERR_WIFI_TIMEOUT on retry exhaustion, or ESP_FAIL on semaphore error.
 */
static esp_err_t wait_for_ip(SemaphoreHandle_t s_ip_ready);

/**
 * @brief Dispatch Wi-Fi/IP events to specialized handlers.
 *
 * Routes events coming from the ESP event loop to Wi-Fi or IP-specific
 * handlers.
 *
 * @param[in] arg        Unused user argument.
 * @param[in] event_base WIFI_EVENT or IP_EVENT.
 * @param[in] event_id   Specific event ID inside the base.
 * @param[in] event_data Event-specific payload (e.g., disconnect reason).
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/**
 * @brief Handle Wi-Fi station events (start/disconnect).
 *
 * Attempts connection on start and manages retry/fail logic on disconnect.
 *
 * @param[in] event_id   WIFI_EVENT_* identifier.
 * @param[in] event_data Event-specific payload (e.g., disconnect reason).
 */
static void handle_wifi_events(int32_t event_id, void *event_data);

/**
 * @brief Handle IP events for the STA interface.
 *
 * Currently processes successful IP acquisition events.
 *
 * @param[in] event_id IP_EVENT_* identifier.
 */
static void handle_ip_events(int32_t event_id);

/**
 * @brief Handle STA disconnections by logging and retrying until limit.
 *
 * Increments the retry counter, logs the disconnect reason when available,
 * and triggers reconnection or signals failure once the retry cap is exceeded.
 *
 * @param[in] event_data Disconnect event data provided by the Wi-Fi stack.
 */
static void handle_wifi_disconnect(wifi_event_sta_disconnected_t *event_data);

esp_err_t wifi_init_sta(void)
{
    esp_err_t err = init_nvs();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "init_nvs failed: (%s)", esp_err_to_name(err));
        return err;
    }    

    err = init_network();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "init_network failed: (%s)", esp_err_to_name(err));
        return err;
    }   

    err = init_event_loop();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "init_event_loop failed: (%s)", esp_err_to_name(err));
        return err;
    }    

    err = init_wifi_driver();    
    if(err != ESP_OK){
        ESP_LOGE(TAG, "init_wifi_driver failed: (%s)", esp_err_to_name(err));
        return err;
    } 

    wifi_config_t wifi_config = {0};
    err = config_sta_params(&wifi_config);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "config_sta_params failed: (%s)", esp_err_to_name(err));
        return err;
    } 

    err = register_event_handlers();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "register_event_handlers failed: (%s)", esp_err_to_name(err));
        return err;
    }    

    s_ip_ready = xSemaphoreCreateBinary();
    if (!s_ip_ready){
        ESP_LOGE(TAG, "xSemaphoreCreateBinary failed");
        return ESP_FAIL;
    }    

    err = set_wifi(&wifi_config);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "set_wifi failed: (%s)", esp_err_to_name(err));
        return err;
    } 
    
    err = start_wifi();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "start_wifi failed: (%s)", esp_err_to_name(err));
        return err;
    }       

    return wait_for_ip(s_ip_ready);
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND || err == ESP_ERR_NVS_NOT_INITIALIZED) {
        err = nvs_flash_erase();
        if (err != ESP_OK){
            ESP_LOGE(TAG, "nvs_flash_erase failed: (%s)", esp_err_to_name(err));
            return err;
        }

        err = nvs_flash_init();
        if (err != ESP_OK){
            ESP_LOGE(TAG, "nvs_flash_init failed: (%s)", esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t init_network(void)
{
    esp_err_t err = esp_netif_init();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_netif_init failed: (%s)", esp_err_to_name(err));
        return err;
    }
    return err;
}

static esp_err_t init_event_loop(void)
{
    esp_err_t err =  esp_event_loop_create_default();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: (%s)", esp_err_to_name(err));
        return err;
    }
    
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    esp_netif_t *netif = NULL;

    netif = esp_netif_new(&cfg);
    if (!netif){
        ESP_LOGE(TAG, "esp_netif_new failed: (%s)", esp_err_to_name(ESP_FAIL));
        return ESP_FAIL;
    }
    err = esp_netif_attach_wifi_station(netif);
    if (err != ESP_OK) {
        esp_netif_destroy(netif);
        ESP_LOGE(TAG, "esp_netif_attach_wifi_station failed: (%s)", esp_err_to_name(ESP_FAIL));
        return ESP_FAIL;
    }
    err = esp_wifi_set_default_wifi_sta_handlers();
    if (err != ESP_OK) {
        esp_netif_destroy(netif);
        ESP_LOGE(TAG, "esp_wifi_set_default_wifi_sta_handlers failed: (%s)", esp_err_to_name(ESP_FAIL));
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t init_wifi_driver(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_init failed: (%s)", esp_err_to_name(err));
        return err;
    }   

    return ESP_OK;
}

static esp_err_t config_sta_params(wifi_config_t *wifi_config)
{
    const char* ap_ssid = settings_get_ap_ssid();
    const char* ap_pwd = settings_get_ap_pwd();
    size_t ssid_len = ap_ssid ? strlen(ap_ssid) : 0;
    size_t pwd_len = ap_pwd ? strlen(ap_pwd) : 0;
    if (ssid_len == 0) {
        ESP_LOGE(TAG, "SSID missing. Please set Wi-Fi credentials in Settings.");
        return ESP_ERR_INVALID_ARG;
    }

    strlcpy((char *)wifi_config->sta.ssid, ap_ssid, sizeof(wifi_config->sta.ssid));
    strlcpy((char *)wifi_config->sta.password, ap_pwd, sizeof(wifi_config->sta.password));
    wifi_config->sta.threshold.authmode = (pwd_len == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_config->sta.pmf_cfg.capable = true;   /**< Protected Management Frames supported */
    wifi_config->sta.pmf_cfg.required = false; /**< PMF not mandatory */
    s_creds_valid = true;

    return ESP_OK;
}

static esp_err_t register_event_handlers(void)
{
    esp_err_t err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_event_handler_register(WIFI_EVENT) failed: (%s)", esp_err_to_name(err));
        return err;
    }       
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_event_handler_register(IP_EVENT) failed: (%s)", esp_err_to_name(err));
        return err;
    }   

    return ESP_OK;
}

static esp_err_t set_wifi(wifi_config_t *wifi_config)
{
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: (%s)", esp_err_to_name(err));
        return err;
    }       
    err = esp_wifi_set_config(WIFI_IF_STA, wifi_config);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_set_config failed: (%s)", esp_err_to_name(err));
        return err;
    }       

    return ESP_OK;
}

static esp_err_t start_wifi(void)
{
    esp_err_t err = esp_wifi_start();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "esp_wifi_start failed: (%s)", esp_err_to_name(err));
        return err;
    }  

    return ESP_OK;
}

static esp_err_t wait_for_ip(SemaphoreHandle_t s_ip_ready)
{
    ESP_LOGI(TAG, "Connecting to WiFi...");
    if (xSemaphoreTake(s_ip_ready, portMAX_DELAY) != pdTRUE){
        ESP_LOGE(TAG, "xSemaphoreTake failed: (%s)", esp_err_to_name(ESP_FAIL));
        return ESP_FAIL;
    }

    if (connection_retry_counter < MAX_WIFI_RETRIES){
        ESP_LOGI(TAG, "WiFi connected, proceeding...");
    }

    if (connection_retry_counter > MAX_WIFI_RETRIES){
        ESP_LOGE(TAG, "Too many retries, Wi-Fi failed.");
        return ESP_ERR_WIFI_TIMEOUT;
    }

    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        handle_wifi_events(event_id, event_data);

    } else if (event_base == IP_EVENT) {
        handle_ip_events(event_id);
    }
}

static void handle_wifi_events(int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START) {
        if (s_creds_valid) {
            esp_wifi_connect();
        } else if (s_ip_ready) {
            /* Unblock waiter immediately when creds are invalid. */
            xSemaphoreGive(s_ip_ready);
        }
    
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        handle_wifi_disconnect(event_data);
    }
}

static void handle_wifi_disconnect(wifi_event_sta_disconnected_t *event_data)
{
    connection_retry_counter++;
    if (connection_retry_counter > MAX_WIFI_RETRIES){
        ESP_LOGE(TAG, "Too many retries, Wi-Fi failed.");
        xSemaphoreGive(s_ip_ready);
        return;
    }   

    const wifi_event_sta_disconnected_t *dis = (const wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "Disconnected (reason=%d). Retrying...", dis ? dis->reason : -1);
    if (dis) {
        switch (dis->reason) {
            case WIFI_REASON_AUTH_FAIL:
            case WIFI_REASON_ASSOC_FAIL:
            case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
                ESP_LOGW(TAG, "Authentication failed. Please verify SSID/password.");
                break;
            case WIFI_REASON_NO_AP_FOUND:
                ESP_LOGW(TAG, "AP not found. Check SSID or signal.");
                break;
            default:
                break;
        }
    }
    esp_wifi_connect();
}

static void handle_ip_events(int32_t event_id)
{
    if (event_id == IP_EVENT_STA_GOT_IP){
        ESP_LOGI(TAG, "Got IP!");
        if (s_ip_ready) {
            xSemaphoreGive(s_ip_ready);
        }
    }
}
