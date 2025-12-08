idf_component_register(
        SRCS "wifi.c" "sntp.c"
        INCLUDE_DIRS "include"
        REQUIRES
                nvs_flash
                esp_event
                esp_netif
                esp_wifi
                lwip
)
