idf_component_register(
        SRCS "wifi.c" "sntp.c"
        INCLUDE_DIRS "include"
        REQUIRES
                esp_event
        PRIV_REQUIRES
                esp_netif
                nvs_flash
                settings
                esp_wifi
                lwip

)
