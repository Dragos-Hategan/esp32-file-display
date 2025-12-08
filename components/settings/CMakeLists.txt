idf_component_register(
    SRCS "settings.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_common  
        lvgl
    PRIV_REQUIRES
        esp_bsp_generic
        touch_xpt2046
        sntp_wifi
        nvs_flash
        esp_timer
        sd_card
        styles
        fonts
)
