idf_component_register(
    SRCS "sd_card.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_bsp_generic 
        esp_common     
        freertos
        lvgl
    PRIV_REQUIRES
        esp_driver_sdspi
        esp_hw_support  
        nvs_flash       
        settings
        styles
        fatfs           
        sdmmc
)
