idf_component_register(
    SRCS "file_manager.c" "text_viewer_screen.c" "fs_navigator.c" "fs_text_ops.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_common     
    PRIV_REQUIRES
        esp_driver_sdspi
        esp_bsp_generic
        esp_hw_support
        image_viewer
        nvs_flash       
        esp_timer  
        settings
        sd_card
        styles
        fonts
        fatfs           
        sdmmc         
        lvgl  
)