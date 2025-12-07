idf_component_register(
            SRCS "styles.c" 
            INCLUDE_DIRS "include"
            REQUIRES
                lvgl
            PRIV_REQUIRES
                settings
)