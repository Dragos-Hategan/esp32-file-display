idf_component_register(SRCS "main.c"
                    REQUIRES 
                        file_manager
                        sntp_wifi
                        settings
                        sd_card
                    )
