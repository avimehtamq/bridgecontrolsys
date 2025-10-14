idf_component_register(SRCS "bcs_main.c"
                    PRIV_REQUIRES esp_wifi esp_http_server driver esp_rom ultrasonic
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES index.html
                    )
