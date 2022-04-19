# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "wifi_connect.c" "ota_task.c" "gap_gatts.c" "timer_gpio.c" "main.c"
                               
                               
                        INCLUDE_DIRS "."
                        )
