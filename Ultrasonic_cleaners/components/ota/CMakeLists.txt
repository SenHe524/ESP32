# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)
# idf_component_register(SRCS "ota_task.c"
#                         INCLUDE_DIRS "include"
#                         )
set(ota_task_srcs "src/ota_task.c")
idf_component_register(SRCS "${ota_task_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES bluetooth wifi
                        REQUIRES nvs_flash app_update esp_http_client bootloader_support 
                                
                        )