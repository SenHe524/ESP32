# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)
# idf_component_register(SRCS "wifi_connect.c"
#                         INCLUDE_DIRS "include"
#                         )
set(wifi_connect_srcs "src/wifi_connect.c")
idf_component_register(SRCS "${wifi_connect_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES bluetooth
                        REQUIRES nvs_flash
                        )