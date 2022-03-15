# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "timer_gpio.c" "wifi_connect.c" "main.c"
                                "ota_test.c"
                               
                        INCLUDE_DIRS "."
                        EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
