# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS  "main.c"
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES bluetooth ota timer wifi
                        )
