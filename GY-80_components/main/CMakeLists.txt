# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "init_Read.c" "main.c"
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES imu i2c kalman_fliter
                        REQUIRES 
                        )
