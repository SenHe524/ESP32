# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "i2c_hardware.c" "KF.c" "IMU.c" "Init_Read.c" "main.c"
                               
                               
                        INCLUDE_DIRS "."
                        )
