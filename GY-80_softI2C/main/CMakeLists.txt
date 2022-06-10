# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "i2c_software.c" "KF.c" "MahonyAHRS.c" "IMU.c" "MadgwickAHRS.c" "Init_Read.c" "main.c"
                               
                               
                        INCLUDE_DIRS "."
                        )
