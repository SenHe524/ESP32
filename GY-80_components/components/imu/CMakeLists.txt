# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)
# idf_component_register(SRCS "ota_task.c"
#                         INCLUDE_DIRS "include"
#                         )
set(imu_srcs "src/imu.c")
idf_component_register(SRCS "${imu_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES 
                        REQUIRES 
                        )