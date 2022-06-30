# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)


set(i2c_hardware_srcs "src/i2c_hardware.c")
idf_component_register(SRCS "${i2c_hardware_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES 
                        REQUIRES 
                        )

