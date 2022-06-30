# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)
# idf_component_register(SRCS "timer_gpio.c"
#                         INCLUDE_DIRS "include"
#                         )
set(timer_gpio_srcs "src/timer_gpio.c")
idf_component_register(SRCS "${timer_gpio_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES bluetooth
                        REQUIRES nvs_flash
                        )