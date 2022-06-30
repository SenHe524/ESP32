# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)
# idf_component_register(SRCS "gap_gatts.c"
#                         INCLUDE_DIRS "include"
#                         )

set(gap_gatts_srcs "src/gap_gatts.c")
idf_component_register(SRCS "${gap_gatts_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES 
                        REQUIRES nvs_flash bt
                        )

