# Embed the server root certificate into the final binary
# idf_build_get_property(project_dir PROJECT_DIR)
# idf_component_register(SRCS "gap_gatts.c"
#                         INCLUDE_DIRS "include"
#                         )

set(kf_srcs "src/kf.c")
idf_component_register(SRCS "${kf_srcs}"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES 
                        REQUIRES 
                        )

