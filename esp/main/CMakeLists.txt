idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "main.c" "smartconfig.c" "display.c" "esp32_i2s_parallel.c"
                    INCLUDE_DIRS ".")
