idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "veml6030.c" "adxl345.c" "i2c.c" "esp32_i2s_parallel_v2.c" "main.c" "smartconfig.c" "display.c" "display2812.c"
                    INCLUDE_DIRS ".")
