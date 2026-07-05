idf_component_register(
    SRCS "regulator.c" "adc_calibration.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_adc esp_driver_ledc
)