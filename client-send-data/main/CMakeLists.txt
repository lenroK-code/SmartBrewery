idf_component_register(SRCS "ds18b20-sensor.c" "main.c" "tcrt5000.c" "mpu9250.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_driver_i2c esp_driver_gpio bt esp_common onewire_bus ds18b20)
