# idf_component_register(SRCS  "main.c" 
idf_component_register(SRCS "tcrt5000.c" "main.c" "tcrt5000.h" "mpu9250.c" "mpu9250.h"
                    PRIV_REQUIRES nvs_flash esp_driver_i2c bt esp_driver_gpio esp_common
                    INCLUDE_DIRS ".")