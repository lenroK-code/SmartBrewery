#pragma once
#include "esp_err.h"

#define EXAMPLE_ONEWIRE_BUS_GPIO    4
#define EXAMPLE_ONEWIRE_MAX_DS18B20 1


esp_err_t temp_init(void);
float temp_read(void);


