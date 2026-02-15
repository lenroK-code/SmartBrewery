#pragma once
#include "esp_err.h"
#define TCRT5000_PIN GPIO_NUM_10

esp_err_t tcrt5000_init(void);
int tcrt5000_read(void);