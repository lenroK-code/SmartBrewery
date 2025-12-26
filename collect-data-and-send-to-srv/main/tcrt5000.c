#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TCRT5000_PIN GPIO_NUM_10  // CHANGE THIS to the pin where TCRT5000 is connected
static const char *TAG = "TCRT5000";

esp_err_t tcrt5000_init()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << TCRT5000_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    ESP_LOGI(TAG, "TCRT5000 initialized on GPIO%d", TCRT5000_PIN);
    return ret;
}

int tcrt5000_read()
{
    // uncomment the following to have actual reading
    // int level = gpio_get_level(TCRT5000_PIN);
    // ESP_LOGD(TAG, "TCRT5000 read level: %d", level);
    // return level;


    // randomly return 0 or 1 for testing purposes
    int level = rand() % 2; 
    return level;
}
