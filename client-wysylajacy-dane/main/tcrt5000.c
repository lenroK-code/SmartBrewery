#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TCRT5000_PIN GPIO_NUM_10  // Zmień na pin, do którego podłączony jest TCRT5000
static const char *TAG_IR = "TCRT5000";

void tcrt5000_init()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << TCRT5000_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGE(TAG_IR, "TCRT5000 initialized on pin %d", TCRT5000_PIN);
}

int tcrt5000_read()
{
    int level = gpio_get_level(TCRT5000_PIN);
    ESP_LOGI(TAG_IR, "TCRT5000 read level: %d", level);
    return level;
}

