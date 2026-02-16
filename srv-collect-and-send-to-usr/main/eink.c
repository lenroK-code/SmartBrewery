#include "eink_driver.h"
#include "eink.h"
#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "EINK";

void screen_init(void)
{
	ESP_LOGI(TAG, "Initializing e-ink driver");
	eink_init(SPI2_HOST, 4, 6, -1, 7, 1, 2, 3, 200, 200);
	vTaskDelay(pdMS_TO_TICKS(2000));
	ESP_LOGI(TAG, "Setting rotation to 90 degrees");
	eink_set_rotation(90);
}

void eink_update_values(float sg, float blg, float temp_c)
{
	// Compose full display lines (labels + values) and perform a full redraw
	int scale = 3;
	int base_x = 0;
	int base_y = 20;

	char line1[32];
	char line2[32];
	char line3[32];
	// keep spacing similar to demo output
	snprintf(line1, sizeof(line1), "SG:   %.3f", sg);
	snprintf(line2, sizeof(line2), "BLG:  %.1f", blg);
	snprintf(line3, sizeof(line3), "T:    %.1fC", temp_c);

	const char *lines[3] = {line1, line2, line3};
	// This will render the lines into a full-screen framebuffer and perform a full update.
	eink_draw_lines_scaled(base_x, base_y, lines, 3, scale);
}