#include "eink_driver.h"
#include "eink.h"
#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "EINK";

void eink_demo(void){
    ESP_LOGI(TAG, "Initializing e-ink driver");
	eink_init(SPI2_HOST, 4, 6, -1, 7, 1, 2, 3, 200, 200);

	// Wait for init task to complete
	vTaskDelay(pdMS_TO_TICKS(2000));

	// apply rotation for demo (0,90,180,270)
	ESP_LOGI(TAG, "Setting rotation to 90 degrees");
	eink_set_rotation(90);

	ESP_LOGI(TAG, "Test 1: hard clear (black then white)");
	eink_hard_clear();
	vTaskDelay(pdMS_TO_TICKS(2000));

	ESP_LOGI(TAG, "Test 4: clear screen (white)");
	// eink_clear();
	vTaskDelay(pdMS_TO_TICKS(3000));

	ESP_LOGI(TAG, "Test 5: draw final message");
	// Demo multiline output (no rotation). Draw each line scaled.
	const char *line1 = "SG:   1.050";
	const char *line2 = "BLG:  12.5"; // fix this degree symbol encoding 
	const char *line3 = "T:    21.5C";
	const char *line4 = "123456789012"; // test long line
	int scale = 3; // larger font
	const char *lines[] = { line1, line2, line3, line4 };
	eink_draw_lines_scaled(0, 20, lines, 4, scale);
	vTaskDelay(pdMS_TO_TICKS(5000));

	ESP_LOGI(TAG, "Putting panel to sleep");
	// eink_sleep();
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

	const char *lines[3] = { line1, line2, line3 };
	// This will render the lines into a full-screen framebuffer and perform a full update.
	eink_draw_lines_scaled(base_x, base_y, lines, 3, scale);
}