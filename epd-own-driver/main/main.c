#include <stdio.h>
#include "eink_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// VSPI_HOST/HSPI_HOST are defined in the SPI driver
#include "driver/spi_master.h"

static const char *TAG = "main_test";

void app_main(void)
{
	// Example GPIO mapping (adjust for your board):
	// sclk=4, mosi=6, miso=-1, cs=7, dc=1, rst=2, busy=3
	// Use SPI2_HOST (alias for HSPI/VSPI depending on IDF); change if your SDK defines different host
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
	eink_clear();
	vTaskDelay(pdMS_TO_TICKS(3000));

	ESP_LOGI(TAG, "Test 5: draw final message");
	// Demo multiline output (no rotation). Draw each line scaled.
	const char *line1 = "SG:   1.050";
	const char *line2 = "BLG:  12.5°";
	const char *line3 = "T:   21.5°C";
	const char *line4 = "Start:08-08";
	int scale = 3; // larger font
	const char *lines[] = { line1, line2, line3, line4 };
	eink_draw_lines_scaled(0, 20, lines, 4, scale);
	vTaskDelay(pdMS_TO_TICKS(5000));

	ESP_LOGI(TAG, "Putting panel to sleep");
	eink_sleep();

	// Finished tests; loop forever
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}