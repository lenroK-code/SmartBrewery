#ifndef EINK_DRIVER_H
#define EINK_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	// Initialize the e-ink driver for ESP-IDF.
	// Parameters: spi_host (HSPI_HOST or VSPI_HOST), sclk, mosi, miso (use -1 if unused),
	// cs, dc, rst, busy GPIO numbers, width and height of the panel in pixels.
	int eink_init(int spi_host, int sclk_gpio, int mosi_gpio, int miso_gpio,
				  int cs_gpio, int dc_gpio, int rst_gpio, int busy_gpio,
				  int width, int height);

	// Draw a NUL-terminated string at (x,y) in pixels using built-in 6x8 font.
	// This performs a full update (blocking) after rendering.
	void eink_draw_text(int x, int y, const char *text);

	// Draw text with integer scaling (scale >= 1). Example: scale=2 -> 12x16 glyphs.
	// Performs a full update after rendering.
	void eink_draw_text_scaled(int x, int y, const char *text, int scale);

	// Draw multiple lines (array of C strings) at (x,y) with given scale; updates once.
	void eink_draw_lines_scaled(int x, int y, const char **lines, int n_lines, int scale);

	// (rotation removed) For multiline text, render each line separately with scaling.

	// Clear the display to white and perform a full update.
	void eink_clear(void);

	// Put the display in deep sleep/hibernate.
	void eink_sleep(void);

	// Perform a stronger clear sequence: full black -> full white to reduce ghosting.
	void eink_hard_clear(void);

	// Set pixel polarity inversion: if non-zero, bytes are inverted before sending.
	void eink_set_invert(int invert);

	// Set display rotation in degrees: 0, 90, 180, or 270. Rotation is applied
	// to framebuffers before sending to the panel. Note: non-square panels may
	// behave unexpectedly; this implementation assumes square displays (200x200).
	void eink_set_rotation(int deg);

#ifdef __cplusplus
}
#endif

#endif // EINK_DRIVER_H
