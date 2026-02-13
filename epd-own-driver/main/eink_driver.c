// Simple ESP-IDF C driver template for an e-paper panel (SSD1681-like / 200x200 example)
// Provides: eink_init(), eink_draw_text(), eink_clear(), eink_sleep()
// Note: This implementation uses a basic full-update sequence inspired by common panels.
// Adjust LUTs/init sequences for your exact panel if needed.

#include "eink_driver.h"

#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <stddef.h>

static const char *TAG = "eink_driver";

static spi_device_handle_t spi = NULL;
static int g_cs = -1, g_dc = -1, g_rst = -1, g_busy = -1;
static int g_width = 200, g_height = 200;
static int g_invert = 0; // 0 = normal, 1 = invert bytes before sending

// Simple 6x8 ASCII font (32..127). Each char is 6 bytes (6x8).
// For brevity this is a compact font taken from public domain 6x8 tables.
static const uint8_t font6x8[] = {
    // 96 characters * 6 bytes = 576 bytes
    // Space (0x20)
    0x00,0x00,0x00,0x00,0x00,0x00,
    // '!' ... (we include a minimal set sufficient for basic ASCII)
    0x00,0x00,0x5F,0x00,0x00,0x00, // !
    0x00,0x07,0x00,0x07,0x00,0x00, // "
    0x14,0x7F,0x14,0x7F,0x14,0x00, // #
    0x24,0x2A,0x7F,0x2A,0x12,0x00, // $
    0x23,0x13,0x08,0x64,0x62,0x00, // %
    0x36,0x49,0x55,0x22,0x50,0x00, // &
    0x00,0x05,0x03,0x00,0x00,0x00, // '
    0x00,0x1C,0x22,0x41,0x00,0x00, // (
    0x00,0x41,0x22,0x1C,0x00,0x00, // )
    0x14,0x08,0x3E,0x08,0x14,0x00, // *
    0x08,0x08,0x3E,0x08,0x08,0x00, // +
    0x00,0x50,0x30,0x00,0x00,0x00, // ,
    0x08,0x08,0x08,0x08,0x08,0x00, // -
    0x00,0x60,0x60,0x00,0x00,0x00, // .
    0x20,0x10,0x08,0x04,0x02,0x00, // /
    0x3E,0x51,0x49,0x45,0x3E,0x00, // 0
    0x00,0x42,0x7F,0x40,0x00,0x00, // 1
    0x72,0x49,0x49,0x49,0x46,0x00, // 2
    0x21,0x41,0x49,0x4D,0x33,0x00, // 3
    0x18,0x14,0x12,0x7F,0x10,0x00, // 4
    0x27,0x45,0x45,0x45,0x39,0x00, // 5
    0x3C,0x4A,0x49,0x49,0x30,0x00, // 6
    0x01,0x71,0x09,0x05,0x03,0x00, // 7
    0x36,0x49,0x49,0x49,0x36,0x00, // 8
    0x06,0x49,0x49,0x29,0x1E,0x00, // 9
    0x00,0x36,0x36,0x00,0x00,0x00, // :
    0x00,0x56,0x36,0x00,0x00,0x00, // ;
    0x08,0x14,0x22,0x41,0x00,0x00, // <
    0x14,0x14,0x14,0x14,0x14,0x00, // =
    0x41,0x22,0x14,0x08,0x00,0x00, // >
    0x02,0x01,0x59,0x09,0x06,0x00, // ?
    0x3E,0x41,0x5D,0x59,0x4E,0x00, // @
    0x7C,0x12,0x11,0x12,0x7C,0x00, // A
    0x7F,0x49,0x49,0x49,0x36,0x00, // B
    0x3E,0x41,0x41,0x41,0x22,0x00, // C
    0x7F,0x41,0x41,0x22,0x1C,0x00, // D
    0x7F,0x49,0x49,0x49,0x41,0x00, // E
    0x7F,0x09,0x09,0x09,0x01,0x00, // F
    0x3E,0x41,0x49,0x49,0x7A,0x00, // G
    0x7F,0x08,0x08,0x08,0x7F,0x00, // H
    0x00,0x41,0x7F,0x41,0x00,0x00, // I
    0x20,0x40,0x41,0x3F,0x01,0x00, // J
    0x7F,0x08,0x14,0x22,0x41,0x00, // K
    0x7F,0x40,0x40,0x40,0x40,0x00, // L
    0x7F,0x02,0x0C,0x02,0x7F,0x00, // M
    0x7F,0x04,0x08,0x10,0x7F,0x00, // N
    0x3E,0x41,0x41,0x41,0x3E,0x00, // O
    0x7F,0x09,0x09,0x09,0x06,0x00, // P
    0x3E,0x41,0x51,0x21,0x5E,0x00, // Q
    0x7F,0x09,0x19,0x29,0x46,0x00, // R
    0x46,0x49,0x49,0x49,0x31,0x00, // S
    0x01,0x01,0x7F,0x01,0x01,0x00, // T
    0x3F,0x40,0x40,0x40,0x3F,0x00, // U
    0x1F,0x20,0x40,0x20,0x1F,0x00, // V
    0x7F,0x20,0x18,0x20,0x7F,0x00, // W
    0x63,0x14,0x08,0x14,0x63,0x00, // X
    0x03,0x04,0x78,0x04,0x03,0x00, // Y
    0x61,0x51,0x49,0x45,0x43,0x00, // Z
    0x00,0x7F,0x41,0x41,0x00,0x00, // [
    0x02,0x04,0x08,0x10,0x20,0x00, // backslash
    0x00,0x41,0x41,0x7F,0x00,0x00, // ]
    0x04,0x02,0x01,0x02,0x04,0x00, // ^
    0x40,0x40,0x40,0x40,0x40,0x00, // _
    0x00,0x01,0x02,0x04,0x00,0x00, // `
    0x20,0x54,0x54,0x54,0x78,0x00, // a
    0x7F,0x48,0x44,0x44,0x38,0x00, // b
    0x38,0x44,0x44,0x44,0x20,0x00, // c
    0x38,0x44,0x44,0x48,0x7F,0x00, // d
    0x38,0x54,0x54,0x54,0x18,0x00, // e
    0x08,0x7E,0x09,0x01,0x02,0x00, // f
    0x0C,0x52,0x52,0x52,0x3E,0x00, // g
    0x7F,0x08,0x04,0x04,0x78,0x00, // h
    0x00,0x44,0x7D,0x40,0x00,0x00, // i
    0x20,0x40,0x44,0x3D,0x00,0x00, // j
    0x7F,0x10,0x28,0x44,0x00,0x00, // k
    0x00,0x41,0x7F,0x40,0x00,0x00, // l
    0x7C,0x04,0x18,0x04,0x78,0x00, // m
    0x7C,0x08,0x04,0x04,0x78,0x00, // n
    0x38,0x44,0x44,0x44,0x38,0x00, // o
    0x7C,0x14,0x14,0x14,0x08,0x00, // p
    0x08,0x14,0x14,0x18,0x7C,0x00, // q
    0x7C,0x08,0x04,0x04,0x08,0x00, // r
    0x48,0x54,0x54,0x54,0x20,0x00, // s
    0x04,0x3F,0x44,0x40,0x20,0x00, // t
    0x3C,0x40,0x40,0x20,0x7C,0x00, // u
    0x1C,0x20,0x40,0x20,0x1C,0x00, // v
    0x3C,0x40,0x30,0x40,0x3C,0x00, // w
    0x44,0x28,0x10,0x28,0x44,0x00, // x
    0x0C,0x50,0x50,0x50,0x3C,0x00, // y
    0x44,0x64,0x54,0x4C,0x44,0x00, // z
    0x00,0x08,0x36,0x41,0x00,0x00, // {
    0x00,0x00,0x7F,0x00,0x00,0x00, // |
    0x00,0x41,0x36,0x08,0x00,0x00, // }
    0x10,0x08,0x08,0x10,0x08,0x00, // ~
    0x00,0x00,0x00,0x00,0x00,0x00  // DEL (pad)
};

// Internal helper: toggle DC and send a command byte
static esp_err_t send_command(uint8_t cmd)
{
    esp_err_t ret;
    if (g_dc >= 0) gpio_set_level(g_dc, 0);
    if (!spi) {
        ESP_LOGE(TAG, "send_command: spi device handle is NULL");
        if (g_dc >= 0) gpio_set_level(g_dc, 1);
        return ESP_ERR_INVALID_STATE;
    }
    spi_transaction_t *t = heap_caps_malloc(sizeof(spi_transaction_t), MALLOC_CAP_DEFAULT);
    if (!t) {
        if (g_dc >= 0) gpio_set_level(g_dc, 1);
        return ESP_ERR_NO_MEM;
    }
    memset(t, 0, sizeof(*t));
    t->length = 8;
    t->tx_buffer = heap_caps_malloc(1, MALLOC_CAP_DMA);
    if (!t->tx_buffer) {
        free(t);
        if (g_dc >= 0) gpio_set_level(g_dc, 1);
        return ESP_ERR_NO_MEM;
    }
    *((uint8_t*)t->tx_buffer) = cmd;
    ret = spi_device_queue_trans(spi, t, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_queue_trans failed: %d", ret);
        free((void*)t->tx_buffer);
        free(t);
        if (g_dc >= 0) gpio_set_level(g_dc, 1);
        return ret;
    }
    // Poll for result with small delays so we don't block the idle task
    spi_transaction_t *rtrans = NULL;
    while (1) {
        esp_err_t r = spi_device_get_trans_result(spi, &rtrans, 10);
        if (r == ESP_OK) break;
        if (r == ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "send_command: waiting for trans result...");
        } else {
            ESP_LOGW(TAG, "send_command: get_trans_result returned %d", r);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free((void*)t->tx_buffer);
    free(t);
    if (g_dc >= 0) gpio_set_level(g_dc, 1);
    return ESP_OK;
}

// Send data buffer
static esp_err_t send_data(const uint8_t* data, int len)
{
    esp_err_t ret;
    if (g_dc >= 0) gpio_set_level(g_dc, 1);
    if (!spi) {
        ESP_LOGE(TAG, "send_data: spi device handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }
    // send in small queued transactions to avoid long blocking calls
    const int CHUNK = 256;
    int sent = 0;
    while (sent < len) {
        int n = (len - sent) > CHUNK ? CHUNK : (len - sent);
        spi_transaction_t *t = heap_caps_malloc(sizeof(spi_transaction_t), MALLOC_CAP_DEFAULT);
        if (!t) return ESP_ERR_NO_MEM;
        memset(t, 0, sizeof(*t));
        t->length = n * 8;
        t->tx_buffer = heap_caps_malloc(n, MALLOC_CAP_DMA);
        if (!t->tx_buffer) {
            free(t);
            return ESP_ERR_NO_MEM;
        }
        memcpy((void*)t->tx_buffer, data + sent, n);
        ret = spi_device_queue_trans(spi, t, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi_device_queue_trans data failed: %d", ret);
            free((void*)t->tx_buffer);
            free(t);
            return ret;
        }
        // wait for this transaction to finish (polling with short timeout)
        spi_transaction_t *rtrans = NULL;
        while (1) {
            esp_err_t r = spi_device_get_trans_result(spi, &rtrans, 10);
            if (r == ESP_OK) break;
            if (r == ESP_ERR_TIMEOUT) {
                ESP_LOGD(TAG, "send_data: waiting for trans result (sent %d/%d)...", sent + n, len);
            } else {
                ESP_LOGW(TAG, "send_data: get_trans_result returned %d", r);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        free((void*)t->tx_buffer);
        free(t);
        sent += n;
    }
    return ESP_OK;
}

static void wait_busy(void)
{
    if (g_busy < 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    // Adaptive busy handling: detect idle level, wait for busy assertion, then wait for idle again
    int idle_level = gpio_get_level(g_busy);
    ESP_LOGD(TAG, "wait_busy: detected idle level=%d", idle_level);
    // wait for busy to assert (level != idle_level)
    const TickType_t assert_timeout = pdMS_TO_TICKS(1000); // 1s to start
    const TickType_t deassert_timeout = pdMS_TO_TICKS(8000); // 8s for refresh
    TickType_t start = xTaskGetTickCount();
    while (gpio_get_level(g_busy) == idle_level) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - start) > assert_timeout) {
            ESP_LOGW(TAG, "wait_busy: busy did not assert within %d ms", 1000);
            break;
        }
    }
    // now wait for busy to return to idle_level
    start = xTaskGetTickCount();
    while (gpio_get_level(g_busy) != idle_level) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - start) > deassert_timeout) {
            ESP_LOGW(TAG, "wait_busy: busy did not deassert within %d ms", 8000);
            break;
        }
    }
}

// forward declaration for functions used before their definition
static void set_ram_area(int x, int y, int w, int h);

// Minimal init sequence (may require tuning for your exact panel)
static int _init_display_done = 0;
static int g_rotation = 0; // rotation in degrees: 0,90,180,270

static void panel_init_sequence(void)
{
    // follow GxEPD2_154_D67::_InitDisplay sequence
    // Reset / soft reset
    if (g_rst >= 0) {
        gpio_set_level(g_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(g_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // soft reset command 0x12
    send_command(0x12);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Driver output control
    send_command(0x01);
    uint8_t drv_out[3] = {0xC7, 0x00, 0x00};
    send_data(drv_out, 3);

    // Border waveform
    send_command(0x3C);
    uint8_t border = 0x05;
    send_data(&border, 1);

    // Read built-in temperature sensor (set mode)
    send_command(0x18);
    uint8_t temp = 0x80;
    send_data(&temp, 1);

    // Set full RAM area to whole panel
    set_ram_area(0, 0, g_width, g_height);

    _init_display_done = 1;

    // Optionally upload LUT (we keep existing LUT arrays as placeholders)
    extern const uint8_t LUTDefault_full[];
    const uint8_t *p = LUTDefault_full;
    const int len = 29; // known LUT length
    if (len > 1) {
        // first byte is the command (0x32)
        send_command(p[0]);
        send_data(&p[1], len - 1);
    }
}

// Power on sequence similar to GxEPD2 _PowerOn
static void power_on(void)
{
    // use GxEPD2_154_D67 power-on sequence: 0x22, 0xE0 then 0x20
    uint8_t data_e0 = 0xE0;
    send_command(0x22);
    send_data(&data_e0, 1);
    send_command(0x20);
    wait_busy();
}

// Initialization task to run panel init/power on/clear without blocking caller
static void eink_init_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "eink_init_task: before panel_init_sequence, busy=%d", g_busy>=0?gpio_get_level(g_busy):-1);
    panel_init_sequence();
    ESP_LOGI(TAG, "eink_init_task: after panel_init_sequence, busy=%d", g_busy>=0?gpio_get_level(g_busy):-1);
    vTaskDelay(pdMS_TO_TICKS(50));
    power_on();
    ESP_LOGI(TAG, "eink_init_task: after power_on, busy=%d", g_busy>=0?gpio_get_level(g_busy):-1);
    vTaskDelay(pdMS_TO_TICKS(50));
    // do a hard clear to remove ghosting
    eink_hard_clear();
    ESP_LOGI(TAG, "eink_init_task: after hard_clear, busy=%d", g_busy>=0?gpio_get_level(g_busy):-1);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "eink init task done");
    vTaskDelete(NULL);
}

// Set RAM area (byte addressing horizontally)
static void set_ram_area(int x, int y, int w, int h)
{
    // x in pixels -> bytes x/8
    int x_start = x / 8;
    int x_end = (x + w - 1) / 8;
    uint8_t d1 = x_start & 0xFF;
    uint8_t d2 = x_end & 0xFF;

    // set ram entry mode: x increase, y increase (normal)
    send_command(0x11);
    uint8_t ram_entry = 0x03;
    send_data(&ram_entry, 1);

    send_command(0x44); // set RAM X address start/end
    send_data((const uint8_t[]){d1, d2}, 2);

    // set RAM Y address start/end (GxEPD2_154_D67 ordering: y_start_l, y_start_h, y_end_l, y_end_h)
    uint8_t y_start_l = (uint8_t)(y & 0xFF);
    uint8_t y_start_h = (uint8_t)((y >> 8) & 0xFF);
    uint8_t y_end_l = (uint8_t)((y + h - 1) & 0xFF);
    uint8_t y_end_h = (uint8_t)(((y + h - 1) >> 8) & 0xFF);
    send_command(0x45);
    send_data((const uint8_t[]){y_start_l, y_start_h, y_end_l, y_end_h}, 4);

    // Set RAM X/Y counters to start
    send_command(0x4E); // RAM X address counter
    send_data((const uint8_t[]){d1}, 1);
    send_command(0x4F); // RAM Y address counter -> set to y start
    send_data((const uint8_t[]){y_start_l, y_start_h}, 2);
}

// Write framebuffer (black channel) with 0x24 command
static void write_framebuffer(const uint8_t* buf, int buf_len, int x, int y, int w, int h)
{
    // If rotation is requested, rotate the incoming 1bpp framebuffer into a
    // temporary buffer before sending. This routine assumes square panels
    // (width==height) for 90/270 degree rotations.
    int bytes_per_line = (w + 7) / 8;
    const uint8_t *to_send = buf;
    uint8_t *rot_buf = NULL;
    int send_w = w;
    int send_h = h;

    if (g_rotation != 0) {
        // allocate rotated buffer (same size as source for square displays)
        rot_buf = heap_caps_malloc(buf_len, MALLOC_CAP_DEFAULT);
        if (rot_buf) {
            memset(rot_buf, 0xFF, buf_len);
            for (int sy = 0; sy < h; ++sy) {
                for (int sx = 0; sx < w; ++sx) {
                    int sbi = sy * bytes_per_line + (sx / 8);
                    int sbit = 7 - (sx % 8);
                    int pixel_black = ((buf[sbi] >> sbit) & 1) == 0;
                    if (!pixel_black) continue;
                    int dx = sx, dy = sy;
                    if (g_rotation == 90) {
                        dx = sy;
                        dy = (w - 1) - sx;
                    } else if (g_rotation == 180) {
                        dx = (w - 1) - sx;
                        dy = (h - 1) - sy;
                    } else if (g_rotation == 270) {
                        dx = (h - 1) - sy;
                        dy = sx;
                    }
                    if (dx < 0 || dx >= w || dy < 0 || dy >= h) continue;
                    int dbi = dy * bytes_per_line + (dx / 8);
                    int dbit = 7 - (dx % 8);
                    rot_buf[dbi] &= ~(1 << dbit);
                }
            }
            to_send = rot_buf;
            // send_w/send_h remain w/h for square panels
        }
    }

    set_ram_area(x, y, send_w, send_h);
    send_command(0x24);
    // Send in chunks to avoid long blocking SPI transfers and allow WDT servicing
    const int CHUNK = 256;
    if (!g_invert) {
        int sent = 0;
        while (sent < buf_len) {
            int n = (buf_len - sent) > CHUNK ? CHUNK : (buf_len - sent);
            send_data(to_send + sent, n);
            sent += n;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    } else {
        // send inverted chunks
        int sent = 0;
        uint8_t tmp[CHUNK];
        while (sent < buf_len) {
            int n = (buf_len - sent) > CHUNK ? CHUNK : (buf_len - sent);
            for (int i = 0; i < n; ++i) tmp[i] = ~(to_send[sent + i]);
            send_data(tmp, n);
            sent += n;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (rot_buf) free(rot_buf);
}

// Trigger full update: mode=false -> full update
static void update_full(void)
{
    // GxEPD2_154_D67 uses 0x22 0xF7 for full update
    send_command(0x22);
    uint8_t data = 0xF7;
    send_data(&data, 1);
    send_command(0x20);
    wait_busy();
    // mark power as off-ish on some panels (GxEPD2 does)
    send_command(0xFF);
}

static void update_part(void)
{
    // GxEPD2_154_D67 uses 0x22 0xFC for partial update
    send_command(0x22);
    uint8_t data = 0xFC;
    send_data(&data, 1);
    send_command(0x20);
    wait_busy();
    send_command(0xFF);
}

// LUTs taken from GxEPD2 drivers (LUTDefault_full and LUTDefault_part)
const uint8_t LUTDefault_full[] =
{
    0x32,  // command
    0x22, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x01, 0x00, 0x00, 0x00, 0x00
};

const uint8_t LUTDefault_part[] =
{
    0x32,  // command
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int eink_init(int spi_host, int sclk_gpio, int mosi_gpio, int miso_gpio,
              int cs_gpio, int dc_gpio, int rst_gpio, int busy_gpio,
              int width, int height)
{
    esp_err_t ret;
    g_cs = cs_gpio; g_dc = dc_gpio; g_rst = rst_gpio; g_busy = busy_gpio;
    g_width = width; g_height = height;

    // Configure GPIOs
    if (g_dc >= 0) gpio_config(&(gpio_config_t){.pin_bit_mask = 1ULL<<g_dc, .mode = GPIO_MODE_OUTPUT});
    if (g_rst >= 0) gpio_config(&(gpio_config_t){.pin_bit_mask = 1ULL<<g_rst, .mode = GPIO_MODE_OUTPUT});
    if (g_busy >= 0) gpio_config(&(gpio_config_t){.pin_bit_mask = 1ULL<<g_busy, .mode = GPIO_MODE_INPUT});

    // Init SPI bus
    spi_bus_config_t buscfg = {0};
    buscfg.mosi_io_num = mosi_gpio;
    buscfg.miso_io_num = miso_gpio;
    buscfg.sclk_io_num = sclk_gpio;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = (width/8) * height + 8;

    ret = spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "spi_bus_initialize: already initialized");
        } else {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
            return -1;
        }
    }

    spi_device_interface_config_t devcfg = {0};
    devcfg.clock_speed_hz = 10*1000*1000; // 10 MHz
    devcfg.mode = 0;
    devcfg.spics_io_num = cs_gpio;
    devcfg.queue_size = 1;

    if (spi == NULL) {
        // Try to add device on the requested host; if that fails due to invalid host
        // try common alternative host identifiers (HSPI/VSPI/SPI2/SPI3) to support different chips/IDF versions.
        int tried_hosts[6];
        int th_count = 0;
        tried_hosts[th_count++] = spi_host;
#ifdef HSPI_HOST
        if (th_count < 6) tried_hosts[th_count++] = HSPI_HOST;
#endif
#ifdef VSPI_HOST
        if (th_count < 6) tried_hosts[th_count++] = VSPI_HOST;
#endif
#ifdef SPI2_HOST
        if (th_count < 6) tried_hosts[th_count++] = SPI2_HOST;
#endif
#ifdef SPI3_HOST
        if (th_count < 6) tried_hosts[th_count++] = SPI3_HOST;
#endif

        esp_err_t last_ret = ESP_ERR_INVALID_ARG;
        for (int i = 0; i < th_count; ++i) {
            int h = tried_hosts[i];
            ret = spi_bus_add_device(h, &devcfg, &spi);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "spi_bus_add_device succeeded on host %d", h);
                break;
            }
            last_ret = ret;
            ESP_LOGW(TAG, "spi_bus_add_device failed for host %d: %d", h, ret);
        }
        if (spi == NULL) {
            ESP_LOGE(TAG, "spi_bus_add_device failed, last err: %d", last_ret);
            spi = NULL;
            return -1;
        }
    } else {
        ESP_LOGI(TAG, "spi device already present, skipping add_device");
    }

    // Start with DC high
    if (g_dc >= 0) gpio_set_level(g_dc, 1);

    // Run heavy init (panel init, power on, clears) in a dedicated task
    // to avoid blocking the main task and triggering the watchdog.
    BaseType_t xt = xTaskCreatePinnedToCore(eink_init_task, "eink_init", 4096, NULL, tskIDLE_PRIORITY+2, NULL, tskNO_AFFINITY);
    if (xt != pdPASS) {
        ESP_LOGW(TAG, "Failed to create eink init task, running inline");
        panel_init_sequence();
        power_on();
        eink_hard_clear();
    }

    ESP_LOGI(TAG, "eink initialized (w=%d h=%d) - init scheduled", g_width, g_height);
    return 0;
}

// Render text into temporary framebuffer (1bpp, packed MSB first) and send full-screen
void eink_draw_text(int x, int y, const char* text)
{
    if (!text) return;
    int bytes_per_line = (g_width / 8);
    int fb_size = bytes_per_line * g_height;
    uint8_t *fb = heap_caps_malloc(fb_size, MALLOC_CAP_DEFAULT);
    if (!fb) return;
    memset(fb, 0xFF, fb_size); // white

    int cx = x;
    const unsigned char *p = (const unsigned char*)text;
    while (*p) {
        // handle UTF-8 degree symbol (0xC2 0xB0)
        if (p[0] == 0xC2 && p[1] == 0xB0) {
            // draw small hollow 3x3 circle at upper-right of glyph box
            int cx0 = cx + 3; // start x of 3x3 box
            int cy0 = y + 1;  // start y
            for (int ry = 0; ry < 3; ++ry) {
                for (int rx = 0; rx < 3; ++rx) {
                    // hollow ring: draw border pixels only
                    if (ry == 1 && rx == 1) continue;
                    int px = cx0 + rx;
                    int py = cy0 + ry;
                    if (px < 0 || py < 0 || px >= g_width || py >= g_height) continue;
                    int bi = py * bytes_per_line + (px / 8);
                    int bbit = 7 - (px % 8);
                    fb[bi] &= ~(1 << bbit);
                }
            }
            p += 2;
            cx += 6;
            continue;
        }
        unsigned char c = *p++;
        if (c < 32 || c > 127) c = '?';
        const uint8_t *glyph = &font6x8[(c - 32) * 6];
        // draw 6x8 glyph
        for (int col = 0; col < 6; ++col) {
            uint8_t coldata = glyph[col];
            for (int row = 0; row < 8; ++row) {
                int px = cx + col;
                int py = y + row;
                if (px < 0 || py < 0 || px >= g_width || py >= g_height) continue;
                int byte_index = py * bytes_per_line + (px / 8);
                int bit = 7 - (px % 8);
                if (coldata & (1 << row)) {
                    fb[byte_index] &= ~(1 << bit); // black pixel
                }
            }
        }
        cx += 6; // glyph width
    }

    write_framebuffer(fb, fb_size, 0, 0, g_width, g_height);
    update_full();
    free(fb);
}

// Scaled text renderer: integer scale >=1. Uses the 6x8 font and scales pixels.
void eink_draw_text_scaled(int x, int y, const char* text, int scale)
{
    if (!text) return;
    if (scale <= 1) { eink_draw_text(x, y, text); return; }

    int bytes_per_line = (g_width / 8);
    int fb_size = bytes_per_line * g_height;
    uint8_t *fb = heap_caps_malloc(fb_size, MALLOC_CAP_DEFAULT);
    if (!fb) return;
    memset(fb, 0xFF, fb_size); // white

    int cx = x;
    const unsigned char *p = (const unsigned char*)text;
    while (*p) {
        // UTF-8 degree symbol handling
        if (p[0] == 0xC2 && p[1] == 0xB0) {
            // scaled hollow circle: draw a hollow box of size (max(3,scale))
            int box = scale < 3 ? 3 : scale;
            int cx0 = cx + 6*scale - (box + 1);
            int cy0 = y + 1;
            for (int ry = 0; ry < box; ++ry) {
                for (int rx = 0; rx < box; ++rx) {
                    if (ry == box/2 && rx == box/2) continue; // keep center empty for hollow look
                    if (ry > 0 && ry < box-1 && rx > 0 && rx < box-1) continue;
                    int px = cx0 + rx;
                    int py = cy0 + ry;
                    if (px < 0 || py < 0 || px >= g_width || py >= g_height) continue;
                    int bi = py * bytes_per_line + (px / 8);
                    int bbit = 7 - (px % 8);
                    fb[bi] &= ~(1 << bbit);
                }
            }
            p += 2;
            cx += 6 * scale;
            continue;
        }
        unsigned char c = *p++;
        if (c < 32 || c > 127) c = '?';
        const uint8_t *glyph = &font6x8[(c - 32) * 6];
        // for each column in glyph
        for (int col = 0; col < 6; ++col) {
            uint8_t coldata = glyph[col];
            // for each bit row in glyph (0..7)
            for (int row = 0; row < 8; ++row) {
                int pixel_on = (coldata & (1 << row)) != 0;
                if (!pixel_on) continue;
                // map to scaled pixels
                for (int sx = 0; sx < scale; ++sx) {
                    for (int sy = 0; sy < scale; ++sy) {
                        int px = cx + col * scale + sx;
                        int py = y + row * scale + sy;
                        if (px < 0 || py < 0 || px >= g_width || py >= g_height) continue;
                        int byte_index = py * bytes_per_line + (px / 8);
                        int bit = 7 - (px % 8);
                        fb[byte_index] &= ~(1 << bit);
                    }
                }
            }
        }
        cx += 6 * scale; // advance by scaled glyph width
    }

    write_framebuffer(fb, fb_size, 0, 0, g_width, g_height);
    update_full();
    free(fb);
}

// Draw multiple lines into one full-screen framebuffer and send a single update.
void eink_draw_lines_scaled(int x, int y, const char** lines, int n_lines, int scale)
{
    if (!lines || n_lines <= 0) return;
    if (scale <= 1) scale = 1;

    int bytes_per_line = (g_width / 8);
    int fb_size = bytes_per_line * g_height;
    uint8_t *fb = heap_caps_malloc(fb_size, MALLOC_CAP_DEFAULT);
    if (!fb) return;
    memset(fb, 0xFF, fb_size); // white

    int line_y = y;
    for (int li = 0; li < n_lines; ++li) {
        const char* text = lines[li];
        if (!text) continue;
        int cx = x;
        const unsigned char *p = (const unsigned char*)text;
        while (*p) {
            // UTF-8 degree symbol handling
            if (p[0] == 0xC2 && p[1] == 0xB0) {
                int box = scale < 3 ? 3 : scale;
                int cx0 = cx + 6*scale - (box + 1);
                int cy0 = line_y + 1;
                for (int ry = 0; ry < box; ++ry) {
                    for (int rx = 0; rx < box; ++rx) {
                        if (ry == box/2 && rx == box/2) continue;
                        if (ry > 0 && ry < box-1 && rx > 0 && rx < box-1) continue;
                        int px = cx0 + rx;
                        int py = cy0 + ry;
                        if (px < 0 || py < 0 || px >= g_width || py >= g_height) continue;
                        int bi = py * bytes_per_line + (px / 8);
                        int bbit = 7 - (px % 8);
                        fb[bi] &= ~(1 << bbit);
                    }
                }
                p += 2;
                cx += 6 * scale;
                continue;
            }
            unsigned char c = *p++;
            if (c < 32 || c > 127) c = '?';
            const uint8_t *glyph = &font6x8[(c - 32) * 6];
            for (int col = 0; col < 6; ++col) {
                uint8_t coldata = glyph[col];
                for (int row = 0; row < 8; ++row) {
                    if (!(coldata & (1 << row))) continue;
                    for (int sx = 0; sx < scale; ++sx) {
                        for (int sy = 0; sy < scale; ++sy) {
                            int px = cx + col * scale + sx;
                            int py = line_y + row * scale + sy;
                            if (px < 0 || py < 0 || px >= g_width || py >= g_height) continue;
                            int byte_index = py * bytes_per_line + (px / 8);
                            int bit = 7 - (px % 8);
                            fb[byte_index] &= ~(1 << bit);
                        }
                    }
                }
            }
            cx += 6 * scale;
        }
        line_y += (8 * scale) + 6; // line spacing
    }

    write_framebuffer(fb, fb_size, 0, 0, g_width, g_height);
    update_full();
    free(fb);
}

// Rotation removed — render multiline by drawing each line scaled separately using eink_draw_text_scaled.

void eink_clear(void)
{
    int bytes_per_line = (g_width / 8);
    int fb_size = bytes_per_line * g_height;
    uint8_t *fb = heap_caps_malloc(fb_size, MALLOC_CAP_DEFAULT);
    if (!fb) return;
    memset(fb, 0xFF, fb_size);
    write_framebuffer(fb, fb_size, 0, 0, g_width, g_height);
    // Some controllers need two updates to fully clear ghosting; do a second full update.
    update_full();
    vTaskDelay(pdMS_TO_TICKS(100));
    update_full();
    free(fb);
}

void eink_sleep(void)
{
    send_command(0x10); // deep sleep
    uint8_t d = 0x01;
    send_data(&d, 1);
}

// Hard clear: write all black, full update, then all white, full update.
void eink_hard_clear(void)
{
    int bytes_per_line = (g_width / 8);
    int fb_size = bytes_per_line * g_height;
    uint8_t *fb = heap_caps_malloc(fb_size, MALLOC_CAP_DEFAULT);
    if (!fb) return;

    // all black
    memset(fb, 0x00, fb_size);
    write_framebuffer(fb, fb_size, 0, 0, g_width, g_height);
    update_full();
    vTaskDelay(pdMS_TO_TICKS(200));

    // all white
    memset(fb, 0xFF, fb_size);
    write_framebuffer(fb, fb_size, 0, 0, g_width, g_height);
    update_full();
    vTaskDelay(pdMS_TO_TICKS(200));

    free(fb);
}

void eink_set_invert(int invert)
{
    g_invert = invert ? 1 : 0;
}

void eink_set_rotation(int deg)
{
    int d = deg % 360;
    if (d < 0) d += 360;
    if (d == 0 || d == 90 || d == 180 || d == 270) {
        g_rotation = d;
        ESP_LOGI(TAG, "rotation set to %d degrees", g_rotation);
    } else {
        ESP_LOGW(TAG, "invalid rotation %d - use 0,90,180,270", deg);
    }
}
