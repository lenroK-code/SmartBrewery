// nie dziala jak cos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include <string.h>

#define DEFAULT_VREF        1100
#define DETECTION_THRESHOLD 1500

static const char *TAG = "TCRT5000_ADC";

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;

void init_adc(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg));

    adc_cali_init_info_t cali_cfg;
    memset(&cali_cfg, 0, sizeof(cali_cfg));
    cali_cfg.unit_id = ADC_UNIT_2;
    cali_cfg.atten = ADC_ATTEN_DB_11;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

    esp_err_t ret = adc_cali_create_scheme(ADC_CALI_SCHEME_VER_CURVE_FITTING, &cali_cfg, &adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Kalibracja ADC niedostępna, użyj surowych odczytów");
        adc_cali_handle = NULL;
    }
}

void app_main(void) {
    init_adc();

    while (1) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw));

        int voltage = 0;
        if (adc_cali_handle) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage));
        } else {
            voltage = raw;
        }

        if (voltage > DETECTION_THRESHOLD) {
            ESP_LOGI(TAG, "Ruch wykryty! Napięcie: %d mV", voltage);
        } else {
            ESP_LOGI(TAG, "Brak ruchu. Napięcie: %d mV", voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
