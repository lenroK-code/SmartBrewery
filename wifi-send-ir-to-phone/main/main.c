#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

// dupa te biblioteki NimBLE nie istnieją 
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TCRT5000_PIN GPIO_NUM_10
static const char *TAG = "TCRT5000_DIGITAL";

static int ble_gap_connected = 0;

static struct ble_gatt_svc_def gatt_services[];

// Bufor do wysyłania danych
static char ble_notify_buf[100];

// UUID usługi i charakterystyki (w tym przykładzie proste UUID 16-bit)
#define SERVICE_UUID        0x180D   // Możesz zmienić na własne UUID
#define CHARACTERISTIC_UUID 0x2A37   // Przykładowe UUID

static uint16_t conn_handle;

// Callback przy zmianie stanu połączenia BLE
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Połączono z klientem");
                ble_gap_connected = 1;
                conn_handle = event->connect.conn_handle;
            } else {
                ESP_LOGI(TAG, "Nieudane połączenie");
                ble_gap_connected = 0;
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Rozłączono");
            ble_gap_connected = 0;
            break;
        default:
            break;
    }
    return 0;
}

// Callback przy zapytaniu o GATT
static int gatt_read(struct ble_gatt_chr_context *chr_ctx, void *arg,
                    struct ble_gatt_access_ctxt *ctxt, void *om) {
    // Tu zwróć opcjonalnie brak danych lub przygotuj odpowiedź jeśli potrzebne
    return BLE_ATT_ERR_CAPACITY;
}

static int gatt_notify(uint16_t conn_handle, const char *log_msg) {
    if (!ble_gap_connected) return -1;

    int rc = ble_gattc_notify_custom(conn_handle, 0x0012, (uint8_t *)log_msg, strlen(log_msg)); // 0x0012 to handle charakterystyki - trzeba ustalić prawidłowo
    return rc;
}

static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "Bluetooth NimBLE zainicjalizowany");
    // Uruchom nadawanie, ustaw urządzenie jako widoczne itp.
    // tryb GAP peripheral
    struct ble_gap_adv_params adv_params;
    ble_gap_adv_params_init(&adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                     &adv_params, ble_gap_event, NULL);
}

void app_main(void)
{
    // Konfiguracja pinu TCRT5000
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TCRT5000_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Inicjalizacja BLE NimBLE...");

    // Inicjalizacja NimBLE
    nimble_port_init();

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // Tutaj powinna być deklaracja usługi/charakterystyki, pominięta dla skrótu

    // Uruchom task obsługujący BLE
    nimble_port_freertos_init(NULL);

    ESP_LOGI(TAG, "Uruchamiam detekcję ruchu TCRT5000...");

    while (1) {
        int sensor_value = gpio_get_level(TCRT5000_PIN);

        if (sensor_value == 0) {
            ESP_LOGI(TAG, "Ruch wykryty! Stan pinu: %d", sensor_value);
            if (ble_gap_connected) {
                snprintf(ble_notify_buf, sizeof(ble_notify_buf), "Ruch wykryty! Stan pinu: %d", sensor_value);
                gatt_notify(conn_handle, ble_notify_buf);
            }
        } else {
            ESP_LOGI(TAG, "Brak ruchu. Stan pinu: %d", sensor_value);
            if (ble_gap_connected) {
                snprintf(ble_notify_buf, sizeof(ble_notify_buf), "Brak ruchu. Stan pinu: %d", sensor_value);
                gatt_notify(conn_handle, ble_notify_buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
