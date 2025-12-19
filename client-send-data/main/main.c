#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "tcrt5000.h"
#include "mpu9250.h"
#include "ds18b20-sensor.h"

static const char *TAG = "NIMBLE_CLIENT";

#define SVC_UUID_16 0x1000
#define CMD_CHAR_UUID_16 0x1001

// ... nagłówki jak wcześniej ...

static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// USTAW TE DWIE WARTOŚCI POD SERWER:
#define CMD_CHAR_HANDLE 0x0025   // handle CMD_CHAR na serwerze
#define INPUT_CHAR_HANDLE 0x0014 // handle FILE_DATA_CHAR na serwerze

// prosty parser nazwy
static void parse_adv_name(const uint8_t *data, uint8_t length_data,
                           char *out_name, size_t out_len)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    if (ble_hs_adv_parse_fields(&fields, data, length_data) != 0)
    {
        out_name[0] = '\0';
        return;
    }
    if (fields.name && fields.name_len > 0 && fields.name_len < out_len)
    {
        memcpy(out_name, fields.name, fields.name_len);
        out_name[fields.name_len] = '\0';
    }
    else
    {
        out_name[0] = '\0';
    }
}

static void start_scan(void);

static void send_frame(int accel_x, int accel_y, int accel_z, int ir_detected, float temp)
{
    char frame[128];
    int len = snprintf(frame, sizeof(frame),
                       "%d,%d,%d,%d,%.1f\n",
                       accel_x, accel_y, accel_z, ir_detected, temp);

    if (len < 0 || len >= sizeof(frame))
    {
        ESP_LOGE("CLIENT", "Frame too long!");
        return;
    }

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ble_gattc_write_flat(conn_handle, INPUT_CHAR_HANDLE,
                             frame, len, NULL, NULL);
        ESP_LOGI("CLIENT", "Sent: %s", frame);
    }
    else
    {
        ESP_LOGW("CLIENT", "No connection, frame skipped");
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI("CLIENT", "Connected, conn=%d", conn_handle);
            ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
        }
        else
        {
            ESP_LOGW("CLIENT", "Connect failed; status=%d", event->connect.status);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; restart scan");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        start_scan();
        return 0;

    case BLE_GAP_EVENT_DISC:
    {
        char name[32];
        parse_adv_name(event->disc.data, event->disc.length_data,
                       name, sizeof(name));
        if (strlen(name) > 0)
        {
            ESP_LOGI(TAG, "Found device: %s", name);
            if (strcmp(name, "ESP_LOG_SERVER") == 0)
            {
                ESP_LOGI(TAG, "Target found, connecting...");

                ble_gap_disc_cancel();

                struct ble_gap_conn_params cp = {
                    .scan_itvl = 0x0010,
                    .scan_window = 0x0010,
                    .itvl_min = 0x0010,
                    .itvl_max = 0x0020,
                    .latency = 0,
                    .supervision_timeout = 0x0100,
                    .min_ce_len = 0x0010,
                    .max_ce_len = 0x0300,
                };

                int rc = ble_gap_connect(own_addr_type, &event->disc.addr,
                                         30000, &cp, gap_event, NULL);
                if (rc != 0)
                {
                    ESP_LOGE(TAG, "ble_gap_connect rc=%d", rc);
                    start_scan();
                }
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete");
        return 0;

    default:
        return 0;
    }
}

static void start_scan(void)
{
    struct ble_gap_disc_params params;
    memset(&params, 0, sizeof(params));
    params.passive = 0;
    params.itvl = 0;
    params.window = 0;
    params.filter_duplicates = 1;

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params,
                          gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Scanning...");
    }
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "id_infer_auto rc=%d", rc);
        return;
    }
    start_scan();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{

     esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "NimBLE client started");

    ret = mpu9250_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mpu9250_init failed: %d", ret);
        // return;
    }

    tcrt5000_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "tcrt5000_init failed: %d", ret);
        // return;
    }

    ret = temp_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "temp_init failed: %d", ret);
        // return;
    }
    ESP_LOGI(TAG, "Sensors initialized");

    mpu9250_accel_t accel;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        int ir_detected = tcrt5000_read(); // 1 - wykryto, 0 - nie wykryto wody w zbiorniku

        if (ESP_OK == read_accelerometer(&accel))
        {
            ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d",
                     accel.accel_x, accel.accel_y, accel.accel_z);
        }
        read_accelerometer(&accel); // Odczytaj dane akcelerometru
        int16_t accel_x = accel.accel_x;
        int16_t accel_y = accel.accel_y;
        int16_t accel_z = accel.accel_z;

        float temp = temp_read();

        // WYŚLIJ RAMKĘ
        send_frame(accel_x, accel_y, accel_z, ir_detected, temp);
    }
}
