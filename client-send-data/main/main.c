#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sleep.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "tcrt5000.h"
#include "mpu9250.h"
#include "ds18b20-sensor.h"

static const char *TAG = "NIMBLE_CLIENT";

#define SVC_UUID_16 0x1000
#define CMD_CHAR_UUID_16 0x1001

static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

static bool cal_mode = false;
static const int normal_delay_ms = 5000;
static const int cal_delay_ms = 10;

const uint64_t uS_TO_S_FACTOR = 1000000ULL;
const uint64_t TIME_TO_SLEEP = 60; // how many seconds to sleep for

// static uint16_t notify_char_handle = 0;

// SET THESE TWO VALUES TO MATCH YOUR SERVER:
#define CMD_CHAR_HANDLE 0x0025        // CMD_CHAR handle on server
#define INPUT_CHAR_HANDLE 0x0014      // FILE_DATA_CHAR handle on server
#define CAL_NOTIFY_CHAR_HANDLE 0x0016 // FILE_DATA_CHAR handle on server

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

static int cccd_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg)
{
    if (error->status == 0)
    {
        ESP_LOGI(TAG, "CCCD enabled successfully for notify! (handle=0x%04x)", attr ? attr->handle : 0);
    }
    else
    {
        ESP_LOGE(TAG, "CCCD write FAILED! status=%d handle=0x%04x - check server CCCD perms/handle",
                 error->status, CAL_NOTIFY_CHAR_HANDLE + 1);
    }
    return error->status;
}

static void start_scan(void);

static void send_frame(int accel_x, int accel_y, int accel_z, float temp)
{
    char frame[128];
    int len = snprintf(frame, sizeof(frame),
                       "%d,%d,%d,%.1f",
                       accel_x, accel_y, accel_z, temp);

    if (len < 0 || len >= sizeof(frame))
    {
        ESP_LOGE("CLIENT", "Frame too long!");
        return;
    }

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ble_gattc_write_flat(conn_handle, INPUT_CHAR_HANDLE,
                             frame, len, NULL, NULL);
        ESP_LOGI("CLIENT", "Sent: %s\n", frame);
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
            uint8_t cccd_val[2] = {0x01, 0x00}; // 0x0001 little-endian
            int rc = ble_gattc_write_flat(conn_handle, CAL_NOTIFY_CHAR_HANDLE + 1,
                                          cccd_val, 2, cccd_write_cb, NULL);
            ESP_LOGI(TAG, "CCCD write queued rc=%d handle=0x%04x", rc, CAL_NOTIFY_CHAR_HANDLE + 1);

            ble_gattc_read(conn_handle, CAL_NOTIFY_CHAR_HANDLE, NULL, NULL);
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

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
        ESP_LOGI(TAG, "NOTIFY RX attr=0x%04x len=%d", event->notify_rx.attr_handle,
                 OS_MBUF_PKTLEN(event->notify_rx.om));
        char msg[32];
        int msg_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (msg_len >= (int)sizeof(msg))
            msg_len = sizeof(msg) - 1;

        os_mbuf_copydata(event->notify_rx.om, 0, msg_len, msg);
        msg[msg_len] = '\0';

        ESP_LOGI(TAG, "'%s'", msg);

        if (strcmp(msg, "CAL_START") == 0)
        {
            cal_mode = true;
            ESP_LOGI(TAG, "CAL ON → 10ms!");
        }
        else if (strcmp(msg, "CAL_DONE") == 0)
        {
            cal_mode = false;
            ESP_LOGI(TAG, "CAL OFF → 5000ms");
        }
        return 0;
    }

    default:
        ESP_LOGI(TAG, "GAP event=%d", event->type); // Show unhandled events for debugging
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
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
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

    ret = temp_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "temp_init failed: %d", ret);
    }
    ESP_LOGI(TAG, "Sensors initialized");

    mpu9250_accel_t accel;
    while (1)
    {
        for (uint8_t num_of_reads = 0; num_of_reads < 5; num_of_reads++)
        {
            if (ESP_OK == read_accelerometer(&accel))
            {
                ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d",
                         accel.accel_x, accel.accel_y, accel.accel_z);
            }
            int16_t accel_x = accel.accel_x;
            int16_t accel_y = accel.accel_y;
            int16_t accel_z = accel.accel_z;

            float temp = temp_read(); // read temperature from DS18B20

            send_frame(accel_x, accel_y, accel_z, temp); // send data to server
            ESP_LOGI(TAG, "Temp: %.1f C", temp);
            int delay_ms = cal_mode ? cal_delay_ms : normal_delay_ms;
            ESP_LOGI(TAG, "DELAY: %d ms", delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
        if (!cal_mode)
        {
            ESP_LOGI(TAG, "Entering deep sleep for %d seconds...", TIME_TO_SLEEP);
            esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
            esp_deep_sleep_start();
        }
    }
}
