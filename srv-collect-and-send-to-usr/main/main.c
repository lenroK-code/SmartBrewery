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
#include "host/ble_gatt.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_pm.h"    // Power Management
#include "esp_sleep.h" // Sleep API

#include "storage.h"
#include "tcrt5000.h"
#include "calibration.h"
#include "eink.h"

static const char *TAG = "NIMBLE_SERVER";

#define SVC_UUID_16 0x1000
#define CMD_CHAR_UUID_16 0x1001
#define FILE_CHAR_UUID_16 0x1002
#define INPUT_CHAR_UUID_16 0x1003
#define CAL_NOTIFY_UUID_16 0x1004

#define FILE_CHUNK_SIZE 250
#define LOG_FILE_PATH "/spiflash/current.csv"

#define MAX_CONNS 3

static uint8_t own_addr_type;
static FILE *s_file = NULL;
static uint16_t conn_cal_handles[MAX_CONNS]; // per connection notify handles

static void snapshot_open_for_read(void)
{
    if (s_file)
    {
        storage_close_log(s_file);
        s_file = NULL;
    }
    s_file = storage_open_log_for_read(LOG_FILE_PATH); // always starts from beginning
    if (!s_file)
    {
        ESP_LOGE(TAG, "snapshot: cannot open current.csv");
    }
    else
    {
        ESP_LOGI(TAG, "snapshot: opened current.csv for read");
    }
}

// Simple helper to read successive chunks from a file
// Returns number of bytes read, or 0 on EOF
static int file_get_next_chunk(uint8_t *buf, size_t buf_len)
{
    if (!s_file)
        return 0;
    size_t n = fread(buf, 1, buf_len, s_file);
    if (n == 0)
    {
        storage_close_log(s_file);
        s_file = NULL;
    }
    return (int)n;
}

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    ESP_LOGI(TAG, "CAL_NOTIFY access: op=%d handle=0x%04x", ctxt->op, attr_handle);

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        if (attr_handle == 0x0016)
        {
            if (conn_handle < MAX_CONNS)
            {
                conn_cal_handles[conn_handle] = attr_handle;
                ESP_LOGI(TAG, "conn=%d SUBSCRIBED handle=0x%04x", conn_handle, attr_handle);
            }
            else
            {
                ESP_LOGW(TAG, "Conn handle %d out of bounds!", conn_handle);
            }
        }
    }
    return ESP_OK;
}
// INPUT_CHAR handler
// Read accelerometer data from buf, read IR sensor, combine and log
static int input_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int len = OS_MBUF_PKTLEN(ctxt->om);
    ESP_LOGI(TAG, "INPUT_CHAR access: conn=%u handle=%u op=%d len=%d",
             conn_handle, attr_handle, ctxt->op, len);

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        uint8_t buf[256];
        if (len >= (int)sizeof(buf) - 32) // More space for logs
            len = sizeof(buf) - 32;
        os_mbuf_copydata(ctxt->om, 0, len, buf);
        buf[len] = '\0'; // Null-terminate for sscanf

        // PARSE: accelX,Y,Z,temp
        int acc_xyz[3];
        float temp_c;
        if (sscanf((const char *)buf, "%d,%d,%d,%f",
                   &acc_xyz[0], &acc_xyz[1], &acc_xyz[2], &temp_c) != 4)
        {
            ESP_LOGE(TAG, "Parse failed! Expected: X,Y,Z,Temp");
            return 0;
        }

        int ir_read = tcrt5000_read(); // ir sensor value

        // convert to tilt, sg, plato
        float tilt = calculate_tilt_from_accel(acc_xyz[0], acc_xyz[1], acc_xyz[2]);
        float sg = calibrate_tilt_to_sg(tilt);
        float plato = sg_to_plato(sg);

        ESP_LOGI(TAG, "Raw: X=%d Y=%d Z=%d T=%.1fC IR=%d",
                 acc_xyz[0], acc_xyz[1], acc_xyz[2], temp_c, ir_read);
        ESP_LOGI(TAG, "SG=%.3f Plato=%.1f Tilt=%.2f", sg, plato, tilt);

        if (calibrate_get_state() == CAL_STATE_MEASURING)
        {
            measurement_add_sample(tilt); // add calibration sample
        }
        else if (calibrate_get_state() == CAL_STATE_IDLE || calibrate_get_state() == CAL_STATE_DONE)
        {
            // save to log file
            char csv_line[64];
            snprintf(csv_line, sizeof(csv_line), "%.3f,%.1f,%.1f,%d",
                     sg, plato, temp_c, ir_read);
            storage_append(LOG_FILE_PATH, csv_line);
            ESP_LOGI(TAG, "saving: '%s'", csv_line);
            eink_update_values(sg, plato, temp_c);
            ESP_LOGI(TAG, "INPUT frame: '%s'", buf);
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* CMD_CHAR handler */
// Commands:
// 0x01 - start sending current.csv from beginning
// 0x02 - delete current.csv
// 0x03 - start calibration
// 0x04 - start module calibration
// 0x05 - stop module calibration and save
static int cmd_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        if (OS_MBUF_PKTLEN(ctxt->om) == 1)
        {
            uint8_t cmd;
            os_mbuf_copydata(ctxt->om, 0, 1, &cmd);
            if (cmd == 0x01)
            {
                // start sending current.csv from beginning
                // if (s_file)
                // {
                snapshot_open_for_read();
                ESP_LOGI(TAG, "CMD 0x01: start transfer current.csv");
                // }
                // s_file = storage_open_log_for_read(LOG_FILE_PATH);
            }
            else if (cmd == 0x02)
            {
                // delete current.csv (new batch)
                if (s_file)
                {
                    storage_close_log(s_file);
                    s_file = NULL;
                }
                storage_delete_log(LOG_FILE_PATH);
                ESP_LOGI(TAG, "CMD 0x02: delete current.csv");
            }
            // *** Calibration ***
            else if (cmd == 0x03)
            {
                calibrate_start(conn_handle);
                return ESP_OK;
            }
            else if (cmd == 0x04 && calibrate_is_ready_for_module_start())
            {
                calibrate_module_start();
                return ESP_OK;
            }
            else if (cmd == 0x05 && calibrate_is_ready_for_save())
            {
                calibrate_module_stop(conn_handle);
                return ESP_OK;
            }
        }
        return ESP_OK;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* FILE_CHAR handler */
// Sends successive chunks of current.csv file
static int file_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    ESP_LOGI(TAG, "FILE_CHAR access: conn=%u handle=%u op=%d",
             conn_handle, attr_handle, ctxt->op);

    uint8_t tmp[FILE_CHUNK_SIZE];
    int len = file_get_next_chunk(tmp, sizeof(tmp));
    ESP_LOGI(TAG, "FILE_CHAR chunk len=%d", len);

    if (len > 0)
    {
        int rc = os_mbuf_append(ctxt->om, tmp, len);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

// GATT definition: 1 service, 3 characteristics
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SVC_UUID_16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                // CMD_CHAR
                .uuid = BLE_UUID16_DECLARE(CMD_CHAR_UUID_16),
                .access_cb = cmd_char_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                // FILE_CHAR
                .uuid = BLE_UUID16_DECLARE(FILE_CHAR_UUID_16),
                .access_cb = file_char_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // INPUT_CHAR
                .uuid = BLE_UUID16_DECLARE(INPUT_CHAR_UUID_16),
                .access_cb = input_char_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                // NOTIFY_CHAR
                .uuid = BLE_UUID16_DECLARE(CAL_NOTIFY_UUID_16),
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
            },
            {0} // terminator
        },
    },
    {0} // end of list
};

static void start_advertising(void);

static int active_conns = 0;
/* GAP events */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            active_conns++;
            uint16_t ch = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected conn=%d, now=%d connection(s)", ch, active_conns);

            struct ble_gap_upd_params params = {
                .itvl_min = 200,           // 250ms
                .itvl_max = 400,           // 500ms
                .latency = 4,              // Skip 4 intervals → 1s-2s effective interval
                .supervision_timeout = 500 // 5s timeout
            };
            ble_gap_update_params(ch, &params);

            if (active_conns < MYNEWT_VAL(BLE_MAX_CONNECTIONS))
            {
                start_advertising();
            }
        }
        else
        {
            ESP_LOGI(TAG, "Connect failed; restart adv");
            start_advertising();
        }
        return ESP_OK;
    case BLE_GAP_EVENT_DISCONNECT:
        active_conns--;
        if (active_conns < 0)
            active_conns = 0;
        ESP_LOGI(TAG, "Client disconnected; now %d connection(s)", active_conns);
        uint16_t ch = event->connect.conn_handle;
        if (ch < MAX_CONNS)
        {
            conn_cal_handles[ch] = 0;
            ESP_LOGI(TAG, "Cleared subscription for conn=%d", ch);
        }
        start_advertising();
        return ESP_OK;

    default:
        return ESP_OK;
    }
}

static void start_advertising(void)
{
    struct ble_gap_adv_params params;
    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.name = (uint8_t *)"ESP_LOG_SERVER";
    fields.name_len = strlen("ESP_LOG_SERVER");
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                      &params, gap_event, NULL);

    ESP_LOGI(TAG, "Advertising as ESP_LOG_SERVER");
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "id_infer_auto rc=%d", rc);
        return;
    }

    // set the name of the device (optional)
    ble_svc_gap_device_name_set("ESP_LOG_SERVER");

    start_advertising();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

int broadcast_cal_notify(uint16_t conn_handle, const char *message)
{
    uint8_t buf[64];
    int len = snprintf((char *)buf, sizeof(buf), "%s", message);

    int success_count = 0;

    for (int i = 0; i < MAX_CONNS; i++)
    {
        uint16_t handle = conn_cal_handles[i];

        // Check if handle is valid (non-zero) and optionally if connection is active
        if (handle != 0)
        {
            // Allocate a new mbuf for each notify_custom call (cannot reuse one mbuf for multiple notify_custom calls!)
            struct os_mbuf *om = ble_hs_mbuf_att_pkt();
            if (!om)
            {
                ESP_LOGE(TAG, "mbuf alloc failed for conn=%d", i);
                continue;
            }
            os_mbuf_append(om, buf, len);

            int rc = ble_gatts_notify_custom(i, handle, om);

            if (rc == 0)
            {
                ESP_LOGI(TAG, " BROADCAST to conn=%d: '%s' OK", i, message);
                success_count++;
            }
            else
            {
                ESP_LOGW(TAG, "BROADCAST fail conn=%d rc=%d", i, rc);
                // If error is e.g. ENOTCONN, we can clear the handle:
                if (rc == BLE_HS_ENOTCONN)
                    conn_cal_handles[i] = 0;
            }
        }
    }

    return (success_count > 0) ? 0 : -1;
}

static void power_management_init(void)
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 2,
        .light_sleep_enable = true};
    esp_err_t rc = esp_pm_configure(&pm_config);
    if (rc == ESP_ERR_NOT_SUPPORTED)
    {
        ESP_LOGW(TAG, "Power management not supported in this configuration; continuing without PM");
    }
    else
    {
        ESP_ERROR_CHECK(rc);
        ESP_LOGI(TAG, "Power management initialized: Light Sleep ENABLED");
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "nvs_flash_init failed: %d", ret);
    memset(conn_cal_handles, 0, sizeof(conn_cal_handles));

    power_management_init();

    storage_init();
    calibration_init();

    if (calibration_is_loaded())
    {
        ESP_LOGI(TAG, "Calibration from NVS is active!");
    }
    else
    {
        ESP_LOGI(TAG, "No calibration found - starting new");
    }

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);

    tcrt5000_init();

    ESP_LOGI(TAG, "NimBLE log server started");
    screen_init();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // test loop every 1 s
        // test calibration
        // float test_tilts[] = {-13.0f, -12.5f, 0.0f, 20.0f};
        // for (int i = 0; i < 4; i++)
        // {
        //     float sg = calibrate_tilt_to_sg(test_tilts[i]);
        //     ESP_LOGI("TEST", "Tilt=%.1f° → SG=%.3f, Plato=%.1f", test_tilts[i], sg, sg_to_plato(sg));
        // }
        // end test calibration
    }
}
