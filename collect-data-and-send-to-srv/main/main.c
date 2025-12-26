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

#include "storage.h"
#include "tcrt5000.h"
#include "calibration.h"

void get_acceleremoter_from_buf(uint8_t*, size_t, float*);

static const char *TAG = "NIMBLE_SERVER";

#define SVC_UUID_16 0x1000
#define CMD_CHAR_UUID_16 0x1001
#define FILE_CHAR_UUID_16 0x1002
#define INPUT_CHAR_UUID_16 0x1003

#define FILE_CHUNK_SIZE 250

#define LOG_FILE_PATH "/spiflash/current.csv" 

static uint8_t own_addr_type; 
static FILE *s_file = NULL;

// Calibration specific
typedef struct {
    float sg;      // specific gravity
    float reading; // raw sensor reading
} CalPoint;

static const float sg_table[6] = {
    1.000, 1.050, 1.040, 1.030, 1.020, 1.010  // z Twojego CSV [file:35]
};

static int cal_state = CAL_STATE_IDLE;
static int cal_point = 0;  // 0-6 punktów
static CalPoint points[7]; // z poprzedniego kodu

//-----------------------------------
// DEBUG function: prints first N bytes of log file to ESP log
// to be removed
static void snapshot_debug(void)
{
    FILE *f = fopen("/spiflash/current.csv", "r");
    if (!f) {
        ESP_LOGE(TAG, "DEBUG: cannot open current.csv");
        return;
    }
    char buf[64];
    size_t n = fread(buf, 1, sizeof(buf)-1, f);
    buf[n] = '\0';
    ESP_LOGI(TAG, "DEBUG: first %d bytes: \n%s", (int)n, buf);
    fclose(f);
}
//-----------------------------------


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
        if (len >= (int)sizeof(buf)-3)
            len = sizeof(buf) - 4;
        os_mbuf_copydata(ctxt->om, 0, len, buf);

        int ir_read = tcrt5000_read();
        // insert gravity reading here 
        float acc_xyz[3];
        get_acceleremoter_from_buf(buf, len, acc_xyz); //this has to be integrated into another function later
        ESP_LOGI(TAG, "Accelerometer data: X=%.2f, Y=%.2f, Z=%.2f", acc_xyz[0], acc_xyz[1], acc_xyz[2]);

        int written = snprintf((char *)&buf[len], sizeof(buf) - len, ",%d", ir_read);
        if (written < 0 || (size_t)written >= sizeof(buf) - len)
        {
            buf[sizeof(buf)-1] = '\0'; // just in case
        } else {
            len += written;
            buf[len] = '\0';
        }


        ESP_LOGI(TAG, "INPUT frame: '%s'", buf);

        // UNCOMMENT TO SAVE TO LOG FILE
// storage_append(LOG_FILE_PATH,(const char *)buf); // save to file
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* CMD_CHAR handler */
// Commands:
// 0x01 - start sending current.csv from beginning
// 0x02 - delete current.csv
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
                    // snapshot_debug(); // do usuniecia
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
            if (cmd == 0x03) {
                if (cal_state == CAL_STATE_IDLE) {
                    cal_state = CAL_STATE_READY;
                    cal_point = 0;
                    ESP_LOGI(TAG, "CAL: START - punkt %d/6 (SG=%.3f)", 
                             cal_point+1, sg_table[cal_point]);
                }
                return ESP_OK;
            }
            else if (cmd == 0x04) {
                if (cal_state == CAL_STATE_READY) {
                    cal_state = CAL_STATE_MEASURING;
                    ESP_LOGI(TAG, "CAL: POMIAR punktu %d/6...", cal_point+1);
                    // Start timera pomiaru (patrz niżej)
                    calibrate_module_start();
                }
                return ESP_OK;
            }
            else if (cmd == 0x05) {
                if (cal_state == CAL_STATE_MEASURING) {
                    // Zapisz punkt i przejdź dalej
                    calibrate_module_stop();
                    cal_state = CAL_STATE_SAVED;
                    ESP_LOGI(TAG, "CAL: ZAPISANO punkt %d/6", cal_point+1);
                    
                    cal_point++;
                    if (cal_point < 6) {
                        // Następny punkt
                        cal_state = CAL_STATE_READY;
                        ESP_LOGI(TAG, "CAL: NASTĘPNY %d/6 (SG=%.3f)", 
                                 cal_point+1, sg_table[cal_point]);
                    } else {
                        // KONIEC - oblicz wielomian!
                        cal_state = CAL_STATE_IDLE;
                        calculate_polynomial(); // i thnk so at least
                        ESP_LOGI(TAG, "CAL: KONIEC! Wielomian gotowy!");
                    }
                }
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
            ESP_LOGI(TAG, "Connected, now=%d connection(s)", active_conns);
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

void get_acceleremoter_from_buf(uint8_t *buf, size_t len, float *acc_xyz) {
        int parsed = sscanf((const char *)buf, "%f,%f,%f", &acc_xyz[0], &acc_xyz[1], &acc_xyz[2]);
        if (parsed != 3) {
            ESP_LOGE(TAG, "Error parsing accelerometer data from buffer");
            acc_xyz[0] = 0.0f;
            acc_xyz[1] = 0.0f;
            acc_xyz[2] = 0.0f;
            return;
        }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "nvs_flash_init failed: %d", ret);

    storage_init();

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "NimBLE log server started");
    // tcrt5000_init();

    while (1)
    {
        // storage_append();          // tu docelowo prawdziwe dane
        // storage_dump_log_to_uart(); // testowo wypisz log do UART
        vTaskDelay(pdMS_TO_TICKS(10000)); // testowo co 10 s
    }
}
