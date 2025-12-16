#include "storage.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_log_mutex;

static const char *TAG_FS = "FS";
static wl_handle_t s_wl_handle;

void storage_init(void)
{
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true,
        .allocation_unit_size = 4096};

    esp_err_t ret = esp_vfs_fat_spiflash_mount_rw_wl(
        "/spiflash", "storage",
        &mount_config, &s_wl_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_FS, "mount failed: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG_FS, "FATFS on /spiflash ready");
    }
}

void storage_append(const char *line)
{
    // TODO: tu wstawisz realne dane z czujników
    // const char *line = "12345,25.5,10,-3,1000,1,3.7\n";

    if (s_log_mutex)
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);

    FILE *f = fopen("/spiflash/current.csv", "a");
    if (!f)
    {
        ESP_LOGE(TAG_FS, "open current.csv failed");
    }
    else
    {
        fputs(line, f);
        fputc('\n', f);
        fclose(f);
    }

    if (s_log_mutex)
        xSemaphoreGive(s_log_mutex);
}

FILE *storage_open_log_for_read(void)
{
    if (s_log_mutex)
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    // UWAGA: nie zwalniamy tutaj – trzyma go czytający, aż skończy
    return fopen("/spiflash/current.csv", "r");
}

void storage_close_log(FILE *f)
{
    if (f)
        fclose(f);
    if (s_log_mutex)
        xSemaphoreGive(s_log_mutex);
}

void storage_delete_log(void)
{
    remove("/spiflash/current.csv");
}

void storage_dump_log_to_uart(void)
{
    FILE *f = storage_open_log_for_read();
    if (!f)
    {
        ESP_LOGE(TAG_FS, "cannot open /spiflash/current.csv for read");
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), f))
    {
        // linia zwykle ma już '\n', więc nie trzeba doklejać
        ESP_LOGI(TAG_FS, "LOG: %s", line);
    }

    fclose(f);
}
