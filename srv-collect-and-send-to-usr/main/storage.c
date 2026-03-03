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

void storage_append(const char *path_to_file, const char *line)
{
    if (s_log_mutex)
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);

    FILE *f = fopen(path_to_file, "a");
    if (!f)
    {
        ESP_LOGE(TAG_FS, "open %s failed", path_to_file);
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

FILE *storage_open_log_for_read(const char *path_to_file)
{
    if (s_log_mutex)
        xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    // WARNING: we do not release it here – the reader holds it until done
    return fopen(path_to_file, "r");
}

void storage_close_log(FILE *f)
{
    if (f)
        fclose(f);
    if (s_log_mutex)
        xSemaphoreGive(s_log_mutex);
}

void storage_delete_log(const char *path_to_file)
{
    remove(path_to_file);
}

void storage_dump_log_to_uart(const char *path_to_file)
{
    FILE *f = storage_open_log_for_read(path_to_file);
    if (!f)
    {
        // ESP_LOGE(TAG_FS, "cannot open /spiflash/current.csv for read");
        ESP_LOGE(TAG_FS, "cannot open %s for read", path_to_file);
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), f))
    {
        ESP_LOGI(TAG_FS, "LOG: %s", line);
    }

    fclose(f);
}
