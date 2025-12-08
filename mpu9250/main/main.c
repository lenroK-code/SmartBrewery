#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6500_ADDR 0x68
#define MPU6500_WHO_AM_I_REG 0x75
#define MPU6500_PWR_MGMT_1_REG 0x6B
#define MPU6500_RESET_BIT 7

#define MPU6500_ACCEL_XOUT_H 0x3B

static const char* TAG = "MPU6500";

// Inicjalizacja I2C (jak wcześniej)
esp_err_t i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t mpu6500_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6500_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6500_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6500_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6500_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Funkcja do odczytu i konwersji wartości akcelerometru
static void read_accelerometer(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
    uint8_t raw_data[6];
    if (mpu6500_register_read(MPU6500_ACCEL_XOUT_H, raw_data, 6) == ESP_OK) {
        *accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        *accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        *accel_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);
    } else {
        *accel_x = *accel_y = *accel_z = 0;
        ESP_LOGE(TAG, "Failed to read accelerometer data");
    }
}

void app_main()
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    ESP_LOGI(TAG, "I2C master initialized");

    uint8_t who_am_i = 0;
    ret = mpu6500_register_read(MPU6500_WHO_AM_I_REG, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != 0x70) {
        ESP_LOGE(TAG, "Failed to detect MPU6500, WHO_AM_I=0x%02X", who_am_i);
        return;
    }
    ESP_LOGI(TAG, "MPU6500 detected, WHO_AM_I=0x%02X", who_am_i);

    // Reset urządzenia
    mpu6500_register_write_byte(MPU6500_PWR_MGMT_1_REG, 1 << MPU6500_RESET_BIT);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        int16_t ax, ay, az;
        read_accelerometer(&ax, &ay, &az);
        ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d", ax, ay, az);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
