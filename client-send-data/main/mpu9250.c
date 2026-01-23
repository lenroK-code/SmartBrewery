#include "mpu9250.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU9250";
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t i2c_dev = NULL;

#define MPU9250_ACCEL_CONFIG_REG 0x1C

esp_err_t mpu9250_init(void)
{

    // 1. Konfiguracja BUS (PINY W BUS_CONFIG!)
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,           // ← port number
        .sda_io_num = I2C_MASTER_SDA_IO, // ← SDA pin
        .scl_io_num = I2C_MASTER_SCL_IO, // ← SCL pin
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0, // auto
        // .trans_queue_depth = 4,
        .flags = {
            .enable_internal_pullup = true, // ← internal pullup
            // .allow_pd = false,
        },
    };


    // 2. TWORZENIE BUS
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_LOGI(TAG, "I2C bus created");

    // 3. DODAWANIE URZĄDZENIA
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 1000, // timeout us
        .flags = {
            .disable_ack_check = false,
        },
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &i2c_dev));
    ESP_LOGI(TAG, "MPU9250 device added");

    uint8_t cmd[2];

    // Reset
    cmd[0] = MPU9250_PWR_MGMT_1_REG;
    cmd[1] = 1 << MPU9250_RESET_BIT; // Bit 7
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, cmd, 2, 1000));
        vTaskDelay(pdMS_TO_TICKS(100));  // Czekaj na reset


    // Wake-up + wybierz zegar gyro Z
    cmd[0] = MPU9250_PWR_MGMT_1_REG;
    cmd[1] = 0x00; // CLKSEL=0b000 (auto), SLEEP=0
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, cmd, 2, 1000));
        vTaskDelay(pdMS_TO_TICKS(100));  // stabilizacja po wybudzeniu

    // KALIBRACJA AKCELEROMETRU ±2g
    cmd[0] = MPU9250_ACCEL_CONFIG_REG; // 0x1C
    cmd[1] = 0x00;                     // AFS_SEL=0 (±2g)
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, cmd, 2, 1000));

    ESP_LOGI(TAG, "MPU9250 calibrated ±2g");
    // koniec kalibracji

    // 4. SPRAWDZENIE TOŻSAMOŚCI
    uint8_t reg_addr = MPU9250_WHO_AM_I_REG;
    uint8_t who_am_i;
    esp_err_t ret = i2c_master_transmit_receive(i2c_dev, &reg_addr, 1, &who_am_i, 1, 1000);
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who_am_i);
    if (ret != ESP_OK)
        return ret;

    // vTaskDelay(pdMS_TO_TICKS(1000));

    ret = i2c_master_receive(i2c_dev, &who_am_i, 1, 1000);
    // if (ret != ESP_OK)
    //     return ret;

    ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who_am_i);

    // GY-9250 = 0x71, MPU6500 = 0x70
    if (who_am_i != 0x71 && who_am_i != 0x70)
    {
        ESP_LOGE(TAG, "MPU not found! Expected 0x70/0x71, got 0x%02X", who_am_i);
        // return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MPU%s DETECTED!", (who_am_i == 0x71) ? "9250" : "6500");
    return ESP_OK;
}

esp_err_t read_accelerometer(mpu9250_accel_t *accel)
{
    // Register read: write reg addr + read 6 bytes
    uint8_t reg_addr = MPU9250_ACCEL_XOUT_H;
    uint8_t raw_data[6];

    esp_err_t ret = i2c_master_transmit_receive(i2c_dev,
                                                &reg_addr, 1, // write register address
                                                raw_data, 6,  // read 6 bytes
                                                1000);        // timeout ms
    ESP_LOGD(TAG, "I2C transmit_receive result: %s", esp_err_to_name(ret));

    // if (ret == ESP_OK)
    // {
        accel->accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        accel->accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        accel->accel_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);
        ESP_LOGD(TAG, "Accel read: X=%d, Y=%d, Z=%d",
                 accel->accel_x, accel->accel_y, accel->accel_z);
        return ESP_OK;
    // }

    accel->accel_x = accel->accel_y = accel->accel_z = 0;
    ESP_LOGE(TAG, "Accel read failed: %s", esp_err_to_name(ret));
    return ret;
}