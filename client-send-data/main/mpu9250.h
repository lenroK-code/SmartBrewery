#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct
{
    int16_t accel_x, accel_y, accel_z;
} mpu9250_accel_t;

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 100000
#define MPU9250_ADDR 0x68
#define MPU9250_WHO_AM_I_REG 0x75
#define MPU9250_PWR_MGMT_1_REG 0x6B
#define MPU9250_RESET_BIT 7
#define MPU9250_ACCEL_XOUT_H 0x3B

esp_err_t mpu9250_init(void);
esp_err_t read_accelerometer(mpu9250_accel_t *accel);
