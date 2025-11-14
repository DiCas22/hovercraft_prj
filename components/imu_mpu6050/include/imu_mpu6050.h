#pragma once
#include "esp_err.h"

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} imu_data_t;

esp_err_t imu_init(void);
esp_err_t imu_read(imu_data_t *out);
