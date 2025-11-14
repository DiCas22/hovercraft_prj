#include "imu_mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B

static const char *TAG = "imu";
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

esp_err_t imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    uint8_t wake[2] = {REG_PWR_MGMT_1, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev, wake, sizeof(wake), 100));

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "MPU6050 ready");
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(dev, &reg, 1, buf, sizeof(buf), 100), TAG, "i2c xfer");

    int16_t ax = (int16_t)((buf[0]<<8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]<<8) | buf[3]);
    int16_t az = (int16_t)((buf[4]<<8) | buf[5]);
    int16_t gx = (int16_t)((buf[8]<<8) | buf[9]);
    int16_t gy = (int16_t)((buf[10]<<8) | buf[11]);
    int16_t gz = (int16_t)((buf[12]<<8) | buf[13]);

    out->ax = ax / 16384.0f;
    out->ay = ay / 16384.0f;
    out->az = az / 16384.0f;
    out->gx = gx / 131.0f;
    out->gy = gy / 131.0f;
    out->gz = gz / 131.0f;
    return ESP_OK;
}
