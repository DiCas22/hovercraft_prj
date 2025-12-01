idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        I2Cdev
        MPU6050
        mqtt
        esp_wifi
        esp_event
        nvs_flash
        driver
)
