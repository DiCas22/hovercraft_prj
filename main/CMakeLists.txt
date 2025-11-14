idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES driver esp_wifi esp_event nvs_flash freertos esp_timer mqtt
)
