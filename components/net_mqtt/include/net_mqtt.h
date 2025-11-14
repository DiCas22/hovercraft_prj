#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa um cliente MQTT interno (URI e keepalive)
esp_err_t net_mqtt_init(const char* uri, int keepalive_s);

// Publica um payload textual (JSON ou não) no tópico informado
esp_err_t net_mqtt_publish(const char* topic, const char* payload, int qos, bool retain);

#ifdef __cplusplus
}
#endif
