#include "net_mqtt.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_check.h" 
#include <string.h>

static const char* TAG = "net_mqtt";
static esp_mqtt_client_handle_t s_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args; (void)base; (void)event_data; // silencia "unused"
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "connected");
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "disconnected");
    }
}

esp_err_t net_mqtt_init(const char* uri, int keepalive_s)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .session.keepalive  = keepalive_s,
    };
    s_client = esp_mqtt_client_init(&cfg);
    ESP_RETURN_ON_FALSE(s_client != NULL, ESP_FAIL, TAG, "client init failed");
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    return esp_mqtt_client_start(s_client);
}

esp_err_t net_mqtt_publish(const char* topic, const char* payload, int qos, bool retain)
{
    if (!s_client) return ESP_ERR_INVALID_STATE;
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}
