// main/hovercraft_main.c

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "driver/i2c.h"
#include "mqtt_client.h"
#include "driver/ledc.h"

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define WIFI_SSID         "BLFIBRA_AP703-2.4G"
#define WIFI_PASS         "olhanomodem"
#define MQTT_URI          "mqtt://broker.hivemq.com:1883"
#define MQTT_TOPIC_PUB    "hovercraft/imu"
#define MQTT_TOPIC_SUB    "hovercraft/cmd"
#define PUB_HZ            20
#define ESC_GPIO          25
#define SERVO_GPIO        26

#define ESC_PWM_MIN_US    1000
#define ESC_PWM_MAX_US    2000
#define SERVO_PWM_MIN_US  1000
#define SERVO_PWM_MAX_US  2000

#define WIFI_CONNECTED_BIT   BIT0
#define MQTT_CONNECTED_BIT   BIT1

static EventGroupHandle_t s_app_events;
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_ok = false;
static char s_client_id[64];

/* ==== ESC / SERVO ==== */
#define ESC_LEDC_TIMER     LEDC_TIMER_0
#define ESC_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define ESC_LEDC_CHANNEL   LEDC_CHANNEL_0
#define ESC_LEDC_DUTY_RES  LEDC_TIMER_16_BIT
#define ESC_PWM_FREQ       50

#define SERVO_LEDC_TIMER     LEDC_TIMER_1
#define SERVO_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL   LEDC_CHANNEL_1
#define SERVO_LEDC_DUTY_RES  LEDC_TIMER_16_BIT
#define SERVO_PWM_FREQ       50

static void esc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = ESC_LEDC_MODE,
        .duty_resolution  = ESC_LEDC_DUTY_RES,
        .timer_num        = ESC_LEDC_TIMER,
        .freq_hz          = ESC_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t chan = {
        .gpio_num   = ESC_GPIO,
        .speed_mode = ESC_LEDC_MODE,
        .channel    = ESC_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = ESC_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan));
}

static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_DUTY_RES,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = SERVO_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t chan = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan));
}

static void pwm_write_us(ledc_channel_t channel, ledc_mode_t mode, uint32_t pulse_us, uint32_t freq, uint32_t duty_res)
{
    const uint32_t max_duty = (1U << duty_res);
    uint64_t duty = (uint64_t)pulse_us * freq * max_duty / 1000000ULL;
    if (duty >= max_duty) duty = max_duty - 1;
    ESP_ERROR_CHECK(ledc_set_duty(mode, channel, (uint32_t)duty));
    ESP_ERROR_CHECK(ledc_update_duty(mode, channel));
}

static void esc_write_us(uint32_t pulse_us)
{
    if (pulse_us < ESC_PWM_MIN_US) pulse_us = ESC_PWM_MIN_US;
    if (pulse_us > ESC_PWM_MAX_US) pulse_us = ESC_PWM_MAX_US;
    pwm_write_us(ESC_LEDC_CHANNEL, ESC_LEDC_MODE, pulse_us, ESC_PWM_FREQ, ESC_LEDC_DUTY_RES);
}

static void servo_write_us(uint32_t pulse_us)
{
    if (pulse_us < SERVO_PWM_MIN_US) pulse_us = SERVO_PWM_MIN_US;
    if (pulse_us > SERVO_PWM_MAX_US) pulse_us = SERVO_PWM_MAX_US;
    pwm_write_us(SERVO_LEDC_CHANNEL, SERVO_LEDC_MODE, pulse_us, SERVO_PWM_FREQ, SERVO_LEDC_DUTY_RES);
}

/* ========== WiFi / MQTT ========== */
// (copie as funções de inicialização WiFi e MQTT do seu código — são as mesmas, igual ao bloco anterior)

// MQTT subscriber handler
static void mqtt_cmd_handler(const char *data, int len)
{
    // Espera string tipo: {"motor_us":1300,"servo_us":1500}
    uint32_t motor_us = 1000, servo_us = 1500;
    sscanf(data, "{\"motor_us\":%u,\"servo_us\":%u}", &motor_us, &servo_us);
    esc_write_us(motor_us);
    servo_write_us(servo_us);
    ESP_LOGI("MQTT_CMD", "CMD: motor_us=%u, servo_us=%u", motor_us, servo_us);
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_ok = true;
        xEventGroupSetBits(s_app_events, MQTT_CONNECTED_BIT);
        esp_mqtt_client_subscribe(s_mqtt, MQTT_TOPIC_SUB, 0);
        ESP_LOGI("MQTT", "Connected and subscribed.");
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_ok = false;
        xEventGroupClearBits(s_app_events, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DATA:
        mqtt_cmd_handler(event->data, event->data_len);
        break;
    default:
        break;
    }
}

static void mqtt_publish_json(const char *json)
{
    if (!s_mqtt_ok) return;
    static int64_t last_us = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_us < (1000000 / PUB_HZ)) return;
    last_us = now;
    int mid = esp_mqtt_client_publish(s_mqtt, MQTT_TOPIC_PUB, json, 0, 0, 0);
    if (mid < 0) ESP_LOGW("MQTT", "publish failed (%d)", mid);
}

/* ========== MPU6050 DMP ========== */
static MPU6050 mpu;
static bool dmpReady = false;
static uint8_t fifoBuffer[64];

static void mpu6050_dmp_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(100)); // ESP32 i2c+MPU needs tempo extra

    // Inicia sensor
    mpu.initialize();
    if (!mpu.testConnection()) {
        ESP_LOGE("MPU", "MPU6050 not connected!");
        vTaskDelete(NULL); }
    if (mpu.dmpInitialize() != 0) {
        ESP_LOGE("MPU", "DMP init failed!");
        vTaskDelete(NULL); }
    mpu.setDMPEnabled(true);
    dmpReady = true;
    ESP_LOGI("MPU", "MPU DMP Ready");

    uint32_t t0_ms = (uint32_t)(esp_timer_get_time() / 1000);

    Quaternion q;
    VectorFloat gravity;
    VectorInt16 aa, aaReal, aaWorld;
    float ypr[3];

    while (1) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

            float ax = aaReal.x / 16384.0f;
            float ay = aaReal.y / 16384.0f;
            float az = aaReal.z / 16384.0f;
            float yaw = ypr[0], pitch = ypr[1], roll = ypr[2];

            uint32_t t_ms = (uint32_t)(esp_timer_get_time()/1000) - t0_ms;
            char json[200];
            snprintf(json, sizeof(json),
                "{\"t_ms\":%lu,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
                "\"yaw\":%.3f,\"pitch\":%.3f,\"roll\":%.3f}",
                (unsigned long)t_ms, ax, ay, az, yaw, pitch, roll);
            mqtt_publish_json(json);
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / PUB_HZ));
    }
}

void app_main(void)
{
    // Init NVS + WiFi/MQTT (copie igual seu código base)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    s_app_events = xEventGroupCreate();
    wifi_init_sta();
    wait_for_ip();
    mqtt_start();

    // ESC/Servo
    esc_init();
    servo_init();

    xTaskCreatePinnedToCore(mpu6050_dmp_task, "mpu6050_dmp_task", 8192, NULL, 6, NULL, 1);
    ESP_LOGI("MAIN", "Setup complete!");
}
