// main/main.cpp

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "esp_wifi.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

// Jeff Rowberg libs (seus componentes)
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"


#include "freertos/semphr.h"

// --- IMU shared state ---
static SemaphoreHandle_t s_imu_mutex = nullptr;

static float s_yaw   = 0.0f;
static float s_pitch = 0.0f;
static float s_roll  = 0.0f;

// Se ainda não tiver:
static MPU6050 mpu;


static const char *TAG = "HOVERCRAFT";

// ============================
// CONFIGURAÇÕES DE HARDWARE
// ============================

// ESC e SERVO – ajuste para os pinos que você realmente está usando
#define ESC_PWM_GPIO        GPIO_NUM_18
#define SERVO_PWM_GPIO      GPIO_NUM_19

// PWM (50 Hz para ESC/servo padrão)
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_HIGH_SPEED_MODE
#define PWM_ESC_CHANNEL     LEDC_CHANNEL_0
#define PWM_SERVO_CHANNEL   LEDC_CHANNEL_1
#define PWM_FREQ_HZ         50
#define PWM_DUTY_RES        LEDC_TIMER_16_BIT
#define PWM_PERIOD_US       (1000000 / PWM_FREQ_HZ)

// I2C – ajuste SDA/SCL se necessário
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ  400000

// ============================
// CONFIG DE REDE / MQTT
// ============================

// *** AJUSTE ESSAS STRINGS ***
#define WIFI_SSID           "BLFIBRA_AP703-2.4G"
#define WIFI_PASS           "olhanomodem"

#define MQTT_URI            "mqtt://broker.hivemq.com:1883" 

#define MQTT_TOPIC_CMD      "hovercraft/cmd"
#define MQTT_TOPIC_IMU      "hovercraft/imu"

// Event group pro Wi-Fi
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static int s_retry_num = 0;

// MQTT client handle
static esp_mqtt_client_handle_t s_mqtt_client = nullptr;

// Últimos comandos usados (pra log e telemetria)
static uint32_t s_last_motor_us = 1000;
static uint32_t s_last_servo_us = 1500;

// ============================
// HELPERS GERAIS
// ============================

static void check_err(esp_err_t err, const char *msg)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", msg, esp_err_to_name(err));
    }
}

// ============================
// INICIALIZAÇÃO DO I2C
// ============================

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "I2C master init OK (SDA=%d, SCL=%d, Freq=%d Hz)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

// I2C helper
static esp_err_t i2c_read_reg(uint8_t dev_addr,
                              uint8_t reg_addr,
                              uint8_t *data,
                              size_t len)
{
    if (len == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err;

    // Write: [S] [ADDR_W] [REG]
    err = i2c_master_start(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) goto cleanup;

    // Restart + read
    err = i2c_master_start(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) goto cleanup;

    if (len > 1) {
        err = i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        if (err != ESP_OK) goto cleanup;
    }
    err = i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    if (err != ESP_OK) goto cleanup;

    err = i2c_master_stop(cmd);
    if (err != ESP_OK) goto cleanup;

    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));

cleanup:
    i2c_cmd_link_delete(cmd);
    return err;
}


// ============================
// PWM PARA ESC E SERVO
// ============================

static uint32_t pwm_usec_to_duty(uint32_t us)
{
    const uint32_t max_duty = (1u << PWM_DUTY_RES) - 1u;
    return static_cast<uint32_t>((static_cast<uint64_t>(us) * max_duty) / PWM_PERIOD_US);
}

static void pwm_init(void)
{
    ledc_timer_config_t timer_conf = {};
    timer_conf.duty_resolution = PWM_DUTY_RES;
    timer_conf.freq_hz = PWM_FREQ_HZ;
    timer_conf.speed_mode = PWM_MODE;
    timer_conf.timer_num = PWM_TIMER;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t esc_ch = {};
    esc_ch.channel = PWM_ESC_CHANNEL;
    esc_ch.duty = pwm_usec_to_duty(s_last_motor_us);
    esc_ch.gpio_num = ESC_PWM_GPIO;
    esc_ch.speed_mode = PWM_MODE;
    esc_ch.hpoint = 0;
    esc_ch.timer_sel = PWM_TIMER;
    ESP_ERROR_CHECK(ledc_channel_config(&esc_ch));

    ledc_channel_config_t servo_ch = {};
    servo_ch.channel = PWM_SERVO_CHANNEL;
    servo_ch.duty = pwm_usec_to_duty(s_last_servo_us);
    servo_ch.gpio_num = SERVO_PWM_GPIO;
    servo_ch.speed_mode = PWM_MODE;
    servo_ch.hpoint = 0;
    servo_ch.timer_sel = PWM_TIMER;
    ESP_ERROR_CHECK(ledc_channel_config(&servo_ch));

    ESP_LOGI(TAG, "PWM init OK (ESC on GPIO %d, SERVO on GPIO %d)",
             ESC_PWM_GPIO, SERVO_PWM_GPIO);
}

static void esc_write_us(uint32_t us)
{
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    uint32_t duty = pwm_usec_to_duty(us);
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_ESC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_ESC_CHANNEL));

    if (us != s_last_motor_us) {
        ESP_LOGI(TAG, "ESC command: %u us", us);
        s_last_motor_us = us;
    }
}

static void servo_write_us(uint32_t us)
{
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    uint32_t duty = pwm_usec_to_duty(us);
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_SERVO_CHANNEL));

    if (us != s_last_servo_us) {
        ESP_LOGI(TAG, "SERVO command: %u us", us);
        s_last_servo_us = us;
    }
}

// ============================
// WIFI STA
// ============================

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "retry to connect to the AP (%d)", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGW(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, &instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Connecting to SSID:%s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP");
    } else {
        ESP_LOGE(TAG, "Failed to connect to AP");
    }
}

// ============================
// MQTT: handler de comandos
// ============================

static void mqtt_cmd_handler(const esp_mqtt_event_handle_t event)
{
    // Copia e termina a string
    char buf[128];
    int len = event->data_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, event->data, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "MQTT CMD payload: %s", buf);

    // Usa unsigned int para casar com %u
    unsigned int motor_us = s_last_motor_us;
    unsigned int servo_us = s_last_servo_us;

    int matched = sscanf(buf, "{\"motor_us\":%u,\"servo_us\":%u}", &motor_us, &servo_us);
    if (matched == 2) {
        esc_write_us(motor_us);
        servo_write_us(servo_us);
        ESP_LOGI(TAG, "Applied CMD: motor=%u us, servo=%u us", motor_us, servo_us);
    } else {
        ESP_LOGW(TAG, "Failed to parse CMD payload");
    }
}

// ============================
// MQTT EVENT HANDLER
// ============================

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, MQTT_TOPIC_CMD, 0);
        ESP_LOGI(TAG, "Subscribed to %s", MQTT_TOPIC_CMD);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA topic=%.*s",
                 event->topic_len, event->topic);
        if (strncmp(event->topic, MQTT_TOPIC_CMD, event->topic_len) == 0) {
            mqtt_cmd_handler(event);
        }
        break;

    default:
        break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_URI;
    mqtt_cfg.credentials.client_id = "hovercraft-esp32";

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_mqtt_client,
        static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        mqtt_event_handler,
        nullptr));
    
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));

    ESP_LOGI(TAG, "MQTT client started (URI: %s)", MQTT_URI);
}

// ============================
// TASK DO MPU6050 + DMP
// ============================

static void mpu6050_dmp_task(void *pvParameters)
{
    // Pequeno delay para tudo (Wi-Fi, I2C, etc) estabilizar
    vTaskDelay(pdMS_TO_TICKS(500));

    // 1) Teste cru do WHO_AM_I diretamente via driver I2C do ESP-IDF
    ESP_LOGI(TAG, "Iniciando teste cru do WHO_AM_I no barramento I2C...");

    uint8_t who_68 = 0, who_69 = 0;
    esp_err_t e68 = i2c_read_reg(0x68, 0x75, &who_68, 1);  // WHO_AM_I @ 0x75
    esp_err_t e69 = i2c_read_reg(0x69, 0x75, &who_69, 1);

    ESP_LOGI(TAG, "WHO_AM_I 0x68 -> err=%s (%d), val=0x%02X",
             esp_err_to_name(e68), (int)e68, who_68);
    ESP_LOGI(TAG, "WHO_AM_I 0x69 -> err=%s (%d), val=0x%02X",
             esp_err_to_name(e69), (int)e69, who_69);

    // 2) Lib Jeff Rowberg
    ESP_LOGI(TAG, "Inicializando MPU6050 (lib Jeff Rowberg)...");
    mpu.initialize();

    bool ok = mpu.testConnection();
    ESP_LOGI(TAG, "mpu.testConnection() = %s", ok ? "true" : "false");

    if (!ok) {
        ESP_LOGE(TAG, "MPU6050 nao conectado (testConnection falhou)!");
        vTaskDelete(nullptr);
        return;
    }

    // 3) Inicializa o DMP
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        ESP_LOGE(TAG, "Falha ao inicializar DMP (code %d)", devStatus);
        vTaskDelete(nullptr);
        return;
    }

    // *** Reduz taxa de amostragem para ~50 Hz ***
    // 1 kHz / (1 + 19) = 50 Hz
    mpu.setRate(19);

    mpu.setDMPEnabled(true);
    ESP_LOGI(TAG, "DMP habilitado.");

    uint16_t packetSize = mpu.dmpGetFIFOPacketSize();
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    // Garante FIFO limpo antes de começar
    mpu.resetFIFO();

    while (true) {

        uint16_t fifoCount = mpu.getFIFOCount();

        // Overflow → reseta FIFO
        if (fifoCount == 1024) {
            ESP_LOGW(TAG, "FIFO overflow, resetando...");
            mpu.resetFIFO();
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Ainda não tem pacote inteiro
        if (fifoCount < packetSize) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Se tiver mais de um pacote no FIFO,
        // vamos ler todos e ficar só com o mais recente.
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        // Converte para Yaw/Pitch/Roll (radianos)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Atualiza variáveis globais protegidas
        if (s_imu_mutex != nullptr) {
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s_yaw   = ypr[0];  // rad
                s_pitch = ypr[1];  // rad
                s_roll  = ypr[2];  // rad
                xSemaphoreGive(s_imu_mutex);
            }
        }

        // Leitura rápida, ~200 Hz máx
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}



// ============================
// app_main (C linkage)
// ============================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "app_main start");

    // Mutex para compartilhar yaw/pitch/roll
    s_imu_mutex = xSemaphoreCreateMutex();
    configASSERT(s_imu_mutex != nullptr);

    // NVS para Wi-Fi/stack
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Hardware
    pwm_init();
    ESP_ERROR_CHECK(i2c_master_init());

    // Rede
    wifi_init_sta();
    mqtt_start();

    // Task que cuida da IMU/DMP
    xTaskCreate(mpu6050_dmp_task,
                "mpu6050_dmp_task",
                4096,
                nullptr,
                5,
                nullptr);

    // ---- Loop principal: só cuida de publicar IMU no MQTT ----

    float last_yaw_deg   = 0.0f;
    float last_pitch_deg = 0.0f;
    float last_roll_deg  = 0.0f;

    const TickType_t pub_period = pdMS_TO_TICKS(50); // 20 Hz máx
    const float ANGLE_EPS = 0.5f;                    // só manda se mudar >0.5°
    TickType_t last_pub = xTaskGetTickCount();

    while (true) {

        TickType_t now = xTaskGetTickCount();
        if (now - last_pub >= pub_period) {

            // Lê valores atuais da IMU (radianos)
            float yaw_rad   = 0.0f;
            float pitch_rad = 0.0f;
            float roll_rad  = 0.0f;

            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                yaw_rad   = s_yaw;
                pitch_rad = s_pitch;
                roll_rad  = s_roll;
                xSemaphoreGive(s_imu_mutex);
            } else {
                // Não conseguiu pegar o mutex, tenta de novo depois
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Converte para graus
            float yaw_deg   = yaw_rad   * 180.0f / M_PI;
            float pitch_deg = pitch_rad * 180.0f / M_PI;
            float roll_deg  = roll_rad  * 180.0f / M_PI;

            // Verifica se mudou o suficiente
            float dy = fabsf(yaw_deg   - last_yaw_deg);
            float dp = fabsf(pitch_deg - last_pitch_deg);
            float dr = fabsf(roll_deg  - last_roll_deg);

            if (dy > ANGLE_EPS || dp > ANGLE_EPS || dr > ANGLE_EPS) {

                char payload[128];
                snprintf(payload, sizeof(payload),
                         "{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f}",
                         yaw_deg, pitch_deg, roll_deg);

                if (s_mqtt_client != nullptr) {
                    esp_mqtt_client_publish(
                        s_mqtt_client,
                        MQTT_TOPIC_IMU,   // "hovercraft/imu"
                        payload,
                        0,                // tamanho (0 = string terminada em '\0')
                        1,                // QoS
                        0);               // retain
                }

                last_yaw_deg   = yaw_deg;
                last_pitch_deg = pitch_deg;
                last_roll_deg  = roll_deg;
            }

            last_pub = now;
        }

        // Pequeno delay para não travar a CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
