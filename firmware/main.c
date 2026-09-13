/*
 * "CentinelaAgro — Know Your Crop Before It Suffers" main.c
 * ESP32-S3 / ESP-IDF firmware
 *
 * Sensors: AHT30 (temp/humidity, I2C), gas sensor (ADC), soil moisture (ADC)
 * Publishes telemetry to ThingsBoard over MQTT
 * Subscribes to shared attributes for pump/LED control commands
 * Drives: status LED + relay-controlled 5V pump (with safety auto shutoff)
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "mqtt_client.h"
#include "cJSON.h"

/* ---------- User configuration: fill these in before flashing ---------- */
#define WIFI_SSID           "******"
#define WIFI_PASS           "********"
#define MQTT_BROKER_URI     "mqtt://mqtt.thingsboard.cloud:1883"
#define MQTT_ACCESS_TOKEN   "**********"  /* get your own token from ThingsBoard */

/* ---------- Pin configuration ---------- */
#define AHT30_SDA_GPIO      GPIO_NUM_8
#define AHT30_SCL_GPIO      GPIO_NUM_9
#define AHT30_ADDR          0x38

#define GAS_ADC_CHANNEL     ADC_CHANNEL_3   /* GPIO4  */
#define SOIL_ADC_CHANNEL    ADC_CHANNEL_9   /* GPIO10 */
#define LED_GPIO            GPIO_NUM_5
#define RELAY_GPIO          GPIO_NUM_6
#define RELAY_ACTIVE_LOW    1               /* most cheap relay modules trigger on LOW — flip to 0 if yours doesn't */

/* ---------- Safety ---------- */
#define PUMP_MAX_RUN_MS     8000            /* hard cap, enforced regardless of any command received */

/* ---------- Calibration placeholders — you MUST adjust these after testing your own sensors ---------- */
#define SOIL_RAW_DRY        2800             /* ADC raw reading with the sensor in dry air */
#define SOIL_RAW_WET        1200             /* ADC raw reading with the sensor in a cup of water */
#define GAS_PPM_DIVISOR     20.0f            /* crude raw-to-ppm scaling — recalibrate against a reference if precision matters */

static const char *TAG = "CENTINELAGRO";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc1_cali_ok = false;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t aht30_dev;

static esp_mqtt_client_handle_t mqtt_client;
static volatile bool mqtt_connected = false;
static esp_timer_handle_t pump_timeout_timer;

typedef enum { STATUS_OK, STATUS_WARNING, STATUS_CRITICAL } crop_status_t;
static volatile crop_status_t g_status = STATUS_OK;
static volatile float g_last_soil_percent = -1.0f;

/* ================= Wi-Fi ================= */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP address");
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

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

/* ================= ADC (gas + soil) ================= */

static void adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, GAS_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_ADC_CHANNEL, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali_handle) == ESP_OK) {
        adc1_cali_ok = true;
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw values");
    }
}

static int adc_read_mv(adc_channel_t channel)
{
    int raw = 0;
    adc_oneshot_read(adc1_handle, channel, &raw);
    if (adc1_cali_ok) {
        int mv = 0;
        adc_cali_raw_to_voltage(adc1_cali_handle, raw, &mv);
        return mv;
    }
    return raw; /* fallback: raw counts if calibration isn't available on this chip/atten combo */
}

static int read_gas_ppm(void)
{
    int mv = adc_read_mv(GAS_ADC_CHANNEL);
    int ppm = (int)(mv / GAS_PPM_DIVISOR);
    return ppm < 0 ? 0 : ppm;
}

static int read_soil_percent(void)
{
    int raw = 0;
    adc_oneshot_read(adc1_handle, SOIL_ADC_CHANNEL, &raw);
    /* Most capacitive soil sensors read HIGHER when dry — adjust the formula if yours is inverted */
    int percent = (int)(100.0f * (SOIL_RAW_DRY - raw) / (float)(SOIL_RAW_DRY - SOIL_RAW_WET));
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}

/* ================= AHT30 (I2C temperature + humidity) ================= */

static void aht30_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = AHT30_SCL_GPIO,
        .sda_io_num = AHT30_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT30_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &aht30_dev));

    vTaskDelay(pdMS_TO_TICKS(40)); /* power-on delay per datasheet */
}

static esp_err_t aht30_read(float *temperature, float *humidity)
{
    uint8_t trigger_cmd[3] = {0xAC, 0x33, 0x00};
    esp_err_t err = i2c_master_transmit(aht30_dev, trigger_cmd, sizeof(trigger_cmd), 1000);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(80)); /* measurement time per datasheet */

    uint8_t data[6] = {0};
    err = i2c_master_receive(aht30_dev, data, sizeof(data), 1000);
    if (err != ESP_OK) return err;

    if (data[0] & 0x80) {
        ESP_LOGW(TAG, "AHT30 still busy, reading may be stale");
    }

    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    *humidity = (raw_humidity / 1048576.0f) * 100.0f;
    *temperature = (raw_temp / 1048576.0f) * 200.0f - 50.0f;

    return ESP_OK;
}

/* ================= Relay / pump control ================= */

static void relay_set(bool on)
{
    int level = RELAY_ACTIVE_LOW ? (on ? 0 : 1) : (on ? 1 : 0);
    gpio_set_level(RELAY_GPIO, level);
}

static void pump_timeout_cb(void *arg)
{
    ESP_LOGW(TAG, "Pump safety timeout reached, forcing OFF");
    relay_set(false);
}

static void trigger_pump(uint32_t duration_ms)
{
    if (duration_ms > PUMP_MAX_RUN_MS) duration_ms = PUMP_MAX_RUN_MS;
    ESP_LOGI(TAG, "Pump ON for %u ms", (unsigned int)duration_ms);
    relay_set(true);
    esp_timer_stop(pump_timeout_timer); /* clear any pending timer before starting a new one */
    esp_timer_start_once(pump_timeout_timer, (uint64_t)duration_ms * 1000);
}

/* ================= LED status task ================= */

static void led_task(void *pvParameters)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    while (1) {
        switch (g_status) {
            case STATUS_OK:
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case STATUS_WARNING:
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(400));
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(400));
                break;
            case STATUS_CRITICAL:
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(120));
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(120));
                break;
        }
    }
}

/* ================= MQTT ================= */

static void publish_telemetry(float temp, float hum, int gas_ppm, int soil_pct)
{
    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT unavailable, telemetry not published");
        return;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temperature", temp);
    cJSON_AddNumberToObject(root, "humidity", hum);
    cJSON_AddNumberToObject(root, "gas_ppm", gas_ppm);
    cJSON_AddNumberToObject(root, "soil_moisture", soil_pct);

    char *payload = cJSON_PrintUnformatted(root);
    int message_id = esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", payload, 0, 1, 0);
    if (message_id >= 0) {
        ESP_LOGI(TAG, "Published: %s", payload);
    } else {
        ESP_LOGE(TAG, "Telemetry publish failed");
    }

    free(payload);
    cJSON_Delete(root);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected");
            esp_mqtt_client_subscribe(mqtt_client, "v1/devices/me/attributes", 1);
            esp_mqtt_client_subscribe(mqtt_client, "v1/devices/me/rpc/request/+", 1);
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_DATA: {
            char topic[64] = {0};
            char data[256] = {0};
            int topic_len = event->topic_len < (int)sizeof(topic) - 1 ? event->topic_len : (int)sizeof(topic) - 1;
            int data_len = event->data_len < (int)sizeof(data) - 1 ? event->data_len : (int)sizeof(data) - 1;
            memcpy(topic, event->topic, topic_len);
            memcpy(data, event->data, data_len);
            ESP_LOGI(TAG, "MQTT data on %s: %s", topic, data);

            cJSON *json = cJSON_Parse(data);
            if (json) {
                cJSON *action = cJSON_GetObjectItem(json, "action");
                cJSON *method = cJSON_GetObjectItem(json, "method");
                const char *command = NULL;
                uint32_t pump_duration_ms = 5000;

                if (cJSON_IsString(action)) {
                    command = action->valuestring;
                } else if (cJSON_IsString(method)) {
                    command = method->valuestring;

                    cJSON *params = cJSON_GetObjectItem(json, "params");
                    if (cJSON_IsNumber(params) && params->valuedouble > 0) {
                        pump_duration_ms = (uint32_t)params->valuedouble;
                    } else if (cJSON_IsObject(params)) {
                        cJSON *duration = cJSON_GetObjectItem(params, "duration_ms");
                        if (cJSON_IsNumber(duration) && duration->valuedouble > 0) {
                            pump_duration_ms = (uint32_t)duration->valuedouble;
                        }
                    }
                }

                if (command) {
                    if (strcmp(command, "irrigate_now") == 0) {
                        ESP_LOGI(TAG, "RPC irrigate_now: triggering pump for %lu ms",
                                 (unsigned long)pump_duration_ms);
                        trigger_pump(pump_duration_ms);
                        g_status = STATUS_WARNING;
                    } else if (strcmp(command, "critical") == 0) {
                        g_status = STATUS_CRITICAL;
                    } else if (strcmp(command, "none") == 0) {
                        g_status = STATUS_OK;
                    } else {
                        ESP_LOGW(TAG, "RPC unknown command: %s", command);
                    }
                }

                if (cJSON_IsString(method)) {
                    const char *request_id = strrchr(topic, '/');
                    cJSON *response = cJSON_CreateObject();
                    if (request_id && request_id[1] != '\0') {
                        cJSON_AddStringToObject(response, "requestId", request_id + 1);
                    }
                    cJSON_AddStringToObject(response, "status", "ok");
                    char *response_payload = cJSON_PrintUnformatted(response);
                    esp_mqtt_client_publish(mqtt_client,
                                            "v1/devices/me/rpc/response",
                                            response_payload, 0, 1, 0);
                    free(response_payload);
                    cJSON_Delete(response);
                }

                cJSON_Delete(json);
            }
            break;
        }

        default:
            break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_ACCESS_TOKEN,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

/* ================= Sensor task ================= */

static void sensor_task(void *pvParameters)
{
    while (1) {
        float temperature = 0, humidity = 0;
        bool aht_ok = (aht30_read(&temperature, &humidity) == ESP_OK);
        if (!aht_ok) {
            ESP_LOGW(TAG, "AHT30 read failed, skipping this cycle's temp/humidity");
        }

        int gas_ppm = read_gas_ppm();
        int soil_pct = read_soil_percent();

        /* Simple local fallback check — the real decision logic lives in the ThingsBoard rule chain */
        if (g_last_soil_percent >= 0) {
            float delta = soil_pct - g_last_soil_percent;
            if (soil_pct < 20 && delta <= 0) {
                g_status = STATUS_WARNING;
            }
        }
        g_last_soil_percent = soil_pct;

        if (aht_ok) {
            publish_telemetry(temperature, humidity, gas_ppm, soil_pct);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ================= App entry point ================= */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_sta();
    adc_setup();
    aht30_init();

    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    relay_set(false); /* make sure the pump is off the instant we boot */

    const esp_timer_create_args_t timer_args = {
        .callback = &pump_timeout_cb,
        .name = "pump_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pump_timeout_timer));

    mqtt_app_start();

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
}
