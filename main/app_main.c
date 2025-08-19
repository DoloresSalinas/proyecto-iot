#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "protocol_examples_common.h"

#define APP_VERSION       "v2.2"
#define BROKER_URI        "mqtts://l46d1e5e.ala.us-east-1.emqxsl.com:8883"
#define MQTT_USER         "big-data-001"
#define MQTT_PASS         "1Q2W3E4R5T6Y"
#define MQTT_CLIENT_ID    "2022371044"
#define TOPIC_SENSOR      "esp32/sensor_data"
#define TOPIC_OTA         "esp32/ota_alert"
#define MANIFEST_URL      "https://proyecto-iot-paq8.onrender.com/firmware/manifest.json"
#define SAMPLE_PERIOD_MS  30000 

const char *TOPIC = "esp32/data"; 

#define DHTPIN            GPIO_NUM_4
#define SOIL_PIN          ADC_CHANNEL_6
#define SOIL_UNIT         ADC_UNIT_1
#define LED_GREEN         GPIO_NUM_15
#define LED_RED           GPIO_NUM_2
#define BUZZER_PIN        GPIO_NUM_13

extern const uint8_t mqtt_eclipseprojects_io_pem_start[] asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");
extern const uint8_t emqxsl_ca_pem_start[] asm("_binary_emqxsl_ca_pem_start");
extern const uint8_t emqxsl_ca_pem_end[]   asm("_binary_emqxsl_ca_pem_end");

typedef enum {
    STATE_INIT = 0,
    STATE_RUN,
    STATE_OTA
} app_state_t;

static app_state_t current_state = STATE_INIT;
static const char *TAG = "MAIN";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static adc_oneshot_unit_handle_t adc_handle;

typedef struct { float t, rh; } dht_reading_t;

/* ===== DHT11 simple ===== */
static bool dht11_read(dht_reading_t *r) {
    gpio_set_direction(DHTPIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHTPIN, 0); esp_rom_delay_us(18000);
    gpio_set_level(DHTPIN, 1); esp_rom_delay_us(30);
    gpio_set_direction(DHTPIN, GPIO_MODE_INPUT);

    int data[5] = {0};
    for (int i = 0; i < 40; i++) {
        int t = 0; while (gpio_get_level(DHTPIN) == 0 && t++ < 200) esp_rom_delay_us(1);
        int h = 0; while (gpio_get_level(DHTPIN) == 1 && h++ < 200) esp_rom_delay_us(1);
        data[i / 8] <<= 1;
        if (h > 40) data[i / 8] |= 1;
    }
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) return false;
    r->rh = data[0];
    r->t = data[2];
    return true;
}

/* ===== Actuadores ===== */
static void actuators_init(void) {
    gpio_config_t io = {.pin_bit_mask = (1ULL << LED_GREEN) | (1ULL << LED_RED), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&io);
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);
    ledc_channel_config_t ccfg = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ccfg);
}

static void apply_outputs(bool ok) {
    gpio_set_level(LED_GREEN, ok);
    gpio_set_level(LED_RED, !ok);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ok ? 0 : (1 << 10) / 2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

/* ===== ADC suelo ===== */
static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t cfg = {.unit_id = SOIL_UNIT};
    adc_oneshot_new_unit(&cfg, &adc_handle);
    adc_oneshot_chan_cfg_t c = {.bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12};
    adc_oneshot_config_channel(adc_handle, SOIL_PIN, &c);
}

static float read_soil(void) {
    int raw = 0;
    adc_oneshot_read(adc_handle, SOIL_PIN, &raw);
    return 100.0f - ((float)raw / 4095.0f) * 100.0f;
}

/* ===== OTA ===== */
static esp_err_t start_ota(const char *binary_url) {
    esp_http_client_config_t http_cfg = {
        .url = binary_url,
        .cert_pem = (const char *)mqtt_eclipseprojects_io_pem_start,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg
    };
    return esp_https_ota(&ota_cfg);
}

static void ota_process(void) {
    ESP_LOGI(TAG, "Iniciando proceso OTA...");

    esp_http_client_config_t http_cfg = {
        .url = MANIFEST_URL,
        .crt_bundle_attach = esp_crt_bundle_attach
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg); 

    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        current_state = STATE_RUN;
        return;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        current_state = STATE_RUN;
        return;
    }
    
    int content_length = esp_http_client_fetch_headers(client);
    bool ota_triggered = false;

    if (content_length > 0 && esp_http_client_get_status_code(client) == 200) {
        char *buf = (char *)malloc(content_length + 1);
        if (buf) {
            int read_len = esp_http_client_read(client, buf, content_length);
            if (read_len > 0) {
                buf[read_len] = '\0';
                cJSON *root = cJSON_Parse(buf);
                if (root) {
                    cJSON *version = cJSON_GetObjectItem(root, "version");
                    cJSON *bin_url = cJSON_GetObjectItem(root, "bin_url");
                    
                    if (cJSON_IsString(version) && cJSON_IsString(bin_url) && strcmp(version->valuestring, APP_VERSION) != 0) {
                        ESP_LOGI(TAG, "Nueva versión encontrada: %s. Iniciando OTA...", version->valuestring);
                        esp_err_t ota_err = start_ota(bin_url->valuestring);
                        ota_triggered = true;
                        
                        if (ota_err == ESP_OK) {
                            ESP_LOGI(TAG, "OTA exitosa. Reiniciando...");
                            esp_restart();
                        } else {
                            ESP_LOGE(TAG, "OTA fallida. Error: %s", esp_err_to_name(ota_err));
                        }
                    }
                    cJSON_Delete(root);
                }
            }
            free(buf);
        }
    }
    esp_http_client_cleanup(client);
    
    if (!ota_triggered) {
        current_state = STATE_RUN;
        ESP_LOGI(TAG, "Volviendo al estado: RUN"); 
    }
}


static void get_manifest(void) {
    esp_http_client_config_t http_cfg = {
        .url = MANIFEST_URL,
        .crt_bundle_attach = esp_crt_bundle_attach
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return;
    }
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }
    int content_length = esp_http_client_fetch_headers(client);
    if (content_length > 0 && esp_http_client_get_status_code(client) == 200) {
        char *buf = (char *)malloc(content_length + 1);
        if (buf) {
            int read_len = esp_http_client_read(client, buf, content_length);
            if (read_len > 0) {
                buf[read_len] = '\0';
                cJSON *root = cJSON_Parse(buf);
                if (root) {
                    cJSON *version = cJSON_GetObjectItem(root, "version");
                    cJSON *bin_url = cJSON_GetObjectItem(root, "bin_url");
                    if (cJSON_IsString(version) && cJSON_IsString(bin_url) && strcmp(version->valuestring, APP_VERSION) != 0) {
                        ESP_LOGI(TAG, "New version found: %s. Starting OTA.", version->valuestring);
                        start_ota(bin_url->valuestring);
                    }
                    cJSON_Delete(root);
                }
            }
            free(buf);
        }
    }
    esp_http_client_cleanup(client);
}

/* ===== MQTT handler ===== */
static void mqtt_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    esp_mqtt_event_handle_t e = data;
    if (e->event_id == MQTT_EVENT_CONNECTED) {
        mqtt_client = e->client;
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_OTA, 1);
        ESP_LOGI(TAG, "MQTT connected, subscribed to %s", TOPIC_OTA);
    } else if (e->event_id == MQTT_EVENT_DISCONNECTED) {
        mqtt_client = NULL;
        ESP_LOGI(TAG, "MQTT disconnected");
    } else if (e->event_id == MQTT_EVENT_DATA) {
        if (strncmp(e->topic, TOPIC_OTA, e->topic_len) == 0) {
            ESP_LOGI(TAG, "OTA alert received. Changing state to OTA.");
            current_state = STATE_OTA;
        }
    }
}

/* ===== Loop principal ===== */
static void run_main_loop(void) {
    static int64_t last = 0;
    int64_t now = esp_timer_get_time() / 1000;
    if (now - last >= SAMPLE_PERIOD_MS) {
        last = now;
        dht_reading_t dht;
        bool ok = dht11_read(&dht);
        float soil = read_soil(); 

        ESP_LOGI(TAG, "Lectura del sensor de humedad del suelo: %.2f%%", soil);

        if (ok) {
            ESP_LOGI(TAG, "Lectura del sensor DHT11: Temp = %.1f C, Humedad = %.1f%%", dht.t, dht.rh);
        } else {
            ESP_LOGE(TAG, "Error al leer el sensor DHT11");
        } 

        if (mqtt_client) {
            cJSON *j = cJSON_CreateObject();
            cJSON_AddStringToObject(j, "matricula", MQTT_CLIENT_ID);
            
            if (ok) {
                cJSON_AddNumberToObject(j, "t", dht.t);
                cJSON_AddNumberToObject(j, "rh", dht.rh);
            } else {
                cJSON_AddStringToObject(j, "dht_status", "fail");
            }

            if (!isnan(soil)) {
                cJSON_AddNumberToObject(j, "soil", soil);
            } else {
                cJSON_AddStringToObject(j, "soil_status", "fail");
            }
            
            char *json_string = cJSON_PrintUnformatted(j);
            esp_mqtt_client_publish(mqtt_client, TOPIC, json_string, 0, 1, 0);
            cJSON_free(json_string);
            cJSON_Delete(j);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ===== MAIN ===== */
void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    example_connect();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
        .broker.verification.certificate = (const char *)emqxsl_ca_pem_start,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
        .credentials.client_id = MQTT_CLIENT_ID,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    actuators_init();
    init_adc();
    gpio_set_direction(DHTPIN, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(DHTPIN, 1);

    while (1) {
        switch (current_state) {
            case STATE_INIT:
                ESP_LOGI(TAG, "Cambiando a estado: RUN");
                current_state = STATE_RUN;
                break;
            case STATE_RUN:
                ESP_LOGI(TAG, "App version: v2.1 - ¡Actualización OTA exitosa! 🎉");
                run_main_loop();
                break;
            case STATE_OTA:
                ota_process();
                break;
        }
    }
}