#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "esp_https_ota.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"  

/* ===================== CONFIG URIs / VERSION ===================== */
#define APP_VERSION                 "v2.0"
#define BROKER_URI                  "mqtts://l46d1e5e.ala.us-east-1.emqxsl.com:8883"
#define MQTT_USER                   "big-data-001"
#define MQTT_PASS                   "1Q2W3E4R5T6Y"
#define MQTT_CLIENT_ID              "2022371044"
#define TOPIC_SENSOR_DATA           "esp32/sensor_data"
#define TOPIC_OTA_ALERT             "esp32/ota_alert"
#define MANIFEST_URL                "https://firmware-host.onrender.com/firmware/manifest.json"
#define OTA_MANIFEST_MAX_SIZE       1024

/* ===================== PINOUT (2 sensores, 2 actuadores) ===================== */
// Sensores
#define DHTPIN                      GPIO_NUM_4     // DHT11
#define SOIL_SENSOR_PIN             ADC_CHANNEL_6 // Corregido para ESP-IDF v5.x
#define SOIL_ADC_UNIT               ADC_UNIT_1

// Actuadores
#define LED_VERDE                   GPIO_NUM_15
#define LED_ROJO                    GPIO_NUM_2
#define BUZZER_PIN                  GPIO_NUM_13

// Constantes NTC
#define SOIL_NTC_BETA 3950.0f
#define SOIL_NTC_R0 10000.0f
#define SOIL_REF_TEMP_K 298.15f

/* ===================== PARAMS DE CONTROL ===================== */
// Muestreo cada 10 s
#define SAMPLE_PERIOD_MS            10000

// Rangos "óptimos" (Ajustables) para plantas aromáticas clima templado-húmedo
// Ajusta según tu cultivo específico (menta, perejil, etc.)
#define OPT_AMBIENT_TEMP_MIN_C      18.0f
#define OPT_AMBIENT_TEMP_MAX_C      29.0f //24 es lo correc
#define OPT_AMBIENT_RH_MIN_PCT      60.0f
#define OPT_AMBIENT_RH_MAX_PCT      80.0f
#define OPT_SOIL_TEMP_MIN_C         40.0f
#define OPT_SOIL_TEMP_MAX_C         70.0f

/* ===================== SERVO/BUZZER LEDC (solo buzzer aquí) ===================== */
#define BUZZER_LEDC_SPEEDMODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER           LEDC_TIMER_1
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_1
#define BUZZER_LEDC_DUTY_RES        LEDC_TIMER_10_BIT
#define BUZZER_LEDC_FREQ_HZ         2000  // 2 kHz tono

/* ===================== ESTADO DE APLICACIÓN ===================== */
typedef enum {
    STATE_INITIALIZING,
    STATE_RUNNING,
    STATE_OTA_PENDING
} app_state_t;

static const char *TAG = "MAIN";
static app_state_t current_state = STATE_INITIALIZING;
static esp_mqtt_client_handle_t mqtt_client_global = NULL;

// ADC
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_calibrated = false;

/* ===================== DHT11 (implementación simple) ===================== */
typedef struct { float temp_c; float rh; } dht_reading_t;

static inline void dht_set_output(void){
    gpio_set_direction(DHTPIN, GPIO_MODE_OUTPUT);
}
static inline void dht_set_input(void){
    gpio_set_direction(DHTPIN, GPIO_MODE_INPUT);
}

static bool dht11_read(dht_reading_t *out)
{
    // Protocolo por temporización (bit-bang). Funciona en DHT11.
    // Deshabilitar interrupciones locales para mayor precisión.
    uint32_t irq_state = portSET_INTERRUPT_MASK_FROM_ISR();

    dht_set_output();
    gpio_set_level(DHTPIN, 0);
    esp_rom_delay_us(18000); // 18 ms
    gpio_set_level(DHTPIN, 1);
    esp_rom_delay_us(30);
    dht_set_input();

    // Esperar respuesta del sensor
    int timeout = 0;
    while (gpio_get_level(DHTPIN) == 1) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);} // 20-40us
    timeout = 0;
    while (gpio_get_level(DHTPIN) == 0) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);} // 80us
    timeout = 0;
    while (gpio_get_level(DHTPIN) == 1) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);} // 80us

    // Leer 40 bits
    uint8_t data[5] = {0};
    for (int i = 0; i < 40; i++) {
        // esperar flanco bajo -> alto (inicio del bit)
        timeout = 0;
        while (gpio_get_level(DHTPIN) == 0) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);} 
        // medir duración del alto
        int t_high = 0;
        while (gpio_get_level(DHTPIN) == 1) { t_high++; if (t_high > 200) break; esp_rom_delay_us(1);} // ~26us -> 0, ~70us -> 1
        // desplazar y escribir bit
        data[i/8] <<= 1;
        if (t_high > 40) data[i/8] |= 1; // umbral medio (~40us)
    }

    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state);

    // Checksum
    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if ((sum & 0xFF) != data[4]) return false;

    // DHT11 formato: RH=int,int; Temp=int,int (parte decimal suele ser 0)
    float rh = (float)data[0];
    float tc = (float)data[2];

    out->rh = rh;
    out->temp_c = tc;
    return true;
}

/* ===================== ADC (temperatura de suelo) ===================== */
// Suponiendo NTC 10k con divisor a 3.3V. Ajusta constantes a tu hardware.
#define SOIL_NTC_R0            10000.0f  // resistencia a 25°C
#define SOIL_NTC_BETA          3950.0f   // Beta típica
#define SOIL_SERIES_R          10000.0f  // resistor en serie
#define SOIL_REF_TEMP_K        298.15f   // 25°C en Kelvin

static esp_err_t init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = SOIL_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12 // hasta ~3.6 V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SOIL_SENSOR_PIN, &chan_cfg));

    // Intentar calibración por curva o línea
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = SOIL_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) {
        adc_calibrated = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = SOIL_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) {
        adc_calibrated = true;
    }
#endif

    return ESP_OK;
}

static float read_soil_moisture_pct(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, SOIL_SENSOR_PIN, &raw) != ESP_OK) return NAN;

    // Convertir el valor de 0-4095 a un porcentaje de 0-100%
    // 0 = suelo húmedo, 4095 = suelo seco
    // La fórmula invierte la escala para que un porcentaje alto signifique más humedad.
    float moisture_pct = 100.0f - (raw / 4095.0f) * 100.0f;
    
    return moisture_pct;
}

/* ===================== Actuadores: LEDs + Buzzer ===================== */
static void actuators_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<LED_VERDE) | (1ULL<<LED_ROJO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    // Buzzer por PWM (LEDC)
    ledc_timer_config_t tcfg = {
        .speed_mode = BUZZER_LEDC_SPEEDMODE,
        .timer_num  = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_LEDC_DUTY_RES,
        .freq_hz = BUZZER_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = BUZZER_LEDC_SPEEDMODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ccfg);
}

static void set_green_on(void){ gpio_set_level(LED_VERDE, 1); }
static void set_green_off(void){ gpio_set_level(LED_VERDE, 0); }
static void set_red_on(void){ gpio_set_level(LED_ROJO, 1); }
static void set_red_off(void){ gpio_set_level(LED_ROJO, 0); }
static void buzzer_on(void){
    // 50% duty
    ledc_set_duty(BUZZER_LEDC_SPEEDMODE, BUZZER_LEDC_CHANNEL, (1<<BUZZER_LEDC_DUTY_RES)/2);
    ledc_update_duty(BUZZER_LEDC_SPEEDMODE, BUZZER_LEDC_CHANNEL);
}
static void buzzer_off(void){
    ledc_set_duty(BUZZER_LEDC_SPEEDMODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_SPEEDMODE, BUZZER_LEDC_CHANNEL);
}

/* ===================== OTA ===================== */
static esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_CONNECTED) {
        esp_http_client_set_header(evt->client, "Accept-Encoding", "identity");
    }
    return ESP_OK;
}

static esp_err_t start_ota_from_url(const char *url)
{
    ESP_LOGI(TAG, "Iniciando OTA: %s", url);
    esp_http_client_config_t http_config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = _ota_http_event_handler,
    };
    esp_https_ota_config_t ota_config = { .http_config = &http_config };
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA exitosa. Reiniciando...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA falló: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* ===================== MQTT ===================== */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            mqtt_client_global = event->client;
            esp_mqtt_client_subscribe(mqtt_client_global, TOPIC_OTA_ALERT, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT desconectado");
            mqtt_client_global = NULL;
            break;
        case MQTT_EVENT_DATA:
            if (event->topic && event->topic_len == strlen(TOPIC_OTA_ALERT) &&
                strncmp(event->topic, TOPIC_OTA_ALERT, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Comando OTA recibido. Cambiando a OTA_PENDING");
                // Nota: no bloqueamos aquí, solo cambiamos el estado.
                // La máquina de estados hará el resto.
                // También podrías parsear payload para URLs personalizadas.
                extern app_state_t current_state; // ya es global
                current_state = STATE_OTA_PENDING;
            }
            break;
        default:
            break;
    }
}

/* ===================== DHT INIT ===================== */
static void dht_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<DHTPIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(DHTPIN, 1); // reposo alto con pull-up
}

/* ===================== SENSORES ===================== */
static esp_err_t init_sensors(void)
{
    dht_init();
    ESP_ERROR_CHECK(init_adc());
    actuators_init();
    ESP_LOGI(TAG, "Sensores y actuadores inicializados");
    return ESP_OK;
}

/* ===================== MANIFIESTO OTA ===================== */
static bool get_manifest_and_maybe_update(void)
{
    bool updated = false;
    esp_http_client_config_t cfg = { .url = MANIFEST_URL, .crt_bundle_attach = esp_crt_bundle_attach };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return false;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) { esp_http_client_cleanup(client); return false; }

    int content_len = esp_http_client_fetch_headers(client);
    if (content_len <= 0 || content_len > OTA_MANIFEST_MAX_SIZE) content_len = OTA_MANIFEST_MAX_SIZE;

    char *buf = malloc(OTA_MANIFEST_MAX_SIZE);
    if (!buf) { esp_http_client_close(client); esp_http_client_cleanup(client); return false; }

    int total = 0; int r;
    while ((r = esp_http_client_read(client, buf + total, OTA_MANIFEST_MAX_SIZE - 1 - total)) > 0) total += r;
    buf[total] = '\0';

    int status = esp_http_client_get_status_code(client);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status == 200 && total > 0) {
        cJSON *root = cJSON_Parse(buf);
        if (root) {
            cJSON *version = cJSON_GetObjectItem(root, "version");
            cJSON *bin_url = cJSON_GetObjectItem(root, "bin_url");
            if (cJSON_IsString(version) && cJSON_IsString(bin_url)) {
                if (strcmp(version->valuestring, APP_VERSION) != 0) {
                    ESP_LOGI(TAG, "Nueva versión %s (actual %s).", version->valuestring, APP_VERSION);
                    if (start_ota_from_url(bin_url->valuestring) == ESP_OK) updated = true;
                } else {
                    ESP_LOGI(TAG, "Firmware actual (%s).", APP_VERSION);
                }
            }
            cJSON_Delete(root);
        }
    }

    free(buf);
    return updated;
}

/* ===================== LÓGICA DE CONTROL ===================== */
static bool conditions_are_optimal(float amb_temp, float amb_rh, float soil_temp)
{
    bool amb_ok  = (amb_temp >= OPT_AMBIENT_TEMP_MIN_C && amb_temp <= OPT_AMBIENT_TEMP_MAX_C) &&
                   (amb_rh   >= OPT_AMBIENT_RH_MIN_PCT && amb_rh   <= OPT_AMBIENT_RH_MAX_PCT);
    bool soil_ok = (soil_temp >= OPT_SOIL_TEMP_MIN_C && soil_temp <= OPT_SOIL_TEMP_MAX_C);
    return amb_ok && soil_ok;
}

static void apply_outputs(bool optimal)
{
    if (optimal) {
        set_green_on();
        set_red_off();
        buzzer_off();
    } else {
        set_green_off();
        set_red_on();
        buzzer_on();
    }
}

/* ===================== ESTADOS ===================== */
static void run_state_initializing(void)
{
    ESP_LOGI(TAG, "Estado: INITIALIZING");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(init_sensors());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
        .credentials.client_id = MQTT_CLIENT_ID,
        .session.keepalive = 60,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    int timeout = 0;
    while (!mqtt_client_global && timeout < 20) { vTaskDelay(pdMS_TO_TICKS(500)); timeout++; }

    current_state = mqtt_client_global ? STATE_RUNNING : STATE_INITIALIZING;
}

static void run_state_running(void)
{
    static int64_t last_sample_ms = 0;
    int64_t now_ms = esp_timer_get_time() / 1000;

    // Muestreo cada 10 segundos
    if (now_ms - last_sample_ms >= SAMPLE_PERIOD_MS) {
        last_sample_ms = now_ms;

        // Leer DHT11
        dht_reading_t dht = {0};
        bool ok_dht = dht11_read(&dht);

        // Leer humedad del suelo (usando la función corregida)
        float soil_moisture_pct = read_soil_moisture_pct();

        ESP_LOGI(TAG, "Sample -> DHT[ok=%d] T=%.1fC RH=%.1f%% | SoilMoisture=%.1f%%",
                 ok_dht, dht.temp_c, dht.rh, soil_moisture_pct);

        bool optimal = false;
        // La condición de "óptimo" ahora usa la humedad del suelo, no la temperatura incorrecta
        if (ok_dht && !isnan(soil_moisture_pct)) {
            optimal = conditions_are_optimal(dht.temp_c, dht.rh, soil_moisture_pct);
            apply_outputs(optimal);
        } else {
            // Si alguna lectura falla, la condición no es óptima
            apply_outputs(false);
        }

        // Publicar por MQTT si disponible
        if (mqtt_client_global) {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "matricula", MQTT_CLIENT_ID);
            if (ok_dht) {
                cJSON_AddNumberToObject(root, "amb_temp_c", dht.temp_c);
                cJSON_AddNumberToObject(root, "amb_rh_pct", dht.rh);
            } else {
                cJSON_AddStringToObject(root, "dht_status", "read_failed");
            }
            if (!isnan(soil_moisture_pct)) {
                cJSON_AddNumberToObject(root, "soil_moisture_pct", soil_moisture_pct);
            } else {
                cJSON_AddStringToObject(root, "soil_moisture_status", "read_failed");
            }
            cJSON_AddStringToObject(root, "status", optimal ? "optimal" : "alert");
            
            char *json = cJSON_PrintUnformatted(root);
            esp_mqtt_client_publish(mqtt_client_global, TOPIC_SENSOR_DATA, json, 0, 1, 0);
            cJSON_Delete(root);
            free(json);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Ciclo corto para el bucle principal
}

static void run_state_ota_pending(void)
{
    ESP_LOGI(TAG, "Estado: OTA_PENDING -> consultando manifiesto");
    get_manifest_and_maybe_update();
    current_state = STATE_RUNNING; // si no actualiza, regresa a RUNNING
}

/* ===================== MAIN ===================== */
void app_main(void)
{
    // NVS init robusto
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    while (1) {
        switch (current_state) {
            case STATE_INITIALIZING: run_state_initializing(); break;
            case STATE_RUNNING:      run_state_running(); break;
            case STATE_OTA_PENDING:  run_state_ota_pending(); break;
            default: ESP_LOGE(TAG, "Estado inválido"); vTaskDelay(pdMS_TO_TICKS(1000)); break;
        }
    }
}
 