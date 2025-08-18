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

/* ===================== CONFIG ===================== */
#define APP_VERSION                 "v1.0"
#define BROKER_URI                  "mqtts://l46d1e5e.ala.us-east-1.emqxsl.com:8883"
#define MQTT_USER                   "big-data-001"
#define MQTT_PASS                   "1Q2W3E4R5T6Y"
#define MQTT_CLIENT_ID              "2022371044"
#define TOPIC_SENSOR_DATA           "esp32/sensor_data"
#define TOPIC_OTA_ALERT             "esp32/ota_alert"
#define MANIFEST_URL                "https://proyecto-iot-paq8.onrender.com/firmware/manifest.json"
#define OTA_MANIFEST_MAX_SIZE       1024

/* ===================== PINOUT ===================== */
// Sensores
#define DHTPIN                      GPIO_NUM_4
#define SOIL_SENSOR_PIN             ADC_CHANNEL_6
#define SOIL_ADC_UNIT               ADC_UNIT_1

// Actuadores
#define LED_VERDE                   GPIO_NUM_15
#define LED_ROJO                    GPIO_NUM_2
#define BUZZER_PIN                  GPIO_NUM_13

// Constantes NTC para humedad de suelo
#define SOIL_NTC_BETA 3950.0f
#define SOIL_NTC_R0 10000.0f
#define SOIL_REF_TEMP_K 298.15f

/* ===================== CONTROL ===================== */
#define SAMPLE_PERIOD_MS            10000

#define OPT_AMBIENT_TEMP_MIN_C      18.0f
#define OPT_AMBIENT_TEMP_MAX_C      29.0f
#define OPT_AMBIENT_RH_MIN_PCT      60.0f
#define OPT_AMBIENT_RH_MAX_PCT      80.0f
#define OPT_SOIL_MOISTURE_MIN_PCT   40.0f
#define OPT_SOIL_MOISTURE_MAX_PCT   70.0f

/* ===================== BUZZER ===================== */
#define BUZZER_LEDC_SPEEDMODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER           LEDC_TIMER_1
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_1
#define BUZZER_LEDC_DUTY_RES        LEDC_TIMER_10_BIT
#define BUZZER_LEDC_FREQ_HZ         2000

/* ===================== ESTADOS ===================== */
typedef enum {
    STATE_INITIALIZING,
    STATE_RUNNING,
    STATE_OTA_PENDING
} app_state_t;

static const char *TAG = "PLANT_MONITOR";
static app_state_t current_state = STATE_INITIALIZING;
static esp_mqtt_client_handle_t mqtt_client_global = NULL;

/* ===================== ADC ===================== */
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_calibrated = false;

/* ===================== DHT ===================== */
typedef struct { float temp_c; float rh; } dht_reading_t;

static inline void dht_set_output(void){ gpio_set_direction(DHTPIN, GPIO_MODE_OUTPUT); }
static inline void dht_set_input(void){ gpio_set_direction(DHTPIN, GPIO_MODE_INPUT); }

static bool dht11_read(dht_reading_t *out)
{
    uint32_t irq_state = portSET_INTERRUPT_MASK_FROM_ISR();

    dht_set_output();
    gpio_set_level(DHTPIN, 0);
    esp_rom_delay_us(18000);
    gpio_set_level(DHTPIN, 1);
    esp_rom_delay_us(30);
    dht_set_input();

    int timeout = 0;
    while (gpio_get_level(DHTPIN) == 1) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);}
    timeout = 0;
    while (gpio_get_level(DHTPIN) == 0) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);}
    timeout = 0;
    while (gpio_get_level(DHTPIN) == 1) { if (++timeout > 200) { portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state); return false; } esp_rom_delay_us(1);}

    uint8_t data[5] = {0};
    for (int i=0;i<40;i++){
        timeout=0;
        while(gpio_get_level(DHTPIN)==0){if(++timeout>200){portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state);return false;} esp_rom_delay_us(1);}
        int t_high=0;
        while(gpio_get_level(DHTPIN)==1){t_high++; if(t_high>200) break; esp_rom_delay_us(1);}
        data[i/8]<<=1;
        if(t_high>40) data[i/8]|=1;
    }

    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq_state);
    uint8_t sum=data[0]+data[1]+data[2]+data[3];
    if(sum!=data[4]) return false;

    out->rh = (float)data[0];
    out->temp_c = (float)data[2];
    return true;
}

/* ===================== ADC ===================== */
static esp_err_t init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = SOIL_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth=ADC_BITWIDTH_12, .atten=ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SOIL_SENSOR_PIN, &chan_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg={.unit_id=SOIL_ADC_UNIT,.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_12};
    if(adc_cali_create_scheme_curve_fitting(&cali_cfg,&adc_cali_handle)==ESP_OK) adc_calibrated=true;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg={.unit_id=SOIL_ADC_UNIT,.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_12};
    if(adc_cali_create_scheme_line_fitting(&cali_cfg,&adc_cali_handle)==ESP_OK) adc_calibrated=true;
#endif

    return ESP_OK;
}

static float read_soil_moisture_pct(void)
{
    int raw=0;
    if(adc_oneshot_read(adc_handle, SOIL_SENSOR_PIN, &raw)!=ESP_OK) return NAN;
    float moisture_pct=100.0f-(raw/4095.0f)*100.0f;
    return moisture_pct;
}

/* ===================== ACTUADORES ===================== */
static void actuators_init(void)
{
    gpio_config_t io={.pin_bit_mask=(1ULL<<LED_VERDE)|(1ULL<<LED_ROJO),.mode=GPIO_MODE_OUTPUT};
    gpio_config(&io);

    ledc_timer_config_t tcfg={.speed_mode=BUZZER_LEDC_SPEEDMODE,.timer_num=BUZZER_LEDC_TIMER,.duty_resolution=BUZZER_LEDC_DUTY_RES,.freq_hz=BUZZER_LEDC_FREQ_HZ,.clk_cfg=LEDC_AUTO_CLK};
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg={.gpio_num=BUZZER_PIN,.speed_mode=BUZZER_LEDC_SPEEDMODE,.channel=BUZZER_LEDC_CHANNEL,.timer_sel=BUZZER_LEDC_TIMER,.duty=0};
    ledc_channel_config(&ccfg);
}

static void set_green_on(void){ gpio_set_level(LED_VERDE,1);}
static void set_green_off(void){ gpio_set_level(LED_VERDE,0);}
static void set_red_on(void){ gpio_set_level(LED_ROJO,1);}
static void set_red_off(void){ gpio_set_level(LED_ROJO,0);}
static void buzzer_on(void){ ledc_set_duty(BUZZER_LEDC_SPEEDMODE,BUZZER_LEDC_CHANNEL,(1<<BUZZER_LEDC_DUTY_RES)/2); ledc_update_duty(BUZZER_LEDC_SPEEDMODE,BUZZER_LEDC_CHANNEL);}
static void buzzer_off(void){ ledc_set_duty(BUZZER_LEDC_SPEEDMODE,BUZZER_LEDC_CHANNEL,0); ledc_update_duty(BUZZER_LEDC_SPEEDMODE,BUZZER_LEDC_CHANNEL);}

/* ===================== OTA ===================== */
static esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt){ if(evt->event_id==HTTP_EVENT_ON_CONNECTED) esp_http_client_set_header(evt->client,"Accept-Encoding","identity"); return ESP_OK; }

static esp_err_t start_ota_from_url(const char *url)
{
    ESP_LOGI(TAG,"Iniciando OTA: %s",url);
    esp_http_client_config_t http_config={.url=url,.crt_bundle_attach=esp_crt_bundle_attach,.event_handler=_ota_http_event_handler};
    esp_https_ota_config_t ota_config={.http_config=&http_config};
    esp_err_t ret=esp_https_ota(&ota_config);
    if(ret==ESP_OK){ ESP_LOGI(TAG,"OTA exitosa, reiniciando..."); esp_restart();}
    else ESP_LOGE(TAG,"OTA falló: %s",esp_err_to_name(ret));
    return ret;
}

/* ===================== MQTT ===================== */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch(event->event_id){
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG,"MQTT conectado"); mqtt_client_global=event->client; esp_mqtt_client_subscribe(mqtt_client_global,TOPIC_OTA_ALERT,1); break;
        case MQTT_EVENT_DISCONNECTED: ESP_LOGI(TAG,"MQTT desconectado"); mqtt_client_global=NULL; break;
        case MQTT_EVENT_DATA:
            if(event->topic && event->topic_len==strlen(TOPIC_OTA_ALERT) && strncmp(event->topic,TOPIC_OTA_ALERT,event->topic_len)==0){
                ESP_LOGI(TAG,"Comando OTA recibido"); current_state=STATE_OTA_PENDING;
            } break;
        default: break;
    }
}

/* ===================== SENSORES ===================== */
static esp_err_t init_sensors(void)
{
    actuators_init();
    ESP_ERROR_CHECK(init_adc());
    ESP_LOGI(TAG,"Sensores y actuadores inicializados");
    return ESP_OK;
}

/* ===================== MANIFIESTO ===================== */
static bool get_manifest_and_maybe_update(void)
{
    bool updated=false;
    esp_http_client_config_t cfg={.url=MANIFEST_URL,.crt_bundle_attach=esp_crt_bundle_attach};
    esp_http_client_handle_t client=esp_http_client_init(&cfg);
    if(!client) return false;

    if(esp_http_client_open(client,0)!=ESP_OK){ esp_http_client_cleanup(client); return false; }

    int content_len=esp_http_client_fetch_headers(client);
    if(content_len<=0 || content_len>OTA_MANIFEST_MAX_SIZE) content_len=OTA_MANIFEST_MAX_SIZE;

    char *buf=malloc(OTA_MANIFEST_MAX_SIZE);
    if(!buf){ esp_http_client_close(client); esp_http_client_cleanup(client); return false; }

    int total=0,r;
    while((r=esp_http_client_read(client,buf+total,OTA_MANIFEST_MAX_SIZE-1-total))>0) total+=r;
    buf[total]='\0';

    int status=esp_http_client_get_status_code(client);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if(status==200 && total>0){
        cJSON *root=cJSON_Parse(buf);
        if(root){
            cJSON *version=cJSON_GetObjectItem(root,"version");
            cJSON *bin_url=cJSON_GetObjectItem(root,"bin_url");
            if(cJSON_IsString(version)&&cJSON_IsString(bin_url)){
                if(strcmp(version->valuestring,APP_VERSION)!=0){
                    ESP_LOGI(TAG,"Nueva versión %s (actual %s).",version->valuestring,APP_VERSION);
                    if(start_ota_from_url(bin_url->valuestring)==ESP_OK) updated=true;
                }else ESP_LOGI(TAG,"Firmware actual (%s).",APP_VERSION);
            }
            cJSON_Delete(root);
        }
    }
    free(buf);
    return updated;
}

/* ===================== CONTROL ===================== */
static bool conditions_are_optimal(float amb_temp,float amb_rh,float soil_moisture){
    bool amb_ok=(amb_temp>=OPT_AMBIENT_TEMP_MIN_C && amb_temp<=OPT_AMBIENT_TEMP_MAX_C)&&
                (amb_rh>=OPT_AMBIENT_RH_MIN_PCT && amb_rh<=OPT_AMBIENT_RH_MAX_PCT);
    bool soil_ok=(soil_moisture>=OPT_SOIL_MOISTURE_MIN_PCT && soil_moisture<=OPT_SOIL_MOISTURE_MAX_PCT);
    return amb_ok && soil_ok;
}

static void apply_outputs(bool optimal){
    if(optimal){ set_green_on(); set_red_off(); buzzer_off(); }
    else { set_green_off(); set_red_on(); buzzer_on(); }
}

/* ===================== ESTADOS ===================== */
static void run_state_initializing(void)
{
    ESP_LOGI(TAG,"Estado: INITIALIZING");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(init_sensors());

    esp_mqtt_client_config_t mqtt_cfg={
        .broker.address.uri=BROKER_URI,
        .broker.verification.crt_bundle_attach=esp_crt_bundle_attach,
        .credentials.username=MQTT_USER,
        .credentials.authentication.password=MQTT_PASS,
        .credentials.client_id=MQTT_CLIENT_ID,
        .session.keepalive=60,
    };
    esp_mqtt_client_handle_t client=esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client,ESP_EVENT_ANY_ID,mqtt_event_handler,NULL);
    esp_mqtt_client_start(client);

    int timeout=0;
    while(!mqtt_client_global && timeout<20){ vTaskDelay(pdMS_TO_TICKS(500)); timeout++; }
    current_state=mqtt_client_global?STATE_RUNNING:STATE_INITIALIZING;
}

static void run_state_running(void)
{
    static int64_t last_sample_ms=0;
    int64_t now_ms=esp_timer_get_time()/1000;
    if(now_ms-last_sample_ms>=SAMPLE_PERIOD_MS){
        last_sample_ms=now_ms;

        dht_reading_t dht={0};
        bool ok_dht=dht11_read(&dht);
        float soil_moisture=read_soil_moisture_pct();

        ESP_LOGI(TAG,"Sample -> DHT[ok=%d] T=%.1fC RH=%.1f%% | SoilMoisture=%.1f%%",
                 ok_dht,dht.temp_c,dht.rh,soil_moisture);

        bool optimal=false;
        if(ok_dht && !isnan(soil_moisture)){
            optimal=conditions_are_optimal(dht.temp_c,dht.rh,soil_moisture);
            apply_outputs(optimal);
        }else apply_outputs(false);

        if(mqtt_client_global){
            cJSON *root=cJSON_CreateObject();
            cJSON_AddStringToObject(root,"matricula",MQTT_CLIENT_ID);
            if(ok_dht){ cJSON_AddNumberToObject(root,"amb_temp_c",dht.temp_c); cJSON_AddNumberToObject(root,"amb_rh_pct",dht.rh);}
            else cJSON_AddStringToObject(root,"dht_status","read_failed");
            if(!isnan(soil_moisture)) cJSON_AddNumberToObject(root,"soil_moisture_pct",soil_moisture);
            else cJSON_AddStringToObject(root,"soil_moisture_status","read_failed");
            cJSON_AddStringToObject(root,"status",optimal?"optimal":"alert");

            char *json=cJSON_PrintUnformatted(root);
            esp_mqtt_client_publish(mqtt_client_global,TOPIC_SENSOR_DATA,json,0,1,0);
            cJSON_Delete(root);
            free(json);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void run_state_ota_pending(void)
{
    ESP_LOGI(TAG,"Estado: OTA_PENDING -> consultando manifiesto");
    get_manifest_and_maybe_update();
    current_state=STATE_RUNNING;
}

/* ===================== MAIN ===================== */
void app_main(void)
{
    esp_err_t ret=nvs_flash_init();
    if(ret==ESP_ERR_NVS_NO_FREE_PAGES || ret==ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    while(1){
        switch(current_state){
            case STATE_INITIALIZING: run_state_initializing(); break;
            case STATE_RUNNING:      run_state_running(); break;
            case STATE_OTA_PENDING:  run_state_ota_pending(); break;
            default: ESP_LOGE(TAG,"Estado inválido"); vTaskDelay(pdMS_TO_TICKS(1000)); break;
        }
    }
}
