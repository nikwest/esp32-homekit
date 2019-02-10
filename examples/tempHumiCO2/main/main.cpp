#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"

#include "hap.h"

#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "dht22.h"
#include "ccs811.h"
//#include "hdc1080.h"
#include "sht3x.h"


#define TAG "THCO2"

#define ACCESSORY_NAME  "TEMP/HUMI/CO2"
#define MANUFACTURER_NAME   "NikWest"
#define MODEL_NAME  "ESP32_ACC"
#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#if 1
#define EXAMPLE_ESP_WIFI_SSID "nikwest"
#define EXAMPLE_ESP_WIFI_PASS "eilemitweile!"
#endif
#if 0
#define EXAMPLE_ESP_WIFI_SSID "NO_RUN"
#define EXAMPLE_ESP_WIFI_PASS "1qaz2wsx"
#endif

#define I2C_BUS       0
#define I2C_FREQ      I2C_FREQ_100K


#define CO2_LIMIT 1500

/* FIXME */
static gpio_num_t DHT22_GPIO = GPIO_NUM_4;

static gpio_num_t I2C_SCL_PIN = GPIO_NUM_14;
static gpio_num_t I2C_SDA_PIN = GPIO_NUM_13;

static EventGroupHandle_t wifi_event_group;
const int WIFI_GOT_IP_BIT = BIT0;
const int WIFI_GOT_IP6_BIT = BIT1;

static void* acc;
static SemaphoreHandle_t ev_mutex;
static int temperature = 0;
static int humidity = 0;
static float temperature_float = 20;
static float humidity_float = 50;
static ccs811_sensor_t* ccs811;
static sht3x_sensor_t* sht3x;
static int co2_detected = 0;
static int co2_level = 0;
static void* _temperature_ev_handle = NULL;
static void* _humidity_ev_handle =  NULL;
static void* _co2_detected_ev_handle =  NULL;
static void* _co2_level_ev_handle =  NULL;

void temperature_humidity_monitoring_task_dht22(void* arm)
{
    while (1) {
        ESP_LOGI("MAIN", "RAM LEFT %d", esp_get_free_heap_size());
        ESP_LOGI("MAIN", "TASK STACK : %d", uxTaskGetStackHighWaterMark(NULL));

        if (dht22_read(DHT22_GPIO, &temperature_float, &humidity_float) < 0) {
            vTaskDelay( 3000 / portTICK_RATE_MS );
            taskYIELD();
            continue;
        }

        temperature = temperature_float * 100;
        humidity = humidity_float * 100;

        //xSemaphoreTake(ev_mutex, 0);

#if 1
        if (_humidity_ev_handle)
            hap_event_response(acc, _humidity_ev_handle, (void*)humidity);

        if (_temperature_ev_handle)
            hap_event_response(acc, _temperature_ev_handle, (void*)temperature);

#endif
        //xSemaphoreGive(ev_mutex);

        printf("%d %d\n", temperature, humidity);
        vTaskDelay( 30000 / portTICK_RATE_MS );
    }
}

void temperature_humidity_monitoring_task_sht3x(void* arm)
{
    while (1) {
        ESP_LOGI("MAIN", "RAM LEFT %d", esp_get_free_heap_size());
        ESP_LOGI("MAIN", "TASK STACK : %d", uxTaskGetStackHighWaterMark(NULL));

        uint8_t duration = sht3x_get_measurement_duration(sht3x_high);
    
        // Trigger one measurement in single shot mode with high repeatability.
        sht3x_start_measurement (sht3x, sht3x_single_shot, sht3x_high);
        
        // Wait until measurement is ready (constant time of at least 30 ms
        // or the duration returned from *sht3x_get_measurement_duration*).
        vTaskDelay (duration);
        
        // retrieve the values and do something with them
        if (sht3x_get_results (sht3x, &temperature_float, &humidity_float)) {
            printf("%.3f SHT3x Sensor: %.2f °C, %.2f %%\n", 
                   (double)sdk_system_get_time()*1e-3, temperature_float, humidity_float);
        }

        temperature = temperature_float * 100;
        humidity = humidity_float * 100;

        //xSemaphoreTake(ev_mutex, 0);

#if 1
        if (_humidity_ev_handle)
            hap_event_response(acc, _humidity_ev_handle, (void*)humidity);

        if (_temperature_ev_handle)
            hap_event_response(acc, _temperature_ev_handle, (void*)temperature);

#endif
        //xSemaphoreGive(ev_mutex);

        printf("%d %d\n", temperature, humidity);
        vTaskDelay( 30000 / portTICK_RATE_MS );
    }
}

static void* _temperature_read(void* arg)
{
    ESP_LOGI("MAIN", "_temperature_read");
    return (void*)temperature;
}

void _temperature_notify(void* arg, void* ev_handle, bool enable)
{
    ESP_LOGI("MAIN", "_temperature_notify");
    //xSemaphoreTake(ev_mutex, 0);

    if (enable) 
        _temperature_ev_handle = ev_handle;
    else 
        _temperature_ev_handle = NULL;

    //xSemaphoreGive(ev_mutex);
}

static void* _humidity_read(void* arg)
{
    ESP_LOGI("MAIN", "_humidity_read");
    return (void*)humidity;
}

void _humidity_notify(void* arg, void* ev_handle, bool enable)
{
    ESP_LOGI("MAIN", "_humidity_notify");
    //xSemaphoreTake(ev_mutex, 0);

    if (enable) 
        _humidity_ev_handle = ev_handle;
    else 
        _humidity_ev_handle = NULL;

    //xSemaphoreGive(ev_mutex);
}


void co2_monitoring_task(void* arm)
{
  uint16_t tvoc = 0;
  uint16_t eco2 = 500;
  while (1) {
    ESP_LOGI("MAIN", "RAM LEFT %d", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "TASK STACK : %d", uxTaskGetStackHighWaterMark(NULL));
    
    //xSemaphoreTake(ev_mutex, 0);
    ccs811_set_environmental_data(ccs811, temperature_float, humidity_float);
    if (ccs811_get_results (ccs811, &tvoc, &eco2, 0, 0)) {
        ESP_LOGI("MAIN", "%d CCS811 Sensor periodic: TVOC %d ppb, eCO2 %d ppm\n",
           ccs811->error_code, tvoc, eco2);
    } else {
        int error = ccs811->error_code;
        ESP_LOGE("MAIN", "CCS811 readout failed: %d", error);
    }
    co2_level = eco2 * 100;
    co2_detected = (eco2 < CO2_LIMIT) ? 0 : 1;
#if 1
    if (_co2_detected_ev_handle) 
        hap_event_response(acc, _co2_detected_ev_handle, (void*)co2_detected);
    if (_co2_level_ev_handle)
        hap_event_response(acc, _co2_level_ev_handle, (void*)co2_level);
        
#endif
    //xSemaphoreGive(ev_mutex);
    
 
    vTaskDelay( 60000 / portTICK_RATE_MS );
  }
}

static void* _co2_detected_read(void* arg)
{
  ESP_LOGI("MAIN", "_co2_detected_read");
  return (void*) co2_detected;
}

static void* _co2_level_read(void* arg)
{
  ESP_LOGI("MAIN", "_co2_level_read");
  return (void*) co2_level;
}

void _co2_detected_notify(void* arg, void* ev_handle, bool enable)
{
  ESP_LOGI("MAIN", "_co2_detected_notify");
  //xSemaphoreTake(ev_mutex, 0);
  
  if (enable)
    _co2_detected_ev_handle = ev_handle;
  else
    _co2_detected_ev_handle = NULL;
  
  //xSemaphoreGive(ev_mutex);
}

void _co2_level_notify(void* arg, void* ev_handle, bool enable)
{
  ESP_LOGI("MAIN", "_co2_level_notify");
  //xSemaphoreTake(ev_mutex, 0);
  
  if (enable)
    _co2_level_ev_handle = ev_handle;
  else
    _co2_level_ev_handle = NULL;
  
  //xSemaphoreGive(ev_mutex);
}


static bool _identifed = false;
void* identify_read(void* arg)
{
    return (void*)true;
}

void hap_object_init(void* arg)
{
    ESP_LOGI(TAG, "hap_object_init");

    void* accessory_object = hap_accessory_add(acc);
    struct hap_characteristic cs[] = {
        {HAP_CHARACTER_IDENTIFY, (void*)true, NULL, identify_read, NULL, NULL},
        {HAP_CHARACTER_MANUFACTURER, (void*)MANUFACTURER_NAME, NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_MODEL, (void*)MODEL_NAME, NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_NAME, (void*)ACCESSORY_NAME, NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_SERIAL_NUMBER, (void*)"0123456789", NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_FIRMWARE_REVISION, (void*)"1.0", NULL, NULL, NULL, NULL},
    };
    hap_service_and_characteristics_add(acc, accessory_object, HAP_SERVICE_ACCESSORY_INFORMATION, cs, ARRAY_SIZE(cs));

    struct hap_characteristic co2_sensor[] = {
      {HAP_CHARACTER_CARBON_DIOXIDE_DETECTED, (void*)co2_detected, NULL, _co2_detected_read, NULL, _co2_detected_notify},
      {HAP_CHARACTER_CARBON_DIOXIDE_LEVEL, (void*)co2_level, NULL, _co2_level_read, NULL, _co2_level_notify},
      {HAP_CHARACTER_NAME, (void*)"CO2" , NULL, NULL, NULL, NULL},
     };
    hap_service_and_characteristics_add(acc, accessory_object, HAP_SERVICE_CARBON_DIOXIDE_SENSOR, co2_sensor, ARRAY_SIZE(co2_sensor));
   
    struct hap_characteristic humidity_sensor[] = {
        {HAP_CHARACTER_CURRENT_RELATIVE_HUMIDITY, (void*)humidity, NULL, _humidity_read, NULL, _humidity_notify},
        {HAP_CHARACTER_NAME, (void*)"HYGROMETER" , NULL, NULL, NULL, NULL},
    };
    hap_service_and_characteristics_add(acc, accessory_object, HAP_SERVICE_HUMIDITY_SENSOR, humidity_sensor, ARRAY_SIZE(humidity_sensor));

    struct hap_characteristic temperature_sensor[] = {
        {HAP_CHARACTER_CURRENT_TEMPERATURE, (void*)temperature, NULL, _temperature_read, NULL, _temperature_notify},
        {HAP_CHARACTER_NAME, (void*)"THERMOMETER" , NULL, NULL, NULL, NULL},
    };
    hap_service_and_characteristics_add(acc, accessory_object, HAP_SERVICE_TEMPERATURE_SENSOR, temperature_sensor, ARRAY_SIZE(temperature_sensor));

}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_GOT_IP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        ESP_LOGI(TAG, "got ip6:%s",
            ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_GOT_IP6_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_GOT_IP_BIT);
        xEventGroupClearBits(wifi_event_group, WIFI_GOT_IP6_BIT);
        break;
    default:
        break;
    }
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    ESP_LOGI(TAG, "EventBits_t %d", bits);
    if(acc == NULL && (bits & 3) == 3)
    {
        ESP_LOGI(TAG, "initializing homekit...");
        hap_init();

        uint8_t mac[6];
        esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
        char accessory_id[32] = {0,};
        sprintf(accessory_id, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        hap_accessory_callback_t callback;
        callback.hap_object_init = hap_object_init;
        acc = hap_accessory_register((char*)ACCESSORY_NAME, accessory_id, (char*)"010-12-680", (char*)MANUFACTURER_NAME, HAP_ACCESSORY_CATEGORY_OTHER, 811, 1, NULL, &callback);
        ESP_LOGI(TAG, "... done.");
    }
    return ESP_OK;
}

void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    memcpy(wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID, strlen(EXAMPLE_ESP_WIFI_SSID));
    memcpy(wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS, strlen(EXAMPLE_ESP_WIFI_PASS));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void i2c_init() {
  // init all I2C bus interfaces at which CCS811 sensors are connected
  i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
  
  // longer clock stretching is required for CCS811
  i2c_set_clock_stretch (I2C_BUS, CCS811_I2C_CLOCK_STRETCH);
  
  // init the sensor with slave address CCS811_I2C_ADDRESS_1 connected I2C_BUS.
  ccs811 = ccs811_init_sensor (I2C_BUS, CCS811_I2C_ADDRESS_1);
  if(ccs811 == NULL) {
    ESP_LOGE("INIT", "Cannot prepare ccs811");
  }
  ccs811_set_mode (ccs811, ccs811_mode_10s);
  
  sht3x = sht3x_init_sensor(I2C_BUS, SHT3x_ADDR_1);
  if(sht3x == NULL) {
    ESP_LOGE("INIT", "Cannot prepare sht3x");
  }
}

extern "C" void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    vSemaphoreCreateBinary(ev_mutex);

    i2c_init();
    if (sht3x) {
        xTaskCreate( &temperature_humidity_monitoring_task_sht3x, "sht3x", 4096, NULL, 5, NULL );
    } else {
        xTaskCreate( &temperature_humidity_monitoring_task_dht22, "dht22", 4096, NULL, 5, NULL );
    }
    if (ccs811) {
        xTaskCreate( &co2_monitoring_task, "ccs811", 4096, NULL, 5, NULL );
    }

    wifi_init_sta();
}
