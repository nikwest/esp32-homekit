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

#include "pcf857x.h"

#include "hap.h"


#define TAG "SWITCH8"

#define ACCESSORY_NAME  "SWITCH8"
#define MANUFACTURER_NAME   "NikWest"
#define MODEL_NAME  "ESP32_ACC"
#define PIN "010-12-680"
#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))
#define NUM_CHANNELS    8

#if 1
#define EXAMPLE_ESP_WIFI_SSID "nikwest"
#define EXAMPLE_ESP_WIFI_PASS "eilemitweile!"
#endif

#define I2C_BUS       0
#define I2C_FREQ      I2C_FREQ_100K
#define PCF8575_ADDR  0x20

static gpio_num_t IRQ_PIN = GPIO_NUM_15;
static gpio_num_t I2C_SCL_PIN = GPIO_NUM_14;
static gpio_num_t I2C_SDA_PIN = GPIO_NUM_13;

static EventGroupHandle_t wifi_event_group;
const int WIFI_GOT_IP_BIT = BIT0;
const int WIFI_GOT_IP6_BIT = BIT1;


static void* acc;
static void* _ev_handle[NUM_CHANNELS];
static pcf857x_t pcf8575;

typedef union {
	uint16_t raw;
    struct {
        uint8_t in;
        uint8_t out;
    };
	struct {
		uint8_t in0: 1;
		uint8_t in1: 1;
		uint8_t in2: 1;
		uint8_t in3: 1;
		uint8_t in4: 1;
		uint8_t in5: 1;
		uint8_t in6: 1;
		uint8_t in7: 1;
        uint8_t out0: 1;
        uint8_t out1: 1;
        uint8_t out2: 1;
        uint8_t out3: 1;
        uint8_t out4: 1;
        uint8_t out5: 1;
        uint8_t out6: 1;
        uint8_t out7: 1;
	};
} state_t;

static state_t gpios;

static inline void set_gpio_out(int pos, bool val, bool write) {
    if (val) {
        gpios.out |= 1 << pos;
    } else {
        gpios.out &= ~(1 << pos);
    } 
    if(write) {
        ESP_ERROR_CHECK(pcf857x_write(&pcf8575, gpios.raw));
        if (_ev_handle[pos]) {
            hap_event_response(acc, _ev_handle[pos], (void*) val);
        }
    }
}

static inline bool get_gpio_out(int pos, bool read) {
    if(read) {
        ESP_ERROR_CHECK(pcf857x_read(&pcf8575, &gpios.raw));
    }
    if(pos == -1) {
        return read;
    }
    return  (gpios.out & (1 << pos)) ? true : false;
}

void* led_read(void* arg) {
    int i = (int) arg;
    bool val = get_gpio_out(i, false);
    ESP_LOGD(TAG, "LED %d READ %d\n", i, val);
    return (void*)val;
}

void led_write(void* arg, void* val, int len) {
    if(len < sizeof(int)) {
        ESP_LOGE(TAG, "argument is not an int. Size too small %d", len);
        return;
    }
    int i = (int) arg;
    bool value = (bool) val;
    ESP_LOGD(TAG, "LED %d WRITE. %c\n", i, value);
    set_gpio_out(i, value, true);

    return;
}

void led_notify(void* arg, void* ev_handle, bool enable) {
    int i = (int) arg;
    if (enable) {
        _ev_handle[i] = ev_handle;
    }
    else {
        _ev_handle[i] = NULL;
    }
}

void* identify_read(void* arg) {
    return (void*)true;
}

void hap_object_init(void* arg) {
    void* accessory_object = hap_accessory_add(acc);
    struct hap_characteristic cs[] = {
        {HAP_CHARACTER_IDENTIFY, (void*)true, NULL, identify_read, NULL, NULL},
        {HAP_CHARACTER_MANUFACTURER, (void*)MANUFACTURER_NAME, NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_MODEL, (void*)MODEL_NAME, NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_NAME, (void*)ACCESSORY_NAME, NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_SERIAL_NUMBER, (void*)"01012680", NULL, NULL, NULL, NULL},
        {HAP_CHARACTER_FIRMWARE_REVISION, (void*)"1.0", NULL, NULL, NULL, NULL},
    };
    hap_service_and_characteristics_add(acc, accessory_object, HAP_SERVICE_ACCESSORY_INFORMATION, cs, ARRAY_SIZE(cs));

    for(int i=0; i < NUM_CHANNELS; i++) {
        struct hap_characteristic cc[] = {
            {HAP_CHARACTER_ON, (void*) get_gpio_out(i, false), (void*) i, led_read, led_write, led_notify}
        };
        hap_service_and_characteristics_add(acc, accessory_object, HAP_SERVICE_SWITCHS, cc, ARRAY_SIZE(cc));
    }
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
        acc = hap_accessory_register((char*)ACCESSORY_NAME, accessory_id, (char*)PIN, (char*)MANUFACTURER_NAME, HAP_ACCESSORY_CATEGORY_OTHER, 811, 1, NULL, &callback);
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
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void pcf8575_run_test() {
    uint16_t write = 0;
    uint16_t read = 0xFF;
    for(int i=0; i<256; i++) {
        if(pcf857x_write(&pcf8575, write) != ESP_OK) {
            ESP_LOGE(TAG, "Could not test write to pcf8575\n");
            continue;
        }
        if(pcf857x_read(&pcf8575, &read) != ESP_OK) {
            ESP_LOGE(TAG, "Could not test read from pcf8575\n");
            continue;
        } 
        if(read != write) {
            ESP_LOGE(TAG, "Couldn't read what was written: %u vs %u\n", read, write);
        }
        write++;
    }

}

void handle_pcf8575_interrupt(uint16_t value) {
    ESP_LOGI(TAG, "handle_pcf8575_interrupt\n");
    for(int i=0; i<NUM_CHANNELS; i++) {
        if( (value & (1 << i)) == 0) {
            ESP_LOGI(TAG, "toggle %d\n", i);
            set_gpio_out(i, !get_gpio_out(i, false), true);
        }
    }
}

void init_pcf8575() {
    while(i2cdev_init() != ESP_OK) {
        ESP_LOGE(TAG, "Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while( pcf857x_init(&pcf8575, PCF8575, I2C_BUS, PCF8575_ADDR, I2C_SDA_PIN, I2C_SCL_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    // read all in
    get_gpio_out(-1, true);

    pcf857x_set_interrupt_handler(&pcf8575, IRQ_PIN, handle_pcf8575_interrupt);    

    //pcf8575_run_test();
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );

    init_pcf8575();
    wifi_init_sta();
}
