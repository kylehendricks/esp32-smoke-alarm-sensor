#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event_loop.h"

#include "mqtt_client.h"

static const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "smoke_alarm";

static EventGroupHandle_t connection_event_group;
static esp_mqtt_client_handle_t mqtt_client = NULL;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(connection_event_group, WIFI_CONNECTED_BIT);
            gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 1);

            ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(connection_event_group, WIFI_CONNECTED_BIT);

            gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 0);
            ESP_LOGW(TAG, "Disconnected from wifi");

            ESP_ERROR_CHECK( esp_wifi_connect() );
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void publish_to_smoke_alarm_topic(const char *msg, int len) {
    esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_ALARM_TOPIC, msg, len, 1, 1);
}

static void task_handle_smoke_alarm_trigger(void *pvParameter) {
    int level = (int) pvParameter;

    if (level) {
        publish_to_smoke_alarm_topic("on", 2);
    } else {
        publish_to_smoke_alarm_topic("off", 3);
    }

    vTaskDelete(NULL);
}

static void isr_smoke_alarm_triggered(void *args) {
    int level = gpio_get_level((gpio_num_t) CONFIG_PIN_SMOKE_ALARM_RELAY);
    xTaskCreate(&task_handle_smoke_alarm_trigger, "smoke_alarm_trigger", 8192, (void *const) level, 5, NULL);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_client = event->client;
            esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_LWT_TOPIC, "online", 6, 1, 1);

            // Send a message with the current state of the alarm
            isr_smoke_alarm_triggered(NULL);

            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_client = NULL;
            break;
        default:
            break;
    }

    return ESP_OK;
}

// Only initializes the mqtt client, connection happens after an IP is gotten
static void init_mqtt() {
    const esp_mqtt_client_config_t settings = {
            .host = CONFIG_MQTT_HOST,
            .username = CONFIG_MQTT_USERNAME,
            .password = CONFIG_MQTT_PASSWORD,
            .port = CONFIG_MQTT_PORT,
            .client_id = "salt_sensor",
            .lwt_topic = CONFIG_MQTT_LWT_TOPIC,
            .lwt_msg = "offline",
            .lwt_qos = 1,
            .lwt_retain = 1,
            .keepalive = 60,
            .event_handle = mqtt_event_handler,
    };

    mqtt_client = esp_mqtt_client_init(&settings);

    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create mqtt client!");
        esp_restart();
    }
}

static void init_wifi()
{
    tcpip_adapter_init();
    connection_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASSWORD,
                    .bssid_set = false,
            }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static void init_smoke_alarm_gpio() {
    gpio_config_t smoke_alarm_pin_config = {
            .pin_bit_mask = (1 << CONFIG_PIN_SMOKE_ALARM_RELAY),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&smoke_alarm_pin_config));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t) CONFIG_PIN_SMOKE_ALARM_RELAY, isr_smoke_alarm_triggered, NULL));
}

void app_main()
{
    nvs_flash_init();

    gpio_pad_select_gpio(CONFIG_PIN_WIFI_STATUS_LED);
    gpio_set_direction((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 0);

    init_mqtt();
    init_wifi();

    init_smoke_alarm_gpio();
}
