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

#define SMOKE_ALARM_READ_INTERVAL_MS 100

static const int WIFI_CONNECTED_BIT = BIT0;
static const int MQTT_CONNECTED_BIT = BIT1;
static const char *TAG = "smoke_alarm";

static EventGroupHandle_t connection_event_group;
static esp_mqtt_client_handle_t mqtt_client = NULL;

enum LastPublishedState {
    LASTPUBLISHEDSTATE_UNPUBLISHED,
    LASTPUBLISHEDSTATE_OFF,
    LASTPUBLISHEDSTATE_ON
};

static enum LastPublishedState last_published_state = LASTPUBLISHEDSTATE_UNPUBLISHED;

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

static int publish_to_smoke_alarm_topic(const char *msg, int len) {
    return esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_ALARM_TOPIC, msg, len, 1, 1);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_client = event->client;
            xEventGroupSetBits(connection_event_group, MQTT_CONNECTED_BIT);
            esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_LWT_TOPIC, "online", 6, 1, 1);

            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(connection_event_group, MQTT_CONNECTED_BIT);
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
            .client_id = "smoke_alarm",
            .lwt_topic = CONFIG_MQTT_LWT_TOPIC,
            .lwt_msg = "offline",
            .lwt_msg_len = 7,
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
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, TAG));
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static void init_gpio() {
    gpio_config_t smoke_alarm_pin_config = {
            .pin_bit_mask = (1 << CONFIG_PIN_SMOKE_ALARM_RELAY),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config_t wifi_led_pin_config = {
            .pin_bit_mask = (1 << CONFIG_PIN_WIFI_STATUS_LED),
            .mode = GPIO_MODE_OUTPUT,
    };

    ESP_ERROR_CHECK(gpio_config(&smoke_alarm_pin_config));
    ESP_ERROR_CHECK(gpio_config(&wifi_led_pin_config));
}

static void task_monitor_smoke_alarm_relay(void *pvParameter) {
    xEventGroupWaitBits(connection_event_group, MQTT_CONNECTED_BIT, 0, 1, portMAX_DELAY);

    while(true) {
        int level = gpio_get_level((gpio_num_t) CONFIG_PIN_SMOKE_ALARM_RELAY);
        if (level && last_published_state != LASTPUBLISHEDSTATE_ON) {
            publish_to_smoke_alarm_topic("on", 2);
            last_published_state = LASTPUBLISHEDSTATE_ON;
        } else if(!level && last_published_state != LASTPUBLISHEDSTATE_OFF) {
            publish_to_smoke_alarm_topic("off", 3);
            last_published_state = LASTPUBLISHEDSTATE_OFF;
        }
        vTaskDelay(pdMS_TO_TICKS(SMOKE_ALARM_READ_INTERVAL_MS));
    }
}

void app_main()
{
    nvs_flash_init();

    init_gpio();
    init_mqtt();
    init_wifi();

    xTaskCreate(&task_monitor_smoke_alarm_relay, "smoke_alarm_trigger", 8192, NULL, 5, NULL);
}
