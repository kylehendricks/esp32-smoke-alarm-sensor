#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event_loop.h"

#include "mqtt_client.h"

static const char *TAG = "smoke_alarm";

enum LastPublishedState {
    LASTPUBLISHEDSTATE_UNPUBLISHED,
    LASTPUBLISHEDSTATE_OFF,
    LASTPUBLISHEDSTATE_ON
};

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    esp_mqtt_client_handle_t mqtt_client = ctx;

    switch(event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 1);

            ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            gpio_set_level((gpio_num_t) CONFIG_PIN_WIFI_STATUS_LED, 0);

            ESP_ERROR_CHECK(esp_mqtt_client_stop(mqtt_client));

            ESP_ERROR_CHECK(esp_wifi_connect());

            break;
        default:
            break;
    }

    return ESP_OK;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_publish(event->client, CONFIG_MQTT_LWT_TOPIC, "online", 6, 1, 1);
            break;
        default:
            break;
    }

    return ESP_OK;
}

// Only initializes the mqtt client, connection happens after an IP is gotten
static esp_mqtt_client_handle_t init_mqtt() {
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

    esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&settings);

    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create mqtt client!");
        esp_restart();
    }

    return mqtt_client;
}

static void init_wifi(esp_mqtt_client_handle_t mqtt_client)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, mqtt_client) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t sta_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASSWORD,
                    .bssid_set = false,
            }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, TAG));
    ESP_ERROR_CHECK(esp_wifi_connect());
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

static int publish_to_smoke_alarm_topic(esp_mqtt_client_handle_t mqtt_client, const char *msg, int len) {
    return esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_ALARM_TOPIC, msg, len, 1, 1);
}

static void task_monitor_smoke_alarm_relay(void *pvParameter) {
    esp_mqtt_client_handle_t mqtt_client = pvParameter;
    enum LastPublishedState last_published_state = LASTPUBLISHEDSTATE_UNPUBLISHED;

    while(true) {
        int level = gpio_get_level((gpio_num_t) CONFIG_PIN_SMOKE_ALARM_RELAY);

        if (level && last_published_state != LASTPUBLISHEDSTATE_ON) {
            if (publish_to_smoke_alarm_topic(mqtt_client, "on", 2) >= 0) {
                last_published_state = LASTPUBLISHEDSTATE_ON;
            }
        } else if(!level && last_published_state != LASTPUBLISHEDSTATE_OFF) {
            if (publish_to_smoke_alarm_topic(mqtt_client, "off", 3) >= 0) {
                last_published_state = LASTPUBLISHEDSTATE_OFF;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_SMOKE_ALARM_PIN_READ_INTERVAL_MS));
    }
}

void app_main()
{
    nvs_flash_init();

    init_gpio();
    esp_mqtt_client_handle_t mqtt_client = init_mqtt();
    init_wifi(mqtt_client);

    xTaskCreate(&task_monitor_smoke_alarm_relay, "smoke_alarm_trigger", 8192, mqtt_client, 5, NULL);
}
