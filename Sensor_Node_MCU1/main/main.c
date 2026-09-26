# FIRMWARE CODE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define WIFI_SSID "FireRobot"
#define WIFI_PASS "fire12345"

#define FLAME_PIN GPIO_NUM_14
#define GAS_ADC_CHANNEL ADC_CHANNEL_6 // GPIO 34 is ADC1 Channel 6
#define NODE_ID 1                      // Change to 2 for second node

static const char *TAG = "SENSOR_NODE";
static bool s_wifi_connected = false;
static adc_oneshot_unit_handle_t s_adc1_handle;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA initialized.");
}

static void send_alert(int flame_detected, int gas_val)
{
    if (!s_wifi_connected) {
        return;
    }

    char url[128];
    snprintf(url, sizeof(url), "http://192.168.4.1/alert?node=%d&flame=%d&gas=%d",
             NODE_ID, flame_detected, gas_val);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 2000,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Alert sent: Node=%d Flame=%d Gas=%d (HTTP status %d)",
                 NODE_ID, flame_detected, gas_val, esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "Failed to send alert: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

void sensor_task(void *pvParameters)
{
    // Initialize GPIO for Digital Flame Sensor
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLAME_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Initialize ADC for MQ2 Analog Gas Sensor
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, GAS_ADC_CHANNEL, &config));

    ESP_LOGI(TAG, "Sensors initialized.");

    while (1) {
        // Read flame sensor (usually Active Low, 0 means flame detected)
        int flame_raw = gpio_get_level(FLAME_PIN);
        int flame_detected = (flame_raw == 0) ? 1 : 0;

        // Read analog gas level
        int gas_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc1_handle, GAS_ADC_CHANNEL, &gas_raw));

        ESP_LOGI(TAG, "Flame: %d | Gas raw: %d", flame_detected, gas_raw);

        // Send updates/alerts to Robot AP
        send_alert(flame_detected, gas_raw);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
