#include "relay.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "robot_pins.h"

#define RELAY_ACTIVE_LEVEL 1

static const char *TAG = "RELAY";
static bool s_relay_on;

void relay_init(void)
{
    gpio_reset_pin(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    relay_off();
    ESP_LOGI(TAG, "Relay initialized");
}

void relay_on(void)
{
    gpio_set_level(RELAY_PIN, RELAY_ACTIVE_LEVEL);
    s_relay_on = true;
    ESP_LOGI(TAG, "Relay ON gpio=%d active=%d", relay_gpio_level(), RELAY_ACTIVE_LEVEL);
}

void relay_off(void)
{
    gpio_set_level(RELAY_PIN, !RELAY_ACTIVE_LEVEL);
    s_relay_on = false;
    ESP_LOGI(TAG, "Relay OFF gpio=%d active=%d", relay_gpio_level(), RELAY_ACTIVE_LEVEL);
}

bool relay_is_on(void)
{
    return s_relay_on;
}

void relay_set_raw_level(bool high)
{
    gpio_set_level(RELAY_PIN, high ? 1 : 0);
    s_relay_on = gpio_get_level(RELAY_PIN) == RELAY_ACTIVE_LEVEL;
    ESP_LOGI(TAG, "Relay raw GPIO set to %d, interpreted_on=%d", relay_gpio_level(), s_relay_on);
}

int relay_gpio_level(void)
{
    return gpio_get_level(RELAY_PIN);
}

int relay_active_level(void)
{
    return RELAY_ACTIVE_LEVEL;
}
