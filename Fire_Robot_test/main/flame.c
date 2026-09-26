#include "flame.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "robot_pins.h"

#define FLAME_ACTIVE_LEVEL 0

static const char *TAG = "FLAME";

static bool read_sensor(gpio_num_t pin)
{
    return gpio_get_level(pin) == FLAME_ACTIVE_LEVEL;
}

void flame_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << FLAME_FRONT_PIN) |
                        (1ULL << FLAME_LEFT_PIN) |
                        (1ULL << FLAME_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&config);
    ESP_LOGI(TAG, "Flame sensors initialized");
}

bool flame_front(void)
{
    return read_sensor(FLAME_FRONT_PIN);
}

bool flame_left(void)
{
    return read_sensor(FLAME_LEFT_PIN);
}

bool flame_right(void)
{
    return read_sensor(FLAME_RIGHT_PIN);
}

bool flame_any(void)
{
    return flame_front() || flame_left() || flame_right();
}
