#include "servo.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "robot_pins.h"

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

static const char *TAG = "SERVO";

void servo_init(void)
{
    ledc_timer_config_t timer =
    {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel =
    {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel);

    ESP_LOGI(TAG,"Servo Initialized");
}

void servo_set_angle(int angle)
{
    if (angle < 0) {
        angle = 0;
    }
    if (angle > 180) {
        angle = 180;
    }

    uint32_t pulse_width =
        SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;

    uint32_t duty =
        (pulse_width * 8191) / 20000;

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        duty);

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0);

    ESP_LOGI(TAG,"Angle = %d",angle);
}

void servo_sweep(int start_angle, int end_angle, int step, int delay_ms)
{
    if (step == 0) {
        step = 10;
    }
    if (start_angle > end_angle && step > 0) {
        step = -step;
    }
    if (start_angle < end_angle && step < 0) {
        step = -step;
    }

    for (int angle = start_angle;
         (step > 0) ? (angle <= end_angle) : (angle >= end_angle);
         angle += step) {
        servo_set_angle(angle);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
