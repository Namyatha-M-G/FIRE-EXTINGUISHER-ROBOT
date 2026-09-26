#include "motor.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "robot_pins.h"

#define MOTOR_PWM_TIMER LEDC_TIMER_1
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define MOTOR_PWM_FREQ_HZ 1000
#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_2
#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_3
#define MOTOR_PWM_MAX_DUTY 1023

static const char *TAG = "MOTOR";
static int s_base_speed_percent = 70;
static motor_state_t s_motor_state = MOTOR_STATE_STOP;

static int clamp_percent(int percent)
{
    if (percent < 0) {
        return 0;
    }
    if (percent > 100) {
        return 100;
    }
    return percent;
}

static void set_pwm(ledc_channel_t channel, int percent)
{
    uint32_t duty = (MOTOR_PWM_MAX_DUTY * clamp_percent(percent)) / 100;
    ledc_set_duty(MOTOR_PWM_MODE, channel, duty);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

void motor_init(void)
{
    gpio_reset_pin(MOTOR_IN1_PIN);
    gpio_reset_pin(MOTOR_IN2_PIN);
    gpio_reset_pin(MOTOR_IN3_PIN);
    gpio_reset_pin(MOTOR_IN4_PIN);

    gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN4_PIN, GPIO_MODE_OUTPUT);

    ledc_timer_config_t timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t left_channel = {
        .gpio_num = MOTOR_ENB_PIN,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_LEFT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&left_channel);

    ledc_channel_config_t right_channel = {
        .gpio_num = MOTOR_ENA_PIN,
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_RIGHT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&right_channel);

    motor_stop();

    ESP_LOGI(TAG, "Motor Initialized");
}

void motor_set_speed(int left_percent, int right_percent)
{
    set_pwm(MOTOR_LEFT_CHANNEL, left_percent);
    set_pwm(MOTOR_RIGHT_CHANNEL, right_percent);
}

void motor_set_base_speed(int percent)
{
    s_base_speed_percent = clamp_percent(percent);
    motor_set_speed(s_base_speed_percent, s_base_speed_percent);
}

void motor_forward(void)
{
    s_motor_state = MOTOR_STATE_FORWARD;
    motor_set_speed(s_base_speed_percent, s_base_speed_percent);
    gpio_set_level(MOTOR_IN1_PIN, 1);
    gpio_set_level(MOTOR_IN2_PIN, 0);

    gpio_set_level(MOTOR_IN3_PIN, 1);
    gpio_set_level(MOTOR_IN4_PIN, 0);

    ESP_LOGI(TAG, "FORWARD");
}

void motor_reverse(void)
{
    s_motor_state = MOTOR_STATE_REVERSE;
    motor_set_speed(s_base_speed_percent, s_base_speed_percent);
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 1);

    gpio_set_level(MOTOR_IN3_PIN, 0);
    gpio_set_level(MOTOR_IN4_PIN, 1);

    ESP_LOGI(TAG, "REVERSE");
}

void motor_left(void)
{
    s_motor_state = MOTOR_STATE_LEFT;
    int turn_speed = s_base_speed_percent;
    if (turn_speed < 85) {
        turn_speed = 85; // boost turn speed to overcome friction
    }
    motor_set_speed(turn_speed, turn_speed);
    // Left motor BACKWARD
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 1);

    // Right motor FORWARD
    gpio_set_level(MOTOR_IN3_PIN, 1);
    gpio_set_level(MOTOR_IN4_PIN, 0);

    ESP_LOGI(TAG, "LEFT (boost speed=%d)", turn_speed);
}

void motor_right(void)
{
    s_motor_state = MOTOR_STATE_RIGHT;
    int turn_speed = s_base_speed_percent;
    if (turn_speed < 85) {
        turn_speed = 85; // boost turn speed to overcome friction
    }
    motor_set_speed(turn_speed, turn_speed);
    // Left motor FORWARD
    gpio_set_level(MOTOR_IN1_PIN, 1);
    gpio_set_level(MOTOR_IN2_PIN, 0);

    // Right motor BACKWARD
    gpio_set_level(MOTOR_IN3_PIN, 0);
    gpio_set_level(MOTOR_IN4_PIN, 1);

    ESP_LOGI(TAG, "RIGHT (boost speed=%d)", turn_speed);
}

void motor_stop(void)
{
    s_motor_state = MOTOR_STATE_STOP;
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 0);

    gpio_set_level(MOTOR_IN3_PIN, 0);
    gpio_set_level(MOTOR_IN4_PIN, 0);
    motor_set_speed(0, 0);

    ESP_LOGI(TAG, "STOP");
}

motor_state_t motor_get_state(void)
{
    return s_motor_state;
}

int motor_get_base_speed(void)
{
    return s_base_speed_percent;
}
