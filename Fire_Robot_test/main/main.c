#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include <math.h>

#include "camera_link.h"
#include "flame.h"
#include "motor.h"
#include "oled.h"
#include "relay.h"
#include "robot_fsm.h"
#include "servo.h"
#include "wifi_server.h"

static const char *TAG = "MAIN";

static volatile bool s_auto_mode = true;
static volatile bool s_pump_on = false;
static robot_state_t s_state = ROBOT_IDLE;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float s_robot_x = 0.0f;
static float s_robot_y = 0.0f;
static float s_robot_heading = 0.0f; // radians

static float s_speed_m_s = 0.20f;
static float s_turn_rad_s = 1.50f;
#define POSITION_UPDATE_MS 50

#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

typedef struct {
    float x;
    float y;
    bool active_alert;
    bool has_flame;
    int gas_level;
    char name[32];
    bool alert_handled;
} sensor_node_t;

static sensor_node_t s_nodes[2] = {
    { .x = 1.5f, .y = 0.0f, .active_alert = false, .has_flame = false, .gas_level = 0, .name = "Node 1", .alert_handled = false },
    { .x = 1.0f, .y = -1.0f, .active_alert = false, .has_flame = false, .gas_level = 0, .name = "Node 2", .alert_handled = false }
};

static int s_current_target_node = -1;

void get_robot_position(float *x, float *y, float *heading) {
    *x = s_robot_x;
    *y = s_robot_y;
    *heading = s_robot_heading;
}

void reset_robot_position(void) {
    s_robot_x = 0.0f;
    s_robot_y = 0.0f;
    s_robot_heading = 0.0f;
}

void set_node_alert(int node_id, bool has_flame, int gas_level) {
    if (node_id >= 1 && node_id <= 2) {
        s_nodes[node_id - 1].has_flame = has_flame;
        s_nodes[node_id - 1].gas_level = gas_level;
        bool is_alerting = has_flame || (gas_level > 400);
        s_nodes[node_id - 1].active_alert = is_alerting;
        if (!is_alerting) {
            s_nodes[node_id - 1].alert_handled = false;
        }
    }
}

void get_node_status(int node_id, bool *has_flame, int *gas_level, bool *active_alert) {
    if (node_id >= 1 && node_id <= 2) {
        *has_flame = s_nodes[node_id - 1].has_flame;
        *gas_level = s_nodes[node_id - 1].gas_level;
        *active_alert = s_nodes[node_id - 1].active_alert;
    }
}

void save_node_config_to_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        // Node 1
        int32_t x1_mm = (int32_t)(s_nodes[0].x * 1000.0f);
        int32_t y1_mm = (int32_t)(s_nodes[0].y * 1000.0f);
        nvs_set_i32(my_handle, "node1_x", x1_mm);
        nvs_set_i32(my_handle, "node1_y", y1_mm);
        nvs_set_str(my_handle, "node1_name", s_nodes[0].name);

        // Node 2
        int32_t x2_mm = (int32_t)(s_nodes[1].x * 1000.0f);
        int32_t y2_mm = (int32_t)(s_nodes[1].y * 1000.0f);
        nvs_set_i32(my_handle, "node2_x", x2_mm);
        nvs_set_i32(my_handle, "node2_y", y2_mm);
        nvs_set_str(my_handle, "node2_name", s_nodes[1].name);

        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS Commit failed!");
        }
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Node configs saved to NVS.");
    } else {
        ESP_LOGE(TAG, "Error opening NVS storage namespace: %s", esp_err_to_name(err));
    }
}

void save_calibration_to_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        int32_t speed_val = (int32_t)(s_speed_m_s * 1000.0f);
        int32_t turn_val = (int32_t)(s_turn_rad_s * 1000.0f);
        nvs_set_i32(my_handle, "cal_speed", speed_val);
        nvs_set_i32(my_handle, "cal_turn", turn_val);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Calibration saved to NVS: speed=%d, turn=%d", speed_val, turn_val);
    } else {
        ESP_LOGE(TAG, "Error opening NVS for calibration save: %s", esp_err_to_name(err));
    }
}

void load_calibration_from_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        int32_t speed_val, turn_val;
        if (nvs_get_i32(my_handle, "cal_speed", &speed_val) == ESP_OK) {
            s_speed_m_s = (float)speed_val / 1000.0f;
        }
        if (nvs_get_i32(my_handle, "cal_turn", &turn_val) == ESP_OK) {
            s_turn_rad_s = (float)turn_val / 1000.0f;
        }
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Calibration loaded from NVS: speed=%.3f, turn=%.3f", s_speed_m_s, s_turn_rad_s);
    } else {
        ESP_LOGI(TAG, "NVS storage namespace not found for calibration. Using defaults.");
    }
}

void get_calibration_values(float *speed, float *turn) {
    *speed = s_speed_m_s;
    *turn = s_turn_rad_s;
}

void set_calibration_values(float speed, float turn) {
    s_speed_m_s = speed;
    s_turn_rad_s = turn;
    save_calibration_to_nvs();
}


void load_node_config_from_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        int32_t x_mm;
        size_t required_len;

        // Node 1
        if (nvs_get_i32(my_handle, "node1_x", &x_mm) == ESP_OK) {
            s_nodes[0].x = (float)x_mm / 1000.0f;
        }
        if (nvs_get_i32(my_handle, "node1_y", &x_mm) == ESP_OK) {
            s_nodes[0].y = (float)x_mm / 1000.0f;
        }
        required_len = sizeof(s_nodes[0].name);
        nvs_get_str(my_handle, "node1_name", s_nodes[0].name, &required_len);

        // Node 2
        if (nvs_get_i32(my_handle, "node2_x", &x_mm) == ESP_OK) {
            s_nodes[1].x = (float)x_mm / 1000.0f;
        }
        if (nvs_get_i32(my_handle, "node2_y", &x_mm) == ESP_OK) {
            s_nodes[1].y = (float)x_mm / 1000.0f;
        }
        required_len = sizeof(s_nodes[1].name);
        nvs_get_str(my_handle, "node2_name", s_nodes[1].name, &required_len);

        nvs_close(my_handle);
        ESP_LOGI(TAG, "Node configs loaded from NVS: Node1=(%.2f, %.2f, %s) Node2=(%.2f, %.2f, %s)",
                 s_nodes[0].x, s_nodes[0].y, s_nodes[0].name,
                 s_nodes[1].x, s_nodes[1].y, s_nodes[1].name);
    } else {
        ESP_LOGI(TAG, "NVS storage namespace not found. Using defaults.");
    }
}

void update_node_config(int id, float x, float y, const char *name) {
    if (id >= 1 && id <= 2) {
        s_nodes[id - 1].x = x;
        s_nodes[id - 1].y = y;
        if (name && strlen(name) > 0) {
            strncpy(s_nodes[id - 1].name, name, sizeof(s_nodes[id - 1].name) - 1);
            s_nodes[id - 1].name[sizeof(s_nodes[id - 1].name) - 1] = '\0';
        }
        save_node_config_to_nvs();
    }
}

void get_node_config(int id, float *x, float *y, char *name, size_t max_len) {
    if (id >= 1 && id <= 2) {
        *x = s_nodes[id - 1].x;
        *y = s_nodes[id - 1].y;
        strncpy(name, s_nodes[id - 1].name, max_len - 1);
        name[max_len - 1] = '\0';
    }
}

static void position_tracker_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    float dt = (float)POSITION_UPDATE_MS / 1000.0f;

    while (1) {
        motor_state_t state = motor_get_state();
        if (state == MOTOR_STATE_FORWARD) {
            s_robot_x += s_speed_m_s * cosf(s_robot_heading) * dt;
            s_robot_y += s_speed_m_s * sinf(s_robot_heading) * dt;
        } else if (state == MOTOR_STATE_REVERSE) {
            s_robot_x -= s_speed_m_s * cosf(s_robot_heading) * dt;
            s_robot_y -= s_speed_m_s * sinf(s_robot_heading) * dt;
        } else if (state == MOTOR_STATE_LEFT) {
            s_robot_heading += s_turn_rad_s * dt;
        } else if (state == MOTOR_STATE_RIGHT) {
            s_robot_heading -= s_turn_rad_s * dt;
        }

        // Keep heading in range [-PI, PI]
        if (s_robot_heading > M_PI) s_robot_heading -= 2.0f * M_PI;
        else if (s_robot_heading < -M_PI) s_robot_heading += 2.0f * M_PI;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POSITION_UPDATE_MS));
    }
}

static void extinguish_fire(void)
{
    ESP_LOGI(TAG, "Extinguishing fire");
    motor_stop();
    relay_on();
    s_pump_on = true;
    servo_sweep(55, 125, 10, 120);
    servo_sweep(125, 55, -10, 120);
    servo_set_angle(90);
    relay_off();
    s_pump_on = false;
}

static void wifi_command_handler(wifi_server_command_t command)
{
    switch (command) {
    case WIFI_CMD_AUTO:
        s_auto_mode = true;
        s_state = ROBOT_IDLE;
        relay_off();
        s_pump_on = false;
        motor_stop();
        break;
    case WIFI_CMD_MANUAL:
        s_auto_mode = false;
        motor_stop();
        break;
    case WIFI_CMD_FORWARD:
        s_auto_mode = false;
        motor_forward();
        break;
    case WIFI_CMD_REVERSE:
        s_auto_mode = false;
        motor_reverse();
        break;
    case WIFI_CMD_LEFT:
        s_auto_mode = false;
        motor_left();
        break;
    case WIFI_CMD_RIGHT:
        s_auto_mode = false;
        motor_right();
        break;
    case WIFI_CMD_STOP:
        s_auto_mode = false;
        relay_off();
        s_pump_on = false;
        motor_stop();
        break;
    case WIFI_CMD_PUMP_ON:
        s_auto_mode = false;
        relay_on();
        s_pump_on = true;
        break;
    case WIFI_CMD_PUMP_OFF:
        relay_off();
        s_pump_on = false;
        break;
    case WIFI_CMD_RELAY_LOW:
        s_auto_mode = false;
        relay_set_raw_level(false);
        s_pump_on = relay_is_on();
        break;
    case WIFI_CMD_RELAY_HIGH:
        s_auto_mode = false;
        relay_set_raw_level(true);
        s_pump_on = relay_is_on();
        break;
    case WIFI_CMD_SWEEP:
        s_auto_mode = false;
        extinguish_fire();
        break;
    default:
        break;
    }
}

static bool search_for_fire_sequence(void)
{
    ESP_LOGI(TAG, "Starting fire search sequence...");

    typedef struct {
        void (*action)(void);
        uint32_t duration_ms;
        const char *name;
    } search_step_t;

    search_step_t steps[] = {
        { motor_left,    600,  "rotating left" },
        { motor_right,   1200, "rotating right" },
        { motor_left,    600,  "rotating back to center" },
        { motor_forward, 600,  "moving forward" },
        { motor_reverse, 600,  "moving backward" },
        { motor_stop,    100,  "stopping" }
    };

    int num_steps = sizeof(steps) / sizeof(steps[0]);

    for (int i = 0; i < num_steps; i++) {
        ESP_LOGI(TAG, "Search step: %s", steps[i].name);
        steps[i].action();

        uint32_t elapsed = 0;
        while (elapsed < steps[i].duration_ms) {
            if (!s_auto_mode) {
                motor_stop();
                return false;
            }
            if (flame_any()) {
                motor_stop();
                ESP_LOGI(TAG, "Fire detected during search step: %s!", steps[i].name);
                return true;
            }
            oled_show_status(s_auto_mode,
                             s_state,
                             flame_left(),
                             flame_front(),
                             flame_right(),
                             s_pump_on);
            vTaskDelay(pdMS_TO_TICKS(50));
            elapsed += 50;
        }
    }

    motor_stop();
    ESP_LOGI(TAG, "Fire search sequence completed, no fire found.");
    return false;
}

static void autonomous_step(void)
{
    // Check for sensor node alerts (ignoring already handled alerts)
    int alert_node = -1;
    for (int i = 0; i < 2; i++) {
        if (s_nodes[i].active_alert && !s_nodes[i].alert_handled) {
            alert_node = i + 1;
            break;
        }
    }

    // State transition logic
    if (s_state == ROBOT_IDLE) {
        if (flame_any()) {
            s_state = ROBOT_SEARCH_FIRE;
            ESP_LOGI(TAG, "Local flame detected in IDLE! Activating fire suppression.");
        } else if (alert_node != -1) {
            s_state = ROBOT_NAV_TO_NODE;
            s_current_target_node = alert_node;
        } else {
            float dx = 0.0f - s_robot_x;
            float dy = 0.0f - s_robot_y;
            float dist_home = sqrtf(dx * dx + dy * dy);
            if (dist_home > 0.15f) {
                s_state = ROBOT_NAV_TO_HOME;
                ESP_LOGI(TAG, "No alert & not home (dist=%.2fm). Going home.", dist_home);
            }
        }
    } else if (s_state == ROBOT_NAV_TO_NODE) {
        if (alert_node == -1) {
            s_state = ROBOT_NAV_TO_HOME;
            s_current_target_node = -1;
            ESP_LOGI(TAG, "Alert cleared. Navigating back home.");
        }
    } else if (s_state == ROBOT_NAV_TO_HOME) {
        if (flame_any()) {
            s_state = ROBOT_SEARCH_FIRE;
            ESP_LOGI(TAG, "Local flame detected while going home! Activating fire suppression.");
        } else if (alert_node != -1) {
            s_state = ROBOT_NAV_TO_NODE;
            s_current_target_node = alert_node;
            ESP_LOGI(TAG, "Alert received while returning home. Re-routing to Node %d.", alert_node);
        }
    }

    ESP_LOGI(TAG, "state=%d flame L=%d F=%d R=%d alert_node=%d current_target=%d", s_state, flame_left(), flame_front(), flame_right(), alert_node, s_current_target_node);

    switch (s_state) {
    case ROBOT_IDLE:
        motor_stop();
        servo_set_angle(90);
        relay_off();
        s_pump_on = false;
        if (s_current_target_node != -1) {
            s_nodes[s_current_target_node - 1].alert_handled = true;
            ESP_LOGI(TAG, "Target node %d alert marked as handled.", s_current_target_node);
            s_current_target_node = -1;
        }
        break;

    case ROBOT_NAV_TO_NODE: {
        if (alert_node == -1) {
            s_state = ROBOT_IDLE;
            motor_stop();
            break;
        }
        float target_x = s_nodes[alert_node - 1].x;
        float target_y = s_nodes[alert_node - 1].y;
        float dx = target_x - s_robot_x;
        float dy = target_y - s_robot_y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance < 0.2f) {
            ESP_LOGI(TAG, "Arrived at Node %d, starting fire scan", alert_node);
            motor_stop();
            s_nodes[alert_node - 1].active_alert = false;
            s_state = ROBOT_SEARCH_FIRE;
        } else {
            float target_heading = atan2f(dy, dx);
            float angle_err = target_heading - s_robot_heading;
            
            if (angle_err > M_PI) angle_err -= 2.0f * M_PI;
            else if (angle_err < -M_PI) angle_err += 2.0f * M_PI;

            if (fabsf(angle_err) > 0.15f) {
                if (angle_err > 0) {
                    motor_left();
                } else {
                    motor_right();
                }
            } else {
                motor_forward();
            }
        }
        break;
    }

    case ROBOT_NAV_TO_HOME: {
        float dx = 0.0f - s_robot_x;
        float dy = 0.0f - s_robot_y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance < 0.15f) {
            // Arrived home, align front heading back to 0.0 degrees (facing forward)
            float angle_err = 0.0f - s_robot_heading;
            if (angle_err > M_PI) angle_err -= 2.0f * M_PI;
            else if (angle_err < -M_PI) angle_err += 2.0f * M_PI;

            if (fabsf(angle_err) > 0.15f) {
                if (angle_err > 0) {
                    motor_left();
                } else {
                    motor_right();
                }
            } else {
                ESP_LOGI(TAG, "Arrived home and aligned. Standing by.");
                motor_stop();
                s_state = ROBOT_IDLE;
            }
        } else {
            // Navigate BACKWARD to home
            float target_heading = atan2f(dy, dx);
            float backward_heading = s_robot_heading + M_PI;
            if (backward_heading > M_PI) {
                backward_heading -= 2.0f * M_PI;
            }
            
            float angle_err = target_heading - backward_heading;
            if (angle_err > M_PI) angle_err -= 2.0f * M_PI;
            else if (angle_err < -M_PI) angle_err += 2.0f * M_PI;

            if (fabsf(angle_err) > 0.15f) {
                if (angle_err > 0) {
                    motor_left();
                } else {
                    motor_right();
                }
            } else {
                motor_reverse();
            }
        }
        break;
    }

    case ROBOT_SEARCH_FIRE: {
        bool found = search_for_fire_sequence();
        if (found) {
            if (flame_front()) {
                s_state = ROBOT_EXTINGUISH;
            } else if (flame_left()) {
                s_state = ROBOT_TURN_LEFT;
            } else if (flame_right()) {
                s_state = ROBOT_TURN_RIGHT;
            } else {
                s_state = ROBOT_EXTINGUISH;
            }
        } else {
            s_state = ROBOT_IDLE;
        }
        break;
    }

    case ROBOT_EXTINGUISH:
        extinguish_fire();
        s_state = flame_any() ? ROBOT_VERIFY : ROBOT_IDLE;
        break;

    case ROBOT_TURN_LEFT:
        motor_left();
        vTaskDelay(pdMS_TO_TICKS(350));
        motor_stop();
        if (flame_front()) {
            s_state = ROBOT_EXTINGUISH;
        } else if (flame_left()) {
            s_state = ROBOT_TURN_LEFT;
        } else if (flame_right()) {
            s_state = ROBOT_TURN_RIGHT;
        } else {
            s_state = ROBOT_IDLE;
        }
        break;

    case ROBOT_TURN_RIGHT:
        motor_right();
        vTaskDelay(pdMS_TO_TICKS(350));
        motor_stop();
        if (flame_front()) {
            s_state = ROBOT_EXTINGUISH;
        } else if (flame_left()) {
            s_state = ROBOT_TURN_LEFT;
        } else if (flame_right()) {
            s_state = ROBOT_TURN_RIGHT;
        } else {
            s_state = ROBOT_IDLE;
        }
        break;

    case ROBOT_VERIFY:
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(500));
        if (flame_any()) {
            if (flame_front()) {
                s_state = ROBOT_EXTINGUISH;
            } else if (flame_left()) {
                s_state = ROBOT_TURN_LEFT;
            } else if (flame_right()) {
                s_state = ROBOT_TURN_RIGHT;
            }
        } else {
            s_state = ROBOT_IDLE;
        }
        break;

    default:
        motor_stop();
        s_state = ROBOT_IDLE;
        break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Fire fighting robot firmware start");

    motor_init();
    motor_set_base_speed(70);
    xTaskCreate(position_tracker_task, "position_tracker", 2048, NULL, 5, NULL);
    servo_init();
    servo_set_angle(90);
    relay_init();
    flame_init();
    oled_init();
    camera_link_init();

    esp_err_t wifi_err = wifi_server_start(wifi_command_handler);
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi server failed: %s", esp_err_to_name(wifi_err));
    }

    extern void load_node_config_from_nvs(void);
    load_node_config_from_nvs();

    extern void load_calibration_from_nvs(void);
    load_calibration_from_nvs();

    while (1) {
        if (s_auto_mode) {
            autonomous_step();
        }

        oled_show_status(s_auto_mode,
                         s_state,
                         flame_left(),
                         flame_front(),
                         flame_right(),
                         s_pump_on);

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

bool get_auto_mode(void) {
    return s_auto_mode;
}

int get_robot_state(void) {
    return (int)s_state;
}
