#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "esp_err.h"

typedef enum {
    WIFI_CMD_AUTO,
    WIFI_CMD_MANUAL,
    WIFI_CMD_FORWARD,
    WIFI_CMD_REVERSE,
    WIFI_CMD_LEFT,
    WIFI_CMD_RIGHT,
    WIFI_CMD_STOP,
    WIFI_CMD_PUMP_ON,
    WIFI_CMD_PUMP_OFF,
    WIFI_CMD_RELAY_LOW,
    WIFI_CMD_RELAY_HIGH,
    WIFI_CMD_SWEEP,
} wifi_server_command_t;

typedef void (*wifi_server_command_cb_t)(wifi_server_command_t command);

esp_err_t wifi_server_start(wifi_server_command_cb_t command_cb);

#endif
