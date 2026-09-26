#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_fsm.h"

esp_err_t oled_init(void);
void oled_clear(void);
void oled_print_line(uint8_t line, const char *text);
void oled_show_status(bool auto_mode,
                      robot_state_t state,
                      bool flame_left,
                      bool flame_front,
                      bool flame_right,
                      bool pump_on);

#endif
