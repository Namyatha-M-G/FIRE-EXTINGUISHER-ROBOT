#ifndef RELAY_H
#define RELAY_H

#include <stdbool.h>

void relay_init(void);

void relay_on(void);
void relay_off(void);
bool relay_is_on(void);
void relay_set_raw_level(bool high);
int relay_gpio_level(void);
int relay_active_level(void);

#endif
