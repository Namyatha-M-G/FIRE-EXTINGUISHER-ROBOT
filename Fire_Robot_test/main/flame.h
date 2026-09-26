#ifndef FLAME_H
#define FLAME_H

#include <stdbool.h>

void flame_init(void);

bool flame_front(void);
bool flame_left(void);
bool flame_right(void);
bool flame_any(void);

#endif
