#ifndef MOTOR_H
#define MOTOR_H

typedef enum {
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_FORWARD,
    MOTOR_STATE_REVERSE,
    MOTOR_STATE_LEFT,
    MOTOR_STATE_RIGHT
} motor_state_t;

void motor_init(void);

void motor_set_speed(int left_percent, int right_percent);
void motor_set_base_speed(int percent);
int motor_get_base_speed(void);

void motor_forward(void);
void motor_reverse(void);

void motor_left(void);
void motor_right(void);

void motor_stop(void);

motor_state_t motor_get_state(void);

#endif
