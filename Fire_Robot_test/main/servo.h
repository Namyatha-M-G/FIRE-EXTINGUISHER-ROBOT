#ifndef SERVO_H
#define SERVO_H

void servo_init(void);
void servo_set_angle(int angle);
void servo_sweep(int start_angle, int end_angle, int step, int delay_ms);

#endif // SERVO_H
