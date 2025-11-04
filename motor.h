#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"

void motor_init(uint m1a_pin, uint m1b_pin, uint m2a_pin, uint m2b_pin);
void motor_direction_set(int dirA, int dirB);
void motor_set_speed(float left, float right);
void motor_stop(void);
// Move robot in basic directions (forward, backward, left, right, stop)
// speed in percentage (0–100), ramp true/false
void move_robot(const char *action, uint32_t duration_ms, bool ramp, float speed);
// Move robot with custom left/right speeds (0–100)
void move_robot_custom(uint32_t duration_ms, bool ramp, float left_speed, float right_speed);


#endif
