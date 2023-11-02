#ifndef _MOTOR_H
#define _MOTOR_H

#define LEFT_WHEEL 14
#define RIGHT_WHEEL 15
#define RIGHT_WHEEL_FORWARD 18
#define RIGHT_WHEEL_BACKWARD 19
#define LEFT_WHEEL_FORWARD 20
#define LEFT_WHEEL_BACKWARD 21
#define CLK_DIV 100
#define WRAP 12500

void init_motor(void *params);
void set_left_speed(float speed);
void set_right_speed(float speed);
void stop(void *params);
void move_forward(void *params);
void turn_left(void *params);
void turn_right(void *params);

#endif