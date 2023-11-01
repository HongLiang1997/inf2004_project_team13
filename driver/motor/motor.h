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

void initMotor(void *params);
void setLeftSpeed(float speed);
void setRightSpeed(float speed);
void stop(void *params);
void moveForward(void *params);
void turnHardLeft(void *params);
void turnHardRight(void *params);

#endif