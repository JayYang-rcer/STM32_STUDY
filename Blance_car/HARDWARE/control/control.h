#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"


#define Middle_angle 0
int EXTI15_10_IRQHandler(void);
int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
void Set_Pwm(int motor_left,int motor_right);
void Key(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle);
void Get_Angle(uint8_t angle_way);
int myabs(int a);
#endif
