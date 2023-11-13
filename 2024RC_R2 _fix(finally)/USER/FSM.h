#ifndef __FSM_H
#define __FSM_H

#include "main.h"
void Robot_Control_Mode(void);
void Robot_stop(void);

extern unsigned char u8_drive_flag;

extern unsigned char STM32_FSM_OPEN_FLAG;
extern unsigned char ROS_AUTO_CONTROL;

#endif
