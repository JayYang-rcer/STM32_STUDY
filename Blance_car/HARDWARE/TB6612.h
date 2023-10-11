#ifndef __TB6612_H 
#define __TB6612_H 
#include "sys.h"

#define PWMB   TIM1->CCR4  //PA11
#define BIN2   PBout(12)
#define BIN1   PBout(13)
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define PWMA   TIM1->CCR1  //PA8

void TB6612_Init(int arr, int psc);
void SetPWM(int pwm);

#endif //定义完毕，或者引用过头文件到达这一步
