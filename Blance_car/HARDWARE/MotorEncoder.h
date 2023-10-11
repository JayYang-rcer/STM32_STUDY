#ifndef __MOTORENCODER_H 
#define __MOTORENCODER_H 
#include "sys.h"
#include "TB6612.h"

void Encoder_Init_TIM2(void); 
void Encoder_Init_TIM4(void);
int Read_Encoder(u8 TIMX);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);

#endif
