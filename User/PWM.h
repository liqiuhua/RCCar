#ifndef __PWM_H_
#define __PWM_H_

#include "stdint.h"
void PWM_INPUT_Init(void);
void PWM_OUTPUT_Init(void);
void PWM_out(uint8_t ch,uint16_t data);
uint32_t getTIM3ICValue(uint8_t channel);
#endif

