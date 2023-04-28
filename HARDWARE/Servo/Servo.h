#ifndef __SERVO_H
#define __SERVO_H
#include "sys.h"

void Duoji_PWM_Init(void);

void TIM8_PWM_Init(uint32_t arr,uint32_t psc);
void TIM10_PWM_Init(uint32_t arr,uint32_t psc);
#endif
