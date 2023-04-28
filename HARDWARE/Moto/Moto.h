#ifndef __MOTO_H
#define __MOTO_H
#include "sys.h"

#define Lmotor 0
#define Rmotor 1

#define Lturn 0
#define Rturn 1

//TB6612FNG双驱动的输入IO
#define STBY    PGout(7)  
#define xianfu  980        //限幅 
void motor_init();
void motor_control(unsigned char ch, int speed);
void TB6612FNG_IO_init();		//双TB6612FNG驱动IO初始化

void SET_Front_left_PWM(int speed);
void SET_Front_right_PWM(int speed);
void SET_Behind_left_PWM(int speed);
void SET_Behind_right_PWM(int speed);

/* 函数功能：绝对值函数 */	
int myabs(int a);

#endif
