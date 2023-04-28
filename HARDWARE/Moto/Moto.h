#ifndef __MOTO_H
#define __MOTO_H
#include "sys.h"

#define Lmotor 0
#define Rmotor 1

#define Lturn 0
#define Rturn 1

//TB6612FNG˫����������IO
#define STBY    PGout(7)  
#define xianfu  980        //�޷� 
void motor_init();
void motor_control(unsigned char ch, int speed);
void TB6612FNG_IO_init();		//˫TB6612FNG����IO��ʼ��

void SET_Front_left_PWM(int speed);
void SET_Front_right_PWM(int speed);
void SET_Behind_left_PWM(int speed);
void SET_Behind_right_PWM(int speed);

/* �������ܣ�����ֵ���� */	
int myabs(int a);

#endif
