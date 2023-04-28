#ifndef __VOICE_H
#define __VOICE_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "delay.h"
#include "usart.h"




#define IO0_GPIO GPIOE
#define IO1_GPIO GPIOE
#define IO2_GPIO GPIOE
#define IO3_GPIO GPIOE
#define IO4_GPIO GPIOE
#define IO5_GPIO GPIOE
#define IO6_GPIO GPIOE
#define IO7_GPIO GPIOE

#define IO0_GPIO_Pin GPIO_Pin_8
#define IO1_GPIO_Pin GPIO_Pin_9
#define IO2_GPIO_Pin GPIO_Pin_10
#define IO3_GPIO_Pin GPIO_Pin_11
#define IO4_GPIO_Pin GPIO_Pin_12
#define IO5_GPIO_Pin GPIO_Pin_13
#define IO6_GPIO_Pin GPIO_Pin_14
#define IO7_GPIO_Pin GPIO_Pin_15

#define USART_MODE 0
#define IO_MODE 1


#define VOICE_MODE USART_MODE      //在这里手动修改模式  串口或IO口



void voice_init(void);
void Set_Volume(unsigned char volume);
void play_voice(unsigned int index);
#endif


