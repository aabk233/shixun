#ifndef __SERIAL_H
#define __SERIAL_H

#include "sys.h"
#include "usart.h"

void BlueTooth_Serial_Init(u32 bound);
void Rasp_Serial_Init(u32 bound);

//两个串口
//一个用于蓝牙通信，控制小车移动，舵机摆动
//一个用于和树莓派通信，接收树莓派传来的传感器数据


#endif