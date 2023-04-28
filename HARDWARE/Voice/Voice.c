#include "voice.h"	
  
#define VOICE_UART USART6
unsigned char set_volume[5]={0xAA,0x13,0x01,0xff};
void Set_Volume(unsigned char volume)//设置扬声器音量
{
	unsigned char i;
	set_volume[3]=volume;
	set_volume[4]=set_volume[0]+set_volume[1]+set_volume[2]+set_volume[3];
	
	for(i=0;i<5;i++)
	{
		USART_SendData(VOICE_UART, set_volume[i]);
		while(USART_GetFlagStatus(VOICE_UART, USART_FLAG_TXE) == RESET);
	}
}



#if VOICE_MODE == USART_MODE

void voice_init(void)
{
	usart6_init(9600);//语音播报
	delay_xms(10);
	Set_Volume(30);
}
/*  指令：AA 07 02 曲目高 曲目低 SM  
	曲目 index
	00001 充电结束
	00002 电量过低
	00003 进入巡检模式
	00004 进入遥控模式
	00005 开始充电
	00006 温度过高
	00007 已开机
	00008 暂停工作
	00009 蓝牙连接成功
	00010 正在检测中
	00011 正在检测一号站点
	00012 正在检测二号站点
	00013 正在检测三号站点

*/

/*  指令：AA 07 02 曲目高 曲目低 SM  
	曲目 index
	00001 开机
	00002 蓝牙连接
	00003 温度超过上限
	00004 温度低于下限
	00005 湿度超过上限
	00006 湿度低于下限
	00007 空气质量好
	00008 空气质量差
*/

unsigned char play_data[6]={0xAA,0x07,0x02,0x00,0x00};
void play_voice(unsigned int index)//index  1-65536        
{
	unsigned char i;
	play_data[3]=index/256;
	play_data[4]=index%256;
	play_data[5]=play_data[0]+play_data[1]+play_data[2]+play_data[3]+play_data[4];

	for(i=0;i<6;i++)
	{
		USART_SendData(VOICE_UART, play_data[i]);
		while(USART_GetFlagStatus(VOICE_UART, USART_FLAG_TXE) == RESET);
	}
}
#else 
#if VOICE_MODE == IO_MODE

void voice_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStruct.GPIO_Pin = IO0_GPIO_Pin|IO1_GPIO_Pin|IO2_GPIO_Pin|IO3_GPIO_Pin|IO4_GPIO_Pin|IO5_GPIO_Pin|IO6_GPIO_Pin|IO7_GPIO_Pin;
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
 	GPIO_Init(IO7_GPIO, &GPIO_InitStruct);	  //初始化

	GPIO_WriteBit(IO0_GPIO,IO0_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO1_GPIO,IO1_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO2_GPIO,IO2_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO3_GPIO,IO3_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO4_GPIO,IO4_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO5_GPIO,IO5_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO6_GPIO,IO6_GPIO_Pin,(BitAction)1);
}

void play_voice(unsigned int index)//index  1-255
{
	unsigned char IO_index=0;
	IO_index=index%256;

	IO_index & 0x01  ?  GPIO_WriteBit(IO0_GPIO,IO0_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO0_GPIO,IO0_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x02  ?  GPIO_WriteBit(IO1_GPIO,IO1_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO1_GPIO,IO1_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x04  ?  GPIO_WriteBit(IO2_GPIO,IO2_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO2_GPIO,IO2_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x08  ?  GPIO_WriteBit(IO3_GPIO,IO3_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO3_GPIO,IO3_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x10  ?  GPIO_WriteBit(IO4_GPIO,IO4_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO4_GPIO,IO4_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x20  ?  GPIO_WriteBit(IO5_GPIO,IO5_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO5_GPIO,IO5_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x40  ?  GPIO_WriteBit(IO6_GPIO,IO6_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO6_GPIO,IO6_GPIO_Pin,(BitAction)1) ;
	IO_index & 0x80  ?  GPIO_WriteBit(IO7_GPIO,IO7_GPIO_Pin,(BitAction)0) : GPIO_WriteBit(IO7_GPIO,IO7_GPIO_Pin,(BitAction)1) ;
	
	delay_ms(100);
	GPIO_WriteBit(IO0_GPIO,IO0_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO1_GPIO,IO1_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO2_GPIO,IO2_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO3_GPIO,IO3_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO4_GPIO,IO4_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO5_GPIO,IO5_GPIO_Pin,(BitAction)1);
	GPIO_WriteBit(IO6_GPIO,IO6_GPIO_Pin,(BitAction)1);
}
#endif

#endif

