#include "voice.h"	
  
#define VOICE_UART USART6
unsigned char set_volume[5]={0xAA,0x13,0x01,0xff};
void Set_Volume(unsigned char volume)//��������������
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
	usart6_init(9600);//��������
	delay_xms(10);
	Set_Volume(30);
}
/*  ָ�AA 07 02 ��Ŀ�� ��Ŀ�� SM  
	��Ŀ index
	00001 ������
	00002 ��������
	00003 ����Ѳ��ģʽ
	00004 ����ң��ģʽ
	00005 ��ʼ���
	00006 �¶ȹ���
	00007 �ѿ���
	00008 ��ͣ����
	00009 �������ӳɹ�
	00010 ���ڼ����
	00011 ���ڼ��һ��վ��
	00012 ���ڼ�����վ��
	00013 ���ڼ������վ��

*/

/*  ָ�AA 07 02 ��Ŀ�� ��Ŀ�� SM  
	��Ŀ index
	00001 ����
	00002 ��������
	00003 �¶ȳ�������
	00004 �¶ȵ�������
	00005 ʪ�ȳ�������
	00006 ʪ�ȵ�������
	00007 ����������
	00008 ����������
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
 	GPIO_Init(IO7_GPIO, &GPIO_InitStruct);	  //��ʼ��

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

