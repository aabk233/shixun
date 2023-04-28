#include "Servo.h"

void Duoji_PWM_Init()
{
//	TIM1_PWM_Init(199,16799);	//TIM1������APB2�����ϣ�APB2���ߵ�ʱ��Ƶ��Ϊ84mhz��APB2�ϵĶ�ʱ��ʱ����2��Ƶ���ģ�Ϊ168mhz
								//168M/16800/200=50Hz�Ķ�ʱ��Ƶ�ʣ�PWM����20ms,�ߵ�ƽ����0.5ms~2.5ms��Ӧ�����0��~180��
//	TIM4_PWM_Init(199,8399);	//TIM4������APB1�����ϣ�APB1���ߵ�ʱ��Ƶ��Ϊ42mhz��APB1�ϵĶ�ʱ��ʱ����2��Ƶ���ģ�Ϊ84mhz
								//ռ�ձȲ����ܷ�Χ0~199����Ч��Χ5~25����Ӧ�����0��~180�ȣ�
								//���鲻Ҫ̫�ӽ���Χ�����ޣ��������ֱ��˶�����׶�ת��ʹ��ѹ���̡����7~23��Χ���
	
	TIM8_PWM_Init(1999,1679);
	TIM10_PWM_Init(1999,1679);
	
	
	TIM_SetCompare4(TIM8,151);//PC9  �����˶������������ֵ�Ӵ�      ��ֵ 151    ��ֵ 110   ��ֵ 190
	TIM_SetCompare1(TIM10,137);//PF6  �����˶������������ֵ�Ӵ�
}

//TIM8 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM8_PWM_Init(uint32_t arr,uint32_t psc)
{		 					 
	//�˲������ֶ��޸�IO������
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);//����GPIOC_Pin6ΪTIM8_Ch1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);//����GPIOC_Pin7ΪTIM8_Ch2
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);//����GPIOC_Pin6ΪTIM8_Ch3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);//����GPIOC_Pin7ΪTIM8_Ch4
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��A
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//Ĭ�Ͼ�Ϊ0
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
 
	//��ʼ��TIM8  PWMģʽ	 
	//PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ�£�
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�ڿ���ʱ���     ��,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������
	
	TIM_OCInitStructure.TIM_Pulse = 0; //����ͨ��1 CCR1��ռ�ձ���ֵ��
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); //Ch��ʼ��
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM8, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����
	
	//��ʼռ�ձ���Ϊ0  ��ֹ���¸�λ��ʱ�ٶ�˲ʱ����
	TIM_SetCompare1(TIM8,0);	//0~100��Ӧռ�ձ�0~100%
	TIM_SetCompare2(TIM8,0);
	TIM_SetCompare3(TIM8,0);
	TIM_SetCompare4(TIM8,0);
						  
} 

//TIM8 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM10_PWM_Init(uint32_t arr,uint32_t psc)
{		 					 
	//�˲������ֶ��޸�IO������
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10);//����GPIOC_Pin6ΪTIM8_Ch1
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOF,&GPIO_InitStructure);               //��ʼ��A
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//Ĭ�Ͼ�Ϊ0
	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
 
	//��ʼ��TIM8  PWMģʽ	 
	//PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ�£�
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�ڿ���ʱ���     ��,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������
	
	TIM_OCInitStructure.TIM_Pulse = 0; //����ͨ��1 CCR1��ռ�ձ���ֵ��
	TIM_OC1Init(TIM10, &TIM_OCInitStructure); //Ch��ʼ��
	
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_ARRPreloadConfig(TIM10,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM10, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM10, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����
	
	//��ʼռ�ձ���Ϊ0  ��ֹ���¸�λ��ʱ�ٶ�˲ʱ����
	TIM_SetCompare1(TIM10,0);	//0~100��Ӧռ�ձ�0~100%
						  
}
