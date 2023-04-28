#include "Moto.h"

extern int error;
int LRate,RRate;
/*

�߼���ʱ��TIM1-PWM�����ʼ��
arr���Զ���װֵ
psc��ʱ��Ԥ��Ƶ��
168M/168/100=10KHz  ��ʱƵ��10ǧHz������0.0001s,PWM�ߵ�ƽ���������0~100��Ӧռ�ձȴ�0~100%

ch1 ��ǰ
ch2 ��ǰ
ch3 ���
ch4 �Һ�

*/
void motor_init()//TIM1_PWM_Init(99,167);            tb6612     1-1000
{  

	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TB6612FNG_IO_init();
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //����GPIOA_Pin8ΪTIM1_Ch1, 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);//����GPIOA_Pin11ΪTIM1_Ch4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); //����GPIOE_Pin9ΪTIM1_Ch3, 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);//����GPIOE_Pin11ΪTIM1_Ch2
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��A
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_11;           //GPIO
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //��ʼ��E
	
	TIM_TimeBaseStructure.TIM_Prescaler=20;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=999;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//Ĭ�Ͼ�Ϊ0
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
 
	//��ʼ��TIM1  PWMģʽ	 
	//PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ�£�
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�ڿ���ʱ���     ��,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������
	
	TIM_OCInitStructure.TIM_Pulse = 0; //����ͨ��1 CCR1��ռ�ձ���ֵ��
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //Ch1��ʼ��
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);//ͨ��2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);//ͨ��3
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//ͨ��4
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR2�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR3�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR4�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����
	
	//��ʼռ�ձ���Ϊ0  ��ֹ���¸�λ��ʱ�ٶ�˲ʱ����
	TIM_SetCompare1(TIM1,0);	 //��ǰ         //0~100��Ӧռ�ձ�0~100%
	TIM_SetCompare2(TIM1,0);	 //��ǰ
	TIM_SetCompare3(TIM1,0);	 //���
	TIM_SetCompare4(TIM1,0);	 //�Һ�
	
	
}

void TB6612FNG_IO_init()		//˫TB6612FNG����IO��ʼ��
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_GPIOD, ENABLE);	 //ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIOA->BSRRL=1<<4;//��ǰ
	GPIOA->BSRRL=1<<12;//

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOG, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOG,GPIO_Pin_0);					
	GPIO_SetBits(GPIOG,GPIO_Pin_1);
	GPIOG->BSRRL=1;    //��ǰ
	GPIOG->BSRRL=1<<1;
	GPIOG->BSRRL=1<<6;//STBY
	GPIOG->BSRRL=1<<7;//STBY

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIOD->BSRRL=1;		//����
	GPIOD->BSRRL=1<<1;
	GPIOD->BSRRL=1<<3;	//�Һ�
	GPIOD->BSRRL=1<<4;
}
volatile int front_left_pwm;//PWM
volatile int front_right_pwm;
volatile int behind_left_pwm;
volatile int behind_right_pwm;
//������������
void SET_Front_left_PWM(int speed)
{
	if(speed>0)	    
	{
		GPIOG->BSRRH=1;    //��ǰ  ǰ��
		GPIOG->BSRRL=1<<1;
	}
	else
	{
		GPIOG->BSRRL=1;    //��ǰ  ����
		GPIOG->BSRRH=1<<1;
	}
	front_left_pwm=myabs(speed);
	if(front_left_pwm>xianfu)front_left_pwm=xianfu;
	TIM_SetCompare1(TIM1,front_left_pwm);
}
void SET_Front_right_PWM(int speed)
{
	if(speed>0)	    
	{
		GPIOA->BSRRH=1<<4;//��ǰ   ǰ��
		GPIOA->BSRRL=1<<12;
	}
	else
	{
		GPIOA->BSRRL=1<<4;//��ǰ   ����
		GPIOA->BSRRH=1<<12;
	}
	front_right_pwm=myabs(speed);
	if(front_right_pwm>xianfu)front_right_pwm=xianfu;
	TIM_SetCompare2(TIM1,front_right_pwm);
}
void SET_Behind_left_PWM(int speed)
{
	if(speed>0)	    
	{
		GPIOD->BSRRL=1;		//��� ǰ��
		GPIOD->BSRRH=1<<1;
	}
	else
	{
		GPIOD->BSRRH=1;		//��� ����
		GPIOD->BSRRL=1<<1;
	}
	behind_left_pwm=myabs(speed);
	if(behind_left_pwm>xianfu)behind_left_pwm=xianfu;
	TIM_SetCompare3(TIM1,behind_left_pwm);
}
void SET_Behind_right_PWM(int speed)
{
	if(speed>0)	    
	{
		GPIOD->BSRRH=1<<3;	//�Һ�  ǰ��
		GPIOD->BSRRL=1<<4;
	}
	else
	{
		GPIOD->BSRRL=1<<3;	//�Һ�  ����
		GPIOD->BSRRH=1<<4;
	}
	behind_right_pwm=myabs(speed);
	if(behind_right_pwm>xianfu)behind_right_pwm=xianfu;
	TIM_SetCompare4(TIM1,behind_right_pwm);
}

/* �������ܣ�����ֵ���� */

int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

//����һ�������ٶ�
// ���룺 �����ţ� �ٶȣ�-100~100�� 

