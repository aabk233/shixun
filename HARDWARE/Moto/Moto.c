#include "Moto.h"

extern int error;
int LRate,RRate;
/*

高级定时器TIM1-PWM输出初始化
arr：自动重装值
psc：时钟预分频数
168M/168/100=10KHz  定时频率10千Hz，周期0.0001s,PWM高电平脉宽参数从0~100对应占空比从0~100%

ch1 左前
ch2 右前
ch3 左后
ch4 右后

*/
void motor_init()//TIM1_PWM_Init(99,167);            tb6612     1-1000
{  

	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TB6612FNG_IO_init();
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTA时钟	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //复用GPIOA_Pin8为TIM1_Ch1, 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);//复用GPIOA_Pin11为TIM1_Ch4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); //复用GPIOE_Pin9为TIM1_Ch3, 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);//复用GPIOE_Pin11为TIM1_Ch2
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化A
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_11;           //GPIO
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化E
	
	TIM_TimeBaseStructure.TIM_Prescaler=20;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=999;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
 
	//初始化TIM1  PWM模式	 
	//PWM 模式 1–– 在递增计数模式下，只要 TIMx_CNT<TIMx_CCR1，通道 1 便为有效状态，否则为无效状态。在递减计数模式下，
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1为正常占空比模式，PWM2为反极性模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低,有效电平为低电平
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//在空闲时输出     低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	
	TIM_OCInitStructure.TIM_Pulse = 0; //输入通道1 CCR1（占空比数值）
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //Ch1初始化
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);//通道2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);//通道3
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//通道4
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR2上的预装载寄存器,CCR自动装载默认也是打开的
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR3上的预装载寄存器,CCR自动装载默认也是打开的
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR4上的预装载寄存器,CCR自动装载默认也是打开的
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
	
	//初始占空比设为0  防止按下复位键时速度瞬时过大
	TIM_SetCompare1(TIM1,0);	 //左前         //0~100对应占空比0~100%
	TIM_SetCompare2(TIM1,0);	 //右前
	TIM_SetCompare3(TIM1,0);	 //左后
	TIM_SetCompare4(TIM1,0);	 //右后
	
	
}

void TB6612FNG_IO_init()		//双TB6612FNG驱动IO初始化
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_GPIOD, ENABLE);	 //使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIOA->BSRRL=1<<4;//右前
	GPIOA->BSRRL=1<<12;//

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOG, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOG,GPIO_Pin_0);					
	GPIO_SetBits(GPIOG,GPIO_Pin_1);
	GPIOG->BSRRL=1;    //左前
	GPIOG->BSRRL=1<<1;
	GPIOG->BSRRL=1<<6;//STBY
	GPIOG->BSRRL=1<<7;//STBY

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIOD->BSRRL=1;		//右左
	GPIOD->BSRRL=1<<1;
	GPIOD->BSRRL=1<<3;	//右后
	GPIOD->BSRRL=1<<4;
}
volatile int front_left_pwm;//PWM
volatile int front_right_pwm;
volatile int behind_left_pwm;
volatile int behind_right_pwm;
//单个单机控制
void SET_Front_left_PWM(int speed)
{
	if(speed>0)	    
	{
		GPIOG->BSRRH=1;    //左前  前进
		GPIOG->BSRRL=1<<1;
	}
	else
	{
		GPIOG->BSRRL=1;    //左前  后退
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
		GPIOA->BSRRH=1<<4;//右前   前进
		GPIOA->BSRRL=1<<12;
	}
	else
	{
		GPIOA->BSRRL=1<<4;//右前   后退
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
		GPIOD->BSRRL=1;		//左后 前进
		GPIOD->BSRRH=1<<1;
	}
	else
	{
		GPIOD->BSRRH=1;		//左后 后退
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
		GPIOD->BSRRH=1<<3;	//右后  前进
		GPIOD->BSRRL=1<<4;
	}
	else
	{
		GPIOD->BSRRL=1<<3;	//右后  后退
		GPIOD->BSRRH=1<<4;
	}
	behind_right_pwm=myabs(speed);
	if(behind_right_pwm>xianfu)behind_right_pwm=xianfu;
	TIM_SetCompare4(TIM1,behind_right_pwm);
}

/* 函数功能：绝对值函数 */

int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

//设置一侧电机的速度
// 输入： 电机序号， 速度（-100~100） 

