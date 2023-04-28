#include "Servo.h"

void Duoji_PWM_Init()
{
//	TIM1_PWM_Init(199,16799);	//TIM1挂载在APB2总线上，APB2总线的时钟频率为84mhz，APB2上的定时器时钟是2倍频过的，为168mhz
								//168M/16800/200=50Hz的定时器频率，PWM周期20ms,高电平脉宽0.5ms~2.5ms对应舵机臂0度~180度
//	TIM4_PWM_Init(199,8399);	//TIM4挂载在APB1总线上，APB1总线的时钟频率为42mhz，APB1上的定时器时钟是2倍频过的，为84mhz
								//占空比参数总范围0~199，有效范围5~25（对应舵机臂0度~180度）
								//建议不要太接近范围上下限，否则那种便宜舵机容易堵转，使稳压发烫。最好7~23范围最保险
	
	TIM8_PWM_Init(1999,1679);
	TIM10_PWM_Init(1999,1679);
	
	
	TIM_SetCompare4(TIM8,151);//PC9  左右运动舵机，向右数值加大      中值 151    左值 110   右值 190
	TIM_SetCompare1(TIM10,137);//PF6  上下运动舵机，向上数值加大
}

//TIM8 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM8_PWM_Init(uint32_t arr,uint32_t psc)
{		 					 
	//此部分需手动修改IO口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTA时钟	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);//复用GPIOC_Pin6为TIM8_Ch1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);//复用GPIOC_Pin7为TIM8_Ch2
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);//复用GPIOC_Pin6为TIM8_Ch3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);//复用GPIOC_Pin7为TIM8_Ch4
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化A
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器1
 
	//初始化TIM8  PWM模式	 
	//PWM 模式 1–– 在递增计数模式下，只要 TIMx_CNT<TIMx_CCR1，通道 1 便为有效状态，否则为无效状态。在递减计数模式下，
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1为正常占空比模式，PWM2为反极性模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低,有效电平为低电平
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//在空闲时输出     低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	
	TIM_OCInitStructure.TIM_Pulse = 0; //输入通道1 CCR1（占空比数值）
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); //Ch初始化
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
	TIM_Cmd(TIM8, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM8, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
	
	//初始占空比设为0  防止按下复位键时速度瞬时过大
	TIM_SetCompare1(TIM8,0);	//0~100对应占空比0~100%
	TIM_SetCompare2(TIM8,0);
	TIM_SetCompare3(TIM8,0);
	TIM_SetCompare4(TIM8,0);
						  
} 

//TIM8 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM10_PWM_Init(uint32_t arr,uint32_t psc)
{		 					 
	//此部分需手动修改IO口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//使能PORTA时钟	
	
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10);//复用GPIOC_Pin6为TIM8_Ch1
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);               //初始化A
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseStructure);//初始化定时器1
 
	//初始化TIM8  PWM模式	 
	//PWM 模式 1–– 在递增计数模式下，只要 TIMx_CNT<TIMx_CCR1，通道 1 便为有效状态，否则为无效状态。在递减计数模式下，
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1为正常占空比模式，PWM2为反极性模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低,有效电平为低电平
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//在空闲时输出     低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	
	TIM_OCInitStructure.TIM_Pulse = 0; //输入通道1 CCR1（占空比数值）
	TIM_OC1Init(TIM10, &TIM_OCInitStructure); //Ch初始化
	
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	TIM_ARRPreloadConfig(TIM10,ENABLE);//ARPE使能 
	TIM_Cmd(TIM10, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM10, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
	
	//初始占空比设为0  防止按下复位键时速度瞬时过大
	TIM_SetCompare1(TIM10,0);	//0~100对应占空比0~100%
						  
}
