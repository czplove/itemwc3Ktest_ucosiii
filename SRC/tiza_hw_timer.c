#define HW_TIMER_GLOBAL

#include "tiza_hw_timer.h"


#define HW_TIMER_DEBUG

extern void TimerFunc(void);

// Time_ms最大100ms, 定时器溢出时间计算方法:Tout=((reload+1)*(prescaler+1))/Ft us.
// 定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数10次为1ms
void OpenTimer(uint8 Time_ms)
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = Time_ms*10-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler= 8400-1;  	//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void CloseTimer(void)
{
	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,DISABLE); //使能定时器4
}

//定时器4中断服务函数
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		TimerFunc();
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
}

