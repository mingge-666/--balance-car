#include "stm32f10x.h"                  // Device header


void Timer_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//开启TIM1的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM1);		//选择TIM1为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;                
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;               
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);             
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);			
	                                                
	                                                
	                                                
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);		//开启TIM1的更新中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	                                                                                       
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;          
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          
	NVIC_Init(&NVIC_InitStructure);                             
	
	/*TIM使能*/
	TIM_Cmd(TIM1, ENABLE);			//使能TIM1，定时器开始运行
}

/* 定时器中断函数，可以复制到使用它的地方
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
*/
