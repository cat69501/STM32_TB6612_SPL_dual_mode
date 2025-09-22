/**        
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  延时函数库
  ******************************************************************************
  * @说  明
  *
  * 1.延时函数使用滴答定时器实现
  * 
  ******************************************************************************
  */

#include "ax_delay.h"
#include "stm32f10x.h"

volatile uint32_t uwTick = 0;

// 删除了 SysTick_Handler 的实现

/**
  * @    时初始化
  * @    
  * @值  
  */
void AX_DELAY_Init(void) 
{   
    // 配置SysTick为1ms中断一次
    SysTick_Config(SystemCoreClock / 1000);
}

/**
  * @    延时函数
  * @参数  us 延时微秒
  * @值  
  */
void AX_Delayus(uint16_t us)
{
	uint32_t temp;
	
	SysTick->LOAD=9*us; 				 		 
	SysTick->VAL=0x00;        				
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  	 
	
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));	 
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; 
	SysTick->VAL =0X00;       				
}
/**
  * @    延时函数
  * @参数  ms 延时毫秒
  * @值  
  * @说明  注意ms的范围，SysTick->LOAD为24位，最大值为0xffffff*8*1000/SYSCLK
  *          例如72M时，ms<=1864ms 
  */
static void Delay_ms(uint16_t ms)
{	 		  	  
	uint32_t temp;	
	
	SysTick->LOAD=(uint32_t)9000*ms;
	SysTick->VAL =0x00;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;      
	SysTick->VAL =0X00;     		  		 	    
}

/**
  * @    延时函数
  * @参数  ms 延时毫秒
  * @值  
  */
void AX_Delayms(uint16_t ms)
{
	uint8_t repeat=ms/500;																
	uint16_t remain=ms%500;
	
	while(repeat)
	{
		Delay_ms(500);
		repeat--;
	}
	
	if(remain)
	{
		Delay_ms(remain);
	}
}

uint32_t AX_DELAY_GetTick(void)
{
    return uwTick;
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/
