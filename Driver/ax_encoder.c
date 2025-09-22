/**           
  ******************************************************************************
  * @author  Qi Hao Ran
 * @website https://ben0724-ace.github.io/
  * @brief   Encoder driver
  *
  ******************************************************************************
  */

#include "ax_encoder.h" 

/**
  * @function  Encoder A initialization
  * @param     cycle: encoder cycle
  * @retval    None
  */
void AX_ENCODER_A_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//Configure IO as floating input - timer channel
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //Floating input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//Speed 100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIM clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //TIM up counting mode
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//Reset counter
	TIM3->CNT = 0;

	TIM_Cmd(TIM3, ENABLE);  
}

/**
  * @function  Encoder A get counter value
  * @param     None
  * @retval    Current counter value
  */
uint16_t AX_ENCODER_A_GetCounter(void)
{
	return (TIM_GetCounter(TIM3)); 
}

/**
  * @function  Encoder A set counter value
  * @param     count  Counter value
  * @retval    None
  */
void AX_ENCODER_A_SetCounter(uint16_t count)
{
	TIM3->CNT = count;
}



/**
  * @function  Encoder B initialization
  * @param     cycle: encoder cycle
  * @retval    None
  */
void AX_ENCODER_B_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//Configure IO as floating input - timer channel
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //Floating input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//Speed 100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIM clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //TIM up counting mode
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	//Reset counter
	TIM2->CNT = 0;

	TIM_Cmd(TIM2, ENABLE);  
}

/**
  * @function  Encoder B get counter value
  * @param     None
  * @retval    Current counter value
  */
uint16_t AX_ENCODER_B_GetCounter(void)
{
	return (TIM_GetCounter(TIM2)); 
}

/**
  * @function  Encoder B set counter value
  * @param     count  Counter value
  * @retval    None
  */
void AX_ENCODER_B_SetCounter(uint16_t count)
{
	TIM2->CNT = count;
}


/******************* (C) Copyright 2025 Qi Hao Ran **************************************/
