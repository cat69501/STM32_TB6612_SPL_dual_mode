/**          
  ******************************************************************************
  * @author  Qi Hao Ran
 * @website https://ben0724-ace.github.io/
  * @brief   Motor PWM control functions
  *
  ******************************************************************************
  * @description
  *
  ******************************************************************************
  */

#include "ax_motor.h" 

/**
  * @function  Motor PWM control initialization
  * @param     None
  * @retval    None
  */
void AX_MOTOR_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//Enable timer 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //Enable GPIOA peripheral clock
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //Enable GPIOB peripheral clock
	
    //Configure pins as alternate function push-pull, output TIM1 CH1 and TIM1 CH4 PWM signals
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;  // Motor A PWM - TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //Alternate function push-pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize motor A PWM GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  // Motor B PWM - TIM1_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //Alternate function push-pull
	GPIO_Init(GPIOA, &GPIO_InitStructure);//Initialize motor B PWM GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8; //Motor direction control pins PB3-PB8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //Output push-pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//Initialize motor direction control GPIO
	
    //Initialize TIM1
	TIM_TimeBaseStructure.TIM_Period = 7200-1; //Set the value to be loaded into the active auto-reload register at the next update event
	TIM_TimeBaseStructure.TIM_Prescaler =0; //Set the prescaler value used to divide the TIMx clock frequency
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //Set clock division: TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM up counting mode
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //Initialize TIMx time base unit according to the parameters specified in TIM_TimeBaseInitStruct
	
	//Initialize TIM1 PWM mode	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //Select timer mode: TIM pulse width modulation mode 1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Enable output
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //Output polarity: TIM output compare polarity high
	TIM_OCInitStructure.TIM_Pulse = 0;	    //Duty cycle initialization
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //Initialize TIM1 OC1 according to the specified parameters
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //Enable TIM1 CCR1 preload register

	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //Initialize TIM1 OC4 for motor B according to specified parameters
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //Enable TIM1 CCR4 preload register 
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE bit enable	
	
	TIM_Cmd(TIM1, ENABLE); 	//Enable TIM1
	// PWM init set to 0. add by YH
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare4(TIM1, 0);
	// Motor init to stop, add by YH
	GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8);
}

/**
  * @function  Motor A PWM speed control
  * @param     speed Motor rotation speed value, range -7200~7200
  * @retval    None
  */
void AX_MOTOR_A_SetSpeed(int16_t speed)
{
	uint16_t temp;

    if(speed > 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	    GPIO_SetBits(GPIOB, GPIO_Pin_3);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	    GPIO_SetBits(GPIOB, GPIO_Pin_4);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	    GPIO_ResetBits(GPIOB, GPIO_Pin_4);
		temp = 0;
	}
	
	TIM_SetCompare1(TIM1,temp);
}

/**
  * @function  Motor B PWM speed control
  * @param     speed Motor rotation speed value, range -7200~7200
  * @retval    None
  */
void AX_MOTOR_B_SetSpeed(int16_t speed)
{
	uint16_t temp;

    if(speed > 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	    GPIO_SetBits(GPIOB, GPIO_Pin_5);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	    GPIO_SetBits(GPIOB, GPIO_Pin_8);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
		temp = 0;
	}
	
	TIM_SetCompare4(TIM1,temp);
}

/******************* (C) Copyright 2025 Qi Hao Ran **************************************/
