/**         
  ******************************************************************************
  * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
  * @文  件  舵机控制函数
  *
  ******************************************************************************
  * @说  明  使用TIM5通道4(PA3)产生舵机控制PWM信号
  *          PWM频率50Hz，周期20ms
  *
  ******************************************************************************
  */

#include "ax_servo.h"

/**
  * @简  述  舵机初始化
  * @参  数  无
  * @返回值  无
  */
void AX_SERVO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* GPIO配置 PA3(TIM5_CH4) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* TIM5时基配置 */
    /* PWM频率 = 72MHz / (预分频+1) / (周期+1) = 72MHz / 72 / 20000 = 50Hz */
    TIM_TimeBaseStructure.TIM_Period = 20000-1;        // 20ms周期
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;        // 72分频，1MHz计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    
    /* PWM模式配置 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = SERVO_CENTER_PULSE;  // 初始位置90度
    
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
    
    /* 使能定时器 */
    TIM_Cmd(TIM5, ENABLE);
}

/**
  * @简  述  设置舵机脉宽
  * @参  数  pulse: 脉宽(us)
  * @返回值  无
  */
void AX_SERVO_SetPulse(uint16_t pulse)
{
    /* 限幅处理 */
    if (pulse < SERVO_MIN_PULSE) pulse = SERVO_MIN_PULSE;
    if (pulse > SERVO_MAX_PULSE) pulse = SERVO_MAX_PULSE;
    
    /* 设置比较值 */
    TIM_SetCompare4(TIM5, pulse);
}

/**
  * @简  述  设置舵机角度
  * @参  数  angle: 角度(0-180度)
  * @返回值  无
  */
void AX_SERVO_SetAngle(uint16_t angle)
{
    uint16_t pulse;
    
    /* 限幅处理 */
    if (angle > 180) angle = 180;
    
    /* 角度转换为脉宽 */
    pulse = SERVO_MIN_PULSE + (uint32_t)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180;
    
    /* 设置脉宽 */
    AX_SERVO_SetPulse(pulse);
}

/**
  * @简  述  设置舵机百分比
  * @参  数  percent: 百分比(-100~100)
  * @返回值  无
  */
void AX_SERVO_SetPercent(int16_t percent)
{
    uint16_t pulse;
    
    /* 限幅处理 */
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    
    /* 百分比转换为脉宽 */
    pulse = SERVO_CENTER_PULSE + (int32_t)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) * percent / 200;
    
    /* 设置脉宽 */
    AX_SERVO_SetPulse(pulse);
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/
