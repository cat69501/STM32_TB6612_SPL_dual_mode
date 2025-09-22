/**          
  ******************************************************************************
  * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
  * @文  件  PWM输入捕获函数
  *
  ******************************************************************************
  * @说  明  使用TIM4捕获两路PWM信号
  *          通道1(PB6) - 电机控制
  *          通道2(PB7) - 舵机控制
  *
  ******************************************************************************
  */

#include "ax_pwm_input.h"

/* 全局变量 */
static volatile uint16_t g_pwm_ch1_period = 0;    // 通道1周期
static volatile uint16_t g_pwm_ch1_pulse = 0;     // 通道1脉宽
static volatile uint32_t g_pwm_ch1_timeout = 0;   // 通道1超时计数

static volatile uint16_t g_pwm_ch2_period = 0;    // 通道2周期
static volatile uint16_t g_pwm_ch2_pulse = 0;     // 通道2脉宽
static volatile uint32_t g_pwm_ch2_timeout = 0;   // 通道2超时计数

/**
  * @简  述  PWM输入捕获初始化
  * @参  数  无
  * @返回值  无
  */
void AX_PWM_INPUT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /* GPIO配置 PB6(TIM4_CH1) PB7(TIM4_CH2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* TIM4时基配置 */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                    // 最大计数值
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;                   // 72分频，1MHz计数频率，1us精度
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    /* 通道1输入捕获配置 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   // 上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    /* 通道2输入捕获配置 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 使能中断 */
    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update, ENABLE);
    
    /* 使能定时器 */
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * @简  述  获取PWM脉宽
  * @参  数  channel: 通道号(1或2)
  * @返回值  脉宽(us)
  */
uint16_t AX_PWM_INPUT_GetPulse(uint8_t channel)
{
    if (channel == PWM_INPUT_CHANNEL_1)
    {
        return g_pwm_ch1_pulse;
    }
    else if (channel == PWM_INPUT_CHANNEL_2)
    {
        return g_pwm_ch2_pulse;
    }
    return 0;
}

/**
  * @简  述  获取PWM百分比
  * @参  数  channel: 通道号(1或2)
  * @返回值  百分比(-100~100)
  */
int16_t AX_PWM_INPUT_GetPercent(uint8_t channel)
{
    uint16_t pulse = AX_PWM_INPUT_GetPulse(channel);
    int16_t percent;
    
    /* 检查信号是否有效 */
    if (pulse < PWM_MIN_PULSE || pulse > PWM_MAX_PULSE)
    {
        return 0;
    }
    
    /* 计算百分比 */
    if (pulse > (PWM_CENTER_PULSE + PWM_DEADBAND))
    {
        percent = (int16_t)((pulse - PWM_CENTER_PULSE) * 100 / (PWM_MAX_PULSE - PWM_CENTER_PULSE));
    }
    else if (pulse < (PWM_CENTER_PULSE - PWM_DEADBAND))
    {
        percent = (int16_t)((pulse - PWM_CENTER_PULSE) * 100 / (PWM_CENTER_PULSE - PWM_MIN_PULSE));
    }
    else
    {
        percent = 0;  // 死区内返回0
    }
    
    /* 限幅 */
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    
    return percent;
}

/**
  * @简  述  检查PWM信号是否有效
  * @参  数  channel: 通道号(1或2)
  * @返回值  1-有效 0-无效
  */
uint8_t AX_PWM_INPUT_IsValid(uint8_t channel)
{
    if (channel == PWM_INPUT_CHANNEL_1)
    {
        return (g_pwm_ch1_timeout < 50000) ? 1 : 0;  // 50ms超时
    }
    else if (channel == PWM_INPUT_CHANNEL_2)
    {
        return (g_pwm_ch2_timeout < 50000) ? 1 : 0;  // 50ms超时
    }
    return 0;
}

/**
  * @简  述  TIM4中断服务函数
  * @参  数  无
  * @返回值  无
  */
void TIM4_IRQHandler(void)
{
    static uint16_t ch1_rise_time = 0;
    static uint16_t ch2_rise_time = 0;
    static uint8_t ch1_capture_flag = 0;
    static uint8_t ch2_capture_flag = 0;
    
    /* 通道1捕获中断 */
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        
        if (ch1_capture_flag == 0)  // 上升沿
        {
            ch1_rise_time = TIM_GetCapture1(TIM4);
            TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);  // 切换为下降沿捕获
            ch1_capture_flag = 1;
        }
        else  // 下降沿
        {
            uint16_t fall_time = TIM_GetCapture1(TIM4);
            if (fall_time >= ch1_rise_time)
            {
                g_pwm_ch1_pulse = fall_time - ch1_rise_time;
            }
            else
            {
                g_pwm_ch1_pulse = 0xFFFF - ch1_rise_time + fall_time + 1;
            }
            TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising);  // 切换为上升沿捕获
            ch1_capture_flag = 0;
            g_pwm_ch1_timeout = 0;  // 清除超时计数
        }
    }
    
    /* 通道2捕获中断 */
    if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
        
        if (ch2_capture_flag == 0)  // 上升沿
        {
            ch2_rise_time = TIM_GetCapture2(TIM4);
            TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);  // 切换为下降沿捕获
            ch2_capture_flag = 1;
        }
        else  // 下降沿
        {
            uint16_t fall_time = TIM_GetCapture2(TIM4);
            if (fall_time >= ch2_rise_time)
            {
                g_pwm_ch2_pulse = fall_time - ch2_rise_time;
            }
            else
            {
                g_pwm_ch2_pulse = 0xFFFF - ch2_rise_time + fall_time + 1;
            }
            TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising);  // 切换为上升沿捕获
            ch2_capture_flag = 0;
            g_pwm_ch2_timeout = 0;  // 清除超时计数
        }
    }
    
    /* 更新中断(溢出) */
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        
        /* 超时计数 */
        if (g_pwm_ch1_timeout < 60000) g_pwm_ch1_timeout++;
        if (g_pwm_ch2_timeout < 60000) g_pwm_ch2_timeout++;
    }
}


/******************* (C) 版权 2025 Qi Hao Ran **************************************/
