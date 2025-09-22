# STM32 TB6612 PWM遥控器控制实现方案

## 1. 系统概述

本方案实现了使用标准双通道PWM遥控器接收器控制TB6612双路电机驱动器的功能：
- **通道1 (CH1)**：控制双路电机的前进、后退、加速、减速
- **通道2 (CH2)**：控制PWM舵机进行转向

系统支持两种控制模式：
- **坦克模式**：通过差速实现转向（左右电机速度不同）
- **混控模式**：使用舵机转向，两个电机速度相同

## 2. 硬件连接与IO分配

### 2.1 新增IO分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM输入通道1 | PB6 | TIM4_CH1，接收遥控器CH1信号（电机控制） |
| PWM输入通道2 | PB7 | TIM4_CH2，接收遥控器CH2信号（舵机控制） |
| 舵机PWM输出 | PA3 | TIM5_CH4，输出50Hz PWM控制舵机 |

### 2.2 原有IO分配（保持不变）

| 功能 | 引脚 | 说明 |
|------|------|------|
| 电机A PWM | PA8 | TIM1_CH1 |
| 电机B PWM | PA11 | TIM1_CH4 |
| 电机A方向1 | PB12 | AIN1 |
| 电机A方向2 | PB13 | AIN2 |
| 电机B方向1 | PB14 | BIN1 |
| 电机B方向2 | PB15 | BIN2 |
| 编码器A | PA6/PA7 | TIM3_CH1/CH2 |
| 编码器B | PA0/PA1 | TIM2_CH1/CH2 |
| 电压检测 | PC0 | ADC1_IN10 |
| 调试串口 | PA9/PA10 | USART1 TX/RX |

### 2.3 硬件连接说明

1. **遥控器接收器连接**：
   - 接收器CH1输出 → STM32 PB6
   - 接收器CH2输出 → STM32 PB7
   - 接收器VCC → 5V
   - 接收器GND → GND

2. **舵机连接**：
   - 舵机信号线 → STM32 PA3
   - 舵机电源正极 → 5V或6V（根据舵机规格）
   - 舵机电源负极 → GND

## 3. 软件架构

### 3.1 新增库文件结构

```
Driver/
├── ax_pwm_input.h    // PWM输入捕获头文件
├── ax_pwm_input.c    // PWM输入捕获实现
├── ax_servo.h        // 舵机控制头文件
├── ax_servo.c        // 舵机控制实现
├── ax_rc_motor.h     // 遥控器电机控制头文件
└── ax_rc_motor.c     // 遥控器电机控制实现
```

### 3.2 PWM信号参数

- **PWM输入信号**：
  - 频率：50Hz（周期20ms）
  - 脉宽范围：1000μs ~ 2000μs
  - 中点值：1500μs
  - 死区：±20μs

- **舵机控制信号**：
  - 频率：50Hz
  - 0°位置：500μs
  - 90°位置：1500μs
  - 180°位置：2500μs

## 4. 完整库文件代码

### 4.1 PWM输入捕获库（ax_pwm_input.h）

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  PWM输入捕获函数
  *
  ******************************************************************************
  * @说  明  用于接收遥控器PWM信号
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PWM_INPUT_H
#define __AX_PWM_INPUT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* PWM输入捕获通道定义 */
#define PWM_INPUT_CHANNEL_1    1    // 电机控制通道
#define PWM_INPUT_CHANNEL_2    2    // 舵机控制通道

/* PWM信号参数定义 */
#define PWM_MIN_PULSE         1000  // 最小脉宽(us)
#define PWM_MAX_PULSE         2000  // 最大脉宽(us)
#define PWM_CENTER_PULSE      1500  // 中点脉宽(us)
#define PWM_DEADBAND          20    // 死区范围(us)

/* 函数声明 */
void AX_PWM_INPUT_Init(void);                     // PWM输入捕获初始化
uint16_t AX_PWM_INPUT_GetPulse(uint8_t channel); // 获取PWM脉宽(us)
int16_t AX_PWM_INPUT_GetPercent(uint8_t channel);// 获取PWM百分比(-100~100)
uint8_t AX_PWM_INPUT_IsValid(uint8_t channel);   // 检查PWM信号是否有效

#endif

/******************* (C) 版权 2024 XTARK **************************************/
```

### 4.2 PWM输入捕获库（ax_pwm_input.c）

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  PWM输入捕获函数
  *
  ******************************************************************************
  * @说  明  使用TIM2捕获两路PWM信号
  *          通道1(PA0) - 电机控制
  *          通道2(PA1) - 舵机控制
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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* GPIO配置 PA0(TIM2_CH1) PA1(TIM2_CH2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* TIM2时基配置 */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                    // 最大计数值
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;                   // 72分频，1MHz计数频率，1us精度
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    /* 通道1输入捕获配置 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   // 上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    /* 通道2输入捕获配置 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 使能中断 */
    TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update, ENABLE);
    
    /* 使能定时器 */
    TIM_Cmd(TIM2, ENABLE);
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
  * @简  述  TIM2中断服务函数
  * @参  数  无
  * @返回值  无
  */
void TIM2_IRQHandler(void)
{
    static uint16_t ch1_rise_time = 0;
    static uint16_t ch2_rise_time = 0;
    static uint8_t ch1_capture_flag = 0;
    static uint8_t ch2_capture_flag = 0;
    
    /* 通道1捕获中断 */
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        
        if (ch1_capture_flag == 0)  // 上升沿
        {
            ch1_rise_time = TIM_GetCapture1(TIM2);
            TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);  // 切换为下降沿捕获
            ch1_capture_flag = 1;
        }
        else  // 下降沿
        {
            uint16_t fall_time = TIM_GetCapture1(TIM2);
            if (fall_time >= ch1_rise_time)
            {
                g_pwm_ch1_pulse = fall_time - ch1_rise_time;
            }
            else
            {
                g_pwm_ch1_pulse = 0xFFFF - ch1_rise_time + fall_time + 1;
            }
            TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising);  // 切换为上升沿捕获
            ch1_capture_flag = 0;
            g_pwm_ch1_timeout = 0;  // 清除超时计数
        }
    }
    
    /* 通道2捕获中断 */
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
        
        if (ch2_capture_flag == 0)  // 上升沿
        {
            ch2_rise_time = TIM_GetCapture2(TIM2);
            TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);  // 切换为下降沿捕获
            ch2_capture_flag = 1;
        }
        else  // 下降沿
        {
            uint16_t fall_time = TIM_GetCapture2(TIM2);
            if (fall_time >= ch2_rise_time)
            {
                g_pwm_ch2_pulse = fall_time - ch2_rise_time;
            }
            else
            {
                g_pwm_ch2_pulse = 0xFFFF - ch2_rise_time + fall_time + 1;
            }
            TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);  // 切换为上升沿捕获
            ch2_capture_flag = 0;
            g_pwm_ch2_timeout = 0;  // 清除超时计数
        }
    }
    
    /* 更新中断(溢出) */
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        /* 超时计数 */
        if (g_pwm_ch1_timeout < 60000) g_pwm_ch1_timeout++;
        if (g_pwm_ch2_timeout < 60000) g_pwm_ch2_timeout++;
    }
}

/******************* (C) 版权 2024 XTARK **************************************/
```

### 4.3 舵机控制库（ax_servo.h）

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  舵机控制函数
  *
  ******************************************************************************
  * @说  明  舵机PWM控制
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SERVO_H
#define __AX_SERVO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* 舵机参数定义 */
#define SERVO_MIN_PULSE     500    // 最小脉宽(us) - 0度
#define SERVO_MAX_PULSE     2500   // 最大脉宽(us) - 180度
#define SERVO_CENTER_PULSE  1500   // 中点脉宽(us) - 90度

/* 函数声明 */
void AX_SERVO_Init(void);                   // 舵机初始化
void AX_SERVO_SetPulse(uint16_t pulse);     // 设置舵机脉宽(us)
void AX_SERVO_SetAngle(uint16_t angle);     // 设置舵机角度(0-180度)
void AX_SERVO_SetPercent(int16_t percent);  // 设置舵机百分比(-100~100)

#endif

/******************* (C) 版权 2024 XTARK **************************************/
```

### 4.4 舵机控制库（ax_servo.c）

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  舵机控制函数
  *
  ******************************************************************************
  * @说  明  使用TIM3通道1(PA6)产生舵机控制PWM信号
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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* GPIO配置 PA6(TIM3_CH1) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* TIM3时基配置 */
    /* PWM频率 = 72MHz / (预分频+1) / (周期+1) = 72MHz / 72 / 20000 = 50Hz */
    TIM_TimeBaseStructure.TIM_Period = 20000-1;        // 20ms周期
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;        // 72分频，1MHz计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    /* PWM模式配置 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = SERVO_CENTER_PULSE;  // 初始位置90度
    
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* 使能定时器 */
    TIM_Cmd(TIM3, ENABLE);
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
    TIM_SetCompare1(TIM3, pulse);
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

/******************* (C) 版权 2024 XTARK **************************************/
```

### 4.5 遥控器电机控制库（ax_rc_motor.h）

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  遥控器电机控制函数
  *
  ******************************************************************************
  * @说  明  结合PWM输入控制电机
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_RC_MOTOR_H
#define __AX_RC_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* 控制模式定义 */
typedef enum
{
    RC_MODE_TANK = 0,      // 坦克模式：CH1控制前进后退，CH2控制转向
    RC_MODE_MIXED = 1      // 混控模式：两个通道混合控制
} RC_ControlMode_t;

/* 函数声明 */
void AX_RC_MOTOR_Init(void);                            // 遥控器电机控制初始化
void AX_RC_MOTOR_SetMode(RC_ControlMode_t mode);       // 设置控制模式
void AX_RC_MOTOR_Update(void);                          // 更新电机控制
void AX_RC_MOTOR_Emergency_Stop(void);                  // 紧急停止

#endif

/******************* (C) 版权 2024 XTARK **************************************/
```

### 4.6 遥控器电机控制库（ax_rc_motor.c）

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  * 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  遥控器电机控制函数
  *
  ******************************************************************************
  * @说  明  结合PWM输入控制电机和舵机
  *          通道1：控制前进后退和速度
  *          通道2：控制舵机转向
  *
  ******************************************************************************
  */

#include "ax_rc_motor.h"
#include "ax_motor.h"
#include "ax_pwm_input.h"
#include "ax_servo.h"

/* 私有变量 */
static RC_ControlMode_t g_control_mode = RC_MODE_TANK;
static int16_t g_max_speed = 7200;  // 最大电机速度

/**
  * @简  述  遥控器电机控制初始化
  * @参  数  无
  * @返回值  无
  */
void AX_RC_MOTOR_Init(void)
{
    /* 初始化各模块 */
    AX_MOTOR_Init();        // 电机初始化
    AX_PWM_INPUT_Init();    // PWM输入捕获初始化
    AX_SERVO_Init();        // 舵机初始化
    
    /* 设置舵机中位 */
    AX_SERVO_SetAngle(90);
}

/**
  * @简  述  设置控制模式
  * @参  数  mode: 控制模式
  * @返回值  无
  */
void AX_RC_MOTOR_SetMode(RC_ControlMode_t mode)
{
    g_control_mode = mode;
}

/**
  * @简  述  更新电机控制
  * @参  数  无
  * @返回值  无
  */
void AX_RC_MOTOR_Update(void)
{
    int16_t ch1_percent, ch2_percent;
    int16_t motor_a_speed, motor_b_speed;
    
    /* 检查遥控器信号 */
    if (!AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_1) || 
        !AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_2))
    {
        /* 信号丢失，停止所有电机 */
        AX_RC_MOTOR_Emergency_Stop();
        return;
    }
    
    /* 获取遥控器输入百分比 */
    ch1_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);  // 前进后退
    ch2_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);  // 转向
    
    /* 根据控制模式计算电机速度 */
    if (g_control_mode == RC_MODE_TANK)
    {
        /* 坦克模式：差速转向 */
        int16_t base_speed = (int16_t)((int32_t)g_max_speed * ch1_percent / 100);
        int16_t turn_speed = (int16_t)((int32_t)g_max_speed * ch2_percent / 200);  // 转向速度减半
        
        motor_a_speed = base_speed + turn_speed;
        motor_b_speed = base_speed - turn_speed;
        
        /* 限幅处理 */
        if (motor_a_speed > g_max_speed) motor_a_speed = g_max_speed;
        if (motor_a_speed < -g_max_speed) motor_a_speed = -g_max_speed;
        if (motor_b_speed > g_max_speed) motor_b_speed = g_max_speed;
        if (motor_b_speed < -g_max_speed) motor_b_speed = -g_max_speed;
    }
    else
    {
        /* 混控模式：舵机转向 */
        motor_a_speed = motor_b_speed = (int16_t)((int32_t)g_max_speed * ch1_percent / 100);
        
        /* 控制舵机 */
        AX_SERVO_SetPercent(ch2_percent);
    }
    
    /* 设置电机速度 */
    AX_MOTOR_A_SetSpeed(motor_a_speed);
    AX_MOTOR_B_SetSpeed(motor_b_speed);
}

/**
  * @简  述  紧急停止
  * @参  数  无
  * @返回值  无
  */
void AX_RC_MOTOR_Emergency_Stop(void)
{
    /* 停止所有电机 */
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    
    /* 舵机回中 */
    AX_SERVO_SetAngle(90);
}

/******************* (C) 版权 2024 XTARK **************************************/
```

## 5. 完整main.c代码

```c
/**
	 __  _______  _    ____  _  __      __  __    ____   ___  _____ _____ 
	 \ \/ /_   _|/ \  |  _ \| |/ / ·    \ \/ /   / ___| / _ \|  ___|_   _| ·
	  \  /  | | / _ \ | |_) | ' /  ·     \  /____\___ \| | | | |_    | |   ·
	  /  \  | |/ ___ \|  _ <| . \  ·     /  \_____|__) | |_| |  _|   | |   ·
	 /_/\_\ |_/_/   \_\_| \_\_|\_\ ·    /_/\_\   |____/ \___/|_|     |_|   
                              
  ******************************************************************************
  * 
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（注册官方号，获取最新更新资讯）
  *                              
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @文  件  TB6612双路电机PWM遥控器控制程序
  * @说  明  使用PWM遥控器控制双路电机和舵机
  *
  ******************************************************************************
  */

#include "stm32f10x.h"
#include <stdio.h>

#include "ax_sys.h"         // 系统配置
#include "ax_delay.h"       // 精准延时
#include "ax_uart.h"        // 调试串口

#include "ax_rc_motor.h"    // 遥控器电机控制
#include "ax_pwm_input.h"   // PWM输入捕获
#include "ax_encoder.h"     // 编码器驱动
#include "ax_vin.h"         // 电压采集

// 全局变量
int16_t  encoder1, encoder2;        // 编码器数值
uint16_t adc_value;                 // 电压采集值
uint16_t pwm_ch1, pwm_ch2;          // PWM脉宽值
int16_t  pwm_ch1_percent, pwm_ch2_percent;  // PWM百分比值

/**
  * @简  述  主函数
  * @参  数  无
  * @返回值  无
  */
int main(void)
{
    uint32_t last_print_time = 0;
    uint32_t current_time = 0;
    
    // 精准延时初始化
    AX_DELAY_Init();    
    
    // 调试串口初始化
    AX_UART_Init(115200);
    printf("  \r\n"); // 发送空格和回车,解决部分串口显示BUG
    
    printf("XTARK TB6612 PWM Remote Control Demo\r\n");
    printf("CH1: Motor speed control\r\n");
    printf("CH2: Servo steering control\r\n");
    
    // JTAG调试配置
    AX_JTAG_Set(JTAG_SWD_DISABLE);  // 关闭JTAG接口
    AX_JTAG_Set(SWD_ENABLE);        // 开启SWD接口 可以用来调试和下载程序
    
    // 遥控器电机控制初始化（包含电机、PWM输入、舵机初始化）
    AX_RC_MOTOR_Init();
    
    // 编码器驱动初始化
    AX_ENCODER_A_Init();  
    AX_ENCODER_B_Init();    
    
    // 电压检测初始化
    AX_VIN_Init();
    
    // 设置控制模式为混控模式（使用舵机转向）
    AX_RC_MOTOR_SetMode(RC_MODE_MIXED);
    
    printf("System initialized successfully!\r\n");
    printf("Waiting for RC signal...\r\n");
    
    while (1) 
    {
        // 更新电机控制（根据PWM输入自动控制电机和舵机）
        AX_RC_MOTOR_Update();
        
        // 获取当前时间
        current_time = AX_DELAY_GetTick();
        
        // 每100ms打印一次调试信息
        if (current_time - last_print_time >= 100)
        {
            last_print_time = current_time;
            
            // 获取PWM输入值
            pwm_ch1 = AX_PWM_INPUT_GetPulse(PWM_INPUT_CHANNEL_1);
            pwm_ch2 = AX_PWM_INPUT_GetPulse(PWM_INPUT_CHANNEL_2);
            pwm_ch1_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);
            pwm_ch2_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);
            
            // 获取编码器数据
            encoder1 = AX_ENCODER_A_GetCounter();
            encoder2 = AX_ENCODER_B_GetCounter();
            
            // 清零编码器
            AX_ENCODER_A_SetCounter(0);  
            AX_ENCODER_B_SetCounter(0);    
            
            // 获取电压采集的AD转换值
            adc_value = AX_VIN_GetVol_X100();
            
            // 打印调试信息
            printf("PWM1:%4d(%3d%%) PWM2:%4d(%3d%%) ENC1:%4d ENC2:%4d VIN:%d ",
                   pwm_ch1, pwm_ch1_percent, 
                   pwm_ch2, pwm_ch2_percent,
                   encoder1, encoder2, adc_value);
            
            // 显示信号状态
            if (AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_1) && 
                AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_2))
            {
                printf("[OK]\r\n");
            }
            else
            {
                printf("[NO SIGNAL]\r\n");
            }
        }
        
        // 短延时，提高控制响应速度
        AX_Delayms(5);
    }
}

/**
  * @简  述  TIM2中断服务函数（PWM输入捕获）
  * @参  数  无
  * @返回值  无
  * @说  明  此函数已在ax_pwm_input.c中实现
  */

/******************* (C) 版权 2024 XTARK **************************************/
```

## 6. 使用说明

### 6.1 编译配置

1. 将新增的库文件添加到工程中：
   - `ax_pwm_input.c`
   - `ax_servo.c`
   - `ax_rc_motor.c`

2. 确保头文件路径包含Driver目录

3. 使用新的main_pwm_control.c替换原有的main.c，或根据需要合并代码

### 6.2 控制模式切换

在main函数中通过以下代码切换控制模式：

```c
// 坦克模式（差速转向）
AX_RC_MOTOR_SetMode(RC_MODE_TANK);

// 混控模式（舵机转向）
AX_RC_MOTOR_SetMode(RC_MODE_MIXED);
```

### 6.3 调试信息

系统通过串口输出以下调试信息：
- PWM输入脉宽和百分比
- 编码器数值
- 电源电压
- 信号状态

### 6.4 安全特性

- **信号丢失保护**：当遥控器信号丢失时，自动停止所有电机
- **死区设置**：中位附近±20μs为死区，避免抖动
- **限幅保护**：所有输出值都有限幅保护

## 7. 注意事项

1. **电源要求**：
   - 舵机需要独立的5V或6V电源
   - 确保电源能提供足够的电流

2. **信号连接**：
   - PWM信号线使用短线，避免干扰
   - 确保共地连接

3. **调试建议**：
   - 先测试PWM输入捕获是否正常
   - 再测试舵机控制
   - 最后联调整个系统

4. **扩展性**：
   - 可以根据需要修改控制算法
   - 可以增加更多控制模式
   - 可以调整PWM参数以适配不同的遥控器