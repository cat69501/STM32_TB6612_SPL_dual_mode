/**         
  ******************************************************************************
  * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
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
#define PWM_MIN_PULSE         900   // 最小脉宽(us) - 扩大范围
#define PWM_MAX_PULSE         2100  // 最大脉宽(us) - 扩大范围  
#define PWM_CENTER_PULSE      1500  // 中点脉宽(us)
#define PWM_DEADBAND          20    // 死区范围(us)

/* 函数声明 */
void AX_PWM_INPUT_Init(void);                     // PWM输入捕获初始化
uint16_t AX_PWM_INPUT_GetPulse(uint8_t channel); // 获取PWM脉宽(us)
int16_t AX_PWM_INPUT_GetPercent(uint8_t channel);// 获取PWM百分比(-100~100)
uint8_t AX_PWM_INPUT_IsValid(uint8_t channel);   // 检查PWM信号是否有效

#endif


