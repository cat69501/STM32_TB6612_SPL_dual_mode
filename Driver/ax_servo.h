/**          
  ******************************************************************************
  * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
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


