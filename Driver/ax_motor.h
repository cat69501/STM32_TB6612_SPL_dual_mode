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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_MOTOR_H
#define __AX_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//X-SOFT interface functions
void AX_MOTOR_Init(void); //Motor PWM control initialization
void AX_MOTOR_A_SetSpeed(int16_t speed);   //Motor A control
void AX_MOTOR_B_SetSpeed(int16_t speed);   //Motor B control
#endif


