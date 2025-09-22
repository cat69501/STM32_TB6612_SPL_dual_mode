/**          
  ******************************************************************************
  * @author  Qi Hao Ran
 * @website https://ben0724-ace.github.io/
  * @brief   Encoder driver
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ENCODER_H
#define __AX_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//X-SOFT interface functions
void AX_ENCODER_A_Init(void);          //Encoder A initialization
uint16_t AX_ENCODER_A_GetCounter(void);          //Encoder A get counter value
void AX_ENCODER_A_SetCounter(uint16_t count);    //Encoder A set counter value

void AX_ENCODER_B_Init(void);          //Encoder B initialization
uint16_t AX_ENCODER_B_GetCounter(void);          //Encoder B get counter value
void AX_ENCODER_B_SetCounter(uint16_t count);    //Encoder B set counter value

#endif


