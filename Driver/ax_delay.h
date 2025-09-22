/**          
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  延时函数库
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_DELAY_H
#define __AX_DELAY_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//X-SOFT�ӿں���
void AX_DELAY_Init(void); //��ʱ��ʼ��
void AX_Delayus(__IO uint16_t us);  //����΢����ʱ
void AX_Delayms(__IO uint16_t ms);  //����������ʱ
uint32_t AX_DELAY_GetTick(void);
extern volatile uint32_t uwTick;

#endif 


