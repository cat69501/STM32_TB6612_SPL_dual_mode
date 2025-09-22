/**        
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  VIN输入电压检测
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_VIN_H
#define __AX_VIN_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//X-SOFT�ӿں���
void AX_VIN_Init(void);  //VIN �����ѹ����ʼ��
uint16_t AX_VIN_GetVol_X100(void);  //VIN ѹ

#endif
