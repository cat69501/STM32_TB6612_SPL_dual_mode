/**       
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  串口通信驱动
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_UART_H
#define __AX_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//X-SOFT�ӿں���
void AX_UART_Init(uint32_t baud);     //UART 调试串口初始化
uint8_t AX_UART_Available(void);     //检查是否有数据可读
uint8_t AX_UART_GetChar(void);       //非阻塞方式读取一个字符

#endif
