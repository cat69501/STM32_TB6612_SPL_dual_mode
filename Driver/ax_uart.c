/**              
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  串口通信驱动
  *
  ******************************************************************************
  * @说  明
  * 
  ******************************************************************************
  */

#include "ax_uart.h"
#include <stdio.h>

/**
  * @作  者  UART DBԴڳʼ
  * @网  址  baud 
  * @文  件	 
  */
void AX_UART_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	
	//**���Դ���USART����******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //�򿪴���GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //�򿪴��������ʱ��
	
	//��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//����USART����
	USART_InitStructure.USART_BaudRate = baud; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	//ʹ�� USART�� �������
	USART_Cmd(USART1, ENABLE);
}

/**************************���ڴ�ӡ��غ����ض���********************************/
/**
  * @作  者  ضputcUSART1	
  */
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
}

/**
  * @作  者  ضgetcUSART1	
  */
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
	{}

	return (int)USART_ReceiveData(USART1);
}

/**
  * @简  述  检查是否有数据可读
  * @参  数  无
  * @返回值  1:有数据可读, 0:无数据
  */
uint8_t AX_UART_Available(void)
{
	return (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) ? 1 : 0;
}

/**
  * @简  述  非阻塞方式读取一个字符
  * @参  数  无
  * @返回值  接收到的字符，如果无数据返回0
  */
uint8_t AX_UART_GetChar(void)
{
	if (AX_UART_Available()) {
		return (uint8_t)USART_ReceiveData(USART1);
	}
	return 0;
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/
