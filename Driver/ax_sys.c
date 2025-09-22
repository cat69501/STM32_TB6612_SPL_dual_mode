/**          
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  系统配置函数
  *
  ******************************************************************************
  * @说  明
  *
  * 1.JTAG SWD配置
  * 
  ******************************************************************************
  */

#include "ax_sys.h"

/**
  * @作  者  JTAG模式设置
  * @网  址  mode:jtag,swd模式;00,全使能;01,使能SWD;10,全关闭;  
						JTAG_SWD_DISABLE   0X02
						SWD_ENABLE         0X01
						JTAG_SWD_ENABLE    0X00	
  * @返回值  无
  */
void AX_JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //使能时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清MAPR[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
} 


