/**          
  ******************************************************************************
  * @作  者  Qi Hao Ran
  * @网  址  https://ben0724-ace.github.io/
  * @文  件  VIN输入电压检测
  *
  ******************************************************************************
  * @说  明
  *
  * 1.使用ADC2对VIN输入电压进行检测，查询方式
	* 2.电池电压检测功能，实现对电池电量的监控
  * 
  ******************************************************************************
  */

#include "ax_vin.h"

/**
  * @作  者  VIN 电压初始化
  * @网  址  
  * @文  件  
  */
void AX_VIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//初始化GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//**初始化ADC******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	//ADC模式设置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);	

	//设置ADC时钟分频
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//设置ADC通道采样时间
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);

	//使能ADC
	ADC_Cmd(ADC2, ENABLE);

	//初始化ADC校准寄存器  
	ADC_ResetCalibration(ADC2);

	//等待校准完成
	while(ADC_GetResetCalibrationStatus(ADC2));

	//ADC开始校准
	ADC_StartCalibration(ADC2);

	//等待校准完成
	while(ADC_GetCalibrationStatus(ADC2)); 	
}

/**
  * @作  者  VIN 获取电压
  * @网  址  
  * @文  件  电压值，单位100，例如720表示7.2V
  */
uint16_t AX_VIN_GetVol_X100(void)
{
	uint16_t Vin_Vol,temp;
	
	//设置ADC通道采样时间
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);

	//启动ADC软件转换
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);  

	//等待转换完成 
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));

	temp = ADC_GetConversionValue(ADC2);
	Vin_Vol = (uint16_t)((3650 * temp)/4095);

	return Vin_Vol;
}


/******************* (C) 版权 2025 Qi Hao Ran **************************************/
