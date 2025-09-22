/**         
  ******************************************************************************
  * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
  * @文  件  遥控器电机控制函数
  *
  ******************************************************************************
  * @说  明  结合PWM输入控制电机
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_RC_MOTOR_H
#define __AX_RC_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* 控制模式定义 */
typedef enum
{
    RC_MODE_TANK = 0,      // 坦克模式：CH1控制前进后退，CH2控制转向
    RC_MODE_MIXED = 1      // 混控模式：两个通道混合控制
} RC_ControlMode_t;

/* 函数声明 */
void AX_RC_MOTOR_Init(void);                            // 遥控器电机控制初始化
void AX_RC_MOTOR_SetMode(RC_ControlMode_t mode);       // 设置控制模式
void AX_RC_MOTOR_SetCalibration(float motor_a_scale, float motor_b_scale, 
                               int16_t motor_a_offset, int16_t motor_b_offset); // 设置电机校准参数
void AX_RC_MOTOR_Update(void);                          // 更新电机控制
void AX_RC_MOTOR_Emergency_Stop(void);                  // 紧急停止

#endif


