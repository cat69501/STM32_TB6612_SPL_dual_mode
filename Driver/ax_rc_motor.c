/**         
  ******************************************************************************
  * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
  * @文  件  遥控器电机控制函数
  *
  ******************************************************************************
  * @说  明  结合PWM输入控制电机和舵机
  *          通道1：控制前进后退和速度
  *          通道2：控制舵机转向
  *
  ******************************************************************************
  */

#include "ax_rc_motor.h"
#include "ax_motor.h"
#include "ax_pwm_input.h"
#include "ax_servo.h"

/* 私有变量 */
static RC_ControlMode_t g_control_mode = RC_MODE_TANK;
static int16_t g_max_speed = 7200;  // 最大电机速度

/* 电机校准系数 - 用于修正电机不同步问题 */
static float g_motor_a_scale = 1.0f;    // 电机A比例系数
static float g_motor_b_scale = 1.0f;    // 电机B比例系数
static int16_t g_motor_a_offset = 0;    // 电机A起始偏移
static int16_t g_motor_b_offset = 0;    // 电机B起始偏移

/**
  * @简  述  遥控器电机控制初始化
  * @参  数  无
  * @返回值  无
  */
void AX_RC_MOTOR_Init(void)
{
    /* 初始化各模块 */
    AX_MOTOR_Init();        // 电机初始化
    AX_PWM_INPUT_Init();    // PWM输入捕获初始化
    AX_SERVO_Init();        // 舵机初始化
    
    /* 设置舵机中位 */
    AX_SERVO_SetAngle(90);
}

/**
  * @简  述  设置电机校准参数
  * @参  数  motor_a_scale: 电机A比例系数
  * @参  数  motor_b_scale: 电机B比例系数  
  * @参  数  motor_a_offset: 电机A起始偏移
  * @参  数  motor_b_offset: 电机B起始偏移
  * @返回值  无
  */
void AX_RC_MOTOR_SetCalibration(float motor_a_scale, float motor_b_scale, 
                               int16_t motor_a_offset, int16_t motor_b_offset)
{
    g_motor_a_scale = motor_a_scale;
    g_motor_b_scale = motor_b_scale;
    g_motor_a_offset = motor_a_offset;
    g_motor_b_offset = motor_b_offset;
}

/**
  * @简  述  设置控制模式
  * @参  数  mode: 控制模式
  * @返回值  无
  */
void AX_RC_MOTOR_SetMode(RC_ControlMode_t mode)
{
    g_control_mode = mode;
}

/**
  * @简  述  更新电机控制
  * @参  数  无
  * @返回值  无
  */
void AX_RC_MOTOR_Update(void)
{
    int16_t ch1_percent, ch2_percent;
    int16_t motor_a_speed, motor_b_speed;
    
    /* 检查遥控器信号 */
    if (!AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_1) || 
        !AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_2))
    {
        /* 信号丢失，停止所有电机 */
        AX_RC_MOTOR_Emergency_Stop();
        return;
    }
    
    /* 获取遥控器输入百分比 */
    ch1_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);  // 前进后退
    ch2_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);  // 转向
    
    /* 修正CH2转向方向 - 适配地面车辆（飞控遥控器转向相反） */
    ch2_percent = ch2_percent;
    
    /* 根据控制模式计算电机速度 */
    if (g_control_mode == RC_MODE_TANK)
    {
        /* 坦克模式：差速转向 */
        int16_t base_speed = (int16_t)((int32_t)g_max_speed * ch1_percent / 100);
        int16_t turn_speed = (int16_t)((int32_t)g_max_speed * ch2_percent / 200);  // 转向速度减半
        
        motor_a_speed = base_speed + turn_speed;  // 恢复原始逻辑
        motor_b_speed = base_speed - turn_speed;
        
        /* 限幅处理 */
        if (motor_a_speed > g_max_speed) motor_a_speed = g_max_speed;
        if (motor_a_speed < -g_max_speed) motor_a_speed = -g_max_speed;
        if (motor_b_speed > g_max_speed) motor_b_speed = g_max_speed;
        if (motor_b_speed < -g_max_speed) motor_b_speed = -g_max_speed;
    }
    else
    {
        /* 混控模式：舵机转向 */
        motor_a_speed = motor_b_speed = (int16_t)((int32_t)g_max_speed * ch1_percent / 100);
        
        /* 控制舵机 */
        AX_SERVO_SetPercent(ch2_percent);
    }
    
    /* 应用电机校准参数 */
    if (motor_a_speed != 0)
    {
        motor_a_speed = (int16_t)(motor_a_speed * g_motor_a_scale + g_motor_a_offset);
    }
    if (motor_b_speed != 0) 
    {
        motor_b_speed = (int16_t)(motor_b_speed * g_motor_b_scale + g_motor_b_offset);
    }
    
    /* 校准后重新限幅处理，防止溢出 */
    if (motor_a_speed > g_max_speed) motor_a_speed = g_max_speed;
    if (motor_a_speed < -g_max_speed) motor_a_speed = -g_max_speed;
    if (motor_b_speed > g_max_speed) motor_b_speed = g_max_speed;
    if (motor_b_speed < -g_max_speed) motor_b_speed = -g_max_speed;
    
    /* 设置电机速度 */
    AX_MOTOR_A_SetSpeed(motor_a_speed);
    AX_MOTOR_B_SetSpeed(motor_b_speed);
}

/**
  * @简  述  紧急停止
  * @参  数  无
  * @返回值  无
  */
void AX_RC_MOTOR_Emergency_Stop(void)
{
    /* 停止所有电机 */
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    
    /* 舵机回中 */
    AX_SERVO_SetAngle(90);
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/
