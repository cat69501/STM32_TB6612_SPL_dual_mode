/**                             
 ******************************************************************************
 * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
 * @文  件  阿克曼转向RC遥控控制程序
 * @说  明  使用后轮驱动+前轮转向的阿克曼转向模式
 *          支持RC遥控器控制，上电即为遥控模式
 *
 ******************************************************************************
 */

#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "ax_sys.h"         // 系统配置
#include "ax_delay.h"       // 精准延时
#include "ax_uart.h"        // 调试串口
#include "ax_motor.h"       // 电机驱动
#include "ax_servo.h"       // 舵机驱动
#include "ax_encoder.h"     // 编码器驱动
#include "ax_vin.h"         // 电压采集
#include "ax_pwm_input.h"   // PWM输入捕获

// 阿克曼转向参数定义
#define REAR_MOTOR_SPEED_MAX      7200    // 最大速度 (100%功率)
#define REAR_MOTOR_SPEED_NORMAL   4000    // 正常速度 (约55%功率)

#define SERVO_STRAIGHT_ANGLE       90      // 前轮直行角度
#define SERVO_MAX_TURN_ANGLE       45      // 前轮最大转向角度
#define SERVO_LEFT_ANGLE          (SERVO_STRAIGHT_ANGLE + SERVO_MAX_TURN_ANGLE)   // 左转角度
#define SERVO_RIGHT_ANGLE         (SERVO_STRAIGHT_ANGLE - SERVO_MAX_TURN_ANGLE)   // 右转角度

#define RC_DEADBAND                5       // RC遥控器死区(百分比)
#define RC_TIMEOUT_MS              1000    // RC信号超时时间(ms)

// 全局变量
int16_t  encoder1, encoder2;                // 编码器数值
uint16_t adc_value;                         // 电压采集值
uint8_t  rc_connected = 0;                  // RC连接状态
uint32_t last_rc_time = 0;                  // 最后接收到RC信号的时间

/**
  * @简  述  阿克曼转向控制 - 设置后轮速度和前轮转向角度
  * @参  数  rear_speed: 后轮速度 (-7200~7200)
  * @参  数  front_angle: 前轮转向角度 (45~135度)
  * @返回值  无
  */
void AckermannDrive_SetSpeedAndAngle(int16_t rear_speed, uint8_t front_angle)
{
    // 限幅处理
    if(rear_speed > REAR_MOTOR_SPEED_MAX) rear_speed = REAR_MOTOR_SPEED_MAX;
    if(rear_speed < -REAR_MOTOR_SPEED_MAX) rear_speed = -REAR_MOTOR_SPEED_MAX;
    if(front_angle < (SERVO_STRAIGHT_ANGLE - SERVO_MAX_TURN_ANGLE)) 
        front_angle = SERVO_STRAIGHT_ANGLE - SERVO_MAX_TURN_ANGLE;
    if(front_angle > (SERVO_STRAIGHT_ANGLE + SERVO_MAX_TURN_ANGLE)) 
        front_angle = SERVO_STRAIGHT_ANGLE + SERVO_MAX_TURN_ANGLE;
    
    // 后轮电机并联驱动 - 同向转动
    AX_MOTOR_A_SetSpeed(rear_speed);   // 后轮电机A
    AX_MOTOR_B_SetSpeed(rear_speed);   // 后轮电机B
    
    // 前轮舵机转向
    AX_SERVO_SetAngle(front_angle);
}

/**
  * @简  述  RC遥控解算 - 将RC输入转换为阿克曼转向控制
  * @参  数  throttle_percent: 油门百分比 (-100~100)
  * @参  数  steering_percent: 转向百分比 (-100~100) 
  * @返回值  无
  */
void RC_AckermannControl(int16_t throttle_percent, int16_t steering_percent)
{
    int16_t rear_speed;
    uint8_t front_angle;
    
    // 死区处理
    if(abs(throttle_percent) < RC_DEADBAND) throttle_percent = 0;
    if(abs(steering_percent) < RC_DEADBAND) steering_percent = 0;
    
    // 计算后轮速度 (根据油门)
    rear_speed = (int32_t)throttle_percent * REAR_MOTOR_SPEED_MAX / 100;
    
    // 计算前轮转向角度 (根据转向量)
    // steering_percent: -100(右转) ~ 0(直行) ~ +100(左转)
    front_angle = SERVO_STRAIGHT_ANGLE + ((int32_t)steering_percent * SERVO_MAX_TURN_ANGLE / 100);
    
    // 设置阿克曼转向
    AckermannDrive_SetSpeedAndAngle(rear_speed, front_angle);
}

/**
  * @简  述  RC模式控制
  * @参  数  无
  * @返回值  无
  */
void RCMode_Control(void)
{
    uint32_t current_time = AX_DELAY_GetTick();
    
    // 检查RC信号是否有效
    if(AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_1) && AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_2))
    {
        // 获取RC信号百分比
        int16_t throttle = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);  // 油门通道
        int16_t steering = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);  // 转向通道
        
        // 执行RC阿克曼转向控制
        RC_AckermannControl(throttle, steering);
        
        // 更新最后接收时间和连接状态
        last_rc_time = current_time;
        if(!rc_connected)
        {
            rc_connected = 1;
            printf("RC signal connected!\r\n");
        }
    }
    else
    {
        // RC信号丢失，检查超时
        if(current_time - last_rc_time > RC_TIMEOUT_MS)
        {
            if(rc_connected)
            {
                printf("RC signal lost! Stopping motors...\r\n");
                rc_connected = 0;
            }
            // 超时，停止电机并保持前轮直行
            AckermannDrive_SetSpeedAndAngle(0, SERVO_STRAIGHT_ANGLE);
        }
    }
}

/**
  * @简  述  主函数
  * @参  数  无
  * @返回值  无
  */
int main(void)
{
    uint32_t last_print_time = 0;
    uint32_t current_time = 0;
    
    // 精准延时初始化
    AX_DELAY_Init();    
    
    // 调试串口初始化
    AX_UART_Init(115200);
    printf("  \r\n"); // 发送空格和回车,解决部分串口显示BUG
    
    printf("========================================\r\n");
    printf("    XTARK Ackermann RC Control Car     \r\n");
    printf("========================================\r\n");
    printf("RC Control: CH1=Throttle, CH2=Steering\r\n");
    printf("Rear Motors: PA8/PA11 (PWM), PB3/4/5/8 (Direction)\r\n");
    printf("Front Servo: PA3 (TIM5_CH4 PWM)\r\n");
    printf("RC Deadband: %d%%, Timeout: %dms\r\n", RC_DEADBAND, RC_TIMEOUT_MS);
    printf("========================================\r\n");
    
    // JTAG调试配置
    AX_JTAG_Set(JTAG_SWD_DISABLE);  // 关闭JTAG接口
    AX_JTAG_Set(SWD_ENABLE);        // 开启SWD接口 可以用来调试和下载程序
    
    // 外设初始化
    AX_MOTOR_Init();        // 电机驱动初始化
    AX_SERVO_Init();        // 舵机驱动初始化
    AX_ENCODER_A_Init();    // 编码器A初始化
    AX_ENCODER_B_Init();    // 编码器B初始化
    AX_VIN_Init();          // 电压检测初始化
    AX_PWM_INPUT_Init();    // PWM输入捕获初始化
    
    printf("System initialized successfully!\r\n");
    printf("Ready for RC control...\r\n\r\n");
    
    // 初始化状态
    last_rc_time = AX_DELAY_GetTick();
    
    while (1) 
    {
        current_time = AX_DELAY_GetTick();
        
        // RC模式控制
        RCMode_Control();
        
        // 每200ms打印一次调试信息
        if (current_time - last_print_time >= 200)
        {
            last_print_time = current_time;
            
            // 获取编码器数据
            encoder1 = AX_ENCODER_A_GetCounter();
            encoder2 = AX_ENCODER_B_GetCounter();
            
            // 清零编码器
            AX_ENCODER_A_SetCounter(0);  
            AX_ENCODER_B_SetCounter(0);    
            
            // 获取电压采集值
            adc_value = AX_VIN_GetVol_X100();
            
            // 获取RC信号
            uint16_t ch1_pulse = AX_PWM_INPUT_GetPulse(PWM_INPUT_CHANNEL_1);
            uint16_t ch2_pulse = AX_PWM_INPUT_GetPulse(PWM_INPUT_CHANNEL_2);
            int16_t ch1_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);
            int16_t ch2_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);
            
            // 打印调试信息
            printf("[%s] ENC_A:%4d ENC_B:%4d VIN:%d.%02dV ", 
                   rc_connected ? "RC_ON" : "RC_OFF",
                   encoder1, encoder2, adc_value/100, adc_value%100);
                   
            printf("CH1:%4dus(%3d%%) CH2:%4dus(%3d%%)\r\n",
                   ch1_pulse, ch1_percent, ch2_pulse, ch2_percent);
        }
        
        // 短延时，提高控制响应速度
        AX_Delayms(10);
    }
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/