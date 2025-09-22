/**                             
 ******************************************************************************
 * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
 * @文  件  阿克曼转向控制演示程序
 * @说  明  演示前进、后退、左转、右转循环动作
 *          使用后轮驱动+前轮转向的阿克曼转向模式
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

// 阿克曼转向参数定义
#define REAR_MOTOR_SPEED_NORMAL    4000    // 后轮正常速度 (约55%功率)
#define REAR_MOTOR_SPEED_TURN      3000    // 后轮转向速度 (约42%功率)
#define REAR_MOTOR_SPEED_SLOW      2000    // 后轮慢速 (约28%功率)

#define SERVO_STRAIGHT_ANGLE       90      // 前轮直行角度
#define SERVO_MAX_TURN_ANGLE       45      // 前轮最大转向角度
#define SERVO_LEFT_ANGLE          (SERVO_STRAIGHT_ANGLE + SERVO_MAX_TURN_ANGLE)   // 左转角度
#define SERVO_RIGHT_ANGLE         (SERVO_STRAIGHT_ANGLE - SERVO_MAX_TURN_ANGLE)   // 右转角度

#define ACTION_DURATION            3000    // 每个动作持续时间(ms)
#define PAUSE_DURATION             1000    // 动作间暂停时间(ms)

// 阿克曼转向动作枚举
typedef enum {
    ACKERMANN_ACTION_FORWARD = 0,     // 前进
    ACKERMANN_ACTION_BACKWARD,        // 后退  
    ACKERMANN_ACTION_TURN_LEFT,       // 左转
    ACKERMANN_ACTION_TURN_RIGHT,      // 右转
    ACKERMANN_ACTION_PAUSE,           // 暂停
    ACKERMANN_ACTION_COUNT            // 动作总数
} AckermannAction_t;

// 全局变量
int16_t  encoder1, encoder2;        // 编码器数值
uint16_t adc_value;                 // 电压采集值
AckermannAction_t current_action = ACKERMANN_ACTION_FORWARD;
uint32_t action_timer = 0;          // 动作计时器
uint32_t action_start_time = 0;     // 动作开始时间

/**
  * @简  述  阿克曼转向控制 - 前进
  * @参  数  speed: 后轮速度值
  * @返回值  无
  */
void AckermannDrive_Forward(int16_t speed)
{
    printf("Ackermann Action: Forward at speed %d\r\n", speed);
    // 后轮电机并联驱动 - 同向转动
    AX_MOTOR_A_SetSpeed(speed);   // 后轮电机A正转
    AX_MOTOR_B_SetSpeed(speed);   // 后轮电机B正转
    // 前轮舵机保持直行
    AX_SERVO_SetAngle(SERVO_STRAIGHT_ANGLE);
}

/**
  * @简  述  阿克曼转向控制 - 后退
  * @参  数  speed: 后轮速度值
  * @返回值  无
  */
void AckermannDrive_Backward(int16_t speed)
{
    printf("Ackermann Action: Backward at speed %d\r\n", speed);
    // 后轮电机并联驱动 - 同向反转
    AX_MOTOR_A_SetSpeed(-speed);  // 后轮电机A反转
    AX_MOTOR_B_SetSpeed(-speed);  // 后轮电机B反转
    // 前轮舵机保持直行
    AX_SERVO_SetAngle(SERVO_STRAIGHT_ANGLE);
}

/**
  * @简  述  阿克曼转向控制 - 左转
  * @参  数  speed: 后轮速度值
  * @返回值  无
  */
void AckermannDrive_TurnLeft(int16_t speed)
{
    printf("Ackermann Action: Turn Left at speed %d, servo angle %d\r\n", speed, SERVO_LEFT_ANGLE);
    // 后轮电机并联驱动 - 同向前进
    AX_MOTOR_A_SetSpeed(speed);   // 后轮电机A正转
    AX_MOTOR_B_SetSpeed(speed);   // 后轮电机B正转
    // 前轮舵机左转
    AX_SERVO_SetAngle(SERVO_LEFT_ANGLE);
}

/**
  * @简  述  阿克曼转向控制 - 右转
  * @参  数  speed: 后轮速度值
  * @返回值  无
  */
void AckermannDrive_TurnRight(int16_t speed)
{
    printf("Ackermann Action: Turn Right at speed %d, servo angle %d\r\n", speed, SERVO_RIGHT_ANGLE);
    // 后轮电机并联驱动 - 同向前进
    AX_MOTOR_A_SetSpeed(speed);   // 后轮电机A正转
    AX_MOTOR_B_SetSpeed(speed);   // 后轮电机B正转
    // 前轮舵机右转
    AX_SERVO_SetAngle(SERVO_RIGHT_ANGLE);
}

/**
  * @简  述  阿克曼转向控制 - 停止
  * @参  数  无
  * @返回值  无
  */
void AckermannDrive_Stop(void)
{
    printf("Ackermann Action: Stop\r\n");
    // 后轮电机停止
    AX_MOTOR_A_SetSpeed(0);       // 后轮电机A停止
    AX_MOTOR_B_SetSpeed(0);       // 后轮电机B停止
    // 前轮舵机回正
    AX_SERVO_SetAngle(SERVO_STRAIGHT_ANGLE);
}

/**
  * @简  述  执行当前阿克曼转向动作
  * @参  数  action: 动作类型
  * @返回值  无
  */
void ExecuteAckermannAction(AckermannAction_t action)
{
    switch(action)
    {
        case ACKERMANN_ACTION_FORWARD:
            AckermannDrive_Forward(REAR_MOTOR_SPEED_NORMAL);
            break;
            
        case ACKERMANN_ACTION_BACKWARD:
            AckermannDrive_Backward(REAR_MOTOR_SPEED_NORMAL);
            break;
            
        case ACKERMANN_ACTION_TURN_LEFT:
            AckermannDrive_TurnLeft(REAR_MOTOR_SPEED_TURN);
            break;
            
        case ACKERMANN_ACTION_TURN_RIGHT:
            AckermannDrive_TurnRight(REAR_MOTOR_SPEED_TURN);
            break;
            
        case ACKERMANN_ACTION_PAUSE:
        default:
            AckermannDrive_Stop();
            break;
    }
}

/**
  * @简  述  获取阿克曼转向动作名称字符串
  * @参  数  action: 动作类型
  * @返回值  动作名称字符串
  */
const char* GetAckermannActionName(AckermannAction_t action)
{
    switch(action)
    {
        case ACKERMANN_ACTION_FORWARD:    return "FORWARD";
        case ACKERMANN_ACTION_BACKWARD:   return "BACKWARD";
        case ACKERMANN_ACTION_TURN_LEFT:  return "TURN_LEFT";
        case ACKERMANN_ACTION_TURN_RIGHT: return "TURN_RIGHT";
        case ACKERMANN_ACTION_PAUSE:      return "PAUSE";
        default:                          return "UNKNOWN";
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
    uint32_t action_duration;
    
    // 精准延时初始化
    AX_DELAY_Init();    
    
    // 调试串口初始化
    AX_UART_Init(115200);
    printf("  \r\n"); // 发送空格和回车,解决部分串口显示BUG
    
    printf("========================================\r\n");
    printf("      XTARK Ackermann Steering Demo     \r\n");
    printf("========================================\r\n");
    printf("Rear Motors: PA8/PA11 (PWM), PB3/4/5/8 (Direction)\r\n");
    printf("Front Servo: PA3 (TIM5_CH4 PWM)\r\n");
    printf("Actions: Forward -> Backward -> Left Turn -> Right Turn\r\n");
    printf("Each action lasts %d seconds\r\n", ACTION_DURATION/1000);
    printf("========================================\r\n");
    
    // JTAG调试配置
    AX_JTAG_Set(JTAG_SWD_DISABLE);  // 关闭JTAG接口
    AX_JTAG_Set(SWD_ENABLE);        // 开启SWD接口 可以用来调试和下载程序
    
    // 电机驱动初始化
    AX_MOTOR_Init();
    
    // 舵机驱动初始化
    AX_SERVO_Init();
    
    // 编码器驱动初始化
    AX_ENCODER_A_Init();  
    AX_ENCODER_B_Init();    
    
    // 电压检测初始化
    AX_VIN_Init();
    
    printf("System initialized successfully!\r\n");
    printf("Starting Ackermann steering demo...\r\n\r\n");
    
    // 初始化动作和计时器
    current_action = ACKERMANN_ACTION_FORWARD;
    action_start_time = AX_DELAY_GetTick();
    ExecuteAckermannAction(current_action);
    
    while (1) 
    {
        // 获取当前时间
        current_time = AX_DELAY_GetTick();
        
        // 确定当前动作应该持续的时间
        action_duration = (current_action == ACKERMANN_ACTION_PAUSE) ? PAUSE_DURATION : ACTION_DURATION;
        
        // 检查是否需要切换到下一个动作
        if(current_time - action_start_time >= action_duration)
        {
            // 切换到下一个动作
            current_action = (AckermannAction_t)((current_action + 1) % ACKERMANN_ACTION_COUNT);
            action_start_time = current_time;
            
            printf("\r\n--- Ackermann Action Switch ---\r\n");
            printf("New Action: %s\r\n", GetAckermannActionName(current_action));
            printf("Duration: %d ms\r\n", 
                   (current_action == ACKERMANN_ACTION_PAUSE) ? PAUSE_DURATION : ACTION_DURATION);
            
            // 执行新动作
            ExecuteAckermannAction(current_action);
        }
        
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
            
            // 获取电压采集的AD转换值
            adc_value = AX_VIN_GetVol_X100();
            
            // 计算剩余时间
            uint32_t elapsed_time = current_time - action_start_time;
            uint32_t remaining_time = action_duration - elapsed_time;
            
            // 打印调试信息
            printf("[%s] Rear_ENC_A:%4d Rear_ENC_B:%4d VIN:%d.%02dV Time:%d.%ds\r\n",
                   GetAckermannActionName(current_action),
                   encoder1, encoder2, 
                   adc_value/100, adc_value%100,
                   remaining_time/1000, (remaining_time%1000)/100);
        }
        
        // 短延时，提高控制响应速度
        AX_Delayms(10);
    }
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/