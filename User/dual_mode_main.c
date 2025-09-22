/**                           
 ******************************************************************************
 * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
 * @文  件  双模式控制演示程序
 * @说  明  支持差速模式和阿克曼转向模式切换
 *          差速模式：双轮独立控制实现转向
 *          阿克曼模式：后轮驱动+前轮转向
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

// 通用控制参数定义
#define MOTOR_SPEED_NORMAL         4000    // 正常速度 (约55%功率)
#define MOTOR_SPEED_TURN           3000    // 转向速度 (约42%功率)
#define MOTOR_SPEED_SLOW           2000    // 慢速 (约28%功率)

// 阿克曼转向舵机参数
#define SERVO_STRAIGHT_ANGLE       90      // 前轮直行角度
#define SERVO_MAX_TURN_ANGLE       45      // 前轮最大转向角度
#define SERVO_LEFT_ANGLE          (SERVO_STRAIGHT_ANGLE + SERVO_MAX_TURN_ANGLE)   // 左转角度
#define SERVO_RIGHT_ANGLE         (SERVO_STRAIGHT_ANGLE - SERVO_MAX_TURN_ANGLE)   // 右转角度

#define ACTION_DURATION            3000    // 每个动作持续时间(ms)
#define PAUSE_DURATION             1000    // 动作间暂停时间(ms)
#define MODE_SWITCH_DURATION       10000   // 模式切换周期(ms) - 仅用于自动切换模式

// 模式切换方式定义
#define MODE_SWITCH_AUTO           0       // 1:自动切换 0:手动切换
#define UART_BUFFER_SIZE           10      // 串口接收缓冲区大小

// 驱动模式枚举
typedef enum {
    DRIVE_MODE_DIFFERENTIAL = 0,    // 差速驱动模式
    DRIVE_MODE_ACKERMANN,           // 阿克曼转向模式
    DRIVE_MODE_COUNT                // 模式总数
} DriveMode_t;

// 动作枚举
typedef enum {
    ACTION_FORWARD = 0,     // 前进
    ACTION_BACKWARD,        // 后退  
    ACTION_TURN_LEFT,       // 左转
    ACTION_TURN_RIGHT,      // 右转
    ACTION_PAUSE,           // 暂停
    ACTION_COUNT            // 动作总数
} Action_t;

// 全局变量
int16_t  encoder1, encoder2;        // 编码器数值
uint16_t adc_value;                 // 电压采集值
DriveMode_t current_mode = DRIVE_MODE_DIFFERENTIAL;
Action_t current_action = ACTION_FORWARD;
uint32_t action_start_time = 0;     // 动作开始时间
uint32_t mode_start_time = 0;       // 模式开始时间

// 串口接收相关变量
char uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_index = 0;
uint8_t command_received = 0;

/**
  * @简  述  差速控制 - 前进
  * @参  数  speed: 速度值
  * @返回值  无
  */
void DifferentialDrive_Forward(int16_t speed)
{
    AX_MOTOR_A_SetSpeed(speed);   // 左轮正转
    AX_MOTOR_B_SetSpeed(speed);   // 右轮正转
}

/**
  * @简  述  差速控制 - 后退
  * @参  数  speed: 速度值
  * @返回值  无
  */
void DifferentialDrive_Backward(int16_t speed)
{
    AX_MOTOR_A_SetSpeed(-speed);  // 左轮反转
    AX_MOTOR_B_SetSpeed(-speed);  // 右轮反转
}

/**
  * @简  述  差速控制 - 左转圈(原地左转)
  * @参  数  speed: 速度值
  * @返回值  无
  */
void DifferentialDrive_TurnLeft(int16_t speed)
{
    AX_MOTOR_A_SetSpeed(-speed);  // 左轮反转
    AX_MOTOR_B_SetSpeed(speed);   // 右轮正转
}

/**
  * @简  述  差速控制 - 右转圈(原地右转)
  * @参  数  speed: 速度值
  * @返回值  无
  */
void DifferentialDrive_TurnRight(int16_t speed)
{
    AX_MOTOR_A_SetSpeed(speed);   // 左轮正转
    AX_MOTOR_B_SetSpeed(-speed);  // 右轮反转
}

/**
  * @简  述  阿克曼转向控制 - 前进
  * @参  数  speed: 后轮速度值
  * @返回值  无
  */
void AckermannDrive_Forward(int16_t speed)
{
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
    // 后轮电机并联驱动 - 同向前进
    AX_MOTOR_A_SetSpeed(speed);   // 后轮电机A正转
    AX_MOTOR_B_SetSpeed(speed);   // 后轮电机B正转
    // 前轮舵机右转
    AX_SERVO_SetAngle(SERVO_RIGHT_ANGLE);
}

/**
  * @简  述  停止所有运动
  * @参  数  无
  * @返回值  无
  */
void Drive_Stop(void)
{
    AX_MOTOR_A_SetSpeed(0);       // 电机A停止
    AX_MOTOR_B_SetSpeed(0);       // 电机B停止
    AX_SERVO_SetAngle(SERVO_STRAIGHT_ANGLE);  // 舵机回正
}

/**
  * @简  述  执行当前动作
  * @参  数  mode: 驱动模式, action: 动作类型
  * @返回值  无
  */
void ExecuteAction(DriveMode_t mode, Action_t action)
{
    int16_t speed = (action == ACTION_TURN_LEFT || action == ACTION_TURN_RIGHT) ? 
                    MOTOR_SPEED_TURN : MOTOR_SPEED_NORMAL;
    
    if (action == ACTION_PAUSE) {
        Drive_Stop();
        return;
    }
    
    switch(mode)
    {
        case DRIVE_MODE_DIFFERENTIAL:
            switch(action)
            {
                case ACTION_FORWARD:
                    DifferentialDrive_Forward(speed);
                    break;
                case ACTION_BACKWARD:
                    DifferentialDrive_Backward(speed);
                    break;
                case ACTION_TURN_LEFT:
                    DifferentialDrive_TurnLeft(speed);
                    break;
                case ACTION_TURN_RIGHT:
                    DifferentialDrive_TurnRight(speed);
                    break;
                default:
                    Drive_Stop();
                    break;
            }
            break;
            
        case DRIVE_MODE_ACKERMANN:
            switch(action)
            {
                case ACTION_FORWARD:
                    AckermannDrive_Forward(speed);
                    break;
                case ACTION_BACKWARD:
                    AckermannDrive_Backward(speed);
                    break;
                case ACTION_TURN_LEFT:
                    AckermannDrive_TurnLeft(speed);
                    break;
                case ACTION_TURN_RIGHT:
                    AckermannDrive_TurnRight(speed);
                    break;
                default:
                    Drive_Stop();
                    break;
            }
            break;
            
        default:
            Drive_Stop();
            break;
    }
}

/**
  * @简  述  获取驱动模式名称字符串
  * @参  数  mode: 驱动模式
  * @返回值  模式名称字符串
  */
const char* GetModeString(DriveMode_t mode)
{
    switch(mode)
    {
        case DRIVE_MODE_DIFFERENTIAL: return "DIFFERENTIAL";
        case DRIVE_MODE_ACKERMANN:    return "ACKERMANN";
        default:                      return "UNKNOWN";
    }
}

/**
  * @简  述  获取动作名称字符串
  * @参  数  action: 动作类型
  * @返回值  动作名称字符串
  */
const char* GetActionString(Action_t action)
{
    switch(action)
    {
        case ACTION_FORWARD:    return "FORWARD";
        case ACTION_BACKWARD:   return "BACKWARD";
        case ACTION_TURN_LEFT:  return "TURN_LEFT";
        case ACTION_TURN_RIGHT: return "TURN_RIGHT";
        case ACTION_PAUSE:      return "PAUSE";
        default:                return "UNKNOWN";
    }
}

/**
  * @简  述  显示帮助信息
  * @参  数  无
  * @返回值  无
  */
void ShowHelp(void)
{
    printf("\r\n========== 控制命令帮助 ==========\r\n");
    printf("模式切换命令:\r\n");
    printf("  1 - 切换到差速驱动模式\r\n");
    printf("  2 - 切换到阿克曼转向模式\r\n");
    printf("手动控制命令:\r\n");
    printf("  w - 前进\r\n");
    printf("  s - 后退\r\n");
    printf("  a - 左转\r\n");
    printf("  d - 右转\r\n");
    printf("  x - 停止\r\n");
    printf("其他命令:\r\n");
    printf("  h - 显示帮助信息\r\n");
    printf("  i - 显示当前状态信息\r\n");
    printf("=================================\r\n\r\n");
}

/**
  * @简  述  显示当前状态信息
  * @参  数  无
  * @返回值  无
  */
void ShowStatus(void)
{
    printf("\r\n========== 当前状态信息 ==========\r\n");
    printf("当前模式: %s\r\n", GetModeString(current_mode));
    printf("当前动作: %s\r\n", GetActionString(current_action));
    printf("切换方式: %s\r\n", MODE_SWITCH_AUTO ? "自动切换" : "手动切换");
    if (MODE_SWITCH_AUTO) {
        uint32_t current_time = AX_DELAY_GetTick();
        uint32_t mode_remaining = MODE_SWITCH_DURATION - (current_time - mode_start_time);
        printf("模式剩余时间: %d.%d秒\r\n", mode_remaining/1000, (mode_remaining%1000)/100);
    }
    printf("编码器A: %d, 编码器B: %d\r\n", encoder1, encoder2);
    printf("电压: %d.%02dV\r\n", adc_value/100, adc_value%100);
    printf("=================================\r\n\r\n");
}

/**
  * @简  述  处理串口接收到的命令
  * @参  数  command: 接收到的命令字符
  * @返回值  无
  */
void ProcessCommand(char command)
{
    switch(command)
    {
        case '1':
            if (current_mode != DRIVE_MODE_DIFFERENTIAL) {
                current_mode = DRIVE_MODE_DIFFERENTIAL;
                current_action = ACTION_FORWARD;
                action_start_time = AX_DELAY_GetTick();
                mode_start_time = action_start_time;
                printf("\r\n>> 切换到差速驱动模式\r\n");
                ExecuteAction(current_mode, current_action);
            }
            break;
            
        case '2':
            if (current_mode != DRIVE_MODE_ACKERMANN) {
                current_mode = DRIVE_MODE_ACKERMANN;
                current_action = ACTION_FORWARD;
                action_start_time = AX_DELAY_GetTick();
                mode_start_time = action_start_time;
                printf("\r\n>> 切换到阿克曼转向模式\r\n");
                ExecuteAction(current_mode, current_action);
            }
            break;
            
        case 'w':
        case 'W':
            current_action = ACTION_FORWARD;
            action_start_time = AX_DELAY_GetTick();
            printf("\r\n>> 手动控制: 前进\r\n");
            ExecuteAction(current_mode, current_action);
            break;
            
        case 's':
        case 'S':
            current_action = ACTION_BACKWARD;
            action_start_time = AX_DELAY_GetTick();
            printf("\r\n>> 手动控制: 后退\r\n");
            ExecuteAction(current_mode, current_action);
            break;
            
        case 'a':
        case 'A':
            current_action = ACTION_TURN_LEFT;
            action_start_time = AX_DELAY_GetTick();
            printf("\r\n>> 手动控制: 左转\r\n");
            ExecuteAction(current_mode, current_action);
            break;
            
        case 'd':
        case 'D':
            current_action = ACTION_TURN_RIGHT;
            action_start_time = AX_DELAY_GetTick();
            printf("\r\n>> 手动控制: 右转\r\n");
            ExecuteAction(current_mode, current_action);
            break;
            
        case 'x':
        case 'X':
            current_action = ACTION_PAUSE;
            action_start_time = AX_DELAY_GetTick();
            printf("\r\n>> 手动控制: 停止\r\n");
            ExecuteAction(current_mode, current_action);
            break;
            
        case 'h':
        case 'H':
            ShowHelp();
            break;
            
        case 'i':
        case 'I':
            ShowStatus();
            break;
            
        default:
            printf("\r\n>> 未知命令: %c, 输入 'h' 查看帮助\r\n", command);
            break;
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
    printf("       XTARK Dual Mode Drive Demo      \r\n");
    printf("========================================\r\n");
    printf("Mode 1: Differential Drive (Tank Mode)\r\n");
    printf("  - Left Motor: PA8(PWM), PB3/4(Dir)\r\n");
    printf("  - Right Motor: PA11(PWM), PB5/8(Dir)\r\n");
    printf("Mode 2: Ackermann Steering (Car Mode)\r\n");
    printf("  - Rear Motors: PA8/PA11(PWM), PB3/4/5/8(Dir)\r\n");
    printf("  - Front Servo: PA3(TIM5_CH4)\r\n");
    if (MODE_SWITCH_AUTO) {
        printf("切换方式: 自动切换 (每%d秒)\r\n", MODE_SWITCH_DURATION/1000);
        printf("Each action lasts %d seconds\r\n", ACTION_DURATION/1000);
    } else {
        printf("切换方式: 手动切换 (通过串口命令)\r\n");
        printf("输入 'h' 查看控制命令帮助\r\n");
    }
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
    printf("Starting dual mode drive demo...\r\n");
    printf("Current Mode: %s\r\n", GetModeString(current_mode));
    
    if (!MODE_SWITCH_AUTO) {
        ShowHelp();
    } else {
        printf("\r\n");
    }
    
    // 初始化计时器
    current_time = AX_DELAY_GetTick();
    action_start_time = current_time;
    mode_start_time = current_time;
    ExecuteAction(current_mode, current_action);
    
    while (1) 
    {
        // 获取当前时间
        current_time = AX_DELAY_GetTick();
        
        // 检查串口接收
        if (AX_UART_Available()) {
            char received_char = (char)AX_UART_GetChar();
            
            // 处理接收到的字符
            if (received_char >= 32 && received_char <= 126) { // 可打印字符
                ProcessCommand(received_char);
            }
        }
        
        // 仅在自动切换模式下检查是否需要切换驱动模式
        if (MODE_SWITCH_AUTO && (current_time - mode_start_time >= MODE_SWITCH_DURATION))
        {
            // 切换到下一个模式
            current_mode = (DriveMode_t)((current_mode + 1) % DRIVE_MODE_COUNT);
            mode_start_time = current_time;
            action_start_time = current_time;
            current_action = ACTION_FORWARD; // 重置动作
            
            printf("\r\n**** AUTO MODE SWITCH ****\r\n");
            printf("New Mode: %s\r\n", GetModeString(current_mode));
            printf("**************************\r\n\r\n");
            
            ExecuteAction(current_mode, current_action);
        }
        
        // 仅在自动切换模式下检查是否需要切换动作
        if (MODE_SWITCH_AUTO) {
            // 确定当前动作应该持续的时间
            action_duration = (current_action == ACTION_PAUSE) ? PAUSE_DURATION : ACTION_DURATION;
            
            // 检查是否需要切换到下一个动作
            if(current_time - action_start_time >= action_duration)
            {
                // 切换到下一个动作
                current_action = (Action_t)((current_action + 1) % ACTION_COUNT);
                action_start_time = current_time;
                
                printf("\r\n--- Auto Action Switch ---\r\n");
                printf("Mode: %s, Action: %s\r\n", GetModeString(current_mode), GetActionString(current_action));
                printf("Duration: %d ms\r\n", 
                       (current_action == ACTION_PAUSE) ? PAUSE_DURATION : ACTION_DURATION);
                
                // 执行新动作
                ExecuteAction(current_mode, current_action);
            }
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
            
            // 打印调试信息
            if (MODE_SWITCH_AUTO) {
                // 自动模式显示剩余时间
                uint32_t elapsed_time = current_time - action_start_time;
                uint32_t remaining_time = action_duration - elapsed_time;
                uint32_t mode_remaining = MODE_SWITCH_DURATION - (current_time - mode_start_time);
                
                printf("[%s:%s] ENC_A:%4d ENC_B:%4d VIN:%d.%02dV T:%d.%ds M:%d.%ds\r\n",
                       GetModeString(current_mode), GetActionString(current_action),
                       encoder1, encoder2, 
                       adc_value/100, adc_value%100,
                       remaining_time/1000, (remaining_time%1000)/100,
                       mode_remaining/1000, (mode_remaining%1000)/100);
            } else {
                // 手动模式仅显示基本信息
                printf("[%s:%s] ENC_A:%4d ENC_B:%4d VIN:%d.%02dV (手动模式)\r\n",
                       GetModeString(current_mode), GetActionString(current_action),
                       encoder1, encoder2, 
                       adc_value/100, adc_value%100);
            }
        }
        
        // 短延时，提高控制响应速度
        AX_Delayms(10);
    }
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/