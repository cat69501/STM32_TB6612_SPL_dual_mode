/**                             
 ******************************************************************************
 * @作  者  Qi Hao Ran
 * @网  址  https://ben0724-ace.github.io/
 * @文  件  RC遥控差速小车控制程序
 * @说  明  支持RC遥控器和自动演示模式
 *
 ******************************************************************************
 */

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "ax_sys.h"         // 系统配置
#include "ax_delay.h"       // 精准延时
#include "ax_uart.h"        // 调试串口
#include "ax_motor.h"       // 电机驱动
#include "ax_encoder.h"     // 编码器驱动
#include "ax_vin.h"         // 电压采集
#include "ax_pwm_input.h"   // PWM输入捕获

// 控制参数定义
#define MOTOR_SPEED_MAX       7200    // 最大速度 (100%功率)
#define MOTOR_SPEED_NORMAL    4000    // 正常速度 (约55%功率)
#define MOTOR_SPEED_TURN      3000    // 转向速度 (约42%功率)

#define RC_DEADBAND           5       // RC遥控器死区(百分比)
#define RC_TIMEOUT_MS         1000    // RC信号超时时间(ms)

#define AUTO_ACTION_DURATION  3000    // 自动模式每个动作持续时间(ms)
#define AUTO_PAUSE_DURATION   1000    // 自动模式动作间暂停时间(ms)

// 控制模式枚举
typedef enum {
    CONTROL_MODE_AUTO = 0,    // 自动演示模式
    CONTROL_MODE_RC           // RC遥控模式
} ControlMode_t;

// 自动模式动作枚举
typedef enum {
    AUTO_ACTION_FORWARD = 0,  // 前进
    AUTO_ACTION_BACKWARD,     // 后退  
    AUTO_ACTION_TURN_LEFT,    // 左转圈
    AUTO_ACTION_TURN_RIGHT,   // 右转圈
    AUTO_ACTION_PAUSE,        // 暂停
    AUTO_ACTION_COUNT         // 动作总数
} AutoAction_t;

// 全局变量
int16_t  encoder1, encoder2;                    // 编码器数值
uint16_t adc_value;                             // 电压采集值
ControlMode_t control_mode = CONTROL_MODE_AUTO; // 当前控制模式
AutoAction_t auto_action = AUTO_ACTION_FORWARD; // 自动模式当前动作
uint32_t auto_action_timer = 0;                 // 自动模式动作计时器
uint32_t last_rc_time = 0;                      // 最后接收到RC信号的时间

/**
  * @简  述  差速控制 - 设置左右轮速度
  * @参  数  left_speed: 左轮速度 (-7200~7200)
  * @参  数  right_speed: 右轮速度 (-7200~7200)
  * @返回值  无
  */
void DifferentialDrive_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    // 限幅处理
    if(left_speed > MOTOR_SPEED_MAX) left_speed = MOTOR_SPEED_MAX;
    if(left_speed < -MOTOR_SPEED_MAX) left_speed = -MOTOR_SPEED_MAX;
    if(right_speed > MOTOR_SPEED_MAX) right_speed = MOTOR_SPEED_MAX;
    if(right_speed < -MOTOR_SPEED_MAX) right_speed = -MOTOR_SPEED_MAX;
    
    AX_MOTOR_B_SetSpeed(left_speed);   // 电机B为左轮
    AX_MOTOR_A_SetSpeed(right_speed);  // 电机A为右轮
}

/**
  * @简  述  RC遥控解算 - 将RC输入转换为差速控制
  * @参  数  throttle_percent: 油门百分比 (-100~100)
  * @参  数  steering_percent: 转向百分比 (-100~100) 
  * @返回值  无
  */
void RC_DifferentialControl(int16_t throttle_percent, int16_t steering_percent)
{
    int16_t left_speed, right_speed;
    int16_t base_speed, turn_diff;
    
    // 死区处理
    if(abs(throttle_percent) < RC_DEADBAND) throttle_percent = 0;
    if(abs(steering_percent) < RC_DEADBAND) steering_percent = 0;
    
    // 计算基础速度 (根据油门)
    base_speed = (int32_t)throttle_percent * MOTOR_SPEED_MAX / 100;
    
    // 计算转向差速 (根据转向量)
    turn_diff = (int32_t)steering_percent * MOTOR_SPEED_MAX / 200; // 转向影响减半
    
    // 差速计算
    left_speed = base_speed - turn_diff;   // 左转时左轮减速
    right_speed = base_speed + turn_diff;  // 左转时右轮加速
    
    // 设置电机速度
    DifferentialDrive_SetSpeed(left_speed, right_speed);
}

/**
  * @简  述  自动模式控制
  * @参  数  无
  * @返回值  无
  */
void AutoMode_Control(void)
{
    uint32_t current_time = AX_DELAY_GetTick();
    uint32_t action_duration = (auto_action == AUTO_ACTION_PAUSE) ? AUTO_PAUSE_DURATION : AUTO_ACTION_DURATION;
    
    // 检查是否需要切换动作
    if(current_time - auto_action_timer >= action_duration)
    {
        // 切换到下一个动作
        auto_action = (AutoAction_t)((auto_action + 1) % AUTO_ACTION_COUNT);
        auto_action_timer = current_time;
        
        printf("Auto Mode - Action: ");
        
        // 执行对应动作
        switch(auto_action)
        {
            case AUTO_ACTION_FORWARD:
                printf("FORWARD\r\n");
                DifferentialDrive_SetSpeed(MOTOR_SPEED_NORMAL, MOTOR_SPEED_NORMAL);
                break;
                
            case AUTO_ACTION_BACKWARD:
                printf("BACKWARD\r\n");
                DifferentialDrive_SetSpeed(-MOTOR_SPEED_NORMAL, -MOTOR_SPEED_NORMAL);
                break;
                
            case AUTO_ACTION_TURN_LEFT:
                printf("TURN_LEFT\r\n");
                DifferentialDrive_SetSpeed(-MOTOR_SPEED_TURN, MOTOR_SPEED_TURN);
                break;
                
            case AUTO_ACTION_TURN_RIGHT:
                printf("TURN_RIGHT\r\n");
                DifferentialDrive_SetSpeed(MOTOR_SPEED_TURN, -MOTOR_SPEED_TURN);
                break;
                
            case AUTO_ACTION_PAUSE:
            default:
                printf("PAUSE\r\n");
                DifferentialDrive_SetSpeed(0, 0);
                break;
        }
    }
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
        
        // 执行RC差速控制
        RC_DifferentialControl(throttle, steering);
        
        // 更新最后接收时间
        last_rc_time = current_time;
    }
    else
    {
        // RC信号丢失，检查超时
        if(current_time - last_rc_time > RC_TIMEOUT_MS)
        {
            // 超时，停止电机并切换到自动模式
            printf("RC signal lost! Switching to Auto Mode...\r\n");
            DifferentialDrive_SetSpeed(0, 0);
            control_mode = CONTROL_MODE_AUTO;
            auto_action_timer = current_time;
        }
    }
}

/**
  * @简  述  检查模式切换
  * @参  数  无
  * @返回值  无
  */
void CheckModeSwitch(void)
{
    static ControlMode_t last_mode = CONTROL_MODE_AUTO;
    
    // 检查是否有有效的RC信号来切换到RC模式
    if(control_mode == CONTROL_MODE_AUTO && 
       AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_1) && 
       AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_2))
    {
        // 检查RC信号是否不在中位(说明用户在操作)
        int16_t throttle = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);
        int16_t steering = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);
        
        if(abs(throttle) > RC_DEADBAND || abs(steering) > RC_DEADBAND)
        {
            control_mode = CONTROL_MODE_RC;
            last_rc_time = AX_DELAY_GetTick();
            printf("RC signal detected! Switching to RC Mode...\r\n");
        }
    }
    
    // 模式切换提示
    if(control_mode != last_mode)
    {
        if(control_mode == CONTROL_MODE_RC)
        {
            printf("=== RC CONTROL MODE ===\r\n");
        }
        else
        {
            printf("=== AUTO DEMO MODE ===\r\n");
        }
        last_mode = control_mode;
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
    printf("    XTARK RC Differential Drive Car    \r\n");
    printf("========================================\r\n");
    printf("RC Control: CH1=Throttle, CH2=Steering\r\n");
    printf("Auto Mode: Forward->Back->Left->Right\r\n");
    printf("RC Deadband: %d%%, Timeout: %dms\r\n", RC_DEADBAND, RC_TIMEOUT_MS);
    printf("========================================\r\n");
    
    // JTAG调试配置
    AX_JTAG_Set(JTAG_SWD_DISABLE);  // 关闭JTAG接口
    AX_JTAG_Set(SWD_ENABLE);        // 开启SWD接口 可以用来调试和下载程序
    
    // 外设初始化
    AX_MOTOR_Init();        // 电机驱动初始化
    AX_ENCODER_A_Init();    // 编码器A初始化
    AX_ENCODER_B_Init();    // 编码器B初始化
    AX_VIN_Init();          // 电压检测初始化
    AX_PWM_INPUT_Init();    // PWM输入捕获初始化
    
    printf("System initialized successfully!\r\n");
    printf("Starting in Auto Mode, RC override available...\r\n\r\n");
    
    // 初始化自动模式计时器
    auto_action_timer = AX_DELAY_GetTick();
    
    while (1) 
    {
        current_time = AX_DELAY_GetTick();
        
        // 检查模式切换
        CheckModeSwitch();
        
        // 根据当前模式执行控制
        if(control_mode == CONTROL_MODE_RC)
        {
            RCMode_Control();
        }
        else
        {
            AutoMode_Control();
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
            
            // 获取电压采集值
            adc_value = AX_VIN_GetVol_X100();
            
            // 获取RC信号
            uint16_t ch1_pulse = AX_PWM_INPUT_GetPulse(PWM_INPUT_CHANNEL_1);
            uint16_t ch2_pulse = AX_PWM_INPUT_GetPulse(PWM_INPUT_CHANNEL_2);
            int16_t ch1_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);
            int16_t ch2_percent = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);
            
            // 打印调试信息
            printf("[%s] ENC_A:%4d ENC_B:%4d VIN:%d.%02dV ", 
                   (control_mode == CONTROL_MODE_RC) ? "RC" : "AUTO",
                   encoder1, encoder2, adc_value/100, adc_value%100);
                   
            printf("CH1:%4dus(%3d%%) CH2:%4dus(%3d%%)\r\n",
                   ch1_pulse, ch1_percent, ch2_pulse, ch2_percent);
        }
        
        // 短延时，提高控制响应速度
        AX_Delayms(10);
    }
}

/******************* (C) 版权 2025 Qi Hao Ran **************************************/