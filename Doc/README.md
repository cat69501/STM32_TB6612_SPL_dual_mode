# STM32F103 双模式驱动控制系统

## 项目概述

本项目基于STM32F103C8T6微控制器，实现了**差速驱动**和**阿克曼转向**两种驱动模式的无缝切换控制系统。采用TB6612FNG双路电机驱动芯片，支持手动串口命令控制和自动模式切换，为智能小车提供了灵活多样的运动控制方案。

### 主要特性

- **双驱动模式**：支持差速驱动（坦克模式）和阿克曼转向（汽车模式）
- **模式无缝切换**：通过串口命令实时切换驱动模式
- **高精度PWM调速**：10kHz PWM频率，支持正反转和速度调节
- **正交编码器接口**：硬件编码器模式，实时速度和位置反馈
- **舵机转向控制**：50Hz PWM信号，支持精确角度控制（阿克曼模式）
- **PWM信号捕获**：支持RC遥控器信号接入
- **串口命令控制**：115200波特率，丰富的控制和查询命令
- **实时状态监控**：电源电压监测、编码器数值、模式状态显示
- **模块化软件架构**：各功能模块独立封装，易于扩展和移植

## 硬件设计

### 🚗 双模式工作原理

#### 1. 差速驱动模式（坦克模式）
- **原理**：左右两个电机独立控制，通过速度差实现转向
- **特点**：可原地转弯，转弯半径为0，适合狭窄空间机动
- **转向方式**：左转时左轮反转/右轮正转，右转时左轮正转/右轮反转

#### 2. 阿克曼转向模式（汽车模式）  
- **原理**：后轮并联驱动提供动力，前轮舵机控制转向角度
- **特点**：转向平稳自然，符合汽车驾驶习惯，适合高速行驶
- **转向方式**：后轮同向转动，前轮舵机转向控制方向

### 系统框图

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   STM32F103C8T6 │     │   TB6612FNG     │     │   电机A + 编码器 │
│                 │     │                 │     │  （左轮/后轮A） │
│ PA8  (PWMA) ────┼────▶│ PWMA       AOUT1├────▶│ M+              │
│ PB3  (AIN1) ────┼────▶│ AIN1       AOUT2├────▶│ M-              │
│ PB4  (AIN2) ────┼────▶│ AIN2            │     │                 │
│                 │     │                 │     │ 编码器A    编码器B│
│ PA6  (E1A)  ◀───┼─────┼─────────────────┼─────┤ A          A    │
│ PA7  (E1B)  ◀───┼─────┼─────────────────┼─────┤ B          B    │
│                 │     │                 │     └─────────────────┘
│ PA11 (PWMB) ────┼────▶│ PWMB       BOUT1├────▶┌─────────────────┐
│ PB5  (BIN1) ────┼────▶│ BIN1       BOUT2├────▶│   电机B + 编码器 │
│ PB8  (BIN2) ────┼────▶│ BIN2            │     │  （右轮/后轮B） │
│                 │     │                 │     │ M+         M-   │
│ PA0  (E2A)  ◀───┼─────┼─────────────────┼─────┤                 │
│ PA1  (E2B)  ◀───┼─────┼─────────────────┼─────┤                 │
│                 │     │ STBY ◀──── 5V   │     └─────────────────┘
│                 │     │ VM   ◀──── VIN  │
│                 │     └─────────────────┘     ┌─────────────────┐
│ PA3  (SERVO)────┼─────────────────────────────▶│   前轮转向舵机   │
│                 │                             │ (阿克曼模式)     │
│ PB6  (RC_CH1)◀──┼───── RC遥控器信号输入        │                 │
│ PB7  (RC_CH2)◀──┼───── RC遥控器信号输入        └─────────────────┘
│                 │
│ PA2  (ADC) ◀────┼───── 电压检测(1/11分压)
│                 │
│ PA9  (TX)  ─────┼───── 串口输出 (命令控制)
│ PA10 (RX)  ─────┼───── 串口输入 (命令控制)
└─────────────────┘
```

### 引脚定义

| 功能分类 | 引脚 | 功能说明 | 备注 |
|---------|------|---------|------|
| **电机A控制** | | | 差速模式：左轮 / 阿克曼模式：后轮A |
| PWMA | PA8 | TIM1_CH1 PWM输出 | 控制电机A速度 |
| AIN1 | PB3 | 电机A方向控制1 | 与AIN2配合控制转向 |
| AIN2 | PB4 | 电机A方向控制2 | AIN1=1,AIN2=0正转 |
| **电机B控制** | | | 差速模式：右轮 / 阿克曼模式：后轮B |
| PWMB | PA11 | TIM1_CH4 PWM输出 | 控制电机B速度 |
| BIN1 | PB5 | 电机B方向控制1 | 与BIN2配合控制转向 |
| BIN2 | PB8 | 电机B方向控制2 | BIN1=1,BIN2=0正转 |
| **舵机控制** | | | 阿克曼模式前轮转向 |
| SERVO | PA3 | TIM5_CH4 PWM输出 | 50Hz PWM，控制转向角度 |
| **编码器接口** | | | |
| E1A | PA6 | TIM3_CH1编码器A相 | 电机A编码器 |
| E1B | PA7 | TIM3_CH2编码器B相 | 电机A编码器 |
| E2A | PA0 | TIM2_CH1编码器A相 | 电机B编码器 |
| E2B | PA1 | TIM2_CH2编码器B相 | 电机B编码器 |
| **PWM输入捕获** | | | 遥控器信号接入 |
| RC_CH1 | PB6 | TIM4_CH1输入捕获 | RC通道1（油门/前进后退） |
| RC_CH2 | PB7 | TIM4_CH2输入捕获 | RC通道2（转向/左右） |
| **其他接口** | | | |
| VIN_DET | PA2 | ADC1_CH2电压检测 | 通过1/11分压 |
| UART_TX | PA9 | USART1_TX | 调试输出和命令控制 |
| UART_RX | PA10 | USART1_RX | 命令接收 |

### TB6612控制逻辑

| AIN1/BIN1 | AIN2/BIN2 | PWM | 电机状态 |
|-----------|-----------|-----|---------|
| 0 | 0 | X | 制动(短路) |
| 0 | 1 | PWM | 反转 |
| 1 | 0 | PWM | 正转 |
| 1 | 1 | X | 停止(高阻) |

## 软件架构

### 目录结构

```
STM32_TB6612_SPL_dual_mode/
├── Doc/                          # 项目文档
│   ├── README.md                # 项目说明（本文档）
│   ├── dual_mode_readme.md      # 双模式详细说明
│   ├── IO_ALLOCATION.md         # 引脚分配表
│   ├── PWM_TEST_GUIDE.md        # PWM测试指南
│   └── PWMcontrol.md            # PWM控制说明
├── Driver/                       # 硬件驱动层
│   ├── ax_delay.h/c             # 精准延时函数
│   ├── ax_encoder.h/c           # 编码器驱动
│   ├── ax_motor.h/c             # 双路电机控制驱动
│   ├── ax_servo.h/c             # 舵机控制驱动 ⭐
│   ├── ax_pwm_input.h/c         # PWM输入捕获驱动 ⭐
│   ├── ax_rc_motor.h/c          # RC遥控电机驱动 ⭐
│   ├── ax_sys.h/c               # 系统配置
│   ├── ax_uart.h/c              # 串口驱动（已增强）
│   └── ax_vin.h/c               # 电压检测驱动
├── Libraries/                    # STM32标准外设库
│   ├── CMSIS/                   # ARM CMSIS库
│   └── STM32F10x_StdPeriph_Driver/  # STM32外设驱动库
├── Project/                      # Keil工程文件
│   ├── DebugConfig/             # 调试配置
│   ├── Objects/                 # 编译目标文件
│   └── xproject.uvprojx         # Keil工程文件
└── User/                         # 用户应用层
    ├── dual_mode_main.c         # 双模式控制主程序 ⭐
    ├── main.c                   # 阿克曼RC遥控主程序
    ├── ackermann_rc_main.c      # 纯阿克曼RC控制程序
    ├── demo_main.c              # 基础功能演示
    ├── diff_demo_main.c         # 差速模式演示
    ├── differential_main.c      # 纯差速模式程序
    ├── stm32f10x_conf.h         # STM32库配置
    ├── stm32f10x_it.c           # 中断服务程序
    └── stm32f10x_it.h           # 中断服务头文件
```

### 软件模块说明

#### 1. 电机控制模块 (ax_motor.c/h)

**功能描述**：
- 使用TIM1产生两路独立的PWM信号
- 控制TB6612FNG驱动芯片实现电机正反转和调速
- 支持差速模式和阿克曼模式的不同控制策略
- PWM频率10kHz，占空比分辨率7200级

**主要API**：
```c
void AX_MOTOR_Init(void);                      // 初始化电机控制
void AX_MOTOR_A_SetSpeed(int16_t speed);      // 设置电机A速度
void AX_MOTOR_B_SetSpeed(int16_t speed);      // 设置电机B速度
```

#### 2. 舵机控制模块 (ax_servo.c/h) ⭐

**功能描述**：
- 使用TIM5_CH4产生50Hz PWM信号控制舵机
- 支持0-180度角度控制，分辨率约1度
- 专用于阿克曼转向模式的前轮转向控制

**主要API**：
```c
void AX_SERVO_Init(void);                     // 初始化舵机控制
void AX_SERVO_SetAngle(uint8_t angle);        // 设置舵机角度(0-180度)
```

#### 3. 编码器模块 (ax_encoder.c/h)

**功能描述**：
- 使用TIM2和TIM3的编码器模式读取两路编码器
- 自动识别正反转，16位计数器
- 提供读取当前值和清零功能

**主要API**：
```c
void AX_ENCODER_A_Init(void);                 // 初始化编码器A
void AX_ENCODER_B_Init(void);                 // 初始化编码器B
int16_t AX_ENCODER_A_GetCounter(void);       // 读取编码器A计数值
int16_t AX_ENCODER_B_GetCounter(void);       // 读取编码器B计数值
void AX_ENCODER_A_SetCounter(int16_t count); // 设置编码器A计数值
void AX_ENCODER_B_SetCounter(int16_t count); // 设置编码器B计数值
```

#### 4. PWM输入捕获模块 (ax_pwm_input.c/h) ⭐

**功能描述**：
- 使用TIM4捕获RC遥控器PWM信号
- 支持双通道信号捕获和解算
- 提供脉宽测量和百分比转换功能
- 内置信号有效性检测和超时保护

**主要API**：
```c
void AX_PWM_INPUT_Init(void);                 // 初始化PWM输入捕获
uint16_t AX_PWM_INPUT_GetPulse(uint8_t ch);   // 获取PWM脉宽(微秒)
int16_t AX_PWM_INPUT_GetPercent(uint8_t ch);  // 获取PWM百分比(-100~+100)
uint8_t AX_PWM_INPUT_IsValid(uint8_t ch);     // 检查信号是否有效
```

#### 5. 串口通信模块 (ax_uart.c/h) - 已增强

**功能描述**：
- USART1配置为115200波特率
- 重定向printf实现调试输出
- 支持中断接收和命令解析
- 增强的字符接收和缓冲功能

**主要API**：
```c
void AX_UART_Init(uint32_t bound);            // 初始化串口
int fputc(int ch, FILE *f);                   // printf重定向
uint8_t AX_UART_Available(void);              // 检查是否有数据可读
uint8_t AX_UART_GetChar(void);                // 读取接收字符
```

#### 6. RC电机控制模块 (ax_rc_motor.c/h) ⭐

**功能描述**：
- 专门用于RC遥控器控制的电机接口
- 集成阿克曼转向控制算法
- 支持RC信号死区处理和安全保护

**主要API**：
```c
void AX_RC_MOTOR_Init(void);                  // 初始化RC电机控制
void AX_RC_MOTOR_Control(int16_t throttle, int16_t steering); // RC控制接口
```

#### 7. 电压检测模块 (ax_vin.c/h)

**功能描述**：
- 使用ADC1通道2检测电源电压
- 1/11分压电路，最大检测电压33V
- 返回值为实际电压×100

**主要API**：
```c
void AX_VIN_Init(void);                       // 初始化ADC
uint16_t AX_VIN_GetVol_X100(void);            // 获取电压值(×100)
```

#### 8. 延时模块 (ax_delay.c/h)

**功能描述**：
- 基于SysTick定时器的精确延时
- 支持微秒级和毫秒级延时
- 提供系统时钟获取功能

**主要API**：
```c
void AX_DELAY_Init(void);                     // 延时模块初始化  
void AX_Delayus(uint32_t nus);                // 微秒延时
void AX_Delayms(uint16_t nms);                // 毫秒延时
uint32_t AX_DELAY_GetTick(void);              // 获取系统时钟(ms)
```

#### 9. 系统配置模块 (ax_sys.c/h)

**功能描述**：
- JTAG/SWD调试接口配置
- GPIO位带操作宏定义

**主要功能**：
```c
void AX_JTAG_Set(uint8_t mode);               // 设置JTAG模式
// 位带操作宏
#define PAout(n) ...                          // PA口输出
#define PAin(n) ...                           // PA口输入
```

## 使用说明

### 🎮 双模式控制示例

本项目提供多种控制模式，主要包括：

1. **dual_mode_main.c** - 双模式串口命令控制（推荐）
2. **main.c** - 阿克曼RC遥控控制  
3. **differential_main.c** - 纯差速模式演示

#### 双模式串口命令控制（dual_mode_main.c）

```c
#include "stm32f10x.h"
#include <stdio.h>
#include "ax_delay.h"       
#include "ax_uart.h"        
#include "ax_motor.h"       
#include "ax_servo.h"       
#include "ax_encoder.h"     
#include "ax_vin.h"         

int main(void)
{
    // 系统初始化
    AX_DELAY_Init();        // 延时模块初始化
    AX_UART_Init(115200);   // 串口初始化
    AX_MOTOR_Init();        // 电机初始化
    AX_SERVO_Init();        // 舵机初始化
    AX_ENCODER_A_Init();    // 编码器A初始化
    AX_ENCODER_B_Init();    // 编码器B初始化
    AX_VIN_Init();          // 电压检测初始化
    
    printf("双模式控制系统已启动!\r\n");
    printf("发送 'h' 查看命令帮助\r\n");
    
    while(1) 
    {
        // 检查串口命令
        if(AX_UART_Available()) {
            char cmd = AX_UART_GetChar();
            ProcessCommand(cmd);  // 处理命令
        }
        
        // 状态监控和数据输出
        // ...
        
        AX_Delayms(10);
    }
}
```

#### 阿克曼RC遥控控制（main.c）

```c
#include "stm32f10x.h"
#include "ax_motor.h"
#include "ax_servo.h" 
#include "ax_pwm_input.h"

int main(void)
{
    // 系统初始化
    AX_MOTOR_Init();        // 电机控制初始化
    AX_SERVO_Init();        // 舵机控制初始化  
    AX_PWM_INPUT_Init();    // PWM输入捕获初始化
    
    while(1)
    {
        // 获取RC遥控器信号
        int16_t throttle = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_1);
        int16_t steering = AX_PWM_INPUT_GetPercent(PWM_INPUT_CHANNEL_2);
        
        // 阿克曼转向控制
        if(AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_1) && 
           AX_PWM_INPUT_IsValid(PWM_INPUT_CHANNEL_2))
        {
            RC_AckermannControl(throttle, steering);
        }
        else 
        {
            // RC信号丢失，停止运动
            AX_MOTOR_A_SetSpeed(0);
            AX_MOTOR_B_SetSpeed(0);
            AX_SERVO_SetAngle(90);  // 舵机回正
        }
        
        AX_Delayms(10);
    }
}
```

### 🎯 串口控制命令

通过串口工具（115200波特率）发送以下命令控制小车：

#### 模式切换命令
| 命令 | 功能 |
|------|------|
| `1` | 切换到差速驱动模式 |
| `2` | 切换到阿克曼转向模式 |

#### 运动控制命令  
| 命令 | 功能 |
|------|------|
| `w` | 前进 |
| `s` | 后退 |
| `a` | 左转 |
| `d` | 右转 |
| `x` | 停止 |

#### 信息查询命令
| 命令 | 功能 |
|------|------|
| `h` | 显示帮助信息 |
| `i` | 显示当前状态信息 |

### 控制说明

#### 差速驱动模式
- **前进**：两电机同向转动
- **后退**：两电机同向反转
- **左转**：左电机反转，右电机正转（原地左转）
- **右转**：左电机正转，右电机反转（原地右转）

#### 阿克曼转向模式  
- **前进**：后轮电机同向正转，前轮舵机保持直行
- **后退**：后轮电机同向反转，前轮舵机保持直行  
- **左转**：后轮电机同向正转，前轮舵机左转
- **右转**：后轮电机同向正转，前轮舵机右转

### 速度参数设置

```c
// 可在代码中调整的参数
#define MOTOR_SPEED_NORMAL    4000    // 正常速度 (约55%功率)
#define MOTOR_SPEED_TURN      3000    // 转向速度 (约42%功率)
#define SERVO_MAX_TURN_ANGLE  45      // 舵机最大转向角度
```

### RC遥控器使用

1. **硬件连接**：
   - RC_CH1 (PB6) ← 遥控器通道1（油门）
   - RC_CH2 (PB7) ← 遥控器通道2（转向）

2. **信号格式**：
   - PWM信号：1000-2000μs
   - 中位：1500μs
   - 死区：±5%

3. **安全保护**：
   - 信号丢失自动停止
   - 超时保护（1秒）

### 注意事项

1. **电源要求**：
   - 逻辑电源：3.3V（STM32供电）
   - 电机电源：根据电机规格选择（TB6612 VM引脚）
   - 舵机电源：5V独立供电（推荐）
   - 确保共地连接

2. **硬件连接**：
   - 确保电机极性正确
   - 编码器需要独立供电（通常3.3V或5V）
   - 注意编码器A/B相序
   - 舵机信号线连接PA3，电源独立供电

3. **模式切换**：
   - 不同模式下电机功能不同，切换前应停止运动
   - 差速模式：两电机独立控制
   - 阿克曼模式：两电机并联作为后轮，舵机控制前轮

4. **PWM参数**：
   - 电机PWM：10kHz，适合大多数小型直流电机
   - 舵机PWM：50Hz，标准舵机控制频率
   - 可根据具体硬件调整参数

5. **串口使用**：
   - 波特率：115200
   - 命令为单字符，立即执行
   - 输入命令后会有确认反馈

## 开发环境配置

### 硬件需求

- **MCU**：STM32F103C8T6（最小系统板）
- **驱动**：TB6612FNG电机驱动模块  
- **电机**：带编码器的直流减速电机×2
- **舵机**：标准5V舵机（阿克曼模式用）
- **电源**：逻辑3.3V，电机电源根据需求，舵机5V
- **调试器**：ST-Link V2或兼容调试器
- **遥控器**：标准PWM信号RC遥控器（可选）

### 软件需求

- **IDE**：Keil MDK v5.23或更高版本
- **固件库**：STM32F10x标准外设库 v3.5.0
- **串口工具**：用于命令控制和调试信息查看

### 快速开始

1. **准备工程**：
   - 打开工程文件：`Project/xproject.uvprojx`
   - 选择合适的main文件作为编译入口
   - 编译工程：Build → Rebuild all target files

2. **下载程序**：
   - 连接ST-Link调试器和目标板
   - 下载程序：Flash → Download

3. **测试功能**：
   - 连接串口工具(115200,8,N,1)
   - 发送 'h' 查看帮助信息
   - 发送 '1' 或 '2' 切换模式
   - 使用 w/s/a/d/x 控制运动

## 扩展开发

### 可添加功能

1. **PID闭环控制**：基于编码器反馈实现精确速度控制
2. **路径规划**：预设运动轨迹和自动巡航
3. **传感器集成**：超声波避障、IMU姿态控制
4. **无线通信**：WiFi/蓝牙远程控制
5. **上位机软件**：图形化控制界面

### 参数优化建议

1. 根据实际机械结构调整舵机转向角度范围
2. 根据电机特性调整PWM频率和速度参数  
3. 根据应用场景优化控制死区和响应速度
4. 添加软启动和刹车减速功能

## 故障排除

### 常见问题

1. **电机不转**：
   - 检查电源连接和STBY引脚（应接高电平）
   - 测试PWM输出和方向控制信号
   - 检查电机驱动器和电机连接

2. **舵机不工作**：
   - 检查PA3引脚PWM输出（示波器测试）
   - 确认舵机电源供应（5V）
   - 检查舵机信号线连接

3. **编码器无读数**：
   - 检查编码器电源和A/B相信号
   - 确认引脚连接正确
   - 使用示波器查看编码器波形

4. **串口无输出**：
   - 检查波特率设置(115200)
   - 确认TX/RX连接正确
   - 检查串口工具配置

5. **RC信号异常**：
   - 检查PB6/PB7引脚连接
   - 确认RC遥控器信号格式（1000-2000μs）
   - 测试PWM输入捕获功能

## 版本历史

- **V1.0** (2024-01) - 初始版本，基本双路电机控制
- **V2.0** (2024-12) - 添加舵机控制和阿克曼转向模式
- **V3.0** (2025-01) - 双模式无缝切换，串口命令控制
- **V3.1** (2025-01) - 添加RC遥控器支持和PWM输入捕获

### 待开发功能

- [ ] PID闭环速度控制
- [ ] 路径规划和自动导航
- [ ] 多传感器融合
- [ ] 上位机控制软件
- [ ] 无线通信模块集成

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 联系方式

如有问题或建议，请通过以下方式联系：
- 项目作者：Qi Hao Ran
- 项目网址：https://ben0724-ace.github.io/  
- 技术支持：[邮箱地址]

---
*最后更新：2025年1月*