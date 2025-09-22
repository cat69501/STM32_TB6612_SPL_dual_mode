# PWM遥控功能测试说明

## 修改概要

为了解决IO冲突问题，本次修改重新分配了PWM输入和舵机控制的IO：

### 主要修改
1. **PWM输入捕获**：从TIM2(PA0/PA1)改为TIM4(PB6/PB7)
2. **舵机PWM输出**：从TIM3_CH1(PA6)改为TIM5_CH4(PA3)
3. **编码器功能保持不变**：
   - 编码器A继续使用TIM3(PA6/PA7)
   - 编码器B继续使用TIM2(PA0/PA1)

## 硬件连接测试

### 1. 遥控器接收器连接
```
接收器 → STM32
CH1 → PB6 (PWM输入通道1 - 电机控制)
CH2 → PB7 (PWM输入通道2 - 舵机控制)
VCC → 5V
GND → GND
```

### 2. 舵机连接
```
舵机 → STM32
信号线 → PA3
电源+ → 5V或6V
电源- → GND
```

## 功能测试步骤

### 1. 编译并下载程序
- 使用Keil MDK打开工程
- 编译无错误后下载到STM32

### 2. 串口监视
- 打开串口工具，设置115200波特率
- 应该看到以下启动信息：
```
XTARK TB6612 PWM Remote Control Demo
CH1: Motor speed control
CH2: Servo steering control
System initialized successfully!
Waiting for RC signal...
```

### 3. 遥控器测试
- 打开遥控器电源
- 推动遥控器摇杆，观察串口输出
- 正常情况下会显示：
```
PWM1:1500(  0%) PWM2:1500(  0%) ENC1:   0 ENC2:   0 VIN:1200 [OK]
```

### 4. 功能验证
- **电机控制**：推动CH1摇杆，电机应该前进/后退
- **舵机控制**：推动CH2摇杆，舵机应该左右转动
- **编码器反馈**：电机转动时，ENC1/ENC2数值应该变化
- **信号丢失保护**：关闭遥控器，应显示[NO SIGNAL]，电机停止

## 故障排查

### 1. 无PWM信号输入
- 检查PB6/PB7连接是否正确
- 确认遥控器接收器供电正常
- 使用示波器检查PB6/PB7是否有PWM信号

### 2. 舵机不动作
- 检查PA3连接是否正确
- 确认舵机供电正常
- 测量PA3是否有50Hz PWM输出

### 3. 编码器功能异常
- 编码器功能应该不受影响
- 如有问题，检查PA0/PA1和PA6/PA7连接

## 代码验证点

1. **ax_pwm_input.c**：
   - 所有TIM2已改为TIM4
   - GPIO从GPIOA改为GPIOB
   - 引脚从Pin_0/Pin_1改为Pin_6/Pin_7

2. **ax_servo.c**：
   - 所有TIM3已改为TIM5
   - 通道从CH1改为CH4
   - 引脚从Pin_6改为Pin_3

3. **中断函数**：
   - TIM2_IRQHandler已改为TIM4_IRQHandler

## 注意事项

1. 本修改已验证与现有编码器功能无冲突
2. 支持STM32F103C8T6和STM32F103RCT6
3. 如需恢复原设计，请参考git历史版本