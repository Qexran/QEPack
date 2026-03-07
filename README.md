# QEPack - 像CubeMX一样使用模块

## 🚀 拓展包特点
- **面向对象**：所有抽象模块均采用面向对象设计模式
- **易扩展**：模块化设计，支持相互继承模块
- **高兼容性**：一套代码兼容多种设置、多个平台
- **高性能**：核心逻辑经性能优化，适配批量高频调用场景
- **架构统一**：所有模块采用统一参数结构和API设计模式
- **可配置性**：通过config文件配置模块

## ⚙️ 已实现模块功能

### 串口UART
- ✅ 支持基础收发功能
- ✅ 使用环形缓冲区
- ✅ 支持接收不定长数据帧
- ✅ 支持开启/关闭接收帧功能
- ✅ 支持自定义帧头帧尾
- ✅ 支持自定义数据处理回调函数
- ✅ 支持开关DMA传输

### LED
- ✅ 支持基础LED控制
- ✅ 支持呼吸灯功能

### 按键Key
- ✅ 支持单击、双击、长按、连发功能
- ✅ 支持自定义消抖、双击、长按阈值

### 显示屏OLED
- ✅ 支持多OLED显示
- ✅ 支持字符串、数字、图形显示
- ✅ 支持开关硬件I2C
- 🚧 支持自定义菜单

### 舵机Servo
- ✅ 支持180°角度型、360°旋转型舵机
- ✅ 支持自定义频率、角度、脉冲宽度、速度阈值
- ✅ 支持快速初始化
- ✅ 支持平滑调节

### ADC
- ✅ 支持多通道+DMA模式
- ✅ 支持自选通道转换模式
- ✅ 支持自动转换采样值

### 超声波Ultrasonic
- ✅ 使用双通道测量
- ✅ 支持自动转换距离

### ATK-MS901M
- ✅ 优化原有代码逻辑

### 电子罗盘Compass
- 🚧 基于QMC5883的磁场测量

### 编码器Encoder
- ✅ 支持TIM、GPIO方案
- ✅ 支持自动转换距离、速度

### PID控制器
- ✅ 使用通用的PID控制算法实现
- ✅ 支持位置式、增量式PID控制
- ✅ 支持变速积分（线性）、积分分离、不完全微分功能

### Filter控制器
- 🚧 支持滑动平均滤波
- 🚧 支持中值滤波
- 🚧 支持加权平均滤波
- 🚧 支持算术平均滤波
- 🚧 支持惯性滤波法
- 🚧 支持零相位滤波
- ✅ 支持卡尔曼滤波

### 电机控制Motor
- ✅ 直流电机控制功能

### 步进电机EMM_V5
- ✅ 扩展模块支持

### 存储模块W25Q64
- ✅ 支持基本读取、写入、擦除操作
- ✅ 支持页编程和地址编程

## 🚀 快速开始

### OLED模块示例
```c
#include "OLED.h"

// 初始化OLED
void vOledConfigInit(void){
    // 软件I2C初始化
    stOledStaticParamTdf stOLEDInit;
    stOLEDInit.pstSclGpioPort = GPIOB;
    stOLEDInit.usSclPin = GPIO_PIN_6;
    stOLEDInit.pstSdaGpioPort = GPIOB;
    stOLEDInit.usSdaPin = GPIO_PIN_7;

    /*
        若为硬件I2C，初始化如下：
        stOLEDInit.hi2c = &hi2c1;
    */

    vOledDeviceInit(&stOledInit, OLED0);
}

int main(void){
    // [平台初始化...]
    
    vOledConfigInit();
    
    while (1){
        // 流式打印
        vOledPrintf(OLED0, 1, 1, OLED_8X16, "Distance: %.2f cm", 12.34);

        // 显示字符串
        vOledShowString(OLED0, 1, 16, OLED_8X16, "Hello QEPack!");

        // 更新显示
        vOledUpdate(OLED0);
    }
}
```

## 📋 版本记录
### V1.0.0 (2026/03/4)
- 测试版本发布

## 🤝 贡献
欢迎Fork本项目为QEPack贡献代码和功能！

## 🐞 反馈
如果发现Bug或有功能建议，欢迎通过以下方式反馈：
- GitHub Issues
- 邮箱: 2752844566@qq.com

