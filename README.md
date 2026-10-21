# STM32F103 Drone

基于 **STM32F103C8T6** 的四轴飞行器，包含**飞控**与**遥控器**两个独立工程。

---

## 项目结构

```
hal/
├── hal.code-workspace      # VS Code 多根工作区（同时打开两个工程）
├── FlightController/       # 飞控工程（待创建）
└── RemoteControl/          # 遥控器工程（CubeMX + HAL + CMake）
```

两个工程**互相独立**，各自有 `.vscode/` 与构建目录，可分别编译、烧录。

## 系统组成

| 设备 | 主控 | 说明 |
| --- | --- | --- |
| 飞控 | STM32F103C8T6 | MPU6050（DMP 姿态）+ NRF24L01 无线 + 电池检测 + OLED |
| 遥控器 | STM32F103C8T6 | 双摇杆 ADC + 12 按键 + NRF24L01 + OLED |

**无线链路**：两端 NRF24L01，地址 `11 22 33 44 55`，信道 2，2 Mbps，1 字节 CRC，自动应答，32 字节固定载荷，无需配对。

## 遥控器功能

读四路摇杆 ADC + 扫描 12 个按键 → 每 100 ms 通过 NRF24L01 发送一帧控制数据 → OLED 显示（摇杆页 / 数据回传页）→ 回传页时接收飞控遥测。

### 引脚分配（遥控器）

| 引脚 | 功能 |
| --- | --- |
| PA0-PA3 | 摇杆 ADC CH0-CH3 |
| PA4 / PA5 | KEY11 / KEY12 |
| PA6 / PA7 | KEY1 / KEY2 |
| PA11 / PA12 | KEY7 / KEY8 |
| PB0 / PB1 / PB2 | KEY3 / KEY4 / KEY5 |
| PB5 / PB6 | KEY9 / KEY10 |
| PB8 / PB9 | OLED SCL / SDA（I2C1 重映射） |
| PB10 | KEY6 |
| PB11 | NRF CE |
| PB12 | NRF CSN（软件片选） |
| PB13 / PB14 / PB15 | NRF SCK / MISO / MOSI（硬件 SPI2） |

### 外设配置

| 外设 | 用途 | 关键参数 |
| --- | --- | --- |
| ADC1 | 四路摇杆 | 单通道单次，55.5 周期，12 MHz |
| TIM1 | 1 ms 节拍 | PSC=71, ARR=999 |
| SPI2 | NRF24L01 | 模式 0（CPOL=0/CPHA=1Edge），9 MHz |
| I2C1 | OLED SSD1306 | 400 kHz，PB8/PB9 重映射 |
| 系统时钟 | — | HSE 8 MHz → PLL×9 → 72 MHz |

> **注意**：NRF24L01 采用**轮询 STATUS** 方式判断收发结果，未使用 IRQ 引脚（该板未引出），这是原设计，请勿改为中断模式。

## 构建

### 前置要求

- [STM32CubeIDE for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=STMicroelectronics.stm32-vscode-extension) 扩展（自带 ninja 与 arm-none-eabi-gcc）
- 或自行安装 `cmake`、`ninja`、`arm-none-eabi-gcc` 并加入 PATH

### 命令行

```bash
cd RemoteControl
cmake -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -S . -B build/Debug -G Ninja
cmake --build build/Debug
```

产物：`build/Debug/RemoteControl.elf`

### VS Code

打开 `hal.code-workspace`，在左侧选择对应工程根目录后构建。

## 通信协议

### 控制帧（遥控器 → 飞控，32 字节）

| 字节 | 内容 | 说明 |
| --- | --- | --- |
| [0] | Mode | 0x00 摇杆页 / 0x01 回传页（触发飞控回遥测） |
| [1] | LH | -100 ~ +100 |
| [2] | LV | -100 ~ +100（前推为负） |
| [3] | RH | -100 ~ +100 |
| [4] | RV | -100 ~ +100 |
| [5] | KEY | 键号 1-12，发送成功后清 0 |
| [6]-[31] | 0 | 填充 |

### 遥测帧（飞控 → 遥控器，32 字节）

| 字节 | 内容 |
| --- | --- |
| [0] | 0x02 帧类型 |
| [1] / [2] | 电机 1 / 2 PWM |
| [4]-[7] | Pitch 角度（float 小端） |
| [8]-[11] | Roll 角度（float 小端） |
| [12]-[15] | 电池电压（float 小端） |

## 摇杆方向

| 轴 | 动作 | 符号 |
| --- | --- | --- |
| LV | 左摇杆前推 | 负值 |
| LV | 左摇杆后拉 | 正值 |
| LH / RH | 左推 / 右推 | 负 / 正 |
| RV | 右摇杆前推 / 后拉 | 负 / 正 |

> 飞控油门映射按「前推 = 负」编写，两侧必须保持一致。

## 许可证

本项目仅用于学习与个人开发。
