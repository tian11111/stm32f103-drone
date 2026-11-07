# STM32F103 Drone

基于 **STM32F103C8T6** 的四轴飞行器，包含**飞控**与**遥控器**两个独立工程。

## 当前状态

| 工程 | 软件状态 | 验证边界 |
| --- | --- | --- |
| [FlightController](FlightController/README.md) | HAL 驱动、双环 PID、混控、无线遥控/遥测、LED/OLED、电池检测及 VOFA 调参已接入 | Debug/Release 构建与相关主机回归通过；尚未完成实板联调和飞行验收 |
| RemoteControl | 四路摇杆、12 按键、无线发送、OLED 双页面和遥测接收已接入 | 本轮未修改或重新验证遥控器固件 |

飞控当前构建记录（2026-11-07）：Debug 与 Release 均使用 Flash **60,220 / 65,536 字节**，静态 RAM **5,552 / 20,480 字节**。这不包含运行时栈水位验证，也不代表已经完成实板烧录或试飞。

## 项目结构

```
hal/
├── hal.code-workspace      # VS Code 多根工作区（同时打开两个工程）
├── FlightController/       # 飞控工程（CubeMX + HAL + CMake）
│   ├── Bsp/                # 业务、硬件适配及主机回归测试
│   ├── README.md           # 飞控配置、构建、保护机制及待验证项
│   └── VOFA使用说明.md     # FireWater 通道和 19 条调参命令模板
└── RemoteControl/          # 遥控器工程（CubeMX + HAL + CMake）
```

两个工程**互相独立**，各自有 `.vscode/` 与构建目录，可分别编译、烧录。

目前仅遥控器提供仓库内的 OpenOCD 烧录/调试任务；飞控已生成 ELF，但尚未提供专用的 `launch.json` 或烧录任务。

## 系统组成

| 设备 | 主控 | 说明 |
| --- | --- | --- |
| 飞控 | STM32F103C8T6 | MPU6050（DMP 姿态）+ NRF24L01 无线 + 电池检测 + OLED |
| 遥控器 | STM32F103C8T6 | 双摇杆 ADC + 12 按键 + NRF24L01 + OLED |

**无线链路**：两端 NRF24L01，地址 `11 22 33 44 55`，信道 2，2 Mbps，1 字节 CRC，自动应答，32 字节固定载荷。飞控使用软件 SPI，遥控器使用硬件 SPI2。

## 飞控功能

主循环读取 MPU6050 DMP、处理遥控帧、蓝牙命令和电池采样；TIM1 提供 1ms 节拍，并在中断中每 **4ms** 执行三轴双环 PID 和四电机混控。OLED 刷新及串口波形使用中断异步发送。

- 电机 PWM：TIM2 四路、20kHz，上电初值及锁机输出为 0。
- 遥控失联保护：约 500ms 无有效控制帧后锁机。
- 姿态保护：超过 50ms 无成功更新的完整姿态样本后锁机；恢复后需释放解锁键并重新解锁。
- VOFA：USART1、9600/8N1、FireWater 波形，支持三轴 PID 与俯仰零点共 19 个参数，修改在 RAM 中生效。

完整飞控引脚表、与原版的差异及剩余事项见 [飞控 README](FlightController/README.md)；VOFA 控件绑定和命令模板见 [VOFA 使用说明](FlightController/VOFA使用说明.md)。

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
| ADC1 | 四路摇杆 | 4 通道扫描 + 连续转换 + DMA 循环搬运，55.5 周期，12 MHz |
| TIM1 | 1 ms 节拍 | PSC=71, ARR=999 |
| SPI2 | NRF24L01 | 模式 0（CPOL=0/CPHA=1Edge），9 MHz |
| I2C1 | OLED SSD1306 | 400 kHz，PB8/PB9 重映射 |
| 系统时钟 | — | HSE 8 MHz → PLL×9 → 72 MHz |

> **注意**：NRF24L01 采用**轮询 STATUS** 方式判断收发结果，未使用 IRQ 引脚（该板未引出），这是原设计，请勿改为中断模式。

## 构建

### 前置要求

- [STM32CubeIDE for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=STMicroelectronics.stm32-vscode-extension) 扩展（自带 ninja 与 arm-none-eabi-gcc）
- 或自行安装 `cmake`、`ninja`、`arm-none-eabi-gcc` 并加入 PATH
- **OpenOCD**（烧录/调试用）：本项目使用全局安装，路径 `D:/dev/tools/openocd/xpack-openocd-0.12.0-7`

### 命令行

在需要构建的工程目录中执行，两套工程均提供 `Debug`、`Release` 预设。例如，从 `hal/` 构建飞控：

```powershell
cd FlightController
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

构建遥控器时进入 `RemoteControl`，执行相同预设命令。产物分别为：

- `FlightController/build/Debug/FlightController.elf`（或 `build/Release/FlightController.elf`）
- `RemoteControl/build/Debug/RemoteControl.elf`（或 `build/Release/RemoteControl.elf`）

飞控 Debug 使用 `-Os -g3`，Release 使用 `-Os -g0`，两者启用 LTO 以适配 64KB Flash；优化可能影响调试时单步与局部变量显示。

### VS Code

打开 `hal.code-workspace`，在左侧选择对应工程根目录后构建。

### 烧录与调试

`RemoteControl/.vscode/launch.json` 提供两套配置，`tasks.json` 提供对应的纯烧录任务：

| 配置 | 探针 | 接口 |
| --- | --- | --- |
| DAPLink 烧录并调试 (10MHz) | CMSIS-DAP（VID `0xc251` / PID `0xf001`） | SWD |
| ST-Link 烧录并调试 (10MHz) | ST-Link | SWD |

先在 `RemoteControl` 完成一次 CMake 配置，再按 `F5` 选择配置烧录并调试；`Ctrl+Shift+P` → `Tasks: Run Task` → `烧录 (DAPLink 10MHz)` 只烧不调试。上述配置仅针对遥控器，并包含本机工具链绝对路径，换电脑需调整。

> **SWD 速率**：10 MHz 对 F103 偏高。若出现 `No ACK`、校验失败或连接不稳，把 `openocd.cfg` / `openocd-stlink.cfg` 里的 `adapter speed` 依次降到 4000 / 2000 / 1000。
>
> **注意**：`adapter speed` 必须写在 `source [find target/stm32f1x.cfg]` **之后**，因为该 target 配置内部自带 `adapter speed 1000`，写在前面会被覆盖。

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
| [1] / [2] | 电机 1 / 2 的 `PWM_OUT / 36`，占空比百分数值 |
| [3] | 0 |
| [4]-[7] | Pitch 角度（float 小端） |
| [8]-[11] | Roll 角度（float 小端） |
| [12]-[15] | 电池电压（float 小端） |
| [16]-[31] | 0 |

飞控在收到 `Mode==0x01` 的控制帧后回传一帧。遥控器当前沿用 `Angle`、`PWM`、`Spd` 页面标签：`Angle` 显示 Pitch，`PWM` 显示前两路 PWM 百分数，`Spd` 的两个数实际为 **Roll 和电池电压**，不是速度。

### VOFA 串口调参

飞控 USART1（PA9/PA10）使用 **9600/8N1**。VOFA 接收协议选 `firewater`，固件发送：

```text
pid:50,0.00,1.25,-3.56,4.12
```

五列依次为累计调用计数时间、目标 Pitch、实际 Pitch、俯仰角度环输出、电池电压，帧尾为真实 CRLF。第一列是普通数据通道，不是自动配置的横轴时间；发送忙时会跳过波形帧。

VOFA 滑条绑定 Str 命令，例如 `[slider,AngleKp,%f]`；手动发送时使用具体数值，如 `[slider,AngleKp,3.20]`。参数立即在 RAM 中生效，重启恢复默认值；没有参数确认回包或 Flash 保存功能。完整 19 条模板见 [VOFA 使用说明](FlightController/VOFA使用说明.md)。

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
