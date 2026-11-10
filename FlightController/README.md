# FlightController（飞控）

STM32F103C8T6 飞控的 **CubeMX + HAL + CMake** 工程。原标准库工程的主流程、双环 PID、混控及外设驱动已接入构建，当前处于**软件移植已接入、实板验收待完成**阶段。

配置参考工作区上一级的 `飞控-CubeMX配置与移植指南.md`；当前代码存在下述主动改动和待补项，不能把指南中的示意接口、周期描述或验证清单当作已实现/已通过的结果。

## 已接入的功能

- MPU6050 硬件 I2C2，默认使用前台 MEKF 姿态解算；可在构建时选择 DMP FIFO 回退。
- 三轴角度外环/角速度内环 PID、X 型四路混控及 TIM2 电机输出。
- NRF24L01 软件 SPI，接收平衡车遥控器控制帧，按需回传遥测。
- LED 状态指示、硬件 I2C1 OLED 显示、电池 ADC 检测。
- USART1 蓝牙接收、19 个在线调参项及 FireWater 异步波形发送。
- 遥控失联锁机、姿态数据超时保护和恢复后的重新解锁要求。

代码集中在 `Bsp/`，真实业务入口为 `Data_Proc.h` 和 `Variable.h`；不依赖指南示例中的 `fly_control.h`、`powersensor.h` 等重命名接口。

## 引脚与外设

| 引脚 | 功能 | 当前配置 |
| --- | --- | --- |
| PA0～PA3 | 电机 PWM 1～4 | TIM2 CH1～CH4，20kHz，高电平有效，初始 CCR=0 |
| PA4 / PA5 | NRF CSN / SCK | 软件 SPI，推挽输出 |
| PB6 / PB7 | NRF MOSI / MISO | 软件 SPI，MISO 上拉输入 |
| PB14 / PB15 | NRF CE / IRQ | CE 输出，IRQ 上拉输入，主循环轮询 |
| PA7 | 状态 LED | 高电平点亮 |
| PB8 / PB9 | OLED SCL / SDA | I2C1 重映射，400kHz，设备地址 `0x3C` |
| PB10 / PB11 | MPU6050 SCL / SDA | I2C2，100kHz，设备地址 `0x68` |
| PB1 | 电池电压 | ADC1 IN9，55.5 周期，ADC 时钟 12MHz |
| PA9 / PA10 | 蓝牙 TX / RX | USART1，9600/8N1，无流控 |
| PA13 / PA14 | 调试 | SWD |

系统使用 8MHz HSE、PLL×9，HCLK=72MHz，APB1=36MHz，APB2=72MHz；TIM1 使用 PSC=71、ARR=999 提供 1ms 节拍。NVIC 分组为 2，TIM1 优先级 1/1、USART1 为 2/2、I2C1 EV/ER 为 3/0。

NRF 引脚跨 GPIOA/GPIOB，不能直接替换成一组硬件 SPI。OLED 使用 PB8/PB9 重映射，不能占用 NRF 的 PB6/PB7。

## 主流程与保护

CubeMX 初始化外设后，程序初始化 DWT 延时、所选姿态源、LED、电机、NRF、电池 ADC、OLED 和串口，最后启动 TIM1 中断。默认 MEKF 上电初始化期间必须物理保持静止；OLED 显示 `IMU INIT / KEEP STILL`，静止初始化完成前姿态无效且保持锁机。首次有效姿态后显示正常数据和 `IMU:OK`；MEKF 初始化或运行中锁存故障时显示 `IMU FAULT / KEEP STILL / RESTART`。检测到启动运动或姿态源故障时会锁存，须静止后重新上电；恒速航向旋转可能被误判为陀螺零偏。DMP 回退的初始化失败进入 `Error_Handler`。详见 [IMU 说明](Bsp/Imu/README.md)。

| 任务 | 执行位置 | 节奏 |
| --- | --- | --- |
| 双环 PID、混控、电机写入 | TIM1 回调 | 稳态每 4ms，与原源码 `++Task_Slow_time > 3` 一致 |
| 姿态读取 | 主循环 | MEKF 约 5ms 调度槽；DMP 约 4ms 调度槽，数据约每 5ms 产生 |
| 遥控接收 | 主循环 | 约 6ms 调度槽 |
| 业务处理、蓝牙调参 | 主循环 | 约 20ms 调度槽 |
| 显示、串口波形 | 主循环 | 约 50ms 调度槽 |
| 电池采样 | 主循环 | 约 10ms 调度槽，100 个有效样本后下一次调用更新平均值 |

后台计数到期后保持待处理，直到主循环领取；实际完成时间受外设通信和主循环负载影响。OLED 使用异步快照分页面发送，主循环持续调用 `OLED_Task()` 推进。波形使用静态缓冲区和 UART 中断发送，忙时跳过新帧。

- 上电 `Lock=1`，锁机或失联时四路 CCR 强制为 0。
- KEY7 上锁；KEY8 仅在低油门、姿态新鲜且满足重新解锁条件时解锁。
- 约 500ms 无有效遥控帧时锁机并清油门。
- 超过 50ms 没有成功更新的完整姿态样本时锁机，清零油门、电机输出和 PID 积分状态；电机层也检查姿态新鲜度。
- 姿态恢复后需先收到非 KEY8 的控制帧，再重新按 KEY8 解锁；MEKF 锁存故障须静止后重新上电。
- 电池电压保持 `3.3 × 2 × ADC / 4096`，采样失败不参与平均。

## 与原标准库版本的主要差异

PID 默认参数、公式、混控符号、遥控轴方向宏、无线寄存器配置和帧布局保留。外环默认 `2.85/0.003/0.048`，内环默认 `4.0/0/0.025`。

除 HAL 外设替换外，当前还有这些运行行为变化：

- OLED、波形异步发送，忙时丢弃本次刷新/波形请求，减少主循环阻塞。
- 默认 MEKF 在前台读取原始数据并解算；DMP 回退每次最多读取四包，仍有较新积压时不发布样本，角速度取同包 DMP 校准结果。
- DMP 回退初始化包含陀螺仪和加速度计自检及偏置写入检查；`get_ms()` 和 FIFO 时间戳接入 HAL 系统时间。
- 增加姿态新鲜度门槛和故障后重新解锁要求；后台任务增加待处理锁存。
- 蓝牙增加接收长度/错误处理和数值检查，波形添加 `pid:` 前缀。原 CSV 本身也符合允许省略前缀的 FireWater 格式。

这些变化已有软件测试覆盖，实际控制响应仍需实板验证，不能据此声明与原版飞行表现相同。

## VOFA+ 波形与调参

VOFA+ 连接蓝牙串口，设置 **9600/8N1**，接收协议选择 **firewater**。帧格式为：

```text
pid:50,0.00,1.25,-3.56,4.12
```

帧尾为真实 CRLF，五列依次为：累计调用计数时间、目标 Pitch、实际 Pitch、俯仰角度环输出、电池电压。第一列不会自动成为 VOFA 的横轴时间，也不是硬件采样时间戳。

滑条使用 Str/ASCII 命令和 `%f` 占位符，例如 `[slider,AngleKp,%f]`；手动发送时填写实际值，如 `[slider,AngleKp,3.20]`。支持三轴 PID 共 18 项及俯仰机械零点 `Offset`，也保留旧 SET 命令。

参数在 RAM 中生效，重启恢复默认值；没有参数确认/读取回包和 Flash 保存命令。接收端只保留一条待处理命令，建议逐条发送、间隔至少 100ms。

完整通道映射、19 条模板、默认值和滑条绑定步骤见 [VOFA 使用说明](VOFA使用说明.md)。说明提供手动配置步骤，未提供或验证 VOFA 工程导入文件。

## 构建与容量

前置条件：CMake、Ninja、`arm-none-eabi-gcc` 在 PATH 中。在 `FlightController/` 执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

默认姿态源为 MEKF；需要 DMP 回退时配置 `cmake --preset Debug -DFLIGHTCONTROLLER_ATTITUDE_SOURCE=DMP`（Release 同理）。已有构建目录的缓存会保留选择，切换时须显式指定姿态源。

产物为 `build/Debug/FlightController.elf`、`build/Release/FlightController.elf`。

### VS Code 烧录与运行

飞控文件夹的 `.vscode/settings.json` 仍将 CMake Tools 指向 `cube-cmake`；如果本机找不到该命令，CMake Tools 可能无法配置。飞控 `tasks.json` 中的配置和构建任务直接使用 `C:/Program Files/CMake/bin/cmake.exe`，并为进程提供 Ninja/ARM 工具链路径；这些本机绝对路径换电脑时需要调整。

- F5 调试：选择 **飞控 DAPLink 烧录并调试 (5MHz)** 或 **飞控 ST-Link 烧录并调试 (10MHz)**，启动时会先配置和构建，再下载固件并停在 `main`；继续运行后才执行初始化和显示。
- 只烧录并运行：命令面板执行 `Tasks: Run Task`，选择 **飞控烧录 (DAPLink 5MHz)** 或 **飞控烧录 (ST-Link 10MHz)**，会配置、构建、校验、复位运行。
- 两种入口只使用 `build/Debug/FlightController.elf`。通用 CMake“运行”按钮不能代替下载到 MCU 的烧录步骤。
- OpenOCD 配置包含通过 SWD 的 `SYSRESETREQ` 软件复位，适用于未接 nRESET 线的探针；DAPLink 默认 5MHz，ST-Link 默认 10MHz，不稳定时在对应 cfg 中降低 `adapter speed`。

飞控正常布局包含 `o:` 油门、`1:`～`4:` 电机 PWM、`B:` 电压、pitch/roll/yaw 零点和 P/R/Y 姿态等字段。出现 `Sig`、`LH/LV/RH/RV` 时，先核对是否烧入了 `RemoteControl.elf`，不要据此修改飞控显示布局。

Debug 使用 `-Os -g3`，Release 使用 `-Os -g0`，两者启用 LTO。Debug 保留调试信息，但优化可能影响单步和局部变量观察。

历史容量记录（2026-11-07，VOFA 适配版本，两种构建一致，并非当前 MEKF 构建）：

| 项目 | 已用 | 容量 | 剩余 |
| --- | --- | --- | --- |
| Flash | 60,220 B | 65,536 B | 5,316 B |
| 静态 RAM（data+bss） | 5,552 B | 20,480 B | 14,928 B |

本次 MEKF Debug/Release 按最高 Flash 加载地址计均使用 64,564 B（剩余 972 B），RAM 含栈堆预留共 9,616 B；DMP 回退 Release 分别为 55,908 B 和 8,632 B。RAM 数值不代表已验证运行时栈水位。原字模数据存在初始化括号警告；本机 LTO 的 make 步骤还会出现 `Error 1 (ignored)` 提示，已观察到最终构建返回 0 并生成 ELF，不能描述成零警告构建。

## 已完成的软件验证

- 本次 MEKF Debug/Release 和 DMP 回退 Release 编译链接通过，容量按 Map 检查；运行时栈水位和实机时序仍待验证。
- `Bsp/BlueSerialTest/`：19 个调参映射、SET 兼容、非法数值、接收边界/错误恢复、FireWater 帧内容、发送忙时缓冲区保持。
- `Bsp/AttitudeTest/`：姿态超时锁机、重新解锁、失联输出门槛、计时回绕、FIFO 有界读取和异常样本拒绝，以及初始化/故障画面状态和锁机遥测编码。
- 本次实机已观察到静止启动进入 `IMU:OK`，启动时晃动显示故障，静止后断电重启恢复；拆桨/电机断电条件下，遥控器两页的 `LOCK`、`UNLOCK` 和断联后的 `UNKNOWN` 已确认。
- `Bsp/OLEDTest/`：异步页面、快照、BUSY 和错误恢复；业务显示帧与移植前已核对的图像一致。
- 串口测试已在主机 GCC 的 `-Os -flto -Wall -Wextra -Werror` 条件下通过。测试不接入固件 CMake 目标，编译方法见各测试源文件头部；姿态测试使用 `python Bsp/AttitudeTest/run.py`，脚本中的主机 GCC 路径需按本机环境调整。

## 待完成与实板验收

- 指南要求的 GPIO 九脉冲 I2C 通用总线恢复尚未实现；当前仅有 OLED BERR 后的外设重初始化，不能替代总线解锁或 MPU 总线自愈。
- `.ioc` 仍设置 `ProjectManager.DeletePrevious=true`，与指南建议不同。重新生成前应备份并检查此选项；实际 CubeMX 再生成的保留效果尚未验证。
- 尚未完成真实 UART/蓝牙链路与 VOFA 控件绑定验证，也未完成无桨实机的姿态采样频率、中断负载、传感器断连和恢复时序验证。
- 拆桨后逐项验证四路 PWM、LED/OLED、电池电压、姿态方向、无线控制和遥测、KEY7/KEY8、500ms 失联停机、姿态超时停机及恢复后重新解锁，再评估后续飞行测试。

无桨实机验证尚未完成，当前不能宣称可飞行；软件检查不替代上板验证。
