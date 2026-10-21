# FlightController（飞控）

CubeMX 工程待创建。配置要点见 `hal/` 上一级的 `飞控-CubeMX配置与移植指南.md`。

与遥控器的关键差异：

- NRF24L01 引脚为 PA4(CSN) / PA5(SCK) / PB6(MOSI) / PB7(MISO)，**跨 GPIOA/GPIOB，凑不成任何硬件 SPI 组** → 只能软件模拟 SPI
- NRF 有 IRQ 引脚（PB15），遥控器没有
- MPU6050 走硬件 I2C2（PB10/PB11）
- OLED 走硬件 I2C1（PB8/PB9，重映射），与遥控器相同
- 需要 TIM 输出 PWM 驱动电机
