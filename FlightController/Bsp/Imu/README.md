# F103 MEKF 姿态模块

当前代码已完成软件集成和 Release 构建，**尚未完成无桨实机验证，不可据此判断具备飞行条件**。上电初始化期间必须让飞行器物理保持静止。源算法无法可靠区分恒速航向旋转和陀螺固定零偏，不能保证所有慢转都被拒绝。

## 文件和数据流

| 来源或职责 | 目标文件 |
|---|---|
| 源工程 `USER/imu/include/*.h` | `Bsp/Imu/include/` 的 7 个核心头文件 |
| 源工程 `USER/imu/src/{imu_math,imu_mpu6050,imu_calibration,imu_stationary,imu_mekf,imu_system}.c` | `Bsp/Imu/src/` 对应 6 个核心文件，数学实现保持原样 |
| F103 HAL 总线、时钟、量程和安装矩阵 | `Bsp/Imu/f103_port.h`、`f103_port.c` |
| 姿态源初始化、故障锁存和业务单位转换 | `Bsp/Attitude/attitude_source.h`、`attitude_mekf.c` |
| 唯一全局姿态发布、50 ms 有效性门和电机保护 | `Bsp/Data_Proc.c` |
| 任务调度和启动位置 | `Core/Src/main.c` |
| OLED/FireWater 的定点小数显示 | `Bsp/FixedFormat.h`、`FixedFormat.c` |

数据流：I2C2 原始帧 → 标定及传感器到机体系旋转 → 静止检测 → 六状态 MEKF → 局部快照 → `Attitude_Data_Proc()` 原子发布。TIM1 控制中断只运行既有控制及调度逻辑；同步 I2C 和滤波均在前台执行。

## 单位、坐标与配置

- 核心机体系为 X 向前、Y 向左、Z 向上；四元数为 Hamilton `[w,x,y,z]`，表示 Body→World。
- 核心比力单位为 m/s²，角速度为 rad/s；发布给现有 PID 的角度为 degree、角速度为 degree/s。水平静止时核心比力约为 `[0, 0, +9.80665]` m/s²。
- Roll/Pitch 为重力倾角；Yaw 是六轴相对航向，会漂移，不能当作长期绝对航向。不同姿态下的重力倾角不必然等同于原 DMP 的 ZYX 欧拉角。
- `f103_port.h` 中的 `IMU_ADDRESS_7BIT=0x68`、`IMU_SERVICE_PERIOD_MS=5`、`IMU_I2C_TIMEOUT_MS=5` 是当前板级配置；HAL 调用时才左移地址。MPU 使用 I2C2、100 kHz、±16 g、±2000 °/s、94 Hz DLPF、器件 1 kHz 输出。
- 默认 `sensor_to_body` 是单位矩阵，**并未证明板上实际轴向**。无桨确认安装方向后，在 `ImuF103_ConfigureCalibration()` 配置 `R_BS` 并做本机标定；没有加载其他器件的 0.991684 专属标定值。
- 静止检测使用 100 样本非重叠窗口，源算法要求连续 3 个静止窗口。5 ms 周期下采集这些窗口理想需约 1.5 s；传感器配置成功后还会先等待 2 s，另有初始化等待。实际周期及启动时间需无桨实测。
- 同一 I2C 突发读取 `INT_STATUS` 与 14 字节传感器帧；DATA_RDY 未置位时拒绝发布并锁存故障。[MPU-6050 寄存器手册](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)第 26–29 页说明了状态位、读取清除与传感器突发读取；本板实际清除时序仍需验证。DATA_RDY 不能证明传感器所有内部冻结都可被检测。

## 调用与安全状态

工程已在 `main.c` 中接好以下流程，移植到其他 HAL 工程时应保持相同先后顺序：

```c
/* 前台启动：I2C2 已初始化，TIM1 控制中断尚未启动。 */
AttitudeSource_Init();

/* 前台循环：由 TIM1 每 5 ms 置位的槽触发，不在中断中读 I2C。 */
if (Attitude_Proc_time == SlowDown) {
    Attitude_Proc_time = 1;
    Attitude_Data_Proc(); /* 读取、解算并只在新结果有效时发布 */
}
```

MEKF 启动时先等待 100 ms，再尝试传感器初始化。仅初始化中的总线错误会重试，最多 3 次（含首次），每次间隔 100 ms；身份或配置错误不会重试。重试仍失败则锁存故障，不发布姿态。成功配置后，先等待 `IMU_STARTUP_SETTLE_WAIT_MS`（暂定 2 s）再开始后续姿态采样；该候选时间能否消除冷启动误判仍需无桨实机验证。这些等待和尝试发生在电机 PWM 与控制定时器启动之前，不能替代 I2C 硬件故障处理。

2 s 等待期间尚未采样，无法检测这段时间内的运动；即使运动在采样前停止，也不会因此锁存。上电后应始终保持飞控静止，直到姿态显示正常并稳定。等待结束后的静止窗口、姿态有效性和故障锁存规则不变。

上电未完成静止初始化时姿态无效，不能解锁。启动静止窗口判为运动，或采样期间出现总线错误、无 DATA_RDY、时间间隔异常或解算异常时，故障保持锁存；本次前台调用即撤销有效位，下一次控制周期锁机并清油门。不能运行中重标定或自动复位故障；应静止后重新上电。50 ms 新鲜性保护仍由业务层维护。

启动期间逐个比较合格静止窗口的平均比力方向与首窗方向，偏移超过 `IMU_STARTUP_DIRECTION_LIMIT_RAD`（暂定 1°）即锁存故障；第三窗口刚产生首帧时也先检查再发布。无效或零比力向量安全失败。该门限尚未经实机验证，低于门限的极慢运动以及恒速航向转动仍可能通过，启动时仍须物理保持静止。任何有效输出若不在正立半球也立即锁存故障，启动和运行阶段均适用。控制中断在检查姿态新鲜度时同时检查锁存故障，故障置位后即使前台尚未撤销有效位也会锁机清油门及 PWM；不在 I2C 或滤波期间关闭中断。

## 构建和测试

```powershell
$env:PATH = 'C:\Users\Neco\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin;' + $env:PATH
cmake -S . -B build/task-mekf-final -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DFLIGHTCONTROLLER_ATTITUDE_SOURCE=MEKF
cmake --build build/task-mekf-final -- -j1
cmake -S . -B build/task-dmp-final -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DFLIGHTCONTROLLER_ATTITUDE_SOURCE=DMP
cmake --build build/task-dmp-final -- -j1
python Bsp/AttitudeTest/run.py
# 单独复现尚未通过的恒速航向慢转验收；当前会输出 FAIL 并返回非零。
./build/task-attitude-host/mekf_test.exe slow-yaw
```

以上命令从 `FlightController` 目录执行。CMake 默认 `MEKF`；选择 `DMP` 时不编译 MEKF 核心，反之不编译 DMP 驱动。

当前 GNU ARM 14.3.1 的 MEKF Debug/Release LTO 构建，按最高 Flash 加载地址计分别为 62,720 B 和 62,624 B，均小于 65,536 B；RAM 含 512 B 堆和 4,096 B 最小栈预留共 9,608 B。DMP 回退 Release 构建占 Flash 55,892 B、RAM 8,632 B。此前记录的静态栈帧大小不代表本版运行时栈高水位，仍需实机测量。

F103 无硬件 FPU，核心使用软件单精度、`math.h` 和 `string.h`，没有 RTOS 或动态分配。整个旧应用仍链接 newlib 的分配器及 `strtof` 路径，不能据此声称整机固件完全不使用动态内存。

`OLED_Printf` 和 `BlueSerial_Printf` 不支持浮点格式。生产路径改用 0–3 位小数的 `FixedFormat`；非有限值或绝对值不小于 2^32 时，OLED 字段显示 `ERR`，FireWater 放弃当前帧。

## 交付前未验证项

- 传感器轴向、量程、电气连接、六面和陀螺标定。
- DATA_RDY 清除时序、有效采样率、MEKF 最坏单帧耗时、4 ms 控制中断时序和运行时栈高水位。
- OLED 实际显示、FireWater、NRF、PID、混控、上锁及失联保护的无桨台架回归。
- 来源核心的许可授权文件；源库未发现明确许可证。
