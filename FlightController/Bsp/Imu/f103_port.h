#ifndef F103_PORT_H
#define F103_PORT_H

#include "imu_system.h"

/*
 * 当前板级配置。传感器安装轴向尚未通过实机台架确认，因此显式使用
 * 单位 R_BS；标定六面与安装方向后只修改 ImuF103_ConfigureCalibration()。
 */
#define IMU_ADDRESS_7BIT 0x68u
#define IMU_SERVICE_PERIOD_MS 5u
#define IMU_I2C_TIMEOUT_MS 5u
/* 上电后首次访问前及总线错误重试间的等待，单位 ms；次数包含首次尝试。 */
#define IMU_STARTUP_READY_WAIT_MS 100u
#define IMU_STARTUP_INIT_MAX_ATTEMPTS 3u
#define IMU_STARTUP_RETRY_DELAY_MS 100u
/* 成功配置后等待传感器稳定的候选时间，须无桨实机验证。 */
#define IMU_STARTUP_SETTLE_WAIT_MS 2000u
#define IMU_RAD_TO_DEG 57.29577951308232f

/*
 * MEKF 的静止窗口按采样帧计数。目标前台有效采样约 200 Hz，
 * 100 点窗口仍对应约 0.5 s，避免源代码注释中的 1 kHz 假设误用。
 */
#define IMU_STATIONARY_WINDOW_SAMPLES 100u
/* 启动窗口方向偏移上限（弧度），暂定 1°，须无桨实机验证。 */
#define IMU_STARTUP_DIRECTION_LIMIT_RAD 0.01745329252f

typedef struct {
    uint32_t last_tick;
    uint64_t elapsed_ms;
} ImuClock;

uint64_t ImuClock_Update(ImuClock *clock, uint32_t tick);
ImuBus ImuF103_Bus(void);
void ImuF103_Config(ImuSystemConfig *config);
void ImuF103_ConfigureCalibration(ImuCalibration *calibration);

#endif
