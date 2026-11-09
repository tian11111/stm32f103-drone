#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef ImuStatus (*ImuBusReadFn)(void *context,
                                  uint8_t device_address_7bit,
                                  uint8_t register_address,
                                  uint8_t *data,
                                  uint16_t length);

typedef ImuStatus (*ImuBusWriteFn)(void *context,
                                   uint8_t device_address_7bit,
                                   uint8_t register_address,
                                   const uint8_t *data,
                                   uint16_t length);

typedef ImuStatus (*ImuBusReadAsyncStartFn)(void *context,
                                            uint8_t device_address_7bit,
                                            uint8_t register_address,
                                            uint8_t *data,
                                            uint16_t length);

typedef ImuStatus (*ImuBusReadAsyncPollFn)(void *context);

typedef void (*ImuDelayMsFn)(void *context, uint32_t delay_ms);

typedef struct {
    void *context;       // 由适配层持有的总线实例上下文
    ImuBusReadFn read;   // 同步寄存器读取；返回前必须完成或报告失败
    ImuBusWriteFn write; // 同步寄存器写入；返回前必须完成或报告失败
    ImuDelayMsFn delay_ms; // 仅在传感器初始化复位等待期间调用
    ImuBusReadAsyncStartFn read_async_start; // 可选异步读取启动；通常映射 DMA
    ImuBusReadAsyncPollFn read_async_poll; // 可选异步完成查询；忙时返回 NO_DATA
} ImuBus;

typedef enum {
    IMU_MPU6050_GYRO_250_DPS = 0,
    IMU_MPU6050_GYRO_500_DPS,
    IMU_MPU6050_GYRO_1000_DPS,
    IMU_MPU6050_GYRO_2000_DPS
} ImuMpu6050GyroRange;

typedef enum {
    IMU_MPU6050_ACCEL_2_G = 0,
    IMU_MPU6050_ACCEL_4_G,
    IMU_MPU6050_ACCEL_8_G,
    IMU_MPU6050_ACCEL_16_G
} ImuMpu6050AccelRange;

typedef enum {
    IMU_MPU6050_DLPF_260_HZ = 0,
    IMU_MPU6050_DLPF_184_HZ = 1,
    IMU_MPU6050_DLPF_94_HZ = 2,
    IMU_MPU6050_DLPF_44_HZ = 3,
    IMU_MPU6050_DLPF_21_HZ = 4,
    IMU_MPU6050_DLPF_10_HZ = 5,
    IMU_MPU6050_DLPF_5_HZ = 6
} ImuMpu6050Dlpf;

/**
 * @brief 驱动已确认的六轴器件型号。
 * @note 型号必须由 WHO_AM_I 精确识别；UNKNOWN 不允许进入采样状态。
 */
typedef enum {
    IMU_MPU_MODEL_UNKNOWN = 0,
    IMU_MPU_MODEL_6050,
    IMU_MPU_MODEL_6500
} ImuMpuModel;

typedef struct {
    uint8_t address_7bit;                 // 7 位 I2C 地址，只允许 0x68/0x69
    uint16_t sample_rate_hz;              // 目标输出采样率，单位 Hz
    ImuMpu6050GyroRange gyro_range;       // 陀螺满量程寄存器枚举
    ImuMpu6050AccelRange accel_range;     // 加速度计满量程寄存器枚举
    ImuMpu6050Dlpf dlpf;                  // MPU6050 CONFIG.DLPF_CFG
    bool data_ready_interrupt;            // 是否使能 DATA_RDY 中断
} ImuMpu6050Config;

typedef struct {
    ImuBus bus;                       // 初始化时复制的抽象总线接口
    ImuMpu6050Config config;          // 实际写入设备的配置快照
    uint32_t sample_sequence;         // 成功原始帧累计序号
    float accel_lsb_per_g;            // 加速度灵敏度，单位 LSB/g
    float gyro_lsb_per_dps;           // 陀螺灵敏度，单位 LSB/(deg/s)
    float temperature_lsb_per_c;      // 芯片温度灵敏度，单位 LSB/摄氏度
    float temperature_offset_c;       // TEMP_OUT=0 时对应温度，单位摄氏度
    uint8_t async_bytes[14];          // 异步总线直接写入的完整六轴寄存器帧
    uint64_t async_timestamp_us;      // DATA_RDY 对应时间戳，单位 us
    uint8_t last_who_am_i;            // 最近一次成功读取的 WHO_AM_I 原值
    ImuMpuModel model;                 // 由 WHO_AM_I 确认的实际器件型号
    bool who_am_i_valid;              // false 表示 WHO_AM_I 总线读取未成功
    bool async_read_pending;          // true 表示异步总线事务尚未收尾
    bool initialized;                 // true 表示全部寄存器配置成功
} ImuMpu6050;

typedef struct {
    uint8_t power_management_1; // PWR_MGMT_1 原值，睡眠位应为 0
    uint8_t power_management_2; // PWR_MGMT_2 原值，六轴待机位应为 0
    uint8_t gyro_config;        // GYRO_CONFIG 原值，用于确认量程写入
    int16_t gyro_raw[3];        // 从 GYRO_XOUT_H 单独同步读取的三轴计数
} ImuMpu6050Diagnostics;

void imu_mpu6050_default_config(ImuMpu6050Config *config);

ImuStatus imu_mpu6050_init(ImuMpu6050 *device,
                           const ImuBus *bus,
                           const ImuMpu6050Config *config);

ImuStatus imu_mpu6050_read_who_am_i(const ImuMpu6050 *device,
                                    uint8_t *who_am_i);

ImuStatus imu_mpu6050_read_raw(ImuMpu6050 *device,
                               uint64_t timestamp_us,
                               ImuRawSample *sample);

ImuStatus imu_mpu6050_start_raw_read(ImuMpu6050 *device,
                                     uint64_t timestamp_us);

ImuStatus imu_mpu6050_finish_raw_read(ImuMpu6050 *device,
                                      ImuRawSample *sample);

ImuStatus imu_mpu6050_read_diagnostics(const ImuMpu6050 *device,
                                       ImuMpu6050Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
