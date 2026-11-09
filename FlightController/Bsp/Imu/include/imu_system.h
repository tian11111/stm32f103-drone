#ifndef IMU_SYSTEM_H
#define IMU_SYSTEM_H

#include "imu_calibration.h"
#include "imu_mekf.h"
#include "imu_stationary.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ImuMpu6050Config sensor; // MPU6050 地址、1 kHz 采样、DLPF 和量程配置
    ImuCalibration calibration; // 原始计数到 B 系物理量的确定性标定
    ImuStationaryConfig stationary; // 启动调零和运行期静止检测配置
    ImuMekfConfig mekf;      // 独立 MEKF 连续噪声、方差和更新门限
} ImuSystemConfig;

typedef struct {
    ImuMpu6050 sensor;       // 无 HAL 的 MPU6050 驱动状态
    ImuCalibration calibration; // 初始化时复制，运行中不隐式修改
    ImuRawSample latest_raw; // 最近一次完整总线读取的原始帧
    ImuSample latest_sample; // latest_raw 对应的预处理物理量
    ImuStationaryDetector stationary_detector; // 独立静止窗口统计状态
    ImuStationaryResult stationary_result; // 最近一次静止检测结果
    ImuMekf mekf;            // 名义 q_WB、唯一 b_g 和 6x6 协方差
    float gyro_only_quaternion_wb[4]; // 同源纯陀螺诊断姿态，顺序 w,x,y,z
    ImuOutput output;         // 启动调零完成后的处理结果
    ImuYawRateProvider yaw_rate_provider; // 可选外部 Body-Z 角速度提供者
    void *yaw_rate_provider_context; // 原样传给外部提供者
    uint32_t last_yaw_rate_source_sequence; // 最近已消费的外部观测序号
    uint64_t previous_timestamp_us; // 上一完整帧时间戳，单位 us
    bool raw_valid;          // latest_raw 是否至少成功发布过一次
    bool sample_valid;       // latest_sample 是否至少成功发布过一次
    bool output_valid;       // 启动调零完成并已发布处理结果
    bool yaw_rate_source_sequence_valid; // 是否已有外部观测序号
} ImuSystem;

void imu_system_default_config(ImuSystemConfig *config);

ImuStatus imu_system_init(ImuSystem *system,
                          const ImuBus *bus,
                          const ImuSystemConfig *config);

ImuStatus imu_system_process_data_ready(ImuSystem *system,
                                        uint64_t timestamp_us);

ImuStatus imu_system_start_data_ready(ImuSystem *system,
                                      uint64_t timestamp_us);

ImuStatus imu_system_finish_data_ready(ImuSystem *system);

void imu_system_set_yaw_rate_provider(ImuSystem *system,
                                      ImuYawRateProvider provider,
                                      void *context);

bool imu_system_get_latest_raw(const ImuSystem *system,
                               ImuRawSample *raw_sample);

bool imu_system_get_latest_sample(const ImuSystem *system,
                                  ImuSample *sample);

bool imu_system_get_stationary_result(const ImuSystem *system,
                                      ImuStationaryResult *result);

bool imu_system_get_output(const ImuSystem *system, ImuOutput *output);

#ifdef __cplusplus
}
#endif

#endif
