#ifndef IMU_STATIONARY_H
#define IMU_STATIONARY_H

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_MOTION_UNKNOWN = 0,
    IMU_MOTION_MOVING,
    IMU_MOTION_ANGULAR_STATIC
} ImuMotionState;

typedef struct {
    uint16_t window_samples;                       // 单个非重叠统计窗口样本数
    uint16_t enter_static_windows;                 // 进入静止所需连续合格窗口数
    uint16_t exit_static_windows;                  // 退出静止所需连续不合格窗口数
    float gyro_pre_mean_max_rad_s;                 // 启动期 ω_pre 均值模长上限，rad/s
    float gyro_pre_std_max_rad_s;                  // ω_pre 任一轴样本标准差上限，rad/s
    float gyro_pre_peak_to_peak_max_rad_s;         // ω_pre 任一轴峰峰值上限，rad/s
    float gyro_corrected_mean_max_rad_s;            // 运行期校正角速度均值模长上限，rad/s
    float accel_norm_tolerance_mps2;               // 比力模长相对 g 的允许偏差，m/s^2
    float accel_norm_std_max_mps2;                 // 比力模长样本标准差上限，m/s^2
} ImuStationaryConfig;

typedef struct {
    uint32_t sequence;                             // 完整窗口最后一帧序号
    uint32_t sample_count;                         // 本次已完成窗口的样本数
    float gyro_pre_mean_rad_s[3];                  // 三轴 ω_pre 窗口均值，rad/s
    float gyro_pre_std_rad_s[3];                   // 三轴 ω_pre 样本标准差，rad/s
    float gyro_pre_peak_to_peak_rad_s[3];          // 三轴 ω_pre 峰峰值，rad/s
    float gyro_corrected_mean_rad_s[3];            // 三轴 ω_pre-b_g 窗口均值，rad/s
    float specific_force_mean_mps2[3];             // 三轴比力窗口均值，m/s^2
    float accel_norm_mean_mps2;                    // 比力模长窗口均值，m/s^2
    float accel_norm_std_mps2;                     // 比力模长样本标准差，m/s^2
    bool candidate_static;                         // 当前完整窗口是否通过全部门限
    bool window_ready;                             // 仅在本次完成非重叠窗口时为 true
    ImuMotionState state;                          // 经连续窗口滞回后的运动状态
} ImuStationaryResult;

typedef struct {
    ImuStationaryConfig config;       // 初始化时复制，运行中不可直接修改
    uint32_t count;                   // 当前未完成窗口的样本数
    float gyro_pre_mean[3];           // Welford 三轴在线均值
    float gyro_pre_m2[3];             // Welford 三轴离均差平方和
    float gyro_pre_min[3];            // 当前窗口三轴最小值
    float gyro_pre_max[3];            // 当前窗口三轴最大值
    float gyro_corrected_mean[3];     // 当前窗口校正角速度均值
    float specific_force_mean[3];     // 当前窗口三轴比力在线均值
    float accel_norm_mean;            // Welford 比力模长在线均值
    float accel_norm_m2;              // Welford 比力模长离均差平方和
    uint16_t static_window_count;     // 连续合格窗口数，饱和累计
    uint16_t moving_window_count;     // 连续不合格窗口数，饱和累计
    ImuMotionState state;             // 当前滞回状态
} ImuStationaryDetector;

void imu_stationary_default_config(ImuStationaryConfig *config);

ImuStatus imu_stationary_init(ImuStationaryDetector *detector,
                              const ImuStationaryConfig *config);

ImuStatus imu_stationary_update(ImuStationaryDetector *detector,
                                const ImuSample *sample,
                                const float gyro_bias_body_rad_s[3],
                                bool gyro_bias_valid,
                                bool startup_mode,
                                ImuStationaryResult *result);

#ifdef __cplusplus
}
#endif

#endif
