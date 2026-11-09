#ifndef IMU_MEKF_H
#define IMU_MEKF_H

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_MEKF_ERROR_STATE_SIZE (6u)

typedef struct {
    float gyro_noise_density_rad_s_sqrt_hz;       // 陀螺白噪声密度，rad/s/sqrt(Hz)
    float gyro_bias_random_walk_rad_s2_sqrt_hz;   // 偏置随机游走密度，rad/s^2/sqrt(Hz)
    float initial_tilt_variance_rad2;              // 初始 Roll/Pitch 小角误差方差，rad^2
    float initial_yaw_variance_rad2;               // 初始 Yaw 小角误差方差，rad^2
    float initial_bias_variance_rad2_s2;           // 初始三轴偏置误差方差，(rad/s)^2
    float specific_force_direction_variance_rad2; // 单次方向观测方差，rad^2
    float zaru_angular_rate_variance_rad2_s2;      // 静止窗口角速度观测方差，(rad/s)^2
    float specific_force_norm_tolerance_mps2;      // 比力模长相对 g 的更新门限，m/s^2
    float minimum_dt_s;                            // 最小合法预测周期，s
    float maximum_dt_s;                            // 最大合法预测周期，s
    float maximum_attitude_correction_rad;         // 单次姿态注入模长上限，rad
    float maximum_bias_correction_rad_s;           // 单次偏置注入模长上限，rad/s
} ImuMekfConfig;

typedef struct {
    float quaternion_wb[4]; // Hamilton Body->World 四元数，顺序 w,x,y,z
    float gyro_bias_body_rad_s[3]; // MEKF 唯一运行时陀螺偏置状态
    float covariance[IMU_MEKF_ERROR_STATE_SIZE][IMU_MEKF_ERROR_STATE_SIZE];
    ImuMekfConfig config; // 初始化时复制，运行中不隐式修改
    uint32_t prediction_count; // 成功预测次数
    uint32_t specific_force_update_count; // 成功比力方向更新次数
    uint32_t rejected_specific_force_count; // 因模长或数值门限拒绝次数
    uint32_t zaru_update_count; // 成功零角速度更新次数
    uint32_t rejected_zaru_count; // 因注入门限拒绝的零角速度更新次数
    uint32_t body_yaw_rate_update_count; // 成功融合的外部 Body-Z 角速度次数
    uint32_t rejected_body_yaw_rate_count; // 非法或过大外部修正拒绝次数
    bool initialized; // 名义姿态、偏置和协方差是否有效
} ImuMekf;

void imu_mekf_default_config(ImuMekfConfig *config);

ImuStatus imu_mekf_init(ImuMekf *mekf,
                        const ImuMekfConfig *config,
                        const float initial_specific_force_body_mps2[3],
                        const float initial_gyro_bias_body_rad_s[3]);

ImuStatus imu_mekf_predict(ImuMekf *mekf,
                           const float gyro_pre_body_rad_s[3],
                           float dt_s);

ImuStatus imu_mekf_update_specific_force_direction(
    ImuMekf *mekf,
    const float specific_force_body_mps2[3]);

ImuStatus imu_mekf_update_zero_angular_rate(
    ImuMekf *mekf,
    const float gyro_pre_mean_body_rad_s[3]);

ImuStatus imu_mekf_update_body_yaw_rate(
    ImuMekf *mekf,
    const float gyro_pre_body_rad_s[3],
    float body_yaw_rate_rad_s,
    float variance_rad2_s2);

bool imu_mekf_get_euler_rad(const ImuMekf *mekf,
                            float *roll_rad,
                            float *pitch_rad,
                            float *yaw_rad);

bool imu_mekf_get_predicted_specific_force_direction_body(
    const ImuMekf *mekf,
    float direction_body[3]);

bool imu_mekf_get_gravity_tilt_rad(const ImuMekf *mekf,
                                   float *gravity_tilt_roll_rad,
                                   float *gravity_tilt_pitch_rad,
                                   bool *upright_hemisphere);

#ifdef __cplusplus
}
#endif

#endif
