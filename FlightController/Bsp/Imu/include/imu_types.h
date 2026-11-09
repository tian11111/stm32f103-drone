#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 所有核心接口共用的返回状态；正值表示非故障状态，负值表示调用失败。 */
typedef enum {
    IMU_STATUS_OK = 0,                 // 操作成功且输出有效
    IMU_STATUS_NO_DATA = 1,            // 本次没有新数据，不属于硬件故障
    IMU_STATUS_INVALID_ARGUMENT = -1,  // 空指针、非法长度或非法调用顺序
    IMU_STATUS_BUS_ERROR = -2,         // 底层总线读写失败
    IMU_STATUS_DEVICE_NOT_FOUND = -3,  // WHO_AM_I 与 MPU6050 不匹配
    IMU_STATUS_CONFIG_ERROR = -4,      // 配置值、矩阵或标定参数非法
    IMU_STATUS_TIME_ERROR = -5,        // 时间戳不递增或 dt 超出允许范围
    IMU_STATUS_NUMERIC_ERROR = -6      // 出现 NaN、Inf 或不可归一化状态
} ImuStatus;

/* 样本状态可按位组合；VALID 只说明本帧读取完整，不掩盖其他告警位。 */
enum {
    IMU_SAMPLE_STATUS_VALID = (1u << 0),
    IMU_SAMPLE_STATUS_TEMP_CLAMPED = (1u << 1),
    IMU_SAMPLE_STATUS_ACCEL_SATURATED = (1u << 2),
    IMU_SAMPLE_STATUS_GYRO_SATURATED = (1u << 3)
};

typedef struct {
    uint32_t sequence;       // 每次成功读取完整 14 字节数据后递增
    uint64_t timestamp_us;   // 数据就绪或读取时刻，单位 us，必须单调递增
    int16_t accel_raw[3];    // MPU6050 S 系三轴加速度原始补码计数
    int16_t gyro_raw[3];     // MPU6050 S 系三轴角速度原始补码计数
    int16_t temperature_raw; // MPU6050 温度原始补码计数
    uint32_t status;         // IMU_SAMPLE_STATUS_* 按位组合
} ImuRawSample;

typedef struct {
    uint32_t sequence;                    // 对应原始帧序号
    uint64_t timestamp_us;                // 对应原始帧时间戳，单位 us
    float dt_s;                           // 与上一有效帧间隔，单位 s
    float specific_force_body_mps2[3];    // B 系校正后比力，单位 m/s^2
    float gyro_pre_body_rad_s[3];         // B 系预处理角速度，单位 rad/s，尚未减 b_g
    float temperature_c;                  // 芯片实测温度，单位摄氏度，未钳位
    uint32_t status;                      // 从原始帧继承并追加预处理告警
} ImuSample;

enum {
    IMU_OUTPUT_STATE_CALIBRATING = (1u << 0),
    IMU_OUTPUT_STATE_READY = (1u << 1),
    IMU_OUTPUT_STATE_STATIC = (1u << 2),
    IMU_OUTPUT_STATE_ATTITUDE_VALID = (1u << 3)
};

typedef struct {
    uint32_t sequence;                    // 对应最近一次成功处理的样本序号
    uint64_t timestamp_us;                // 对应样本时间戳，单位 us
    float quaternion_wb[4];               // Hamilton Body->World，顺序 w,x,y,z
    float roll_rad;                       // 诊断用 ZYX Roll，+-90 deg Pitch 附近奇异
    float pitch_rad;                      // 诊断用 ZYX Pitch，单位 rad
    float yaw_rad;                        // 诊断用 ZYX Yaw，奇异区与 Roll 耦合
    float mekf_fused_yaw_rad;             // 推荐相对航向，倾斜解耦合，单位 rad
    float gyro_only_fused_yaw_rad;        // 同源纯陀螺诊断姿态的 fused yaw，单位 rad
    float gyro_only_pitch_rad;            // 同源纯陀螺诊断姿态的 ZYX Pitch，单位 rad
    float gravity_tilt_roll_rad;          // 推荐业务 Roll：asin(f_pred_B.y)，无 Yaw 耦合
    float gravity_tilt_pitch_rad;         // 推荐业务 Pitch：asin(-f_pred_B.x)，无 Yaw 耦合
    float predicted_specific_force_direction_body[3]; // R_BW*[0,0,1]，无量纲
    float attitude_error_variance_rad2[3]; // delta_theta x/y/z 误差方差
    float initial_roll_rad;               // 静止平均比力得到的初始 Roll，单位 rad
    float initial_pitch_rad;              // 静止平均比力得到的初始 Pitch，单位 rad
    float accel_roll_rad;                 // 当前比力方向得到的诊断 Roll，单位 rad
    float accel_pitch_rad;                // 当前比力方向得到的诊断 Pitch，单位 rad
    float gyro_bias_body_rad_s[3];        // MEKF 唯一运行时 b_g，单位 rad/s
    float gyro_corrected_body_rad_s[3];   // omega_pre-b_g，单位 rad/s
    float temperature_c;                  // 当前芯片温度，单位摄氏度
    uint32_t specific_force_update_count; // 已成功注入的静止比力方向观测总数
    uint32_t zaru_update_count;           // 已成功注入的静止零角速度观测总数
    uint32_t state_flags;                 // IMU_OUTPUT_STATE_* 按位组合
    bool upright_hemisphere;              // true 表示 f_pred_B.z>=0，机体未倒置
} ImuOutput;

/* 外部角速度的坐标语义；差速轮默认提供 BODY_Z。 */
typedef enum {
    IMU_YAW_RATE_BODY_Z = 0,
    IMU_YAW_RATE_WORLD_HEADING = 1
} ImuYawRateSemantic;

typedef struct {
    uint64_t timestamp_us;          // 外部观测对应时刻，单位 us
    float rate_rad_s;               // 指定 semantic 下的角速度，单位 rad/s
    float variance_rad2_s2;         // 观测方差，单位 (rad/s)^2，必须大于零
    uint32_t source_sequence;       // 提供者帧序号，用于拒绝重复融合
    ImuYawRateSemantic semantic;    // BODY_Z 或 WORLD_HEADING_RATE
    bool valid;                     // false 时其余观测字段不得参与融合
    bool stationary_valid;          // true 时 vehicle_stationary 字段才有效
    bool vehicle_stationary;        // 外部系统确认车辆完全静止
} ImuYawRateObservation;

typedef bool (*ImuYawRateProvider)(void *context,
                                   uint64_t imu_timestamp_us,
                                   ImuYawRateObservation *observation);

#ifdef __cplusplus
}
#endif

#endif
