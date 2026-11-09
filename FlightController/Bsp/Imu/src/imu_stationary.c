#include "imu_stationary.h"

#include "imu_math_internal.h"

#include <math.h>
#include <string.h>

#define IMU_STANDARD_GRAVITY_MPS2 (9.80665f)

/**
 * @brief 验证窗口长度、滞回计数和全部正值门限。
 * @param config 待验证静止检测配置。
 * @return true 配置可用于初始化；false 存在零窗口或非正门限。
 */
static bool imu_stationary_config_is_valid(const ImuStationaryConfig *config)
{
    return config != 0 &&
           config->window_samples >= 2u &&
           config->enter_static_windows > 0u &&
           config->exit_static_windows > 0u &&
           config->gyro_pre_mean_max_rad_s > 0.0f &&
           config->gyro_pre_std_max_rad_s > 0.0f &&
           config->gyro_pre_peak_to_peak_max_rad_s > 0.0f &&
           config->gyro_corrected_mean_max_rad_s > 0.0f &&
           config->accel_norm_tolerance_mps2 > 0.0f &&
           config->accel_norm_std_max_mps2 > 0.0f;
}

/**
 * @brief 清空当前非重叠统计窗口，不改变滞回计数和运动状态。
 * @param detector 静止检测实例。
 * @note 每发布一个完整窗口后调用，确保后续 ZARU 不重复使用同一批样本。
 */
static void imu_stationary_reset_window(ImuStationaryDetector *detector)
{
    uint32_t axis;

    detector->count = 0u;
    detector->accel_norm_mean = 0.0f;
    detector->accel_norm_m2 = 0.0f;
    for (axis = 0u; axis < 3u; ++axis) {
        detector->gyro_pre_mean[axis] = 0.0f;
        detector->gyro_pre_m2[axis] = 0.0f;
        detector->gyro_pre_min[axis] = 0.0f;
        detector->gyro_pre_max[axis] = 0.0f;
        detector->gyro_corrected_mean[axis] = 0.0f;
        detector->specific_force_mean[axis] = 0.0f;
    }
}

/**
 * @brief 计算三维向量模长。
 * @param vector 输入三维向量，调用前必须完成有限性检查。
 * @return 欧氏模长，单位继承输入向量。
 */
static float imu_stationary_vec3_magnitude(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] +
                 vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

/**
 * @brief 填充适用于 1 kHz、200 点窗口的保守静止检测门限。
 * @param config 输出配置。
 * @note 默认按 1 kHz 下 200 点非重叠窗口设置，但所有门限仍需用目标
 *       MPU6050、DLPF 和机械安装的静止/慢转数据重新标定。
 */
void imu_stationary_default_config(ImuStationaryConfig *config)
{
    if (config == 0) {
        return;
    }
    config->window_samples = 200u;
    config->enter_static_windows = 3u;
    config->exit_static_windows = 1u;
    config->gyro_pre_mean_max_rad_s = 0.35f;
    /* 覆盖典型静止噪声，同时拒绝落入单个窗口的缓慢起转斜坡。 */
    config->gyro_pre_std_max_rad_s = 0.003f;
    config->gyro_pre_peak_to_peak_max_rad_s = 0.012f;
    /* 严格限制运行期均值，避免把缓慢真实旋转吸收到 b_g。 */
    config->gyro_corrected_mean_max_rad_s = 0.0015f;
    /*
     * 通用启动不能依赖上一颗器件的六面标定。允许约 +/-15% 的未标定比例
     * 误差；恒定运动仍由模长标准差、三轴陀螺均值/标准差/峰峰值联合拒绝。
     * 用户装载本机六面标定后可按实测噪声把该值收紧到 0.8 m/s^2 左右。
     */
    config->accel_norm_tolerance_mps2 = 1.5f;
    config->accel_norm_std_max_mps2 = 0.12f;
}

/**
 * @brief 初始化非重叠窗口统计和静止/运动滞回状态。
 * @param detector 调用者持有的检测实例。
 * @param config 窗口长度、连续窗口数和物理量门限。
 * @return IMU_STATUS_OK 初始化成功；参数非法返回 IMU_STATUS_INVALID_ARGUMENT。
 * @note 初始状态为 IMU_MOTION_UNKNOWN，不会在没有完整窗口时宣称静止。
 */
ImuStatus imu_stationary_init(ImuStationaryDetector *detector,
                              const ImuStationaryConfig *config)
{
    if (detector == 0 || !imu_stationary_config_is_valid(config)) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    memset(detector, 0, sizeof(*detector));
    detector->config = *config;
    detector->state = IMU_MOTION_UNKNOWN;
    imu_stationary_reset_window(detector);
    return IMU_STATUS_OK;
}

/**
 * @brief 累加一帧预处理 IMU 数据，并在窗口结束时更新静止滞回状态。
 * @param detector 已初始化检测实例。
 * @param sample 当前有效预处理帧；比力单位 m/s^2，角速度单位 rad/s。
 * @param gyro_bias_body_rad_s 当前 MEKF 三轴 b_g，单位 rad/s；仅 bias_valid 时读取。
 * @param gyro_bias_valid true 表示 b_g 可用于计算运行期校正角速度均值。
 * @param startup_mode true 表示启动调零阶段，禁止依赖尚不可信的校正角速度。
 * @param result 本次结果；window_ready=false 时其窗口统计字段不得用于 ZARU。
 * @return IMU_STATUS_OK 表示样本已接收；参数或数值无效时返回明确错误。
 * @note 函数每 window_samples 帧只发布一次非重叠窗口。启动阶段依据
 *       omega_pre 的均值/方差/峰峰值与比力稳定性；运行阶段才追加
 *       omega_pre-b_g 均值门限，避免偏置未收敛导致静止检测死锁。
 */
ImuStatus imu_stationary_update(ImuStationaryDetector *detector,
                                const ImuSample *sample,
                                const float gyro_bias_body_rad_s[3],
                                bool gyro_bias_valid,
                                bool startup_mode,
                                ImuStationaryResult *result)
{
    float accel_norm;
    float corrected[3];
    float inverse_count;
    float delta;
    float delta_after_mean;
    float gyro_pre_mean_norm;
    float gyro_corrected_mean_norm;
    bool candidate;
    uint32_t axis;

    if (detector == 0 || sample == 0 || result == 0 ||
        (sample->status & IMU_SAMPLE_STATUS_VALID) == 0u ||
        !imu_math_vec3_is_finite(sample->specific_force_body_mps2) ||
        !imu_math_vec3_is_finite(sample->gyro_pre_body_rad_s) ||
        (gyro_bias_valid && !imu_math_vec3_is_finite(gyro_bias_body_rad_s))) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));
    result->state = detector->state;
    accel_norm = imu_stationary_vec3_magnitude(
        sample->specific_force_body_mps2);
    if (!imu_math_is_finite(accel_norm)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }

    /* Welford 在线统计避免 sum(x*x)-mean^2 在 float 下发生严重相消。 */
    ++detector->count;
    inverse_count = 1.0f / (float)detector->count;
    delta = accel_norm - detector->accel_norm_mean;
    detector->accel_norm_mean += delta * inverse_count;
    delta_after_mean = accel_norm - detector->accel_norm_mean;
    detector->accel_norm_m2 += delta * delta_after_mean;

    for (axis = 0u; axis < 3u; ++axis) {
        corrected[axis] = sample->gyro_pre_body_rad_s[axis] -
                          (gyro_bias_valid ? gyro_bias_body_rad_s[axis] : 0.0f);
        delta = sample->gyro_pre_body_rad_s[axis] -
                detector->gyro_pre_mean[axis];
        detector->gyro_pre_mean[axis] += delta * inverse_count;
        delta_after_mean = sample->gyro_pre_body_rad_s[axis] -
                           detector->gyro_pre_mean[axis];
        detector->gyro_pre_m2[axis] += delta * delta_after_mean;
        detector->gyro_corrected_mean[axis] +=
            (corrected[axis] - detector->gyro_corrected_mean[axis]) *
            inverse_count;
        detector->specific_force_mean[axis] +=
            (sample->specific_force_body_mps2[axis] -
             detector->specific_force_mean[axis]) * inverse_count;
        if (detector->count == 1u) {
            detector->gyro_pre_min[axis] = sample->gyro_pre_body_rad_s[axis];
            detector->gyro_pre_max[axis] = sample->gyro_pre_body_rad_s[axis];
        } else {
            if (sample->gyro_pre_body_rad_s[axis] <
                detector->gyro_pre_min[axis]) {
                detector->gyro_pre_min[axis] =
                    sample->gyro_pre_body_rad_s[axis];
            }
            if (sample->gyro_pre_body_rad_s[axis] >
                detector->gyro_pre_max[axis]) {
                detector->gyro_pre_max[axis] =
                    sample->gyro_pre_body_rad_s[axis];
            }
        }
    }

    if (detector->count < detector->config.window_samples) {
        return IMU_STATUS_OK;
    }

    result->sequence = sample->sequence;
    result->sample_count = detector->count;
    result->accel_norm_mean_mps2 = detector->accel_norm_mean;
    result->accel_norm_std_mps2 =
        sqrtf(detector->accel_norm_m2 / (float)(detector->count - 1u));
    for (axis = 0u; axis < 3u; ++axis) {
        result->gyro_pre_mean_rad_s[axis] = detector->gyro_pre_mean[axis];
        result->gyro_pre_std_rad_s[axis] =
            sqrtf(detector->gyro_pre_m2[axis] /
                  (float)(detector->count - 1u));
        result->gyro_pre_peak_to_peak_rad_s[axis] =
            detector->gyro_pre_max[axis] - detector->gyro_pre_min[axis];
        result->gyro_corrected_mean_rad_s[axis] =
            detector->gyro_corrected_mean[axis];
        result->specific_force_mean_mps2[axis] =
            detector->specific_force_mean[axis];
    }

    gyro_pre_mean_norm =
        imu_stationary_vec3_magnitude(result->gyro_pre_mean_rad_s);
    gyro_corrected_mean_norm =
        imu_stationary_vec3_magnitude(result->gyro_corrected_mean_rad_s);
    candidate =
        gyro_pre_mean_norm <= detector->config.gyro_pre_mean_max_rad_s &&
        fabsf(result->accel_norm_mean_mps2 - IMU_STANDARD_GRAVITY_MPS2) <=
            detector->config.accel_norm_tolerance_mps2 &&
        result->accel_norm_std_mps2 <=
            detector->config.accel_norm_std_max_mps2;
    for (axis = 0u; axis < 3u; ++axis) {
        candidate = candidate &&
            result->gyro_pre_std_rad_s[axis] <=
                detector->config.gyro_pre_std_max_rad_s &&
            result->gyro_pre_peak_to_peak_rad_s[axis] <=
                detector->config.gyro_pre_peak_to_peak_max_rad_s;
    }
    if (!startup_mode && gyro_bias_valid) {
        candidate = candidate &&
            gyro_corrected_mean_norm <=
                detector->config.gyro_corrected_mean_max_rad_s;
    }

    result->candidate_static = candidate;
    result->window_ready = true;
    /* 连续窗口滞回只在完整窗口边界推进，单帧毛刺不会直接切换状态。 */
    if (candidate) {
        detector->moving_window_count = 0u;
        if (detector->static_window_count < UINT16_MAX) {
            ++detector->static_window_count;
        }
        if (detector->static_window_count >=
            detector->config.enter_static_windows) {
            detector->state = IMU_MOTION_ANGULAR_STATIC;
        }
    } else {
        detector->static_window_count = 0u;
        if (detector->moving_window_count < UINT16_MAX) {
            ++detector->moving_window_count;
        }
        if (detector->moving_window_count >=
            detector->config.exit_static_windows) {
            detector->state = IMU_MOTION_MOVING;
        }
    }
    result->state = detector->state;
    imu_stationary_reset_window(detector);
    return IMU_STATUS_OK;
}
