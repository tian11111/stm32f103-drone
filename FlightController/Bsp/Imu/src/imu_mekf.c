#include "imu_mekf.h"

#include "imu_math_internal.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define IMU_MEKF_STANDARD_GRAVITY_MPS2 (9.80665f)
#define IMU_MEKF_PI_RAD                 (3.14159265358979323846f)
#define IMU_MEKF_NEGATIVE_VARIANCE_EPS  (1.0e-7f)

/**
 * @brief 检查 MEKF 配置中的连续噪声、方差、时间和注入门限。
 * @param config 待验证配置。
 * @return true 全部字段有限且物理范围合法；false 配置不可使用。
 */
static bool imu_mekf_config_is_valid(const ImuMekfConfig *config)
{
    return config != 0 &&
           imu_math_is_finite(config->gyro_noise_density_rad_s_sqrt_hz) &&
           config->gyro_noise_density_rad_s_sqrt_hz > 0.0f &&
           imu_math_is_finite(
               config->gyro_bias_random_walk_rad_s2_sqrt_hz) &&
           config->gyro_bias_random_walk_rad_s2_sqrt_hz > 0.0f &&
           imu_math_is_finite(config->initial_tilt_variance_rad2) &&
           config->initial_tilt_variance_rad2 > 0.0f &&
           imu_math_is_finite(config->initial_yaw_variance_rad2) &&
           config->initial_yaw_variance_rad2 > 0.0f &&
           imu_math_is_finite(config->initial_bias_variance_rad2_s2) &&
           config->initial_bias_variance_rad2_s2 > 0.0f &&
           imu_math_is_finite(
               config->specific_force_direction_variance_rad2) &&
           config->specific_force_direction_variance_rad2 > 0.0f &&
           imu_math_is_finite(
               config->zaru_angular_rate_variance_rad2_s2) &&
           config->zaru_angular_rate_variance_rad2_s2 > 0.0f &&
           imu_math_is_finite(
               config->specific_force_norm_tolerance_mps2) &&
           config->specific_force_norm_tolerance_mps2 > 0.0f &&
           imu_math_is_finite(config->minimum_dt_s) &&
           config->minimum_dt_s > 0.0f &&
           imu_math_is_finite(config->maximum_dt_s) &&
           config->maximum_dt_s >= config->minimum_dt_s &&
           imu_math_is_finite(config->maximum_attitude_correction_rad) &&
           config->maximum_attitude_correction_rad > 0.0f &&
           imu_math_is_finite(config->maximum_bias_correction_rad_s) &&
           config->maximum_bias_correction_rad_s > 0.0f;
}

/**
 * @brief 将公开数组转换为数学层 Hamilton 四元数。
 * @param quaternion_wb 输入顺序固定为 w,x,y,z。
 * @return 数学层四元数值。
 */
static ImuQuaternion imu_mekf_quaternion_from_array(
    const float quaternion_wb[4])
{
    ImuQuaternion quaternion;

    quaternion.w = quaternion_wb[0];
    quaternion.x = quaternion_wb[1];
    quaternion.y = quaternion_wb[2];
    quaternion.z = quaternion_wb[3];
    return quaternion;
}

/**
 * @brief 将数学层四元数写回公开 w,x,y,z 数组。
 * @param quaternion 输入 Hamilton 四元数。
 * @param quaternion_wb 输出数组。
 */
static void imu_mekf_quaternion_to_array(ImuQuaternion quaternion,
                                         float quaternion_wb[4])
{
    quaternion_wb[0] = quaternion.w;
    quaternion_wb[1] = quaternion.x;
    quaternion_wb[2] = quaternion.y;
    quaternion_wb[3] = quaternion.z;
}

/**
 * @brief 提取 q_WB 的 fused yaw，并拒绝唯一的 180 度倾倒奇异点。
 * @param quaternion 单位 Hamilton Body->World 四元数。
 * @param fused_yaw_rad 输出范围 [-pi,pi] 的倾斜解耦航向，单位 rad。
 * @return true 输出有效；false 参数非法、数值非法或 w=z=0。
 */
static bool imu_mekf_try_get_fused_yaw_rad(ImuQuaternion quaternion,
                                            float *fused_yaw_rad)
{
    float heading_norm_squared;

    if (fused_yaw_rad == 0 || !imu_math_is_finite(quaternion.w) ||
        !imu_math_is_finite(quaternion.x) ||
        !imu_math_is_finite(quaternion.y) ||
        !imu_math_is_finite(quaternion.z)) {
        return false;
    }
    heading_norm_squared = quaternion.w * quaternion.w +
                           quaternion.z * quaternion.z;
    if (!(heading_norm_squared > 0.0f) ||
        !imu_math_is_finite(heading_norm_squared)) {
        return false;
    }
    *fused_yaw_rad = imu_math_quaternion_get_fused_yaw_rad(quaternion);
    return imu_math_is_finite(*fused_yaw_rad);
}

/**
 * @brief 左乘世界 Z 轴旋转，把候选四元数恢复到指定 fused-yaw 基准。
 * @param quaternion 输入输出单位 q_WB。
 * @param reference_yaw_rad 更新前的航向基准，单位 rad。
 * @return true 已恢复航向；false 候选姿态处于 180 度倾倒或数值非法。
 * @note 左乘世界 Z 轴不会改变 R_BW*[0,0,1]。右乘误差坐标也保持不变，
 *       因此该不可观测 gauge 选择不需要再次变换误差状态协方差。
 */
static bool imu_mekf_restore_fused_yaw(ImuQuaternion *quaternion,
                                       float reference_yaw_rad)
{
    ImuQuaternion world_yaw_correction;
    ImuQuaternion corrected;
    float candidate_yaw_rad;
    float correction_rad;

    if (quaternion == 0 || !imu_math_is_finite(reference_yaw_rad) ||
        !imu_mekf_try_get_fused_yaw_rad(*quaternion,
                                         &candidate_yaw_rad)) {
        return false;
    }
    correction_rad = reference_yaw_rad - candidate_yaw_rad;
    if (correction_rad > IMU_MEKF_PI_RAD) {
        correction_rad -= 2.0f * IMU_MEKF_PI_RAD;
    } else if (correction_rad < -IMU_MEKF_PI_RAD) {
        correction_rad += 2.0f * IMU_MEKF_PI_RAD;
    }
    world_yaw_correction.w = cosf(0.5f * correction_rad);
    world_yaw_correction.x = 0.0f;
    world_yaw_correction.y = 0.0f;
    world_yaw_correction.z = sinf(0.5f * correction_rad);
    corrected = imu_math_quaternion_multiply(world_yaw_correction,
                                               *quaternion);
    if (!imu_math_quaternion_normalize(&corrected)) {
        return false;
    }
    *quaternion = corrected;
    return true;
}

/**
 * @brief 检查候选四元数、偏置和协方差能否安全提交。
 * @param quaternion_wb 候选单位四元数。
 * @param gyro_bias_body_rad_s 候选三轴偏置。
 * @param covariance 候选 6x6 协方差。
 * @return true 全部有限且协方差对角线非负；false 数值状态损坏。
 */
static bool imu_mekf_state_is_valid(
    const float quaternion_wb[4],
    const float gyro_bias_body_rad_s[3],
    const float covariance[IMU_MEKF_ERROR_STATE_SIZE]
                          [IMU_MEKF_ERROR_STATE_SIZE])
{
    uint32_t row;
    uint32_t column;

    if (!imu_math_is_finite(quaternion_wb[0]) ||
        !imu_math_is_finite(quaternion_wb[1]) ||
        !imu_math_is_finite(quaternion_wb[2]) ||
        !imu_math_is_finite(quaternion_wb[3]) ||
        !imu_math_vec3_is_finite(gyro_bias_body_rad_s)) {
        return false;
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        if (!imu_math_is_finite(covariance[row][row]) ||
            covariance[row][row] < -IMU_MEKF_NEGATIVE_VARIANCE_EPS) {
            return false;
        }
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            if (!imu_math_is_finite(covariance[row][column])) {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief 强制 6x6 协方差对称并清除舍入产生的微小负方差。
 * @param covariance 输入输出协方差。
 */
static void imu_mekf_symmetrize_covariance(
    float covariance[IMU_MEKF_ERROR_STATE_SIZE]
                    [IMU_MEKF_ERROR_STATE_SIZE])
{
    uint32_t row;
    uint32_t column;

    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = row + 1u;
             column < IMU_MEKF_ERROR_STATE_SIZE;
             ++column) {
            float symmetric_value =
                0.5f * (covariance[row][column] +
                        covariance[column][row]);
            covariance[row][column] = symmetric_value;
            covariance[column][row] = symmetric_value;
        }
        if (covariance[row][row] < 0.0f &&
            covariance[row][row] >= -IMU_MEKF_NEGATIVE_VARIANCE_EPS) {
            covariance[row][row] = 0.0f;
        }
    }
}

/**
 * @brief 求有限 3x3 矩阵逆，用于创新协方差。
 * @param matrix 输入矩阵。
 * @param inverse 输出逆矩阵。
 * @return true 行列式具有安全裕量且结果有限；false 矩阵退化。
 */
static bool imu_mekf_inverse_mat3(const float matrix[3][3],
                                  float inverse[3][3])
{
    float cofactor00 = matrix[1][1] * matrix[2][2] -
                       matrix[1][2] * matrix[2][1];
    float cofactor01 = matrix[1][2] * matrix[2][0] -
                       matrix[1][0] * matrix[2][2];
    float cofactor02 = matrix[1][0] * matrix[2][1] -
                       matrix[1][1] * matrix[2][0];
    float determinant = matrix[0][0] * cofactor00 +
                        matrix[0][1] * cofactor01 +
                        matrix[0][2] * cofactor02;
    float inverse_determinant;
    float maximum_element = 0.0f;
    float determinant_scale;
    uint32_t row;
    uint32_t column;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            float magnitude = fabsf(matrix[row][column]);
            if (!imu_math_is_finite(magnitude)) {
                return false;
            }
            if (magnitude > maximum_element) {
                maximum_element = magnitude;
            }
        }
    }
    determinant_scale = maximum_element * maximum_element * maximum_element;
    /*
     * determinant 的量纲是矩阵元素的三次方，不能用固定绝对阈值。
     * 按最大元素三次方缩放后，方向观测和角速度观测可共用同一求逆器。
     */
    if (!imu_math_is_finite(determinant) || maximum_element <= 0.0f ||
        fabsf(determinant) <= FLT_EPSILON * determinant_scale) {
        return false;
    }
    inverse_determinant = 1.0f / determinant;
    inverse[0][0] = cofactor00 * inverse_determinant;
    inverse[0][1] = (matrix[0][2] * matrix[2][1] -
                     matrix[0][1] * matrix[2][2]) * inverse_determinant;
    inverse[0][2] = (matrix[0][1] * matrix[1][2] -
                     matrix[0][2] * matrix[1][1]) * inverse_determinant;
    inverse[1][0] = cofactor01 * inverse_determinant;
    inverse[1][1] = (matrix[0][0] * matrix[2][2] -
                     matrix[0][2] * matrix[2][0]) * inverse_determinant;
    inverse[1][2] = (matrix[0][2] * matrix[1][0] -
                     matrix[0][0] * matrix[1][2]) * inverse_determinant;
    inverse[2][0] = cofactor02 * inverse_determinant;
    inverse[2][1] = (matrix[0][1] * matrix[2][0] -
                     matrix[0][0] * matrix[2][1]) * inverse_determinant;
    inverse[2][2] = (matrix[0][0] * matrix[1][1] -
                     matrix[0][1] * matrix[1][0]) * inverse_determinant;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            if (!imu_math_is_finite(inverse[row][column])) {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief 执行共用三维线性误差状态更新、Joseph 协方差和右侧注入/Reset。
 * @param mekf 已初始化估计器。
 * @param residual 三维创新 z-h(x_nominal)。
 * @param measurement_jacobian 对 [delta_theta,delta_b_g] 的 3x6 Jacobian。
 * @param measurement_variance 三轴独立同方差观测噪声。
 * @param attitude_gain_projection 施加到 Kalman 增益姿态三行左侧的 3x3
 *        投影。单位阵表示完整姿态更新；重力切平面投影禁止修改航向自由度；
 *        零矩阵表示只更新偏置。
 * @param bias_gain_projection 施加到 Kalman 增益偏置三行左侧的 3x3 投影。
 *        重力观测使用切平面投影，禁止修改当前绕重力方向的不可观偏置；
 *        ZARU 使用单位阵，保留完整三轴偏置观测。
 * @param preserved_fused_yaw_rad 非空时，候选四元数提交前恢复到该 fused
 *        yaw；仅 180 度倾倒奇异点自动跳过。空指针表示不执行 gauge 约束。
 * @return OK 已提交；NO_DATA 表示注入过大而整次拒绝；负值表示数值故障。
 * @note 状态和协方差只在全部候选结果通过检查后一起提交，避免只截断
 *       状态注入却仍按原始 Kalman 增益收缩 P。
 */
static ImuStatus imu_mekf_apply_measurement3(
    ImuMekf *mekf,
    const float residual[3],
    const float measurement_jacobian[3][IMU_MEKF_ERROR_STATE_SIZE],
    float measurement_variance,
    const float attitude_gain_projection[3][3],
    const float bias_gain_projection[3][3],
    const float *preserved_fused_yaw_rad)
{
    float innovation_covariance[3][3] = {{0.0f}};
    float innovation_covariance_inverse[3][3];
    float covariance_h_transpose[IMU_MEKF_ERROR_STATE_SIZE][3] = {{0.0f}};
    float kalman_gain[IMU_MEKF_ERROR_STATE_SIZE][3] = {{0.0f}};
    float projected_attitude_gain[3][3] = {{0.0f}};
    float projected_bias_gain[3][3] = {{0.0f}};
    float error_state[IMU_MEKF_ERROR_STATE_SIZE] = {0.0f};
    float joseph_left[IMU_MEKF_ERROR_STATE_SIZE]
                      [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float temporary[IMU_MEKF_ERROR_STATE_SIZE]
                   [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float covariance_joseph[IMU_MEKF_ERROR_STATE_SIZE]
                           [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float reset[IMU_MEKF_ERROR_STATE_SIZE]
               [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float reset_temporary[IMU_MEKF_ERROR_STATE_SIZE]
                         [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float covariance_new[IMU_MEKF_ERROR_STATE_SIZE]
                        [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float reset_attitude[3][3];
    float attitude_correction_rad[3];
    float bias_correction_rad_s[3];
    float gyro_bias_new_rad_s[3];
    float quaternion_new_array[4];
    ImuQuaternion quaternion;
    ImuQuaternion increment;
    ImuQuaternion quaternion_new;
    uint32_t row;
    uint32_t column;
    uint32_t index;

    if (mekf == 0 || !mekf->initialized || attitude_gain_projection == 0 ||
        bias_gain_projection == 0 ||
        !imu_math_vec3_is_finite(residual) ||
        !imu_math_is_finite(measurement_variance) ||
        measurement_variance <= 0.0f) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < 3u; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                covariance_h_transpose[row][column] +=
                    mekf->covariance[row][index] *
                    measurement_jacobian[column][index];
            }
        }
    }
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                innovation_covariance[row][column] +=
                    measurement_jacobian[row][index] *
                    covariance_h_transpose[index][column];
            }
            if (row == column) {
                innovation_covariance[row][column] += measurement_variance;
            }
        }
    }
    if (!imu_mekf_inverse_mat3(innovation_covariance,
                               innovation_covariance_inverse)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < 3u; ++column) {
            for (index = 0u; index < 3u; ++index) {
                kalman_gain[row][column] +=
                    covariance_h_transpose[row][index] *
                    innovation_covariance_inverse[index][column];
            }
        }
    }
    /*
     * 观测可观测性必须同时约束状态注入和协方差更新。姿态、偏置增益
     * 分别左乘调用者给定投影，后续误差状态与 Joseph 形式共用受约束 K。
     */
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            for (index = 0u; index < 3u; ++index) {
                projected_attitude_gain[row][column] +=
                    attitude_gain_projection[row][index] *
                    kalman_gain[index][column];
            }
            for (index = 0u; index < 3u; ++index) {
                projected_bias_gain[row][column] +=
                    bias_gain_projection[row][index] *
                    kalman_gain[index + 3u][column];
            }
        }
    }
    /*
     * 两个投影都必须完整读取原始 K 后才写回。若逐行覆盖，后续行会
     * 使用已经投影的前行，破坏倾斜姿态下的切平面约束与偏置更新。
     */
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            kalman_gain[row][column] = projected_attitude_gain[row][column];
            kalman_gain[row + 3u][column] = projected_bias_gain[row][column];
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < 3u; ++column) {
            error_state[row] += kalman_gain[row][column] * residual[column];
        }
    }
    for (index = 0u; index < 3u; ++index) {
        attitude_correction_rad[index] = error_state[index];
        bias_correction_rad_s[index] = error_state[index + 3u];
    }
    if (imu_math_vec3_norm(attitude_correction_rad) >
            mekf->config.maximum_attitude_correction_rad ||
        imu_math_vec3_norm(bias_correction_rad_s) >
            mekf->config.maximum_bias_correction_rad_s) {
        return IMU_STATUS_NO_DATA;
    }

    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            joseph_left[row][column] = (row == column) ? 1.0f : 0.0f;
            for (index = 0u; index < 3u; ++index) {
                joseph_left[row][column] -=
                    kalman_gain[row][index] *
                    measurement_jacobian[index][column];
            }
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                temporary[row][column] +=
                    joseph_left[row][index] * mekf->covariance[index][column];
            }
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                covariance_joseph[row][column] +=
                    temporary[row][index] * joseph_left[column][index];
            }
            for (index = 0u; index < 3u; ++index) {
                covariance_joseph[row][column] +=
                    measurement_variance * kalman_gain[row][index] *
                    kalman_gain[column][index];
            }
        }
    }

    quaternion = imu_mekf_quaternion_from_array(mekf->quaternion_wb);
    increment = imu_math_quaternion_exp(attitude_correction_rad);
    quaternion_new = imu_math_quaternion_multiply(quaternion, increment);
    if (!imu_math_quaternion_normalize(&quaternion_new)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    if (preserved_fused_yaw_rad != 0) {
        (void)imu_mekf_restore_fused_yaw(&quaternion_new,
                                         *preserved_fused_yaw_rad);
    }
    imu_mekf_quaternion_to_array(quaternion_new, quaternion_new_array);
    for (index = 0u; index < 3u; ++index) {
        gyro_bias_new_rad_s[index] =
            mekf->gyro_bias_body_rad_s[index] + bias_correction_rad_s[index];
    }

    imu_math_so3_right_jacobian(attitude_correction_rad, reset_attitude);
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        reset[row][row] = 1.0f;
    }
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            reset[row][column] = reset_attitude[row][column];
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                reset_temporary[row][column] +=
                    reset[row][index] * covariance_joseph[index][column];
            }
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                covariance_new[row][column] +=
                    reset_temporary[row][index] * reset[column][index];
            }
        }
    }
    imu_mekf_symmetrize_covariance(covariance_new);
    if (!imu_mekf_state_is_valid(quaternion_new_array,
                                 gyro_bias_new_rad_s,
                                 covariance_new)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    memcpy(mekf->quaternion_wb, quaternion_new_array,
           sizeof(mekf->quaternion_wb));
    memcpy(mekf->gyro_bias_body_rad_s, gyro_bias_new_rad_s,
           sizeof(mekf->gyro_bias_body_rad_s));
    memcpy(mekf->covariance, covariance_new, sizeof(mekf->covariance));
    return IMU_STATUS_OK;
}

/**
 * @brief 填充适用于 MPU6050 1 kHz 解算的保守默认参数。
 * @param config 输出配置。
 * @note 两个过程噪声字段是连续时间密度，不是每采样点标准差。默认值用于
 *       通用启动，量产或高精度应用应由 Allan 方差和真实动态回放重新辨识。
 */
void imu_mekf_default_config(ImuMekfConfig *config)
{
    float tilt_standard_deviation_rad = 5.0f * IMU_MEKF_PI_RAD / 180.0f;
    float yaw_standard_deviation_rad = 180.0f * IMU_MEKF_PI_RAD / 180.0f;
    float bias_standard_deviation_rad_s =
        0.5f * IMU_MEKF_PI_RAD / 180.0f;
    float force_direction_standard_deviation_rad =
        3.0f * IMU_MEKF_PI_RAD / 180.0f;

    if (config == 0) {
        return;
    }
    config->gyro_noise_density_rad_s_sqrt_hz = 0.00035f;
    config->gyro_bias_random_walk_rad_s2_sqrt_hz = 0.00005f;
    config->initial_tilt_variance_rad2 =
        tilt_standard_deviation_rad * tilt_standard_deviation_rad;
    config->initial_yaw_variance_rad2 =
        yaw_standard_deviation_rad * yaw_standard_deviation_rad;
    config->initial_bias_variance_rad2_s2 =
        bias_standard_deviation_rad_s * bias_standard_deviation_rad_s;
    config->specific_force_direction_variance_rad2 =
        force_direction_standard_deviation_rad *
        force_direction_standard_deviation_rad;
    /* 200 点窗口均值仍受 DLPF 样本相关性影响，按 1.5 mrad/s 设置。 */
    config->zaru_angular_rate_variance_rad2_s2 = 2.25e-6f;
    config->specific_force_norm_tolerance_mps2 = 0.8f;
    config->minimum_dt_s = 0.0002f;
    config->maximum_dt_s = 0.02f;
    config->maximum_attitude_correction_rad = 0.35f;
    config->maximum_bias_correction_rad_s = 0.01f;
}

/**
 * @brief 用静止比力方向和启动陀螺偏置初始化名义状态与 6x6 协方差。
 * @param mekf 调用者持有的估计器实例。
 * @param config 连续噪声、初始方差、观测和时间门限。
 * @param initial_specific_force_body_mps2 静止平均 B 系比力，m/s^2。
 * @param initial_gyro_bias_body_rad_s 启动调零得到的唯一 b_g，rad/s。
 * @return IMU_STATUS_OK 初始化成功；否则返回参数、配置或数值错误。
 * @note 初始 Yaw 固定为零；Roll/Pitch 使 R_BW*[0,0,1] 与实测比力方向一致。
 */
ImuStatus imu_mekf_init(ImuMekf *mekf,
                        const ImuMekfConfig *config,
                        const float initial_specific_force_body_mps2[3],
                        const float initial_gyro_bias_body_rad_s[3])
{
    float force_direction[3];
    float lateral_norm;
    float roll_rad;
    float pitch_rad;
    float half_roll;
    float half_pitch;
    ImuMekfConfig config_copy;
    ImuQuaternion quaternion;
    uint32_t axis;

    if (mekf == 0 || !imu_mekf_config_is_valid(config) ||
        !imu_math_vec3_is_finite(initial_specific_force_body_mps2) ||
        !imu_math_vec3_is_finite(initial_gyro_bias_body_rad_s)) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    config_copy = *config;
    force_direction[0] = initial_specific_force_body_mps2[0];
    force_direction[1] = initial_specific_force_body_mps2[1];
    force_direction[2] = initial_specific_force_body_mps2[2];
    if (!imu_math_vec3_normalize(force_direction)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }

    lateral_norm = sqrtf(force_direction[1] * force_direction[1] +
                         force_direction[2] * force_direction[2]);
    roll_rad = atan2f(force_direction[1], force_direction[2]);
    pitch_rad = atan2f(-force_direction[0], lateral_norm);
    half_roll = 0.5f * roll_rad;
    half_pitch = 0.5f * pitch_rad;
    quaternion.w = cosf(half_pitch) * cosf(half_roll);
    quaternion.x = cosf(half_pitch) * sinf(half_roll);
    quaternion.y = sinf(half_pitch) * cosf(half_roll);
    quaternion.z = -sinf(half_pitch) * sinf(half_roll);
    if (!imu_math_quaternion_normalize(&quaternion)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }

    memset(mekf, 0, sizeof(*mekf));
    mekf->config = config_copy;
    imu_mekf_quaternion_to_array(quaternion, mekf->quaternion_wb);
    for (axis = 0u; axis < 3u; ++axis) {
        mekf->gyro_bias_body_rad_s[axis] =
            initial_gyro_bias_body_rad_s[axis];
        mekf->covariance[axis][axis] =
            (axis == 2u) ? config_copy.initial_yaw_variance_rad2 :
                           config_copy.initial_tilt_variance_rad2;
        mekf->covariance[axis + 3u][axis + 3u] =
            config_copy.initial_bias_variance_rad2_s2;
    }
    mekf->initialized = true;
    return IMU_STATUS_OK;
}

/**
 * @brief 用一帧 omega_pre 和实际 dt 传播名义四元数及误差协方差。
 * @param mekf 已初始化估计器。
 * @param gyro_pre_body_rad_s 尚未减 b_g 的 B 系角速度，rad/s。
 * @param dt_s 本帧与上一帧间隔，s。
 * @return IMU_STATUS_OK 预测已提交；否则状态保持为上一次有效值。
 * @note 使用 q<-q*Exp((omega_pre-b_g)dt)、精确 SO(3) Phi 姿态块和文档规定
 *       的各向同性 Qd 三阶近似；b_g 只在此处减一次。
 */
ImuStatus imu_mekf_predict(ImuMekf *mekf,
                           const float gyro_pre_body_rad_s[3],
                           float dt_s)
{
    float corrected_angular_rate_rad_s[3];
    float rotation_vector_rad[3];
    float negative_rotation_vector_rad[3];
    float attitude_transition[3][3];
    float right_jacobian[3][3];
    float transition[IMU_MEKF_ERROR_STATE_SIZE]
                    [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float temporary[IMU_MEKF_ERROR_STATE_SIZE]
                   [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float covariance_new[IMU_MEKF_ERROR_STATE_SIZE]
                        [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float quaternion_new_array[4];
    float gyro_noise_psd;
    float bias_noise_psd;
    float dt_squared;
    float dt_cubed;
    ImuQuaternion quaternion;
    ImuQuaternion increment;
    ImuQuaternion quaternion_new;
    uint32_t row;
    uint32_t column;
    uint32_t index;
    uint32_t axis;

    if (mekf == 0 || !mekf->initialized ||
        !imu_math_vec3_is_finite(gyro_pre_body_rad_s) ||
        !imu_math_is_finite(dt_s)) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    if (dt_s < mekf->config.minimum_dt_s ||
        dt_s > mekf->config.maximum_dt_s) {
        return IMU_STATUS_TIME_ERROR;
    }
    for (index = 0u; index < 3u; ++index) {
        corrected_angular_rate_rad_s[index] =
            gyro_pre_body_rad_s[index] -
            mekf->gyro_bias_body_rad_s[index];
        rotation_vector_rad[index] =
            corrected_angular_rate_rad_s[index] * dt_s;
        negative_rotation_vector_rad[index] = -rotation_vector_rad[index];
    }

    quaternion = imu_mekf_quaternion_from_array(mekf->quaternion_wb);
    increment = imu_math_quaternion_exp(rotation_vector_rad);
    quaternion_new = imu_math_quaternion_multiply(quaternion, increment);
    if (!imu_math_quaternion_normalize(&quaternion_new)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    imu_mekf_quaternion_to_array(quaternion_new, quaternion_new_array);

    imu_math_so3_exp(negative_rotation_vector_rad, attitude_transition);
    imu_math_so3_right_jacobian(rotation_vector_rad, right_jacobian);
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            transition[row][column] = attitude_transition[row][column];
            transition[row][column + 3u] =
                -dt_s * right_jacobian[row][column];
        }
        transition[row + 3u][row + 3u] = 1.0f;
    }

    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u;
             column < IMU_MEKF_ERROR_STATE_SIZE;
             ++column) {
            for (index = 0u;
                 index < IMU_MEKF_ERROR_STATE_SIZE;
                 ++index) {
                temporary[row][column] +=
                    transition[row][index] *
                    mekf->covariance[index][column];
            }
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u;
             column < IMU_MEKF_ERROR_STATE_SIZE;
             ++column) {
            for (index = 0u;
                 index < IMU_MEKF_ERROR_STATE_SIZE;
                 ++index) {
                covariance_new[row][column] +=
                    temporary[row][index] *
                    transition[column][index];
            }
        }
    }

    gyro_noise_psd =
        mekf->config.gyro_noise_density_rad_s_sqrt_hz *
        mekf->config.gyro_noise_density_rad_s_sqrt_hz;
    bias_noise_psd =
        mekf->config.gyro_bias_random_walk_rad_s2_sqrt_hz *
        mekf->config.gyro_bias_random_walk_rad_s2_sqrt_hz;
    dt_squared = dt_s * dt_s;
    dt_cubed = dt_squared * dt_s;
    for (axis = 0u; axis < 3u; ++axis) {
        covariance_new[axis][axis] +=
            gyro_noise_psd * dt_s +
            bias_noise_psd * dt_cubed / 3.0f;
        covariance_new[axis][axis + 3u] -=
            0.5f * bias_noise_psd * dt_squared;
        covariance_new[axis + 3u][axis] -=
            0.5f * bias_noise_psd * dt_squared;
        covariance_new[axis + 3u][axis + 3u] +=
            bias_noise_psd * dt_s;
    }
    imu_mekf_symmetrize_covariance(covariance_new);
    if (!imu_mekf_state_is_valid(quaternion_new_array,
                                 mekf->gyro_bias_body_rad_s,
                                 covariance_new)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }

    memcpy(mekf->quaternion_wb, quaternion_new_array,
           sizeof(mekf->quaternion_wb));
    memcpy(mekf->covariance, covariance_new, sizeof(mekf->covariance));
    ++mekf->prediction_count;
    return IMU_STATUS_OK;
}

/**
 * @brief 用准静态比力方向执行三维秩二 MEKF 测量更新。
 * @param mekf 已初始化估计器。
 * @param specific_force_body_mps2 当前或窗口平均 B 系比力，m/s^2。
 * @return OK 更新成功；NO_DATA 表示模长门限拒绝；其他负值表示数值故障。
 * @note 残差固定为 normalize(f_meas) x R_BW(q)*[0,0,1]，右乘误差
 *       Jacobian 为 I-f_pred*f_pred^T。更新采用 Joseph 形式，随后右侧注入
 *       q<-q*Exp(delta_theta) 并用 J_r(delta_theta) 完成协方差 Reset。
 */
ImuStatus imu_mekf_update_specific_force_direction(
    ImuMekf *mekf,
    const float specific_force_body_mps2[3])
{
    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    float measured_direction[3];
    float predicted_direction[3];
    float residual[3];
    float measurement_jacobian[3][IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float gravity_tangent_projection[3][3];
    float force_norm;
    float fused_yaw_before_update_rad;
    const float *preserved_fused_yaw_rad = 0;
    ImuStatus status;
    ImuQuaternion quaternion;
    uint32_t row;
    uint32_t column;

    if (mekf == 0 || !mekf->initialized ||
        !imu_math_vec3_is_finite(specific_force_body_mps2)) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    force_norm = imu_math_vec3_norm(specific_force_body_mps2);
    if (!(force_norm > 0.0f) || !imu_math_is_finite(force_norm)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    if (fabsf(force_norm - IMU_MEKF_STANDARD_GRAVITY_MPS2) >
        mekf->config.specific_force_norm_tolerance_mps2) {
        ++mekf->rejected_specific_force_count;
        return IMU_STATUS_NO_DATA;
    }
    measured_direction[0] = specific_force_body_mps2[0] / force_norm;
    measured_direction[1] = specific_force_body_mps2[1] / force_norm;
    measured_direction[2] = specific_force_body_mps2[2] / force_norm;

    quaternion = imu_mekf_quaternion_from_array(mekf->quaternion_wb);
    if (imu_mekf_try_get_fused_yaw_rad(
            quaternion,
            &fused_yaw_before_update_rad)) {
        preserved_fused_yaw_rad = &fused_yaw_before_update_rad;
    }
    imu_math_quaternion_rotate_world_to_body(quaternion,
                                             world_up,
                                             predicted_direction);
    if (!imu_math_vec3_normalize(predicted_direction)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    imu_math_vec3_cross(measured_direction, predicted_direction, residual);
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            gravity_tangent_projection[row][column] =
                ((row == column) ? 1.0f : 0.0f) -
                predicted_direction[row] * predicted_direction[column];
            measurement_jacobian[row][column] =
                gravity_tangent_projection[row][column];
        }
    }

    status = imu_mekf_apply_measurement3(
        mekf,
        residual,
        measurement_jacobian,
        mekf->config.specific_force_direction_variance_rad2,
        gravity_tangent_projection,
        gravity_tangent_projection,
        preserved_fused_yaw_rad);
    if (status == IMU_STATUS_OK) {
        ++mekf->specific_force_update_count;
    } else if (status == IMU_STATUS_NO_DATA) {
        ++mekf->rejected_specific_force_count;
    }
    return status;
}

/**
 * @brief 静止时用窗口 omega_pre 均值执行三轴零角速度更新。
 * @param mekf 已初始化估计器。
 * @param gyro_pre_mean_body_rad_s 静止窗口 omega_pre 均值，rad/s。
 * @return OK 已更新唯一 b_g；NO_DATA 表示注入门限拒绝；负值表示参数或数值错误。
 * @note z=0，h=omega_pre-b_g，创新=-(omega_pre-b_g)，对误差偏置的
 *       Jacobian 为 -I。该观测的姿态增益被约束为零，只更新 b_g 和对应
 *       协方差，不回改四元数、不硬锁 Yaw，也不覆盖输出。
 */
ImuStatus imu_mekf_update_zero_angular_rate(
    ImuMekf *mekf,
    const float gyro_pre_mean_body_rad_s[3])
{
    float residual[3];
    float measurement_jacobian[3][IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    const float zero_attitude_gain[3][3] = {{0.0f}};
    const float full_bias_gain[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    ImuStatus status;
    uint32_t axis;

    if (mekf == 0 || !mekf->initialized ||
        !imu_math_vec3_is_finite(gyro_pre_mean_body_rad_s)) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    for (axis = 0u; axis < 3u; ++axis) {
        residual[axis] = -(gyro_pre_mean_body_rad_s[axis] -
                           mekf->gyro_bias_body_rad_s[axis]);
        measurement_jacobian[axis][axis + 3u] = -1.0f;
    }
    status = imu_mekf_apply_measurement3(
        mekf,
        residual,
        measurement_jacobian,
        mekf->config.zaru_angular_rate_variance_rad2_s2,
        zero_attitude_gain,
        full_bias_gain,
        0);
    if (status == IMU_STATUS_OK) {
        ++mekf->zaru_update_count;
    } else if (status == IMU_STATUS_NO_DATA) {
        ++mekf->rejected_zaru_count;
    }
    return status;
}

/**
 * @brief 融合外部 Body-Z 角速度，并只修正 Z 轴陀螺偏置。
 *
 * @param mekf 已初始化的估计器。
 * @param gyro_pre_body_rad_s 尚未减去 b_g 的 B 系角速度，单位 rad/s。
 * @param body_yaw_rate_rad_s 外部测得的真实 B 系 Z 轴角速度，单位 rad/s。
 * @param variance_rad2_s2 外部观测方差，单位 (rad/s)^2，必须大于 0。
 * @return IMU_STATUS_OK 已融合；IMU_STATUS_NO_DATA 修正超过安全门限；
 *         其他负值表示参数或数值状态非法。
 * @note H=[0,0,0,0,0,-1]。限制 Kalman 增益只有 b_g,z 一行非零，避免
 *       角速度观测被误用成航向角观测；协方差仍使用 Joseph 形式更新。
 */
ImuStatus imu_mekf_update_body_yaw_rate(
    ImuMekf *mekf,
    const float gyro_pre_body_rad_s[3],
    float body_yaw_rate_rad_s,
    float variance_rad2_s2)
{
    float innovation_variance;
    float residual;
    float gain;
    float bias_correction;
    float transition[IMU_MEKF_ERROR_STATE_SIZE]
                    [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float temporary[IMU_MEKF_ERROR_STATE_SIZE]
                   [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float covariance_new[IMU_MEKF_ERROR_STATE_SIZE]
                        [IMU_MEKF_ERROR_STATE_SIZE] = {{0.0f}};
    float gyro_bias_new[3];
    uint32_t row;
    uint32_t column;
    uint32_t index;

    if (mekf == 0 || !mekf->initialized ||
        !imu_math_vec3_is_finite(gyro_pre_body_rad_s) ||
        !imu_math_is_finite(body_yaw_rate_rad_s) ||
        !imu_math_is_finite(variance_rad2_s2) || variance_rad2_s2 <= 0.0f) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    innovation_variance = mekf->covariance[5][5] + variance_rad2_s2;
    if (!imu_math_is_finite(innovation_variance) ||
        innovation_variance <= FLT_EPSILON) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    residual = body_yaw_rate_rad_s -
               (gyro_pre_body_rad_s[2] -
                mekf->gyro_bias_body_rad_s[2]);
    gain = -mekf->covariance[5][5] / innovation_variance;
    bias_correction = gain * residual;
    if (!imu_math_is_finite(bias_correction)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    if (fabsf(bias_correction) >
        mekf->config.maximum_bias_correction_rad_s) {
        ++mekf->rejected_body_yaw_rate_count;
        return IMU_STATUS_NO_DATA;
    }

    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        transition[row][row] = 1.0f;
    }
    /* A=I-KH，且受约束 K 只有第 5 行、H[5]=-1。 */
    transition[5][5] += gain;
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                temporary[row][column] +=
                    transition[row][index] * mekf->covariance[index][column];
            }
        }
    }
    for (row = 0u; row < IMU_MEKF_ERROR_STATE_SIZE; ++row) {
        for (column = 0u; column < IMU_MEKF_ERROR_STATE_SIZE; ++column) {
            for (index = 0u; index < IMU_MEKF_ERROR_STATE_SIZE; ++index) {
                covariance_new[row][column] +=
                    temporary[row][index] * transition[column][index];
            }
        }
    }
    covariance_new[5][5] += gain * variance_rad2_s2 * gain;
    imu_mekf_symmetrize_covariance(covariance_new);

    memcpy(gyro_bias_new, mekf->gyro_bias_body_rad_s,
           sizeof(gyro_bias_new));
    gyro_bias_new[2] += bias_correction;
    if (!imu_mekf_state_is_valid(mekf->quaternion_wb,
                                 gyro_bias_new,
                                 covariance_new)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    memcpy(mekf->gyro_bias_body_rad_s, gyro_bias_new,
           sizeof(mekf->gyro_bias_body_rad_s));
    memcpy(mekf->covariance, covariance_new, sizeof(mekf->covariance));
    ++mekf->body_yaw_rate_update_count;
    return IMU_STATUS_OK;
}

/**
 * @brief 将当前 q_WB 转换为 ZYX 的 Roll、Pitch、Yaw。
 * @param mekf 已初始化估计器。
 * @param roll_rad 输出绕 B/X 的 Roll，rad。
 * @param pitch_rad 输出 Pitch，rad，范围 [-pi/2,pi/2]。
 * @param yaw_rad 输出相对 Yaw，rad，范围 [-pi,pi]。
 * @return true 输出有效；false 参数为空或估计器未初始化。
 */
bool imu_mekf_get_euler_rad(const ImuMekf *mekf,
                            float *roll_rad,
                            float *pitch_rad,
                            float *yaw_rad)
{
    float w;
    float x;
    float y;
    float z;
    ImuQuaternion quaternion;

    if (mekf == 0 || !mekf->initialized || roll_rad == 0 ||
        pitch_rad == 0 || yaw_rad == 0) {
        return false;
    }
    w = mekf->quaternion_wb[0];
    x = mekf->quaternion_wb[1];
    y = mekf->quaternion_wb[2];
    z = mekf->quaternion_wb[3];
    quaternion = imu_mekf_quaternion_from_array(mekf->quaternion_wb);
    *roll_rad = atan2f(2.0f * (w * x + y * z),
                       1.0f - 2.0f * (x * x + y * y));
    *pitch_rad = imu_math_quaternion_get_zyx_pitch_rad(quaternion);
    *yaw_rad = atan2f(2.0f * (w * z + x * y),
                      1.0f - 2.0f * (y * y + z * z));
    return imu_math_is_finite(*roll_rad) &&
           imu_math_is_finite(*pitch_rad) &&
           imu_math_is_finite(*yaw_rad);
}

/**
 * @brief 输出名义姿态预测的静止比力单位方向，供控制与奇异区诊断使用。
 * @param mekf 已初始化估计器。
 * @param direction_body 输出 B 系单位方向，等于 R_BW(q_WB)*[0,0,1]。
 * @return true 三个分量有效；false 参数为空、未初始化或出现非有限数。
 * @note 该方向指向静止加速度计测得的 +g 方向，并非世界重力向量 -g；
 *       它由四元数直接得到，不受 ZYX 欧拉角在 Pitch=+-90度处的跳变影响。
 */
bool imu_mekf_get_predicted_specific_force_direction_body(
    const ImuMekf *mekf,
    float direction_body[3])
{
    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    ImuQuaternion quaternion;

    if (mekf == 0 || direction_body == 0 || !mekf->initialized) {
        return false;
    }
    quaternion = imu_mekf_quaternion_from_array(mekf->quaternion_wb);
    imu_math_quaternion_rotate_world_to_body(quaternion,
                                             world_up,
                                             direction_body);
    return imu_math_vec3_is_finite(direction_body);
}

/**
 * @brief 计算不受 Yaw 和 ZYX Pitch 奇异点影响的两个重力倾角。
 * @param mekf 已初始化估计器。
 * @param gravity_tilt_roll_rad 输出 asin(f_pred_B.y)，范围 [-pi/2,pi/2]。
 * @param gravity_tilt_pitch_rad 输出 asin(-f_pred_B.x)，范围 [-pi/2,pi/2]。
 * @param upright_hemisphere 输出 f_pred_B.z>=0；用于区分越过 90 度前后。
 * @return true 输出有效；false 参数为空、未初始化或方向出现非有限数。
 * @note 两个倾角只描述重力相对机体的方向，不包含 Yaw。任一方向越过
 *       90 度后角度会连续折返，因此完整姿态仍必须使用 q_WB。
 */
bool imu_mekf_get_gravity_tilt_rad(const ImuMekf *mekf,
                                   float *gravity_tilt_roll_rad,
                                   float *gravity_tilt_pitch_rad,
                                   bool *upright_hemisphere)
{
    float direction_body[3];
    float roll_sine;
    float pitch_sine;

    if (gravity_tilt_roll_rad == 0 || gravity_tilt_pitch_rad == 0 ||
        upright_hemisphere == 0 ||
        !imu_mekf_get_predicted_specific_force_direction_body(
            mekf, direction_body)) {
        return false;
    }
    roll_sine = direction_body[1];
    pitch_sine = -direction_body[0];
    if (roll_sine > 1.0f) {
        roll_sine = 1.0f;
    } else if (roll_sine < -1.0f) {
        roll_sine = -1.0f;
    }
    if (pitch_sine > 1.0f) {
        pitch_sine = 1.0f;
    } else if (pitch_sine < -1.0f) {
        pitch_sine = -1.0f;
    }
    *gravity_tilt_roll_rad = asinf(roll_sine);
    *gravity_tilt_pitch_rad = asinf(pitch_sine);
    *upright_hemisphere = direction_body[2] >= 0.0f;
    return imu_math_is_finite(*gravity_tilt_roll_rad) &&
           imu_math_is_finite(*gravity_tilt_pitch_rad);
}
