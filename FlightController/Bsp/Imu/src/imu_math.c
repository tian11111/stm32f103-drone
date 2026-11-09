#include "imu_math_internal.h"

#include <float.h>
#include <math.h>

#define IMU_MATH_SMALL_ANGLE_SQ (1.0e-8f)

/**
 * @brief 构造向量的反对称矩阵，使 skew(v) * x 等于 v 与 x 的叉积。
 * @param vector 输入三维向量。
 * @param skew 输出 3x3 反对称矩阵，允许与后续局部临时量复用。
 */
static void imu_math_skew(const float vector[3], float skew[3][3])
{
    skew[0][0] = 0.0f;
    skew[0][1] = -vector[2];
    skew[0][2] = vector[1];
    skew[1][0] = vector[2];
    skew[1][1] = 0.0f;
    skew[1][2] = -vector[0];
    skew[2][0] = -vector[1];
    skew[2][1] = vector[0];
    skew[2][2] = 0.0f;
}

/**
 * @brief 判断单精度浮点数是否既不是 NaN 也不是正负无穷。
 * @param value 待检查数值。
 * @return true 数值有限；false 数值不可用于滤波计算。
 */
bool imu_math_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/**
 * @brief 检查三维向量指针和全部分量的有限性。
 * @param vector 待检查三维向量，不允许为空。
 * @return true 三个分量均有限；false 指针为空或包含非法浮点数。
 */
bool imu_math_vec3_is_finite(const float vector[3])
{
    return (vector != 0) &&
           imu_math_is_finite(vector[0]) &&
           imu_math_is_finite(vector[1]) &&
           imu_math_is_finite(vector[2]);
}

/**
 * @brief 计算有限三维向量的欧氏模长。
 * @param vector 输入三维向量。
 * @return 向量模长；输入为空或含非法值时返回 0，调用方仍应检查原向量。
 */
float imu_math_vec3_norm(const float vector[3])
{
    if (!imu_math_vec3_is_finite(vector)) {
        return 0.0f;
    }
    return sqrtf(vector[0] * vector[0] +
                 vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

/**
 * @brief 原地归一化三维向量。
 * @param vector 输入输出三维向量。
 * @return true 归一化成功；false 指针为空、模长为零或计算结果非法。
 * @note 失败时不保证 vector 保持原值，调用方不得继续发布该结果。
 */
bool imu_math_vec3_normalize(float vector[3])
{
    float norm;

    if (vector == 0) {
        return false;
    }
    norm = imu_math_vec3_norm(vector);
    if (!(norm > 0.0f) || !imu_math_is_finite(norm)) {
        return false;
    }
    vector[0] /= norm;
    vector[1] /= norm;
    vector[2] /= norm;
    return imu_math_vec3_is_finite(vector);
}

/**
 * @brief 计算两个三维向量的叉积 a x b。
 * @param a 左输入向量。
 * @param b 右输入向量。
 * @param result 输出向量；内部使用临时量，因此允许与 a 或 b 指向同一数组。
 */
void imu_math_vec3_cross(const float a[3], const float b[3], float result[3])
{
    float temporary[3];

    temporary[0] = a[1] * b[2] - a[2] * b[1];
    temporary[1] = a[2] * b[0] - a[0] * b[2];
    temporary[2] = a[0] * b[1] - a[1] * b[0];
    result[0] = temporary[0];
    result[1] = temporary[1];
    result[2] = temporary[2];
}

/**
 * @brief 将 3x3 矩阵设置为单位阵。
 * @param matrix 输出矩阵。
 */
void imu_math_mat3_identity(float matrix[3][3])
{
    uint32_t row;
    uint32_t column;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            matrix[row][column] = (row == column) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 计算 3x3 矩阵与三维列向量的乘积。
 * @param matrix 左乘矩阵。
 * @param vector 右侧列向量。
 * @param result 输出三维向量；允许与 vector 指向同一数组。
 */
void imu_math_mat3_mul_vec3(const float matrix[3][3],
                            const float vector[3],
                            float result[3])
{
    float temporary[3];
    uint32_t row;

    for (row = 0u; row < 3u; ++row) {
        temporary[row] = matrix[row][0] * vector[0] +
                         matrix[row][1] * vector[1] +
                         matrix[row][2] * vector[2];
    }
    result[0] = temporary[0];
    result[1] = temporary[1];
    result[2] = temporary[2];
}

/**
 * @brief 返回不产生旋转的 Hamilton 单位四元数。
 * @return ImuQuaternion [w,x,y,z]=[1,0,0,0]。
 */
ImuQuaternion imu_math_quaternion_identity(void)
{
    ImuQuaternion result = {1.0f, 0.0f, 0.0f, 0.0f};
    return result;
}

/**
 * @brief 计算 Hamilton 四元数乘积 left 与 right。
 * @param left 左乘四元数。
 * @param right 右乘四元数；MEKF 的 Body 局部增量放在此参数。
 * @return 未主动归一化的乘积，状态更新后应显式检查并归一化。
 */
ImuQuaternion imu_math_quaternion_multiply(ImuQuaternion left,
                                            ImuQuaternion right)
{
    ImuQuaternion result;

    result.w = left.w * right.w - left.x * right.x -
               left.y * right.y - left.z * right.z;
    result.x = left.w * right.x + left.x * right.w +
               left.y * right.z - left.z * right.y;
    result.y = left.w * right.y - left.x * right.z +
               left.y * right.w + left.z * right.x;
    result.z = left.w * right.z + left.x * right.y -
               left.y * right.x + left.z * right.w;
    return result;
}

/**
 * @brief 原地归一化 Hamilton 四元数。
 * @param quaternion 输入输出四元数。
 * @return true 归一化成功；false 指针为空、模长为零或包含非法值。
 */
bool imu_math_quaternion_normalize(ImuQuaternion *quaternion)
{
    float norm_squared;
    float inverse_norm;

    if (quaternion == 0) {
        return false;
    }
    norm_squared = quaternion->w * quaternion->w +
                   quaternion->x * quaternion->x +
                   quaternion->y * quaternion->y +
                   quaternion->z * quaternion->z;
    if (!(norm_squared > 0.0f) || !imu_math_is_finite(norm_squared)) {
        return false;
    }
    inverse_norm = 1.0f / sqrtf(norm_squared);
    quaternion->w *= inverse_norm;
    quaternion->x *= inverse_norm;
    quaternion->y *= inverse_norm;
    quaternion->z *= inverse_norm;
    return imu_math_is_finite(quaternion->w) &&
           imu_math_is_finite(quaternion->x) &&
           imu_math_is_finite(quaternion->y) &&
           imu_math_is_finite(quaternion->z);
}

/**
 * @brief 将 SO(3) 旋转向量映射为单位 Hamilton 四元数。
 * @param rotation_vector_rad 旋转轴乘转角，三个分量单位均为 rad。
 * @return 对应的增量四元数；小角度使用级数避免 sin(theta/2)/theta 失稳。
 */
ImuQuaternion imu_math_quaternion_exp(const float rotation_vector_rad[3])
{
    ImuQuaternion result;
    float angle_squared;
    float scalar_scale;

    angle_squared = rotation_vector_rad[0] * rotation_vector_rad[0] +
                    rotation_vector_rad[1] * rotation_vector_rad[1] +
                    rotation_vector_rad[2] * rotation_vector_rad[2];
    if (angle_squared < IMU_MATH_SMALL_ANGLE_SQ) {
        scalar_scale = 0.5f - angle_squared / 48.0f;
        result.w = 1.0f - angle_squared / 8.0f;
    } else {
        float angle = sqrtf(angle_squared);
        float half_angle = 0.5f * angle;
        scalar_scale = sinf(half_angle) / angle;
        result.w = cosf(half_angle);
    }
    result.x = scalar_scale * rotation_vector_rad[0];
    result.y = scalar_scale * rotation_vector_rad[1];
    result.z = scalar_scale * rotation_vector_rad[2];
    return result;
}

/**
 * @brief 从 Hamilton Body->World 四元数提取 fused yaw。
 * @param quaternion 输入单位四元数，顺序 w,x,y,z。
 * @return 范围 [-pi,pi] 的倾斜解耦航向角，单位 rad。
 * @note 定义为 wrap(2*atan2(z,w))。它只用于诊断航向，不修改姿态状态，
 *       也不能让无磁力计系统的绝对 Yaw 变得可观。调用方传入的四元数已由
 *       状态传播归一化；设备精确倒置时该航向定义本身存在奇异点。
 */
float imu_math_quaternion_get_fused_yaw_rad(ImuQuaternion quaternion)
{
    return atan2f(2.0f * quaternion.w * quaternion.z,
                  quaternion.w * quaternion.w -
                  quaternion.z * quaternion.z);
}

/**
 * @brief 从 Hamilton Body->World 四元数提取 ZYX Pitch。
 * @param quaternion 输入单位四元数，顺序 w,x,y,z。
 * @return 范围 [-pi/2,pi/2] 的 Pitch，单位 rad。
 * @note 对反正弦输入限幅，避免浮点舍入使单位四元数在正负 90 度附近
 *       产生 NaN。本函数只读取姿态，不修改滤波状态。
 */
float imu_math_quaternion_get_zyx_pitch_rad(ImuQuaternion quaternion)
{
    float pitch_sine = 2.0f * (quaternion.w * quaternion.y -
                               quaternion.z * quaternion.x);

    if (pitch_sine > 1.0f) {
        pitch_sine = 1.0f;
    } else if (pitch_sine < -1.0f) {
        pitch_sine = -1.0f;
    }
    return asinf(pitch_sine);
}

/**
 * @brief 使用主动旋转 q_WB 将 B 系向量变换到 W 系。
 * @param q_wb Hamilton Body->World 四元数，调用前应保证为单位四元数。
 * @param vector_body B 系输入向量。
 * @param vector_world W 系输出向量，不允许与 vector_body 重叠。
 */
void imu_math_quaternion_rotate_body_to_world(ImuQuaternion q_wb,
                                              const float vector_body[3],
                                              float vector_world[3])
{
    float q_vector[3] = {q_wb.x, q_wb.y, q_wb.z};
    float cross_once[3];
    float cross_twice[3];

    imu_math_vec3_cross(q_vector, vector_body, cross_once);
    imu_math_vec3_cross(q_vector, cross_once, cross_twice);
    vector_world[0] = vector_body[0] +
                      2.0f * (q_wb.w * cross_once[0] + cross_twice[0]);
    vector_world[1] = vector_body[1] +
                      2.0f * (q_wb.w * cross_once[1] + cross_twice[1]);
    vector_world[2] = vector_body[2] +
                      2.0f * (q_wb.w * cross_once[2] + cross_twice[2]);
}

/**
 * @brief 使用 q_WB 的共轭将 W 系向量变换到 B 系。
 * @param q_wb Hamilton Body->World 四元数，调用前应保证为单位四元数。
 * @param vector_world W 系输入向量。
 * @param vector_body B 系输出向量，不允许与 vector_world 重叠。
 */
void imu_math_quaternion_rotate_world_to_body(ImuQuaternion q_wb,
                                              const float vector_world[3],
                                              float vector_body[3])
{
    q_wb.x = -q_wb.x;
    q_wb.y = -q_wb.y;
    q_wb.z = -q_wb.z;
    imu_math_quaternion_rotate_body_to_world(q_wb, vector_world, vector_body);
}

/**
 * @brief 通过 Rodrigues 公式计算 SO(3) 指数映射 Exp([phi]x)。
 * @param rotation_vector_rad 旋转向量 phi，单位 rad。
 * @param rotation_matrix 输出主动旋转矩阵。
 * @note 小角度分支使用稳定级数，供 MEKF 的固定维度状态转移使用。
 */
void imu_math_so3_exp(const float rotation_vector_rad[3],
                      float rotation_matrix[3][3])
{
    float skew[3][3];
    float skew_squared[3][3];
    float angle_squared;
    float coefficient_a;
    float coefficient_b;
    uint32_t row;
    uint32_t column;
    uint32_t index;

    angle_squared = rotation_vector_rad[0] * rotation_vector_rad[0] +
                    rotation_vector_rad[1] * rotation_vector_rad[1] +
                    rotation_vector_rad[2] * rotation_vector_rad[2];
    if (angle_squared < IMU_MATH_SMALL_ANGLE_SQ) {
        coefficient_a = 1.0f - angle_squared / 6.0f;
        coefficient_b = 0.5f - angle_squared / 24.0f;
    } else {
        float angle = sqrtf(angle_squared);
        coefficient_a = sinf(angle) / angle;
        coefficient_b = (1.0f - cosf(angle)) / angle_squared;
    }

    imu_math_skew(rotation_vector_rad, skew);
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            skew_squared[row][column] = 0.0f;
            for (index = 0u; index < 3u; ++index) {
                skew_squared[row][column] += skew[row][index] *
                                             skew[index][column];
            }
        }
    }

    imu_math_mat3_identity(rotation_matrix);
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            rotation_matrix[row][column] +=
                coefficient_a * skew[row][column] +
                coefficient_b * skew_squared[row][column];
        }
    }
}

/**
 * @brief 计算 SO(3) 旋转向量的右 Jacobian J_r(phi)。
 * @param rotation_vector_rad 旋转向量 phi，单位 rad。
 * @param jacobian 输出 3x3 右 Jacobian。
 * @note 小角度分支使用二阶级数，供 MEKF 状态转移和 Error Reset 共用。
 */
void imu_math_so3_right_jacobian(const float rotation_vector_rad[3],
                                 float jacobian[3][3])
{
    float skew[3][3];
    float skew_squared[3][3];
    float angle_squared;
    float coefficient_a;
    float coefficient_b;
    uint32_t row;
    uint32_t column;
    uint32_t index;

    angle_squared = rotation_vector_rad[0] * rotation_vector_rad[0] +
                    rotation_vector_rad[1] * rotation_vector_rad[1] +
                    rotation_vector_rad[2] * rotation_vector_rad[2];
    if (angle_squared < IMU_MATH_SMALL_ANGLE_SQ) {
        coefficient_a = 0.5f - angle_squared / 24.0f;
        coefficient_b = 1.0f / 6.0f - angle_squared / 120.0f;
    } else {
        float angle = sqrtf(angle_squared);
        coefficient_a = (1.0f - cosf(angle)) / angle_squared;
        coefficient_b = (angle - sinf(angle)) / (angle_squared * angle);
    }

    imu_math_skew(rotation_vector_rad, skew);
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            skew_squared[row][column] = 0.0f;
            for (index = 0u; index < 3u; ++index) {
                skew_squared[row][column] += skew[row][index] *
                                             skew[index][column];
            }
        }
    }

    imu_math_mat3_identity(jacobian);
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            jacobian[row][column] -= coefficient_a * skew[row][column];
            jacobian[row][column] += coefficient_b * skew_squared[row][column];
        }
    }
}
