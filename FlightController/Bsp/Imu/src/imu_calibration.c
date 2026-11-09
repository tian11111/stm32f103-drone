#include "imu_calibration.h"

#include "imu_math_internal.h"

#include <string.h>

#define IMU_STANDARD_GRAVITY_MPS2 (9.80665f)
#define IMU_DEGREES_TO_RADIANS    (0.01745329251994329577f)

/**
 * @brief 检查 3x3 标定矩阵全部元素是否为有限浮点数。
 * @param matrix 待检查矩阵。
 * @return true 全部元素有限；false 至少包含一个 NaN 或 Inf。
 */
static bool imu_calibration_matrix_is_finite(const float matrix[3][3])
{
    uint32_t row;
    uint32_t column;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            if (!imu_math_is_finite(matrix[row][column])) {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief 生成不改变轴向、比例和零偏的标定参数。
 * @param calibration 输出标定结构。
 * @note 温度有效区间暂设为 MPU6050 常用工作范围 -40 至 85 摄氏度；
 *       单位矩阵只用于首次采集，不能替代六面和温度标定结果。
 */
void imu_calibration_set_identity(ImuCalibration *calibration)
{
    if (calibration == 0) {
        return;
    }
    memset(calibration, 0, sizeof(*calibration));
    calibration->temperature_reference_c = 25.0f;
    calibration->temperature_min_c = -40.0f;
    calibration->temperature_max_c = 85.0f;
    imu_math_mat3_identity(calibration->gyro_correction_matrix);
    imu_math_mat3_identity(calibration->accel_correction_matrix);
    imu_math_mat3_identity(calibration->sensor_to_body);
}

/**
 * @brief 验证温区、三轴零偏和所有标定矩阵的数值有效性。
 * @param calibration 待验证标定参数。
 * @return IMU_STATUS_OK 参数有限且温区有序；否则返回 IMU_STATUS_CONFIG_ERROR。
 * @note 此处只做运行时数值边界检查；正交性、比例范围和六面残差由标定工具验证。
 */
ImuStatus imu_calibration_validate(const ImuCalibration *calibration)
{
    uint32_t index;

    if (calibration == 0 ||
        !imu_math_is_finite(calibration->temperature_reference_c) ||
        !imu_math_is_finite(calibration->temperature_min_c) ||
        !imu_math_is_finite(calibration->temperature_max_c) ||
        calibration->temperature_min_c > calibration->temperature_max_c ||
        !imu_calibration_matrix_is_finite(calibration->gyro_temperature_coeff_sensor) ||
        !imu_calibration_matrix_is_finite(calibration->gyro_correction_matrix) ||
        !imu_calibration_matrix_is_finite(calibration->accel_correction_matrix) ||
        !imu_calibration_matrix_is_finite(calibration->sensor_to_body)) {
        return IMU_STATUS_CONFIG_ERROR;
    }
    for (index = 0u; index < 3u; ++index) {
        if (!imu_math_is_finite(calibration->accel_bias_sensor_mps2[index])) {
            return IMU_STATUS_CONFIG_ERROR;
        }
    }
    return IMU_STATUS_OK;
}

/**
 * @brief 把 MPU6050/MPU6500 原始帧转换为经过确定性标定的 B 系物理量。
 * @param calibration 加速度 b_a/C_a、陀螺温度模型/C_g 和安装旋转 R_BS。
 * @param device 提供本次量程对应的 LSB/g 与 LSB/(deg/s)。
 * @param raw 输入 S 系原始计数和时间信息。
 * @param dt_s 当前帧与上一有效帧间隔，单位 s；首帧允许传入 0。
 * @param sample 输出 B 系比力、尚未减 MEKF b_g 的角速度和芯片温度。
 * @return IMU_STATUS_OK 表示输出有限；参数非法或计算异常返回对应错误。
 * @note 温度模型只在拟合区间内求值，越界时钳位模型输入并保留实测温度，
 *       同时设置 IMU_SAMPLE_STATUS_TEMP_CLAMPED。函数不修改 calibration/device/raw。
 */
ImuStatus imu_calibration_apply(const ImuCalibration *calibration,
                                const ImuMpu6050 *device,
                                const ImuRawSample *raw,
                                float dt_s,
                                ImuSample *sample)
{
    ImuStatus calibration_status;
    float temperature_c;
    float model_temperature_c;
    float temperature_delta;
    float accel_sensor[3];
    float accel_unrotated[3];
    float gyro_sensor[3];
    float gyro_unrotated[3];
    float gyro_temperature_bias[3];
    uint32_t index;

    if (calibration == 0 || device == 0 || raw == 0 || sample == 0 ||
        !(device->accel_lsb_per_g > 0.0f) ||
        !(device->gyro_lsb_per_dps > 0.0f) ||
        !(device->temperature_lsb_per_c > 0.0f) ||
        !imu_math_is_finite(device->temperature_offset_c) ||
        !imu_math_is_finite(dt_s) || dt_s < 0.0f) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    calibration_status = imu_calibration_validate(calibration);
    if (calibration_status != IMU_STATUS_OK) {
        return calibration_status;
    }

    memset(sample, 0, sizeof(*sample));
    sample->sequence = raw->sequence;
    sample->timestamp_us = raw->timestamp_us;
    sample->dt_s = dt_s;
    sample->status = raw->status;

    /* 温度系数由设备驱动按 WHO_AM_I 选择，避免混用 6050/6500 公式。 */
    temperature_c = (float)raw->temperature_raw /
                    device->temperature_lsb_per_c +
                    device->temperature_offset_c;
    model_temperature_c = temperature_c;
    if (model_temperature_c < calibration->temperature_min_c) {
        model_temperature_c = calibration->temperature_min_c;
        sample->status |= IMU_SAMPLE_STATUS_TEMP_CLAMPED;
    } else if (model_temperature_c > calibration->temperature_max_c) {
        model_temperature_c = calibration->temperature_max_c;
        sample->status |= IMU_SAMPLE_STATUS_TEMP_CLAMPED;
    }
    sample->temperature_c = temperature_c;
    temperature_delta = model_temperature_c -
                        calibration->temperature_reference_c;

    /*
     * 预处理顺序固定为：原始计数 -> SI -> S 系零偏/温漂 ->
     * 确定性校正矩阵 -> R_BS。此处不减运行时 MEKF 偏置 b_g。
     */
    for (index = 0u; index < 3u; ++index) {
        accel_sensor[index] =
            ((float)raw->accel_raw[index] / device->accel_lsb_per_g) *
            IMU_STANDARD_GRAVITY_MPS2 -
            calibration->accel_bias_sensor_mps2[index];
        gyro_sensor[index] =
            ((float)raw->gyro_raw[index] / device->gyro_lsb_per_dps) *
            IMU_DEGREES_TO_RADIANS;
        gyro_temperature_bias[index] =
            calibration->gyro_temperature_coeff_sensor[index][0] +
            calibration->gyro_temperature_coeff_sensor[index][1] *
                temperature_delta +
            calibration->gyro_temperature_coeff_sensor[index][2] *
                temperature_delta * temperature_delta;
        gyro_sensor[index] -= gyro_temperature_bias[index];
    }

    imu_math_mat3_mul_vec3(calibration->accel_correction_matrix,
                           accel_sensor,
                           accel_unrotated);
    imu_math_mat3_mul_vec3(calibration->sensor_to_body,
                           accel_unrotated,
                           sample->specific_force_body_mps2);
    imu_math_mat3_mul_vec3(calibration->gyro_correction_matrix,
                           gyro_sensor,
                           gyro_unrotated);
    imu_math_mat3_mul_vec3(calibration->sensor_to_body,
                           gyro_unrotated,
                           sample->gyro_pre_body_rad_s);

    if (!imu_math_vec3_is_finite(sample->specific_force_body_mps2) ||
        !imu_math_vec3_is_finite(sample->gyro_pre_body_rad_s) ||
        !imu_math_is_finite(sample->temperature_c)) {
        return IMU_STATUS_NUMERIC_ERROR;
    }
    return IMU_STATUS_OK;
}
