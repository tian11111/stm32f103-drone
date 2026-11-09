#include "imu_system.h"
#include "imu_math_internal.h"

#include <string.h>
#include <math.h>

/**
 * @brief 用与 MEKF 相同的右乘约定推进纯陀螺诊断四元数。
 * @param system 已完成启动调零的系统实例。
 * @param gyro_pre_body_rad_s 本帧尚未减 b_g 的 B 系角速度，单位 rad/s。
 * @param dt_s 本帧积分周期，单位 s。
 * @return true 姿态成功推进并归一化；false 输入或结果出现数值错误。
 * @note 使用推进前的同一个 MEKF b_g，但不接受比力、ZARU 或其他观测更新，
 *       因而可区分陀螺传播误差与 MEKF measurement update 引入的误差。
 */
static bool imu_system_predict_gyro_only(
    ImuSystem *system,
    const float gyro_pre_body_rad_s[3],
    float dt_s)
{
    ImuQuaternion quaternion;
    ImuQuaternion delta_quaternion;
    float rotation_vector_rad[3];
    uint32_t axis;

    quaternion.w = system->gyro_only_quaternion_wb[0];
    quaternion.x = system->gyro_only_quaternion_wb[1];
    quaternion.y = system->gyro_only_quaternion_wb[2];
    quaternion.z = system->gyro_only_quaternion_wb[3];
    for (axis = 0u; axis < 3u; ++axis) {
        rotation_vector_rad[axis] =
            (gyro_pre_body_rad_s[axis] -
             system->mekf.gyro_bias_body_rad_s[axis]) * dt_s;
    }
    delta_quaternion = imu_math_quaternion_exp(rotation_vector_rad);
    quaternion = imu_math_quaternion_multiply(quaternion,
                                               delta_quaternion);
    if (!imu_math_quaternion_normalize(&quaternion)) {
        return false;
    }
    system->gyro_only_quaternion_wb[0] = quaternion.w;
    system->gyro_only_quaternion_wb[1] = quaternion.x;
    system->gyro_only_quaternion_wb[2] = quaternion.y;
    system->gyro_only_quaternion_wb[3] = quaternion.z;
    return true;
}

/**
 * @brief 发布一帧原始数据并生成与其严格对应的预处理物理量。
 * @param system 已初始化系统实例。
 * @param raw 已由同步读取或异步读取完整解析的原始帧。
 * @return IMU_STATUS_OK 两级快照均发布；否则返回时间或标定错误。
 * @note raw 总会先发布；标定失败不会用失败帧覆盖 latest_sample。
 */
static ImuStatus imu_system_publish_raw(ImuSystem *system,
                                        const ImuRawSample *raw)
{
    ImuStatus status;
    ImuSample sample;
    ImuStationaryResult stationary_result;
    float dt_s = 0.0f;
    uint32_t axis;

    if (system->raw_valid && raw->timestamp_us <= system->previous_timestamp_us) {
        return IMU_STATUS_TIME_ERROR;
    }
    if (system->raw_valid) {
        dt_s = (float)(raw->timestamp_us - system->previous_timestamp_us) * 1.0e-6f;
    }

    system->latest_raw = *raw;
    system->raw_valid = true;
    system->previous_timestamp_us = raw->timestamp_us;

    status = imu_calibration_apply(&system->calibration,
                                   &system->sensor,
                                   raw,
                                   dt_s,
                                   &sample);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    system->latest_sample = sample;
    system->sample_valid = true;

    status = imu_stationary_update(&system->stationary_detector,
                                   &sample,
                                   system->output.gyro_bias_body_rad_s,
                                   system->output_valid,
                                   !system->output_valid,
                                   &stationary_result);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    system->stationary_result = stationary_result;

    if (!system->output_valid && stationary_result.window_ready &&
        stationary_result.state == IMU_MOTION_ANGULAR_STATIC) {
        float lateral_norm;

        memset(&system->output, 0, sizeof(system->output));
        for (axis = 0u; axis < 3u; ++axis) {
            system->output.gyro_bias_body_rad_s[axis] =
                stationary_result.gyro_pre_mean_rad_s[axis];
        }
        lateral_norm = sqrtf(
            stationary_result.specific_force_mean_mps2[1] *
            stationary_result.specific_force_mean_mps2[1] +
            stationary_result.specific_force_mean_mps2[2] *
            stationary_result.specific_force_mean_mps2[2]);
        system->output.initial_roll_rad = atan2f(
            stationary_result.specific_force_mean_mps2[1],
            stationary_result.specific_force_mean_mps2[2]);
        system->output.initial_pitch_rad = atan2f(
            -stationary_result.specific_force_mean_mps2[0], lateral_norm);
        status = imu_mekf_init(
            &system->mekf,
            &system->mekf.config,
            stationary_result.specific_force_mean_mps2,
            system->output.gyro_bias_body_rad_s);
        if (status != IMU_STATUS_OK) {
            return status;
        }
        for (axis = 0u; axis < 4u; ++axis) {
            system->gyro_only_quaternion_wb[axis] =
                system->mekf.quaternion_wb[axis];
        }
        system->output_valid = true;
    } else if (system->output_valid) {
        status = imu_mekf_predict(&system->mekf,
                                  sample.gyro_pre_body_rad_s,
                                  sample.dt_s);
        if (status != IMU_STATUS_OK) {
            return status;
        }
        if (!imu_system_predict_gyro_only(system,
                                          sample.gyro_pre_body_rad_s,
                                          sample.dt_s)) {
            return IMU_STATUS_NUMERIC_ERROR;
        }
        /*
         * 比力模长在匀速转动时仍可稳定，不能单独证明方向代表当前重力。
         * 仅使用同时通过三轴角速度、峰峰值和比力稳定性检查的静止窗口，
         * 避免把 200 点运动均值作为窗口末端的当前姿态注入 MEKF。
         */
        if (stationary_result.window_ready &&
            stationary_result.candidate_static &&
            stationary_result.accel_norm_std_mps2 <=
                system->stationary_detector.config.accel_norm_std_max_mps2) {
            status = imu_mekf_update_specific_force_direction(
                &system->mekf,
                stationary_result.specific_force_mean_mps2);
            if (status < IMU_STATUS_OK) {
                return status;
            }
        }
        if (stationary_result.window_ready &&
            stationary_result.candidate_static &&
            stationary_result.state == IMU_MOTION_ANGULAR_STATIC) {
            status = imu_mekf_update_zero_angular_rate(
                &system->mekf,
                stationary_result.gyro_pre_mean_rad_s);
            if (status < IMU_STATUS_OK) {
                return status;
            }
        }
        if (system->yaw_rate_provider != 0) {
            ImuYawRateObservation observation = {0};

            if (system->yaw_rate_provider(system->yaw_rate_provider_context,
                                          sample.timestamp_us,
                                          &observation) &&
                observation.valid &&
                observation.semantic == IMU_YAW_RATE_BODY_Z &&
                imu_math_is_finite(observation.rate_rad_s) &&
                imu_math_is_finite(observation.variance_rad2_s2) &&
                observation.variance_rad2_s2 > 0.0f &&
                (!system->yaw_rate_source_sequence_valid ||
                 observation.source_sequence !=
                     system->last_yaw_rate_source_sequence)) {
                status = imu_mekf_update_body_yaw_rate(
                    &system->mekf,
                    sample.gyro_pre_body_rad_s,
                    observation.rate_rad_s,
                    observation.variance_rad2_s2);
                if (status < IMU_STATUS_OK) {
                    return status;
                }
                if (status == IMU_STATUS_OK) {
                    system->last_yaw_rate_source_sequence =
                        observation.source_sequence;
                    system->yaw_rate_source_sequence_valid = true;
                }
            }
        }
    }
    if (system->output_valid) {
        ImuQuaternion mekf_quaternion;
        ImuQuaternion gyro_only_quaternion;
        float lateral_norm = sqrtf(
            sample.specific_force_body_mps2[1] *
            sample.specific_force_body_mps2[1] +
            sample.specific_force_body_mps2[2] *
            sample.specific_force_body_mps2[2]);

        if (!imu_mekf_get_euler_rad(&system->mekf,
                                     &system->output.roll_rad,
                                     &system->output.pitch_rad,
                                     &system->output.yaw_rad)) {
            return IMU_STATUS_NUMERIC_ERROR;
        }
        mekf_quaternion.w = system->mekf.quaternion_wb[0];
        mekf_quaternion.x = system->mekf.quaternion_wb[1];
        mekf_quaternion.y = system->mekf.quaternion_wb[2];
        mekf_quaternion.z = system->mekf.quaternion_wb[3];
        gyro_only_quaternion.w = system->gyro_only_quaternion_wb[0];
        gyro_only_quaternion.x = system->gyro_only_quaternion_wb[1];
        gyro_only_quaternion.y = system->gyro_only_quaternion_wb[2];
        gyro_only_quaternion.z = system->gyro_only_quaternion_wb[3];
        system->output.mekf_fused_yaw_rad =
            imu_math_quaternion_get_fused_yaw_rad(mekf_quaternion);
        system->output.gyro_only_fused_yaw_rad =
            imu_math_quaternion_get_fused_yaw_rad(gyro_only_quaternion);
        system->output.gyro_only_pitch_rad =
            imu_math_quaternion_get_zyx_pitch_rad(gyro_only_quaternion);
        if (!imu_mekf_get_predicted_specific_force_direction_body(
                &system->mekf,
                system->output.predicted_specific_force_direction_body)) {
            return IMU_STATUS_NUMERIC_ERROR;
        }
        if (!imu_mekf_get_gravity_tilt_rad(
                &system->mekf,
                &system->output.gravity_tilt_roll_rad,
                &system->output.gravity_tilt_pitch_rad,
                &system->output.upright_hemisphere)) {
            return IMU_STATUS_NUMERIC_ERROR;
        }
        system->output.sequence = sample.sequence;
        system->output.timestamp_us = sample.timestamp_us;
        system->output.temperature_c = sample.temperature_c;
        system->output.specific_force_update_count =
            system->mekf.specific_force_update_count;
        system->output.zaru_update_count = system->mekf.zaru_update_count;
        system->output.state_flags = IMU_OUTPUT_STATE_READY |
                                     IMU_OUTPUT_STATE_ATTITUDE_VALID;
        /*
         * 保留单帧比力倾角作为独立诊断，不参与 MEKF 状态反馈：
         * roll=atan2(f_y,f_z)，pitch=atan2(-f_x,sqrt(f_y^2+f_z^2))。
         * 车辆存在平动加速度时该诊断不是可信姿态，禁止用于正式控制。
         */
        system->output.accel_roll_rad = atan2f(
            sample.specific_force_body_mps2[1],
            sample.specific_force_body_mps2[2]);
        system->output.accel_pitch_rad = atan2f(
            -sample.specific_force_body_mps2[0], lateral_norm);
        if (stationary_result.state == IMU_MOTION_ANGULAR_STATIC) {
            system->output.state_flags |= IMU_OUTPUT_STATE_STATIC;
        }
        for (axis = 0u; axis < 3u; ++axis) {
            system->output.quaternion_wb[axis] =
                system->mekf.quaternion_wb[axis];
            system->output.gyro_bias_body_rad_s[axis] =
                system->mekf.gyro_bias_body_rad_s[axis];
            system->output.gyro_corrected_body_rad_s[axis] =
                sample.gyro_pre_body_rad_s[axis] -
                system->output.gyro_bias_body_rad_s[axis];
            system->output.attitude_error_variance_rad2[axis] =
                system->mekf.covariance[axis][axis];
        }
        system->output.quaternion_wb[3] = system->mekf.quaternion_wb[3];
    }
    return IMU_STATUS_OK;
}

/**
 * @brief 填充 IMU 驱动、标定、静止检测和 MEKF 的默认配置。
 * @param config 输出系统配置。
 * @note MPU6050 默认 1 kHz、94 Hz DLPF；标定矩阵为单位阵，仅用于先采集
 *       原始数据，不能替代六面加速度标定和陀螺温度标定。
 */
void imu_system_default_config(ImuSystemConfig *config)
{
    if (config == 0) {
        return;
    }
    imu_mpu6050_default_config(&config->sensor);
    imu_calibration_set_identity(&config->calibration);
    imu_stationary_default_config(&config->stationary);
    imu_mekf_default_config(&config->mekf);
}

/**
 * @brief 初始化无依赖 IMU 核心、确定性标定和 MPU6050 寄存器。
 * @param system 调用者持有的系统实例，函数开始时清零。
 * @param bus 使用者注入的同步总线和初始化延时回调。
 * @param config 传感器与确定性标定配置。
 * @return IMU_STATUS_OK 初始化完成；否则返回参数、标定、总线或器件错误。
 * @note 本函数会触发 MPU6050 复位并阻塞约 110 ms，只能在前台启动初始化时调用。
 *       失败时 raw_valid/sample_valid 均保持 false，不发布伪造样本。
 */
ImuStatus imu_system_init(ImuSystem *system,
                          const ImuBus *bus,
                          const ImuSystemConfig *config)
{
    ImuStatus status;

    if (system == 0 || bus == 0 || config == 0) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    status = imu_calibration_validate(&config->calibration);
    if (status != IMU_STATUS_OK) {
        return status;
    }

    memset(system, 0, sizeof(*system));
    system->calibration = config->calibration;
    system->mekf.config = config->mekf;
    status = imu_stationary_init(&system->stationary_detector,
                                 &config->stationary);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_init(&system->sensor, bus, &config->sensor);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    return IMU_STATUS_OK;
}

/**
 * @brief 在一次 DATA_RDY 事件后读取原始帧并生成预处理物理量。
 * @param system 已成功初始化的系统实例。
 * @param timestamp_us 本帧数据就绪或前台开始读取的时刻，单位 us，必须递增。
 * @return IMU_STATUS_OK 原始帧和预处理帧均已发布；否则返回明确失败状态。
 * @note 每个 DATA_RDY 最多调用一次。总线成功后立即发布 raw；若后续标定失败，
 *       raw 仍可由调试接口读取，但 sample_valid 不会被新失败帧覆盖。
 *       本函数执行同步 I2C，禁止从 EXTI 中断直接调用。
 */
ImuStatus imu_system_process_data_ready(ImuSystem *system,
                                        uint64_t timestamp_us)
{
    ImuStatus status;
    ImuRawSample raw;

    if (system == 0 || !system->sensor.initialized) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    if (system->raw_valid && timestamp_us <= system->previous_timestamp_us) {
        return IMU_STATUS_TIME_ERROR;
    }

    status = imu_mpu6050_read_raw(&system->sensor, timestamp_us, &raw);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    return imu_system_publish_raw(system, &raw);
}

/**
 * @brief 为一次 DATA_RDY 启动无阻塞原始帧读取。
 * @param system 已初始化且总线实现异步回调的系统实例。
 * @param timestamp_us DATA_RDY 对应时间戳，单位 us，必须晚于已发布帧。
 * @return IMU_STATUS_OK 异步读取已启动；否则返回调用顺序或总线错误。
 * @note 本函数不等待 DMA 完成，也不发布样本。
 */
ImuStatus imu_system_start_data_ready(ImuSystem *system,
                                      uint64_t timestamp_us)
{
    if (system == 0 || !system->sensor.initialized) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    if (system->raw_valid && timestamp_us <= system->previous_timestamp_us) {
        return IMU_STATUS_TIME_ERROR;
    }
    return imu_mpu6050_start_raw_read(&system->sensor, timestamp_us);
}

/**
 * @brief 查询异步读取并在完成后发布 raw 与预处理样本。
 * @param system 已调用 start_data_ready 的系统实例。
 * @return NO_DATA 表示 DMA 仍忙；OK 表示已发布；负值表示读取或预处理失败。
 * @note 调用者应高频轮询；解析、单位换算和标定均在前台而非 DMA ISR 中完成。
 */
ImuStatus imu_system_finish_data_ready(ImuSystem *system)
{
    ImuStatus status;
    ImuRawSample raw;

    if (system == 0 || !system->sensor.initialized) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    status = imu_mpu6050_finish_raw_read(&system->sensor, &raw);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    return imu_system_publish_raw(system, &raw);
}

/**
 * @brief 注册或清除外部车体系 Z 轴角速度提供者。
 *
 * @param system IMU 系统实例。
 * @param provider 提供者回调；传入空指针表示禁用外部辅助。
 * @param context 原样传给 provider 的用户上下文。
 * @note provider 在前台处理新 IMU 帧时调用。只有新的有效 BODY_Z 观测才会
 *       更新 b_g,z；回调不得阻塞，也不得重复使用 source_sequence。
 */
void imu_system_set_yaw_rate_provider(ImuSystem *system,
                                      ImuYawRateProvider provider,
                                      void *context)
{
    if (system == 0) {
        return;
    }
    system->yaw_rate_provider = provider;
    system->yaw_rate_provider_context = context;
    system->yaw_rate_source_sequence_valid = false;
    system->last_yaw_rate_source_sequence = 0u;
}

/**
 * @brief 复制最近一次完整 MPU6050 原始帧。
 * @param system IMU 系统实例。
 * @param raw_sample 输出原始计数、时间戳、序号和状态位。
 * @return true 已经成功读取过至少一帧；false 参数为空或尚无原始数据。
 * @note 本接口不触发 I2C，也不清除新数据状态；同一序号可重复读取。
 *       默认要求与 process_data_ready 在同一前台上下文调用。
 */
bool imu_system_get_latest_raw(const ImuSystem *system,
                               ImuRawSample *raw_sample)
{
    if (system == 0 || raw_sample == 0 || !system->raw_valid) {
        return false;
    }
    *raw_sample = system->latest_raw;
    return true;
}

/**
 * @brief 复制最近一次与原始帧对应的预处理 B 系物理量。
 * @param system IMU 系统实例。
 * @param sample 输出比力、omega_pre、温度、dt、序号和状态位。
 * @return true 已经成功生成过至少一帧；false 参数为空或预处理尚未成功。
 * @note 本接口不减 MEKF b_g；gyro_pre_body_rad_s 仍是偏置状态观测输入。
 */
bool imu_system_get_latest_sample(const ImuSystem *system,
                                  ImuSample *sample)
{
    if (system == 0 || sample == 0 || !system->sample_valid) {
        return false;
    }
    *sample = system->latest_sample;
    return true;
}

/**
 * @brief 复制最近一次完整的非重叠静止检测窗口结果。
 * @param system IMU 系统实例。
 * @param result 输出窗口末帧序号、运动状态和统计量。
 * @return true 当前保存的是刚完成的窗口；false 参数为空或本帧未完成窗口。
 * @note 本接口只读且无消费语义。调用者应使用 sequence 去重；下一帧处理后
 *       window_ready 会恢复为 false，禁止把旧统计量误认为新窗口。
 */
bool imu_system_get_stationary_result(const ImuSystem *system,
                                      ImuStationaryResult *result)
{
    if (system == 0 || result == 0 ||
        !system->stationary_result.window_ready) {
        return false;
    }
    *result = system->stationary_result;
    return true;
}

/**
 * @brief 复制启动静止调零后的处理结果。
 * @param system IMU 系统实例。
 * @param output 输出 q_WB、欧拉角、误差方差、唯一陀螺偏置和减偏角速度。
 * @return true 启动静止标定已完成；false 尚在标定或参数为空。
 * @note initial_roll/pitch 仅记录启动条件；实时姿态以 q_WB 和欧拉角为准。
 */
bool imu_system_get_output(const ImuSystem *system, ImuOutput *output)
{
    if (system == 0 || output == 0 || !system->output_valid) {
        return false;
    }
    *output = system->output;
    return true;
}
