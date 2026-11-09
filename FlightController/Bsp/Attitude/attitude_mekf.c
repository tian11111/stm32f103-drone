#include "attitude_source.h"
#include "f103_port.h"
#include "main.h"
#include <math.h>

static ImuSystem system_state;
static ImuClock clock_state;
static uint32_t previous_tick;
static uint32_t previous_sequence;
static bool initialized, active, sampled, startup_direction_valid;
static volatile bool failed;
static float startup_direction[3];

static bool startup_direction_ok(const float force[3])
{
    float direction[3], norm, cross[3], sine, cosine;
    unsigned i;
    for (i = 0; i < 3; ++i) if (!isfinite(force[i])) return false;
    norm = sqrtf(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]);
    if (!isfinite(norm) || norm <= 0.0f) return false;
    for (i = 0; i < 3; ++i) direction[i] = force[i] / norm;
    if (!startup_direction_valid) {
        for (i = 0; i < 3; ++i) startup_direction[i] = direction[i];
        startup_direction_valid = true;
        return true;
    }
    cross[0] = startup_direction[1]*direction[2] - startup_direction[2]*direction[1];
    cross[1] = startup_direction[2]*direction[0] - startup_direction[0]*direction[2];
    cross[2] = startup_direction[0]*direction[1] - startup_direction[1]*direction[0];
    sine = sqrtf(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]);
    cosine = startup_direction[0]*direction[0] + startup_direction[1]*direction[1] + startup_direction[2]*direction[2];
    return isfinite(sine) && isfinite(cosine) &&
           atan2f(sine, cosine) <= IMU_STARTUP_DIRECTION_LIMIT_RAD;
}

void AttitudeSource_Init(void)
{
    ImuSystemConfig config;
    ImuBus bus = ImuF103_Bus();
    ImuStatus status = IMU_STATUS_BUS_ERROR;
    uint32_t attempt;

    ImuF103_Config(&config);
    active = sampled = startup_direction_valid = false;
    previous_sequence = 0u;
    clock_state.elapsed_ms = 0u;
    /* 仅在上电初始化时等待并重试总线错误；运行中的故障仍然锁存。 */
    HAL_Delay(IMU_STARTUP_READY_WAIT_MS);
    for (attempt = 1u; attempt <= IMU_STARTUP_INIT_MAX_ATTEMPTS; ++attempt) {
        status = imu_system_init(&system_state, &bus, &config);
        if (status != IMU_STATUS_BUS_ERROR ||
            attempt == IMU_STARTUP_INIT_MAX_ATTEMPTS) {
            break;
        }
        HAL_Delay(IMU_STARTUP_RETRY_DELAY_MS);
    }
    if (status == IMU_STATUS_OK) {
        /* 尚未采样，姿态无效；初始化返回前 PWM 和控制定时器尚未启动。 */
        HAL_Delay(IMU_STARTUP_SETTLE_WAIT_MS);
    }
    initialized = status == IMU_STATUS_OK;
    failed = !initialized;
    clock_state.last_tick = HAL_GetTick();
    previous_tick = clock_state.last_tick;
}

bool AttitudeSource_IsFaulted(void)
{
    return failed;
}

bool AttitudeSource_Read(AttitudeOutput *output)
{
    ImuOutput result;
    uint32_t tick = HAL_GetTick();
    uint32_t elapsed = tick - previous_tick;
    uint64_t timestamp;
    if (!output || !initialized || failed || elapsed < IMU_SERVICE_PERIOD_MS) return false;
    previous_tick = tick;
    timestamp = ImuClock_Update(&clock_state, tick);
    if ((sampled && elapsed > 20u) ||
        imu_system_process_data_ready(&system_state, timestamp) != IMU_STATUS_OK) {
        failed = true;
        return false;
    }
    sampled = true;
    /* 检查启动阶段每个完整候选窗口，包括核心刚生成首帧的窗口。 */
    if (!active && system_state.stationary_result.window_ready &&
        system_state.stationary_result.candidate_static &&
        !startup_direction_ok(system_state.stationary_result.specific_force_mean_mps2)) {
        failed = true;
        return false;
    }
    if (!imu_system_get_output(&system_state, &result)) {
        /*
         * 上电后一旦完成运动窗口，就不再接受本次启动的静止标定。
         * 用户必须让飞行器静止后重新上电，避免运行中重标定或静默恢复。
         */
        if (system_state.stationary_result.window_ready &&
            system_state.stationary_result.state == IMU_MOTION_MOVING) {
            failed = true;
        }
        return false;
    }
    if (!result.upright_hemisphere ||
        (active && result.sequence == previous_sequence) ||
        !isfinite(result.gravity_tilt_pitch_rad) || !isfinite(result.gravity_tilt_roll_rad) ||
        !isfinite(result.mekf_fused_yaw_rad) ||
        !isfinite(result.gyro_corrected_body_rad_s[0]) ||
        !isfinite(result.gyro_corrected_body_rad_s[1]) ||
        !isfinite(result.gyro_corrected_body_rad_s[2])) {
        failed = true;
        return false;
    }
    output->pitch = result.gravity_tilt_pitch_rad * IMU_RAD_TO_DEG;
    output->roll = result.gravity_tilt_roll_rad * IMU_RAD_TO_DEG;
    output->yaw = result.mekf_fused_yaw_rad * IMU_RAD_TO_DEG;
    output->gx = result.gyro_corrected_body_rad_s[0] * IMU_RAD_TO_DEG;
    output->gy = result.gyro_corrected_body_rad_s[1] * IMU_RAD_TO_DEG;
    output->gz = result.gyro_corrected_body_rad_s[2] * IMU_RAD_TO_DEG;
    output->sequence = previous_sequence = result.sequence;
    output->tick = tick;
    active = true;
    return true;
}
