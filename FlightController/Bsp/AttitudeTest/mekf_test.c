#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "f103_port.h"
#include "attitude_source.h"

I2C_HandleTypeDef hi2c2;
static uint32_t tick;
static int fail_read, reads, data_ready = 1;
static int who_am_i = 0x68, fail_who_reads, fail_reset_writes;
static int reset_writes, signal_resets, i2c_ops, delay_calls;
static uint32_t delays[16], retry_after_first_delay;
static int delay_ops[16], sample_reads;
static uint64_t first_sample_timestamp_us;
static int16_t accel_raw[3] = {0, 0, 2048};
static int16_t gyro_raw[3];

ImuStatus __real_imu_system_process_data_ready(ImuSystem *system, uint64_t timestamp_us);
ImuStatus __wrap_imu_system_process_data_ready(ImuSystem *system, uint64_t timestamp_us)
{
    if (sample_reads == 0) first_sample_timestamp_us = timestamp_us;
    return __real_imu_system_process_data_ready(system, timestamp_us);
}

static void encode_i16(uint8_t *dst, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    dst[0] = (uint8_t)(bits >> 8);
    dst[1] = (uint8_t)bits;
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms)
{
    if (delay_calls == 0u && i2c_ops != 0) retry_after_first_delay = 1;
    if (delay_calls < sizeof(delays) / sizeof(delays[0])) {
        delays[delay_calls] = ms;
        delay_ops[delay_calls] = i2c_ops;
    }
    ++delay_calls;
    tick += ms;
}
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                     uint16_t width, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(bus == &hi2c2 && addr == 0xd0 && width == 1 && timeout == 5);
    ++i2c_ops;
    ++reads;
    if (fail_read) return 1;
    memset(data, 0, len);
    if (reg == 0x75) {
        if (fail_who_reads != 0) {
            if (fail_who_reads > 0) --fail_who_reads;
            return 1;
        }
        data[0] = (uint8_t)who_am_i;
    }
    if (reg == 0x3a && len == 15) {
        int axis;
        ++sample_reads;
        data[0] = data_ready ? 1 : 0; /* INT_STATUS.DATA_RDY */
        for (axis = 0; axis < 3; ++axis) {
            encode_i16(&data[1 + 2 * axis], accel_raw[axis]);
            encode_i16(&data[9 + 2 * axis], gyro_raw[axis]);
        }
    }
    return HAL_OK;
}
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                      uint16_t width, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(bus == &hi2c2 && addr == 0xd0 && width == 1 && timeout == 5);
    ++i2c_ops;
    if (reg == 0x6b && len == 1 && data[0] == 0x80) {
        ++reset_writes;
        if (fail_reset_writes > 0) {
            --fail_reset_writes;
            return 1;
        }
    }
    if (reg == 0x68 && len == 1 && data[0] == 0x07) ++signal_resets;
    return HAL_OK;
}

static void test_calibration_units_and_rotation(void)
{
    ImuCalibration calibration;
    ImuMpu6050 device = {0};
    ImuRawSample raw = {0};
    ImuSample sample;
    imu_calibration_set_identity(&calibration);
    device.accel_lsb_per_g = 2048.0f;
    device.gyro_lsb_per_dps = 16.4f;
    device.temperature_lsb_per_c = 340.0f;
    device.temperature_offset_c = 36.53f;
    raw.accel_raw[0] = 2048; /* S 系 +X 方向 1 g */
    raw.gyro_raw[1] = 164;  /* S 系 +Y 方向 10 deg/s */
    raw.status = IMU_SAMPLE_STATUS_VALID;

    /* R_BS 为绕 Z 轴 +90°：S 的 +X -> B 的 +Y，+Y -> -X。 */
    calibration.sensor_to_body[0][0] = 0.0f;
    calibration.sensor_to_body[0][1] = -1.0f;
    calibration.sensor_to_body[1][0] = 1.0f;
    calibration.sensor_to_body[1][1] = 0.0f;
    assert(imu_calibration_apply(&calibration, &device, &raw, 0.005f, &sample) == IMU_STATUS_OK);
    assert(fabsf(sample.specific_force_body_mps2[0]) < 0.001f);
    assert(fabsf(sample.specific_force_body_mps2[1] - 9.80665f) < 0.001f);
    assert(fabsf(sample.gyro_pre_body_rad_s[0] + 0.17453293f) < 0.0001f);
    assert(fabsf(sample.gyro_pre_body_rad_s[1]) < 0.0001f);
}

int main(int argc, char **argv)
{
    ImuClock clock = { 0xfffffffdu, 0 };
    AttitudeOutput out = {0};
    int i, published = 0, count;
    assert(ImuClock_Update(&clock, 2) == 5000);
    assert(ImuClock_Update(&clock, 2) == 5000);
    test_calibration_units_and_rotation();

    if (argc > 1 && strcmp(argv[1], "init-retry") == 0) {
        fail_who_reads = 1;
    } else if (argc > 1 && strcmp(argv[1], "init-reset-retry") == 0) {
        fail_reset_writes = 1;
    } else if (argc > 1 && strcmp(argv[1], "init-bus-limit") == 0) {
        fail_who_reads = -1;
    } else if (argc > 1 && strcmp(argv[1], "init-wrong-id") == 0) {
        who_am_i = 0x00;
    }
    AttitudeSource_Init();

    /* The settle delay occurs once, after all configuration I2C operations,
       before any sample; failed initialization never takes this delay. */
    count = 0;
    for (i = 0; i < delay_calls; ++i) {
        if (delays[i] == IMU_STARTUP_SETTLE_WAIT_MS) {
            ++count;
            assert(i == delay_calls - 1 && delay_ops[i] == i2c_ops);
        }
    }
    assert(count == (AttitudeSource_IsFaulted() ? 0 : 1));
    assert(sample_reads == 0);

    if (argc == 1) {
        assert(!retry_after_first_delay && delay_calls == 4);
        assert(delays[0] == IMU_STARTUP_READY_WAIT_MS);
        assert(delays[1] == 100u && delays[2] == 100u);
        assert(reset_writes == 1 && signal_resets == 1);
        assert(!AttitudeSource_IsFaulted());
        count = i2c_ops;
        assert(!AttitudeSource_Read(&out) && i2c_ops == count);
        tick += IMU_SERVICE_PERIOD_MS - 1u;
        assert(!AttitudeSource_Read(&out) && i2c_ops == count);
        ++tick;
        assert(!AttitudeSource_Read(&out) && sample_reads == 1);
        assert(first_sample_timestamp_us == IMU_SERVICE_PERIOD_MS * 1000u);
        assert(!AttitudeSource_IsFaulted());
    }

    if (argc > 1 && strcmp(argv[1], "init-reset-retry") == 0) {
        assert(!retry_after_first_delay && delay_calls == 5);
        assert(delays[0] == IMU_STARTUP_READY_WAIT_MS);
        assert(delays[1] == IMU_STARTUP_RETRY_DELAY_MS);
        assert(delays[2] == 100u && delays[3] == 100u);
        assert(reset_writes == 2 && signal_resets == 1);
        assert(!AttitudeSource_IsFaulted());
        for (i = 0; i < 610; ++i) {
            tick += 5;
            if (AttitudeSource_Read(&out)) ++published;
        }
        assert(published > 0);
        puts("MEKF first reset write retry checks passed");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "init-retry") == 0) {
        assert(!retry_after_first_delay && delay_calls == 7);
        assert(delays[0] == IMU_STARTUP_READY_WAIT_MS);
        assert(delays[1] == 100u && delays[2] == 100u);
        assert(delays[3] == IMU_STARTUP_RETRY_DELAY_MS);
        assert(delays[4] == 100u && delays[5] == 100u);
        assert(reset_writes == 2 && signal_resets == 2);
        assert(!AttitudeSource_IsFaulted());
        for (i = 0; i < 610; ++i) {
            tick += 5;
            if (AttitudeSource_Read(&out)) ++published;
        }
        assert(published > 0);
        puts("MEKF bounded startup retry checks passed");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "init-bus-limit") == 0) {
        assert(!retry_after_first_delay);
        assert(delay_calls == 9);
        assert(delays[0] == IMU_STARTUP_READY_WAIT_MS);
        assert(delays[3] == IMU_STARTUP_RETRY_DELAY_MS);
        assert(delays[6] == IMU_STARTUP_RETRY_DELAY_MS);
        assert(reset_writes == IMU_STARTUP_INIT_MAX_ATTEMPTS);
        assert(signal_resets == IMU_STARTUP_INIT_MAX_ATTEMPTS);
        assert(AttitudeSource_IsFaulted());
        fail_who_reads = 0;
        count = i2c_ops;
        tick += IMU_SERVICE_PERIOD_MS;
        assert(!AttitudeSource_Read(&out) && i2c_ops == count);
        puts("MEKF bounded startup retry exhaustion checks passed");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "init-wrong-id") == 0) {
        assert(reset_writes == 1 && signal_resets == 1);
        assert(AttitudeSource_IsFaulted());
        who_am_i = 0x68;
        count = i2c_ops;
        tick += IMU_SERVICE_PERIOD_MS;
        assert(!AttitudeSource_Read(&out) && i2c_ops == count);
        puts("MEKF non-bus startup failure checks passed");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "startup-moving") == 0) {
        for (i = 0; i < 100; ++i) {
            gyro_raw[0] = (i & 1) ? 400 : -400;
            tick += 5;
            assert(!AttitudeSource_Read(&out));
        }
        assert(AttitudeSource_IsFaulted());
        count = reads; gyro_raw[0] = 0; tick += 5;
        assert(!AttitudeSource_Read(&out) && reads == count);
        puts("MEKF startup movement latch checks passed");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "stationary-tilted") == 0) {
        /* A fixed 30-degree roll is still stationary. */
        accel_raw[1] = 1024;
        accel_raw[2] = 1774;
        for (i = 0; i < 610; ++i) {
            tick += 5;
            if (AttitudeSource_Read(&out)) ++published;
        }
        assert(published > 0 && !AttitudeSource_IsFaulted());
        assert(fabsf(out.roll - 30.0f) < 0.2f);
        puts("MEKF tilted stationary startup checks passed");
        return 0;
    }

    if (argc > 1 && (strcmp(argv[1], "slow-roll") == 0 ||
                     strcmp(argv[1], "inverted") == 0 ||
                     strcmp(argv[1], "direction-below") == 0 ||
                     strcmp(argv[1], "direction-above") == 0)) {
        int inverted = strcmp(argv[1], "inverted") == 0;
        int slow = strcmp(argv[1], "slow-roll") == 0;
        int below = strcmp(argv[1], "direction-below") == 0;
        for (i = 0; i < 610; ++i) {
            float angle = slow ? 0.1f * (float)i * 0.005f :
                          (i < 100 ? 0.0f : (below ? 0.015f : 0.020f));
            accel_raw[1] = inverted ? 0 : (int16_t)lroundf(2048.0f * sinf(angle));
            accel_raw[2] = inverted ? -2048 : (int16_t)lroundf(2048.0f * cosf(angle));
            gyro_raw[0] = slow ? 94 : 0;
            tick += 5;
            if (AttitudeSource_Read(&out)) ++published;
            if (AttitudeSource_IsFaulted()) break;
        }
        if (below) {
            assert(published > 0 && !AttitudeSource_IsFaulted());
        } else {
            assert(!published && AttitudeSource_IsFaulted());
            count = reads; tick += 5;
            assert(!AttitudeSource_Read(&out) && reads == count);
        }
        puts("MEKF startup direction safety checks passed");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "bias") == 0) gyro_raw[0] = 82;
    if (argc > 1 && strcmp(argv[1], "slow-yaw") == 0) gyro_raw[2] = 100;
    for (i = 0; i < 610; ++i) {
        tick += 5;
        if (AttitudeSource_Read(&out)) ++published;
    }
    if (argc > 1 && strcmp(argv[1], "slow-yaw") == 0) {
        /* 独立的未通过验收：恒速 Yaw 与未知陀螺零偏无法仅靠六轴可靠区分。 */
        if (published != 0) {
            printf("FAIL: constant yaw accepted: frames=%d gz=%.3f fault=%d\n",
                   published, (double)out.gz, AttitudeSource_IsFaulted());
            return 1;
        }
        assert(AttitudeSource_IsFaulted());
        puts("constant yaw was rejected");
        return 0;
    }
    assert(published > 0 && isfinite(out.pitch) && fabsf(out.pitch) < 0.1f);
    assert(fabsf(out.roll) < 0.1f && fabsf(out.yaw) < 0.1f);
    if (argc > 1 && strcmp(argv[1], "runtime-inverted") == 0) {
        gyro_raw[0] = 470; /* ~0.5 rad/s */
        for (i = 0; i < 1600 && !AttitudeSource_IsFaulted(); ++i) {
            float angle = 0.5f * (float)(i + 1) * 0.005f;
            accel_raw[1] = (int16_t)lroundf(2048.0f * sinf(angle));
            accel_raw[2] = (int16_t)lroundf(2048.0f * cosf(angle));
            tick += 5;
            AttitudeSource_Read(&out);
        }
        assert(AttitudeSource_IsFaulted());
        count = reads; tick += 5;
        assert(!AttitudeSource_Read(&out) && reads == count);
        puts("MEKF runtime inverted latch checks passed");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "bias") == 0) {
        assert(fabsf(out.gx) < 0.05f);
        gyro_raw[0] = 246; /* 保持约 5 deg/s 初始零偏，再增加 10 deg/s。 */
        tick += 5;
        assert(AttitudeSource_Read(&out));
        assert(fabsf(out.gx - 10.0f) < 0.1f);
        puts("MEKF initial gyro bias and rate checks passed");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "late") == 0) {
        tick += 21;
        assert(!AttitudeSource_Read(&out) && AttitudeSource_IsFaulted());
        puts("MEKF excessive sample interval latch checks passed");
        return 0;
    }
    /* 独立进程分别测试 I2C 失败和 DATA_RDY 未置位；两者都必须锁存。 */
    if (argc > 1 && strcmp(argv[1], "no-ready") == 0) data_ready = 0;
    else fail_read = 1;
    tick += 5;
    assert(!AttitudeSource_Read(&out));
    assert(AttitudeSource_IsFaulted());
    count = reads; fail_read = 0; data_ready = 1; tick += 5;
    assert(!AttitudeSource_Read(&out) && reads == count);
    puts("MEKF adapter, wrap and fatal latch checks passed");
    return 0;
}
