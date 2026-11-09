#include "f103_port.h"
#include "i2c.h"
#include <string.h>

/* INT_STATUS 紧邻 14 字节传感器数据；同一突发读取可检查新数据标志。 */
#define MPU_REG_INT_STATUS       0x3au
#define MPU_REG_ACCEL_XOUT_H     0x3bu
#define MPU_INT_DATA_READY       0x01u
#define MPU_RAW_FRAME_SIZE       14u

uint64_t ImuClock_Update(ImuClock *clock, uint32_t tick)
{
    clock->elapsed_ms += (uint32_t)(tick - clock->last_tick);
    clock->last_tick = tick;
    return clock->elapsed_ms * 1000u;
}

static ImuStatus bus_read(void *context, uint8_t address, uint8_t reg,
                          uint8_t *data, uint16_t length)
{
    if (!context || address > 0x7fu || !data || !length) return IMU_STATUS_INVALID_ARGUMENT;
    if (reg == MPU_REG_ACCEL_XOUT_H && length == MPU_RAW_FRAME_SIZE) {
        uint8_t frame[MPU_RAW_FRAME_SIZE + 1u];
        if (HAL_I2C_Mem_Read(context, (uint16_t)address << 1,
                             MPU_REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT,
                             frame, sizeof frame, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
            return IMU_STATUS_BUS_ERROR;
        }
        /* 当前服务以 5 ms 读取 1 kHz 器件；没有 DATA_RDY 时不发布旧帧。 */
        if ((frame[0] & MPU_INT_DATA_READY) == 0u) return IMU_STATUS_NO_DATA;
        memcpy(data, &frame[1], MPU_RAW_FRAME_SIZE);
        return IMU_STATUS_OK;
    }
    return HAL_I2C_Mem_Read(context, (uint16_t)address << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, length,
                            IMU_I2C_TIMEOUT_MS) == HAL_OK ? IMU_STATUS_OK : IMU_STATUS_BUS_ERROR;
}

static ImuStatus bus_write(void *context, uint8_t address, uint8_t reg,
                           const uint8_t *data, uint16_t length)
{
    if (!context || address > 0x7fu || !data || !length) return IMU_STATUS_INVALID_ARGUMENT;
    return HAL_I2C_Mem_Write(context, (uint16_t)address << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length,
                             IMU_I2C_TIMEOUT_MS) == HAL_OK ? IMU_STATUS_OK : IMU_STATUS_BUS_ERROR;
}

static void bus_delay(void *context, uint32_t delay_ms)
{
    (void)context;
    HAL_Delay(delay_ms);
}

ImuBus ImuF103_Bus(void)
{
    ImuBus bus = { &hi2c2, bus_read, bus_write, bus_delay, 0, 0 };
    return bus;
}

void ImuF103_Config(ImuSystemConfig *config)
{
    /* Core defaults include identity sensor-to-body and calibration matrices,
       stationary windows and MEKF noise. Replace only after device calibration. */
    imu_system_default_config(config);
    config->sensor.address_7bit = IMU_ADDRESS_7BIT;
    config->sensor.sample_rate_hz = 1000u;
    config->sensor.gyro_range = IMU_MPU6050_GYRO_2000_DPS;
    config->sensor.accel_range = IMU_MPU6050_ACCEL_16_G;
    config->sensor.dlpf = IMU_MPU6050_DLPF_94_HZ;
    config->stationary.window_samples = IMU_STATIONARY_WINDOW_SAMPLES;
    ImuF103_ConfigureCalibration(&config->calibration);
}

void ImuF103_ConfigureCalibration(ImuCalibration *calibration)
{
    /*
     * TODO(硬件): 实机确认 MPU6050 的 X/Y/Z 朝向和符号后，设置 R_BS。
     * 当前配置明确采用单位矩阵，不声称现有 DMP 的安装方向已经验证。
     */
    imu_calibration_set_identity(calibration);
}
