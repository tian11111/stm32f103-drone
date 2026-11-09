#include "imu_mpu6050.h"

#include <string.h>

#define MPU6050_REG_SMPLRT_DIV  (0x19u)
#define MPU6050_REG_CONFIG      (0x1Au)
#define MPU6050_REG_GYRO_CONFIG (0x1Bu)
#define MPU6050_REG_ACCEL_CONFIG (0x1Cu)
#define MPU6500_REG_ACCEL_CONFIG_2 (0x1Du)
#define MPU6050_REG_FIFO_EN     (0x23u)
#define MPU6050_REG_INT_PIN_CFG (0x37u)
#define MPU6050_REG_INT_ENABLE  (0x38u)
#define MPU6050_REG_ACCEL_XOUT_H (0x3Bu)
#define MPU6050_REG_GYRO_XOUT_H (0x43u)
#define MPU6050_REG_USER_CTRL   (0x6Au)
#define MPU6050_REG_SIGNAL_PATH_RESET (0x68u)
#define MPU6050_REG_PWR_MGMT_1  (0x6Bu)
#define MPU6050_REG_PWR_MGMT_2  (0x6Cu)
#define MPU6050_REG_WHO_AM_I    (0x75u)

#define MPU6050_PWR_DEVICE_RESET (0x80u)
#define MPU6050_CLOCK_PLL_X_GYRO (0x01u)
#define MPU6050_RESET_ALL_SIGNAL_PATHS (0x07u)
#define MPU6050_INT_READ_CLEAR   (0x10u)
#define MPU6050_INT_DATA_READY   (0x01u)
#define MPU6050_WHO_AM_I_VALUE   (0x68u)
#define MPU6500_WHO_AM_I_VALUE   (0x70u)

#define MPU6050_TEMP_LSB_PER_C   (340.0f)
#define MPU6050_TEMP_OFFSET_C    (36.53f)
#define MPU6500_TEMP_LSB_PER_C   (333.87f)
#define MPU6500_TEMP_OFFSET_C    (21.0f)

/**
 * @brief 通过抽象总线写入一个 MPU6050 寄存器。
 * @param device 已绑定总线和 7 位地址的设备实例。
 * @param register_address 目标寄存器地址。
 * @param value 待写入的单字节值。
 * @return 底层总线返回状态；本函数不重试、不复位 I2C，也不隐藏失败。
 */
static ImuStatus imu_mpu6050_write_u8(const ImuMpu6050 *device,
                                      uint8_t register_address,
                                      uint8_t value)
{
    return device->bus.write(device->bus.context,
                             device->config.address_7bit,
                             register_address,
                             &value,
                             1u);
}

/**
 * @brief 验证 MPU6050 地址、量程、DLPF 和采样分频能否精确表示。
 * @param config 待验证配置。
 * @return true 配置可直接写入 8 位 SMPLRT_DIV；false 配置非法或需取整。
 * @note DLPF=260 Hz 时陀螺输出基频按 8 kHz，其余档位按 1 kHz。
 */
static bool imu_mpu6050_config_is_valid(const ImuMpu6050Config *config)
{
    uint32_t base_rate_hz;
    uint32_t divider_plus_one;

    if (config == 0 ||
        (config->address_7bit != 0x68u && config->address_7bit != 0x69u) ||
        config->gyro_range > IMU_MPU6050_GYRO_2000_DPS ||
        config->accel_range > IMU_MPU6050_ACCEL_16_G ||
        config->dlpf > IMU_MPU6050_DLPF_5_HZ ||
        config->sample_rate_hz == 0u) {
        return false;
    }

    base_rate_hz = (config->dlpf == IMU_MPU6050_DLPF_260_HZ) ? 8000u : 1000u;
    if (config->sample_rate_hz > base_rate_hz ||
        (base_rate_hz % config->sample_rate_hz) != 0u) {
        return false;
    }
    divider_plus_one = base_rate_hz / config->sample_rate_hz;
    return divider_plus_one >= 1u && divider_plus_one <= 256u;
}

/**
 * @brief 将 MPU6050 高字节在前的二进制补码解码为 int16_t。
 * @param bytes 指向连续两个字节，高字节在 bytes[0]。
 * @return 解码后的有符号原始计数。
 */
static int16_t imu_mpu6050_decode_i16(const uint8_t *bytes)
{
    uint16_t value = ((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1];
    return (int16_t)value;
}

/**
 * @brief 判断原始计数是否到达 int16_t 数字边界。
 * @param value 单轴原始补码计数。
 * @return true 到达正或负数字边界；false 未触发本项饱和诊断。
 * @note 该检查只能发现数字满码，不能替代结合量程和动态范围的物理饱和判断。
 */
static bool imu_mpu6050_is_saturated(int16_t value)
{
    return value == INT16_MIN || value == INT16_MAX;
}

/**
 * @brief 把完整 14 字节寄存器快照解析为带序号的原始样本。
 * @param device 设备状态，成功时递增 sample_sequence。
 * @param bytes 连续 Acc/Temp/Gyro 字节，必须来自同一次突发读取。
 * @param timestamp_us 与该帧对应的 DATA_RDY 时间戳，单位 us。
 * @param sample 输出原始样本。
 * @return IMU_STATUS_OK；任一指针为空返回参数错误。
 */
static ImuStatus imu_mpu6050_decode_raw(ImuMpu6050 *device,
                                         const uint8_t *bytes,
                                         uint64_t timestamp_us,
                                         ImuRawSample *sample)
{
    uint32_t index;

    if (device == 0 || bytes == 0 || sample == 0) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    sample->sequence = ++device->sample_sequence;
    sample->timestamp_us = timestamp_us;
    sample->accel_raw[0] = imu_mpu6050_decode_i16(&bytes[0]);
    sample->accel_raw[1] = imu_mpu6050_decode_i16(&bytes[2]);
    sample->accel_raw[2] = imu_mpu6050_decode_i16(&bytes[4]);
    sample->temperature_raw = imu_mpu6050_decode_i16(&bytes[6]);
    sample->gyro_raw[0] = imu_mpu6050_decode_i16(&bytes[8]);
    sample->gyro_raw[1] = imu_mpu6050_decode_i16(&bytes[10]);
    sample->gyro_raw[2] = imu_mpu6050_decode_i16(&bytes[12]);
    sample->status = IMU_SAMPLE_STATUS_VALID;

    for (index = 0u; index < 3u; ++index) {
        if (imu_mpu6050_is_saturated(sample->accel_raw[index])) {
            sample->status |= IMU_SAMPLE_STATUS_ACCEL_SATURATED;
        }
        if (imu_mpu6050_is_saturated(sample->gyro_raw[index])) {
            sample->status |= IMU_SAMPLE_STATUS_GYRO_SATURATED;
        }
    }
    return IMU_STATUS_OK;
}

/**
 * @brief 填充 MPU6050/MPU6500 的安全默认配置。
 * @param config 输出配置，不允许为空。
 * @note 默认 1 kHz、94 Hz DLPF、陀螺 +/-2000 dps、加速度 +/-16 g；
 *       1 kHz 是当前 I2C 六轴采集链路基准，量程和带宽仍需由真实数据确定。
 */
void imu_mpu6050_default_config(ImuMpu6050Config *config)
{
    if (config == 0) {
        return;
    }
    config->address_7bit = 0x68u;
    config->sample_rate_hz = 1000u;
    config->gyro_range = IMU_MPU6050_GYRO_2000_DPS;
    config->accel_range = IMU_MPU6050_ACCEL_16_G;
    config->dlpf = IMU_MPU6050_DLPF_94_HZ;
    config->data_ready_interrupt = true;
}

/**
 * @brief 读取 MPU6050 WHO_AM_I 寄存器。
 * @param device 已绑定抽象总线的设备实例，初始化探测阶段也可调用。
 * @param who_am_i 输出寄存器原值。
 * @return IMU_STATUS_OK 或底层总线错误；本函数不判断器件型号。
 */
ImuStatus imu_mpu6050_read_who_am_i(const ImuMpu6050 *device,
                                    uint8_t *who_am_i)
{
    if (device == 0 || who_am_i == 0 || device->bus.read == 0) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    return device->bus.read(device->bus.context,
                            device->config.address_7bit,
                            MPU6050_REG_WHO_AM_I,
                            who_am_i,
                            1u);
}

/**
 * @brief 复位 MPU6050/MPU6500、验证器件身份并写入采样配置。
 * @param device 输出设备状态；函数开始时清零，调用者持有其生命周期。
 * @param bus 同步读写和毫秒延时回调，初始化时复制到 device。
 * @param config 地址、采样率、DLPF、量程和数据就绪中断配置。
 * @return IMU_STATUS_OK 表示全部寄存器写入完成；否则返回首个明确失败。
 * @note 本函数包含 100 ms 复位等待和 10 ms 唤醒等待，只能在前台初始化调用。
 *       任一步失败时 initialized 保持 false，不自动重试或重置 MCU I2C 外设。
 */
ImuStatus imu_mpu6050_init(ImuMpu6050 *device,
                           const ImuBus *bus,
                           const ImuMpu6050Config *config)
{
    ImuStatus status;
    uint8_t who_am_i;
    uint32_t base_rate_hz;
    uint8_t sample_divider;
    static const float accel_lsb_per_g[4] = {
        16384.0f, 8192.0f, 4096.0f, 2048.0f
    };
    static const float gyro_lsb_per_dps[4] = {
        131.0f, 65.5f, 32.8f, 16.4f
    };

    if (device == 0 || bus == 0 || bus->read == 0 || bus->write == 0 ||
        bus->delay_ms == 0 || !imu_mpu6050_config_is_valid(config)) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    memset(device, 0, sizeof(*device));
    device->bus = *bus;
    device->config = *config;
    device->accel_lsb_per_g = accel_lsb_per_g[config->accel_range];
    device->gyro_lsb_per_dps = gyro_lsb_per_dps[config->gyro_range];

    status = imu_mpu6050_write_u8(device, MPU6050_REG_PWR_MGMT_1,
                                  MPU6050_PWR_DEVICE_RESET);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    device->bus.delay_ms(device->bus.context, 100u);

    status = imu_mpu6050_write_u8(device, MPU6050_REG_PWR_MGMT_1,
                                  MPU6050_CLOCK_PLL_X_GYRO);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_PWR_MGMT_2, 0u);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    /*
     * 复位温度、加速度和陀螺数字信号通道，清除上电或热插拔后可能残留的
     * 通道状态。复位位由器件自动清零，不改变后续量程与 DLPF 配置。
     */
    status = imu_mpu6050_write_u8(device, MPU6050_REG_SIGNAL_PATH_RESET,
                                  MPU6050_RESET_ALL_SIGNAL_PATHS);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    /* 显式解除六轴待机并复位信号通道后，等待模拟前端重新稳定。 */
    device->bus.delay_ms(device->bus.context, 100u);

    status = imu_mpu6050_read_who_am_i(device, &who_am_i);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    device->last_who_am_i = who_am_i;
    device->who_am_i_valid = true;
    if (who_am_i == MPU6050_WHO_AM_I_VALUE) {
        device->model = IMU_MPU_MODEL_6050;
        device->temperature_lsb_per_c = MPU6050_TEMP_LSB_PER_C;
        device->temperature_offset_c = MPU6050_TEMP_OFFSET_C;
    } else if (who_am_i == MPU6500_WHO_AM_I_VALUE) {
        device->model = IMU_MPU_MODEL_6500;
        device->temperature_lsb_per_c = MPU6500_TEMP_LSB_PER_C;
        device->temperature_offset_c = MPU6500_TEMP_OFFSET_C;
    } else {
        return IMU_STATUS_DEVICE_NOT_FOUND;
    }

    base_rate_hz = (config->dlpf == IMU_MPU6050_DLPF_260_HZ) ? 8000u : 1000u;
    sample_divider = (uint8_t)((base_rate_hz / config->sample_rate_hz) - 1u);
    status = imu_mpu6050_write_u8(device, MPU6050_REG_SMPLRT_DIV,
                                  sample_divider);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_CONFIG,
                                  (uint8_t)config->dlpf);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_GYRO_CONFIG,
                                  (uint8_t)((uint8_t)config->gyro_range << 3));
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_ACCEL_CONFIG,
                                  (uint8_t)((uint8_t)config->accel_range << 3));
    if (status != IMU_STATUS_OK) {
        return status;
    }
    /*
     * MPU6500 将加速度计 DLPF 独立放在 ACCEL_CONFIG_2；MPU6050 没有该
     * 寄存器。只对已识别的 6500 写入，保证两种器件得到一致的带宽档位。
     */
    if (device->model == IMU_MPU_MODEL_6500) {
        status = imu_mpu6050_write_u8(device, MPU6500_REG_ACCEL_CONFIG_2,
                                      (uint8_t)config->dlpf);
        if (status != IMU_STATUS_OK) {
            return status;
        }
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_FIFO_EN, 0u);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_USER_CTRL, 0u);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_INT_PIN_CFG,
                                  MPU6050_INT_READ_CLEAR);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = imu_mpu6050_write_u8(device, MPU6050_REG_INT_ENABLE,
                                  config->data_ready_interrupt ?
                                  MPU6050_INT_DATA_READY : 0u);
    if (status != IMU_STATUS_OK) {
        return status;
    }

    device->initialized = true;
    return IMU_STATUS_OK;
}

/**
 * @brief 同步读取并解析一帧 Acc、Temp、Gyro 连续原始寄存器。
 * @param device 已成功初始化的设备实例。
 * @param timestamp_us 本帧数据就绪或读取时刻，单位 us；驱动不自行读取时钟。
 * @param sample 输出原始帧。
 * @return IMU_STATUS_OK 表示 14 字节完整读取；总线失败时不递增序号、不发布新帧。
 * @note 本函数只解析原始计数并标记数字满码，不做滤波、调零、单位转换或姿态解算。
 */
ImuStatus imu_mpu6050_read_raw(ImuMpu6050 *device,
                               uint64_t timestamp_us,
                               ImuRawSample *sample)
{
    ImuStatus status;
    uint8_t bytes[14];

    if (device == 0 || sample == 0 || !device->initialized ||
        device->async_read_pending) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    status = device->bus.read(device->bus.context,
                              device->config.address_7bit,
                              MPU6050_REG_ACCEL_XOUT_H,
                              bytes,
                              (uint16_t)sizeof(bytes));
    if (status != IMU_STATUS_OK) {
        return status;
    }

    return imu_mpu6050_decode_raw(device, bytes, timestamp_us, sample);
}

/**
 * @brief 启动一帧 Acc/Temp/Gyro 异步突发读取。
 * @param device 已初始化且绑定异步总线回调的设备。
 * @param timestamp_us 触发本次读取的 DATA_RDY 时间戳，单位 us。
 * @return IMU_STATUS_OK 已启动；配置、调用顺序或底层启动失败返回明确错误。
 * @note 函数返回成功只表示 DMA/异步事务已开始，必须继续调用 finish_raw_read。
 */
ImuStatus imu_mpu6050_start_raw_read(ImuMpu6050 *device,
                                     uint64_t timestamp_us)
{
    ImuStatus status;

    if (device == 0 || !device->initialized || device->async_read_pending ||
        device->bus.read_async_start == 0 ||
        device->bus.read_async_poll == 0) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    status = device->bus.read_async_start(device->bus.context,
                                          device->config.address_7bit,
                                          MPU6050_REG_ACCEL_XOUT_H,
                                          device->async_bytes,
                                          (uint16_t)sizeof(device->async_bytes));
    if (status != IMU_STATUS_OK) {
        return status;
    }
    device->async_timestamp_us = timestamp_us;
    device->async_read_pending = true;
    return IMU_STATUS_OK;
}

/**
 * @brief 查询异步读取并在完成后解析原始样本。
 * @param device 已启动异步读取的设备。
 * @param sample 成功时输出完整原始样本。
 * @return IMU_STATUS_NO_DATA 事务仍在进行；OK 已解析；负值表示失败。
 * @note 总线完成或失败后都会清除 pending；只有成功才递增样本序号。
 */
ImuStatus imu_mpu6050_finish_raw_read(ImuMpu6050 *device,
                                      ImuRawSample *sample)
{
    ImuStatus status;

    if (device == 0 || sample == 0 || !device->initialized ||
        !device->async_read_pending || device->bus.read_async_poll == 0) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    status = device->bus.read_async_poll(device->bus.context);
    if (status == IMU_STATUS_NO_DATA) {
        return status;
    }
    device->async_read_pending = false;
    if (status != IMU_STATUS_OK) {
        return status;
    }
    return imu_mpu6050_decode_raw(device,
                                  device->async_bytes,
                                  device->async_timestamp_us,
                                  sample);
}

/**
 * @brief 同步读取电源、量程和独立陀螺输出，供板级启动诊断使用。
 * @param device 已完成初始化的传感器实例。
 * @param diagnostics 输出寄存器快照和三轴陀螺原始计数。
 * @return IMU_STATUS_OK 全部读取成功；否则返回参数或首个总线错误。
 * @note 本函数不改变采样序号和发布快照；只能在无 DMA 事务时由前台调用。
 */
ImuStatus imu_mpu6050_read_diagnostics(const ImuMpu6050 *device,
                                       ImuMpu6050Diagnostics *diagnostics)
{
    ImuStatus status;
    uint8_t gyro_bytes[6];

    if (device == 0 || diagnostics == 0 || !device->initialized ||
        device->async_read_pending || device->bus.read == 0) {
        return IMU_STATUS_INVALID_ARGUMENT;
    }
    status = device->bus.read(device->bus.context, device->config.address_7bit,
                              MPU6050_REG_PWR_MGMT_1,
                              &diagnostics->power_management_1, 1u);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = device->bus.read(device->bus.context, device->config.address_7bit,
                              MPU6050_REG_PWR_MGMT_2,
                              &diagnostics->power_management_2, 1u);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = device->bus.read(device->bus.context, device->config.address_7bit,
                              MPU6050_REG_GYRO_CONFIG,
                              &diagnostics->gyro_config, 1u);
    if (status != IMU_STATUS_OK) {
        return status;
    }
    status = device->bus.read(device->bus.context, device->config.address_7bit,
                              MPU6050_REG_GYRO_XOUT_H,
                              gyro_bytes, (uint16_t)sizeof(gyro_bytes));
    if (status != IMU_STATUS_OK) {
        return status;
    }
    diagnostics->gyro_raw[0] = imu_mpu6050_decode_i16(&gyro_bytes[0]);
    diagnostics->gyro_raw[1] = imu_mpu6050_decode_i16(&gyro_bytes[2]);
    diagnostics->gyro_raw[2] = imu_mpu6050_decode_i16(&gyro_bytes[4]);
    return IMU_STATUS_OK;
}
