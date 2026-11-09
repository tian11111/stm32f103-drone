#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature_reference_c;                 // 温度多项式参考点，单位摄氏度
    float temperature_min_c;                       // 温度模型拟合下界，单位摄氏度
    float temperature_max_c;                       // 温度模型拟合上界，单位摄氏度
    float gyro_temperature_coeff_sensor[3][3];     // S 系逐轴 c0/c1/c2，输出 rad/s
    float gyro_correction_matrix[3][3];            // S 系陀螺比例/非正交校正 C_g
    float accel_bias_sensor_mps2[3];                // S 系加速度计零偏 b_a，单位 m/s^2
    float accel_correction_matrix[3][3];            // S 系加速度比例/非正交校正 C_a
    float sensor_to_body[3][3];                     // S→B 固定安装旋转 R_BS
} ImuCalibration;

void imu_calibration_set_identity(ImuCalibration *calibration);

ImuStatus imu_calibration_validate(const ImuCalibration *calibration);

ImuStatus imu_calibration_apply(const ImuCalibration *calibration,
                                const ImuMpu6050 *device,
                                const ImuRawSample *raw,
                                float dt_s,
                                ImuSample *sample);

#ifdef __cplusplus
}
#endif

#endif
