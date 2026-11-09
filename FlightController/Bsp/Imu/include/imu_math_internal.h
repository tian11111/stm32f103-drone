#ifndef IMU_MATH_INTERNAL_H
#define IMU_MATH_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float w; // Hamilton 四元数标量部
    float x; // Hamilton 四元数 X 虚部
    float y; // Hamilton 四元数 Y 虚部
    float z; // Hamilton 四元数 Z 虚部
} ImuQuaternion;

bool imu_math_is_finite(float value);
bool imu_math_vec3_is_finite(const float vector[3]);
float imu_math_vec3_norm(const float vector[3]);
bool imu_math_vec3_normalize(float vector[3]);
void imu_math_vec3_cross(const float a[3], const float b[3], float result[3]);
void imu_math_mat3_identity(float matrix[3][3]);
void imu_math_mat3_mul_vec3(const float matrix[3][3],
                            const float vector[3],
                            float result[3]);

ImuQuaternion imu_math_quaternion_identity(void);
ImuQuaternion imu_math_quaternion_multiply(ImuQuaternion left,
                                            ImuQuaternion right);
bool imu_math_quaternion_normalize(ImuQuaternion *quaternion);
ImuQuaternion imu_math_quaternion_exp(const float rotation_vector_rad[3]);
float imu_math_quaternion_get_fused_yaw_rad(ImuQuaternion quaternion);
float imu_math_quaternion_get_zyx_pitch_rad(ImuQuaternion quaternion);
void imu_math_quaternion_rotate_body_to_world(ImuQuaternion q_wb,
                                              const float vector_body[3],
                                              float vector_world[3]);
void imu_math_quaternion_rotate_world_to_body(ImuQuaternion q_wb,
                                              const float vector_world[3],
                                              float vector_body[3]);
void imu_math_so3_exp(const float rotation_vector_rad[3],
                      float rotation_matrix[3][3]);
void imu_math_so3_right_jacobian(const float rotation_vector_rad[3],
                                 float jacobian[3][3]);

#endif
