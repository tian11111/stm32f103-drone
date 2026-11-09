#ifndef TEST_I2C_H
#define TEST_I2C_H
#include <stdint.h>
typedef struct { int unused; } I2C_HandleTypeDef;
extern I2C_HandleTypeDef hi2c2;
#define HAL_OK 0
#define I2C_MEMADD_SIZE_8BIT 1
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
void HAL_Delay(uint32_t);
#endif
