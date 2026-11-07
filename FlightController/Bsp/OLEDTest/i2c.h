#ifndef OLED_TEST_I2C_H
#define OLED_TEST_I2C_H
#include <stdint.h>
typedef struct { uint32_t State; uint32_t ErrorCode; } I2C_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define HAL_I2C_STATE_READY 0U
#define I2C_FLAG_BUSY 1U
#define HAL_I2C_ERROR_BERR 1U
#define I2C_MEMADD_SIZE_8BIT 1U
#define RESET 0U
extern I2C_HandleTypeDef hi2c1, hi2c2;
extern uint32_t test_busy;
#define __HAL_I2C_GET_FLAG(handle, flag) (test_busy)
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
void HAL_Delay(uint32_t);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);
#endif
