#ifndef BLUETOOTH_TEST_USART_H
#define BLUETOOTH_TEST_USART_H

#include <stdint.h>

typedef struct {
    uint32_t ErrorCode;
    uint32_t RxState;
    uint32_t gState;
} UART_HandleTypeDef;

typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
#define HAL_UART_STATE_BUSY_TX 2U
#define __HAL_UART_CLEAR_PEFLAG(handle) test_uart_clear_error(handle)

extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
void test_uart_clear_error(UART_HandleTypeDef *);
void Error_Handler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

#endif
