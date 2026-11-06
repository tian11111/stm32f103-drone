#include "main.h"
#include "MySPI.h"

void MySPI_W_MOSI(uint8_t Value)
{
    HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin,
                      Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MySPI_W_SCLK(uint8_t Value)
{
    HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin,
                      Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MySPI_W_CSN(uint8_t Value)
{
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin,
                      Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MySPI_W_CE(uint8_t Value)
{
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin,
                      Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t MySPI_R_MISO(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port,
                                   NRF_MISO_Pin);
}

uint8_t MySPI_SwapData(uint8_t Byte)
{
    uint8_t ReceiveByte = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        MySPI_W_MOSI(Byte & (0x80u >> i));

        MySPI_W_SCLK(1);
        if (MySPI_R_MISO())
        {
            ReceiveByte |= (0x80u >> i);
        }
        MySPI_W_SCLK(0);
    }

    return ReceiveByte;
}