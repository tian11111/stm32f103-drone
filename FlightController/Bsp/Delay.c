#include "main.h"
#include "Delay.h"

void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Delay_us(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
        __NOP();
    }
}

/* 保留 MPU/DMP 使用的毫秒延时接口 */
void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}