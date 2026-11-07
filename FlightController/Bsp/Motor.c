#include "main.h"
#include "tim.h"
#include "Motor.h"
#include "Variable.h"
#include "Data_Proc.h"

uint8_t Motor_time = 4;

void Motor_Init(void)
{
    /* 启动输出前，四路占空比清零 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void Motor_Control(void)
{
    if (Lock == 0 && connect_protect == 0 && Attitude_IsFresh())
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_OUT1);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_OUT2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_OUT3);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_OUT4);
    }
    else
    {
        /* 上锁或失联时，四路输出强制清零 */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
}