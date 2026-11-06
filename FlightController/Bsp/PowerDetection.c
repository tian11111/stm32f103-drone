#include "main.h"
#include "adc.h"
#include "PowerDetection.h"
#include "Variable.h"

/* ADC1 已由 MX_ADC1_Init() 初始化，这里只校准 */
void AD_Init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 正常返回 0～4095；失败返回 UINT16_MAX */
uint16_t Get_ADC(void)
{
    uint16_t raw = UINT16_MAX;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return UINT16_MAX;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
    {
        return UINT16_MAX;
    }

    return raw;
}

void GetBattery_Init(void)
{
    AD_Init();
}

/* 返回电池电压；采样失败返回 -1 */
float GetBattery(void)
{
    uint16_t raw = Get_ADC();

    if (raw == UINT16_MAX)
    {
        return -1.0f;
    }

    /* 保留原来的参考电压、分压比和换算公式 */
    return 3.3f * 2.0f * (float)raw / 4096.0f;
}

void GetCur_Power(void)
{
    if (Battery_num < 100)
    {
        float voltage = GetBattery();

        /* 失败样本不参与平均，保留上次平均电压 */
        if (voltage < 0.0f)
        {
            return;
        }

        Battery_temp = voltage;
        Battery_total += Battery_temp;
        Battery_num++;
    }
    else
    {
        /* 保留原来的节奏：100 次采样，下一次调用更新 */
        Battery_average = Battery_total / 100.0f;
        Battery_total = 0.0f;
        Battery_num = 0;
    }
}