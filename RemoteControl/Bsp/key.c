#include "main.h"

/*由 TIM1 中断（Key_Tick）写、主循环（Key_GetNum）读，必须 volatile*/
volatile uint8_t Key_Num;


/**
  * 函    数：获取按键键码
  * 参    数：无
  * 返 回 值：按键键码，范围：0~12，0 表示无按键事件，取走后自动清零
  */
uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

/**
  * 函    数：获取当前按键状态
  * 参    数：无
  * 返 回 值：当前按下按键的键码，范围：0~12，0 表示无按键按下
  */
uint8_t Key_GetState(void)
{
	if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
	{
		return 1;
	}
	if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
	{
		return 2;
	}
	if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
	{
		return 3;
	}
	if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET)
	{
		return 4;
	}
	if (HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin) == GPIO_PIN_RESET)
	{
		return 5;
	}
	if (HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) == GPIO_PIN_RESET)
	{
		return 6;
	}
	if (HAL_GPIO_ReadPin(KEY7_GPIO_Port, KEY7_Pin) == GPIO_PIN_RESET)
	{
		return 7;
	}
	if (HAL_GPIO_ReadPin(KEY8_GPIO_Port, KEY8_Pin) == GPIO_PIN_RESET)
	{
		return 8;
	}
	if (HAL_GPIO_ReadPin(KEY9_GPIO_Port, KEY9_Pin) == GPIO_PIN_RESET)
	{
		return 9;
	}
	if (HAL_GPIO_ReadPin(KEY10_GPIO_Port, KEY10_Pin) == GPIO_PIN_RESET)
	{
		return 10;
	}
	if (HAL_GPIO_ReadPin(KEY11_GPIO_Port, KEY11_Pin) == GPIO_PIN_RESET)
	{
		return 11;
	}
	if (HAL_GPIO_ReadPin(KEY12_GPIO_Port, KEY12_Pin) == GPIO_PIN_RESET)
	{
		return 12;
	}
	return 0;
}

/**
  * 函    数：按键扫描驱动，需放在 1ms 周期的中断里调用
  * 参    数：无
  * 返 回 值：无
  * 说    明：内部每 20ms 真正扫描一次 GPIO，实现 20ms 消抖。
  *           按键为松手触发：按下再松开才产生一次键码
  */
void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	Count ++;
	if (Count >= 20)
	{
		Count = 0;
		
		PrevState = CurrState;
		CurrState = Key_GetState();
		
		if (CurrState == 0 && PrevState != 0)
		{
			Key_Num = PrevState;
		}
	}
}
