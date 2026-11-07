#include "main.h"
#include "LED.h"

static uint16_t LED_Time_Light=0;
static uint16_t LED_Time_Dark=0;
static uint8_t LED_Light_Dark_Bit=0;

void LED_Init(void)
{
	LED_Time_Light=0;
	LED_Time_Dark=0;
	LED_Light_Dark_Bit=0;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Light(uint8_t LED_bit)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_Time(uint16_t LED_Time_number)
{

	
	if(LED_Light_Dark_Bit==1)//执行亮
	{
		LED_Light(1);
		LED_Time_Light++;  //记录亮的时间，必须与LED_Time值一致才能保证亮的时间与暗的时间一致
		if(LED_Time_Light==LED_Time_number) 
		{
			LED_Time_Light=0;//清零
			LED_Light_Dark_Bit=0;
		}
	}
	else if(LED_Light_Dark_Bit==0)//执行暗
	{
		LED_Light(0);
	  LED_Time_Dark++;
		if(LED_Time_Dark==LED_Time_number)
		{
			LED_Time_Dark=0;//清零
		  LED_Light_Dark_Bit=1;
		}
	}
}
