#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "BlueSerial.h"
#include "Variable.h"

char BlueSerial_RxPacket[100];
volatile uint8_t BlueSerial_RxFlag;

static uint8_t rx_byte;
static uint8_t rx_length;
static uint8_t rx_discard;

/* USART1 is configured by MX_USART1_UART_Init before this call. */
void BlueSerial_Init(void)
{
	rx_length = 0;
	rx_discard = 0;
	BlueSerial_RxFlag = 0;
	BlueSerial_RxPacket[0] = '\0';
	if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
	{
		Error_Handler();
	}
}

void BlueSerial_SendByte(uint8_t Byte)
{
	BlueSerial_SendArray(&Byte, 1);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	if (Length != 0)
	{
		/* 10 bits per byte at 9600 baud, plus scheduling margin. */
		uint32_t timeout = ((uint32_t)Length * 10U * 1000U + 9599U) / 9600U + 20U;
		(void)HAL_UART_Transmit(&huart1, Array, Length, timeout);
	}
}

void BlueSerial_SendString(char *String)
{
	/* HAL's length is uint16_t; send long strings in bounded chunks. */
	size_t length = strlen(String);
	while (length != 0)
	{
		uint16_t chunk = length > UINT16_MAX ? UINT16_MAX : (uint16_t)length;
		BlueSerial_SendArray((uint8_t *)String, chunk);
		String += chunk;
		length -= chunk;
	}
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsnprintf(String, sizeof String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

/* Bracketed packets and unbracketed lines share the same bounded buffer. */
static void BlueSerial_RxByte(uint8_t byte)
{
	uint8_t delimiter = (byte == ']' || byte == '\r' || byte == '\n');

	if (BlueSerial_RxFlag != 0)
	{
		/* Do not accept a suffix if the pending packet is consumed mid-frame. */
		rx_discard = !delimiter;
		return;
	}
	if (rx_discard)
	{
		if (delimiter) rx_discard = 0;
		return;
	}
	if (delimiter)
	{
		if (rx_length != 0)
		{
			BlueSerial_RxPacket[rx_length] = '\0';
			rx_length = 0;
			BlueSerial_RxFlag = 1;
		}
	}
	else if (byte == '[')
	{
		rx_length = 0;
	}
	else if (rx_length < sizeof BlueSerial_RxPacket - 1U)
	{
		BlueSerial_RxPacket[rx_length++] = (char)byte;
	}
	else
	{
		rx_length = 0;
		rx_discard = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1 && huart->ErrorCode == HAL_UART_ERROR_NONE)
	{
		BlueSerial_RxByte(rx_byte);
		(void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		/* A damaged frame cannot safely be used for tuning. */
		rx_length = 0;
		rx_discard = 1;
		if (huart->RxState != HAL_UART_STATE_READY)
		{
			(void)HAL_UART_AbortReceive(huart);
		}
		__HAL_UART_CLEAR_PEFLAG(huart);
		(void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
	}
}

static int BlueSerial_ParseFloat(const char *text, float *value)
{
	char *end;
	float parsed = strtof(text, &end);
	if (end == text || !isfinite(parsed)) return 0;
	while (isspace((unsigned char)*end)) end++;
	if (*end != '\0' && *end != ',') return 0;
	*value = parsed;
	return 1;
}

/* Online PID tuning, called every 20 ms from Data_Proc().
   Values are written straight into the PID globals and take effect at once. */
void BlueSerial_Proc(void)
{
	if (BlueSerial_RxFlag == 1)
	{
		if (strncmp(BlueSerial_RxPacket, "SET", 3) == 0)
		{
			/* SET frame tunes the pitch angle loop (same semantics as the 09 demo). */
			char *p = strstr(BlueSerial_RxPacket, "P:");
			char *in = strstr(BlueSerial_RxPacket, "I:");
			char *d = strstr(BlueSerial_RxPacket, "D:");
			float v;
			if (p && BlueSerial_ParseFloat(p + 2, &v)) pitch_angle_balance_Kp = v;
			if (in && BlueSerial_ParseFloat(in + 2, &v)) pitch_angle_balance_Ki = v;
			if (d && BlueSerial_ParseFloat(d + 2, &v)) pitch_angle_balance_Kd = v;
		}
		else
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (Tag != NULL)
			{
				/* slider,<Name>,<Value>: friendly name mapped onto one PID global. */
				if (strcmp(Tag, "slider") == 0)
				{
					char *Name = strtok(NULL, ",");
					char *Value = strtok(NULL, ",");
					float v;
					if (Name != NULL && Value != NULL && BlueSerial_ParseFloat(Value, &v))
					{
						if (strcmp(Name, "AngleKp") == 0) pitch_angle_balance_Kp = v;
						else if (strcmp(Name, "AngleKi") == 0) pitch_angle_balance_Ki = v;
						else if (strcmp(Name, "AngleKd") == 0) pitch_angle_balance_Kd = v;
						else if (strcmp(Name, "SpeedKp") == 0) pitch_gyro_balance_Kp = v;
						else if (strcmp(Name, "SpeedKi") == 0) pitch_gyro_balance_Ki = v;
						else if (strcmp(Name, "SpeedKd") == 0) pitch_gyro_balance_Kd = v;
						else if (strcmp(Name, "TurnKp") == 0) yaw_angle_balance_Kp = v;
						else if (strcmp(Name, "TurnKi") == 0) yaw_angle_balance_Ki = v;
						else if (strcmp(Name, "TurnKd") == 0) yaw_angle_balance_Kd = v;
						else if (strcmp(Name, "Offset") == 0) fly_pitch_zero = v;
						else if (strcmp(Name, "RollAKp") == 0) roll_angle_balance_Kp = v;
						else if (strcmp(Name, "RollAKi") == 0) roll_angle_balance_Ki = v;
						else if (strcmp(Name, "RollAKd") == 0) roll_angle_balance_Kd = v;
						else if (strcmp(Name, "RollGKp") == 0) roll_gyro_balance_Kp = v;
						else if (strcmp(Name, "RollGKi") == 0) roll_gyro_balance_Ki = v;
						else if (strcmp(Name, "RollGKd") == 0) roll_gyro_balance_Kd = v;
						else if (strcmp(Name, "YawGKp") == 0) yaw_gyro_balance_Kp = v;
						else if (strcmp(Name, "YawGKi") == 0) yaw_gyro_balance_Ki = v;
						else if (strcmp(Name, "YawGKd") == 0) yaw_gyro_balance_Kd = v;
					}
				}
				/* "key" and "joystick" tags are ignored on purpose:
				   Bluetooth APP buttons must never control the aircraft. */
			}
		}
		BlueSerial_RxFlag = 0;
	}
}

/* Waveform observer, called once from the 50 ms display slot in main.c.
   FireWater output: pid:ms,pitch_target,pitch_actual,angle_loop_out,battery_volts */
void BlueSerial_SendWave(void)
{
	static uint32_t wave_ms = 0;
	static char wave_buffer[100];
	int length;
	wave_ms += 50;	/* Advance even when a busy transmitter drops this sample. */
	if (huart1.gState != HAL_UART_STATE_READY) return;
	length = snprintf(wave_buffer, sizeof wave_buffer, "pid:%lu,%.2f,%.2f,%.2f,%.2f\r\n",
	                  (unsigned long)wave_ms, fly_pitch_zero + getpitch_SET,
	                  Pitch, Pitch_Angle_out, Battery_average);
	if (length > 0 && (size_t)length < sizeof wave_buffer)
	{
		(void)HAL_UART_Transmit_IT(&huart1, (uint8_t *)wave_buffer, (uint16_t)length);
	}
}
