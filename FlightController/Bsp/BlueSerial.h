#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include <stdint.h>

extern char BlueSerial_RxPacket[100];
extern volatile uint8_t BlueSerial_RxFlag;

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);
void BlueSerial_Proc(void);
void BlueSerial_SendWave(void);

#endif
