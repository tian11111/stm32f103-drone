#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

void Delay_Init(void);
void Delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif