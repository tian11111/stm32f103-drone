#ifndef __AD_H
#define __AD_H

#include <stdint.h>

/*DMA 搬运的原始值缓冲区，顺序为 CH0(左横)/CH1(左纵)/CH2(右横)/CH3(右纵)*/
extern volatile uint16_t AD_Value[4];

void AD_Init(void);
void AD_Read(uint16_t *LH, uint16_t *LV, uint16_t *RH, uint16_t *RV);

#endif
