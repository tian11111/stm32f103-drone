#ifndef __DATA_PROC_H
#define __DATA_PROC_H

#include <stdint.h>

void Data_Proc(void);
void Data_Display(void);
void Data_Receive(void);
void Data_Send(void);
void Get_fly_zero(void);
void Fly_task(void);
void Connect_Portect_Test(void);
void Attitude_Data_Proc(void);
uint8_t Attitude_IsFresh(void);
void LED_Detect(void);

#endif
