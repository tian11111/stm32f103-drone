"""Host test of the production flight-control and FIFO functions with hardware stubs."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / "build/Debug/attitude_host"
out.mkdir(parents=True, exist_ok=True)

def source(path):
    raw = (root / path).read_bytes()
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("gbk")

def no_includes(text):
    return "\n".join(line for line in text.splitlines() if not line.startswith("#include"))

prelude = r'''
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "Variable.h"
typedef uint8_t u8;
static uint32_t tick, mask;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask=1; }
void __set_PRIMASK(uint32_t value) { mask=value; }
static int ccr[4], htim2;
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define __HAL_TIM_SET_COMPARE(t,c,v) (ccr[c]=(v))
#define HAL_OK 0
int HAL_TIM_PWM_Start(int *h, int c) { (void)h; (void)c; return 0; }
void Error_Handler(void) { assert(0); }
#define INV_XYZ_GYRO 0x70
#define INV_WXYZ_QUAT 0x100
#define q30 1073741824.0f
static int remaining, reads, fifo_error, fail_at, bad_sensors, bad_quat;
int dmp_read_fifo(short *g, short *a, long *q, unsigned long *t, short *s, unsigned char *more) {
    (void)a; (void)t; ++reads;
    if (fifo_error || reads == fail_at || remaining == 0) return 1;
    g[0]=164*remaining; g[1]=328*remaining; g[2]=-164*remaining;
    q[0]=bad_quat ? 0 : 1073741824L; q[1]=q[2]=q[3]=0;
    *s=bad_sensors ? INV_WXYZ_QUAT : INV_WXYZ_QUAT|INV_XYZ_GYRO;
    *more=(unsigned char)(--remaining != 0); return 0;
}
u8 MPU6050_DMP_Get_Sample(float*,float*,float*,float*,float*,float*);
uint8_t Attitude_IsFresh(void);
void Motor_Control(void);
uint8_t NRF24L01_R_IRQ(void) { return 0; }
uint8_t Receive_NRF24L01_Data(uint8_t *p) { (void)p; return 1; }
void Send(uint8_t *p) { (void)p; }
void BlueSerial_Proc(void) {}
void LED_Time(int x) { (void)x; }
void LED_Light(int x) { (void)x; }
#define OLED_6X8 0
void OLED_Printf(int x,int y,int font,const char *fmt,...) { (void)x;(void)y;(void)font;(void)fmt; }
void OLED_UpdateAsync(void) {}
'''

data = no_includes(source("Bsp/Data_Proc.c"))
motor = no_includes(source("Bsp/Motor.c"))
fifo = source("Bsp/Mpu6050/inv_mpu.c")
fifo = fifo[fifo.index("u8 MPU6050_DMP_Get_Sample("):]
variables = no_includes(source("Bsp/Variable.c"))
test = r'''
static void zeros(void) { assert(!ccr[0] && !ccr[1] && !ccr[2] && !ccr[3]); }
static void packet(int key) { Receive_Data[0]=0; Receive_Data[2]=0; Receive_Data[5]=key; Data_Receive(); }
int main(void) {
    float p=77,r=77,y=77,x=77,g=77,z=77;
    Lock=0; connect_protect=0; PWM_OUT1=PWM_OUT2=PWM_OUT3=PWM_OUT4=300;
    Motor_Control(); zeros(); Fly_task(); assert(Lock==1);
    remaining=6; reads=0; assert(MPU6050_DMP_Get_Sample(&p,&r,&y,&x,&g,&z)==3 && reads==4 && p==77 && x==77);
    assert(MPU6050_DMP_Get_Sample(&p,&r,&y,&x,&g,&z)==0 && reads==6 && x==10 && g==20 && z==-10);
    remaining=2; reads=0; fail_at=2; assert(MPU6050_DMP_Get_Sample(&p,&r,&y,&x,&g,&z)!=0 && p==0 && x==10); fail_at=0;
    remaining=1; bad_sensors=1; assert(MPU6050_DMP_Get_Sample(&p,&r,&y,&x,&g,&z)!=0 && x==10); bad_sensors=0;
    remaining=1; bad_quat=1; assert(MPU6050_DMP_Get_Sample(&p,&r,&y,&x,&g,&z)!=0); bad_quat=0;
    tick=100; remaining=1; mask=1; Attitude_Data_Proc(); assert(mask==1 && Attitude_IsFresh() && gx==10); mask=0;
    Lock=1; PWM_OUT1=PWM_OUT2=PWM_OUT3=PWM_OUT4=300; Motor_Control(); zeros();
    packet(0); packet(8); assert(Lock==0); Motor_Control(); assert(ccr[0]==300 && ccr[3]==300);
    tick=140; fifo_error=1; Attitude_Data_Proc(); Fly_task(); assert(Lock==0 && Attitude_IsFresh());
    tick=151; Oil_Get=30; Oil_Set=900; PID_Pitch_Gyro_Integral=42; PID_Pitch_Gyro_err_last=42;
    Attitude_Data_Proc(); Fly_task(); assert(Lock==1 && !Oil_Get && !Oil_Set && !PWM_OUT1 && !PWM_OUT4 && !PID_Pitch_Gyro_Integral && !PID_Pitch_Gyro_err_last); zeros();
    packet(8); fifo_error=0; remaining=1; Attitude_Data_Proc(); packet(8); assert(Lock==1);
    packet(0); packet(8); assert(Lock==0);
    connect_protect=1; Motor_Control(); zeros(); connect_protect=0;
    tick=0xfffffff0u; remaining=1; Attitude_Data_Proc(); tick=0x20; assert(Attitude_IsFresh()); tick=0x23; assert(!Attitude_IsFresh());
    tick=0x24; remaining=1; Attitude_Data_Proc(); p=Pitch; x=gx;
    tick=0x40; remaining=1; bad_sensors=1; Attitude_Data_Proc(); assert(Pitch==p && gx==x); tick=0x57; assert(!Attitude_IsFresh());
    puts("attitude and FIFO host checks passed"); return 0;
}
'''
generated = out / "attitude_test.c"
generated.write_text(prelude + variables + motor + data + fifo + test, encoding="utf-8")
exe = out / "attitude_test.exe"
subprocess.run(["D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os", "-flto", "-ffunction-sections", "-fdata-sections", "-I", str(root / "Bsp"), str(generated), "-Wl,--gc-sections", "-lm", "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)
