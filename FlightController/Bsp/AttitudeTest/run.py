"""Host test of the production flight-control and FIFO functions with hardware stubs."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / "build/task-attitude-host"
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
static uint8_t sent_packet[32];
static int sent_count;
void Send(uint8_t *p) { memcpy(sent_packet, p, sizeof sent_packet); ++sent_count; }
void BlueSerial_Proc(void) {}
void LED_Time(int x) { (void)x; }
void LED_Light(int x) { (void)x; }
#define OLED_6X8 0
#define OLED_8X16 1
static char oled_status[32];
static int oled_ok, oled_still, oled_restart;
void OLED_Clear(void) { oled_status[0] = '\0'; oled_ok = oled_still = oled_restart = 0; }
void OLED_Printf(int x,int y,int font,const char *fmt,...) { (void)x;(void)y;(void)font;(void)fmt; }
static int oled_pitch_cleared, oled_pitch_error;
void OLED_ClearArea(int x,int y,int width,int height) {
    if (x==88 && y==28 && width==40 && height==8) oled_pitch_cleared++;
}
void OLED_ShowString(int x,int y,char *text,int font) {
    (void)font;
    if (x==0 && y==0) { strncpy(oled_status, text, sizeof oled_status - 1); oled_status[sizeof oled_status - 1] = '\0'; }
    if (x==0 && y==40 && strcmp(text,"IMU:OK")==0) oled_ok++;
    if (x==0 && y==16 && strcmp(text,"KEEP STILL")==0) oled_still++;
    if (x==0 && y==32 && strcmp(text,"RESTART")==0) oled_restart++;
    if (x==88 && y==28 && strcmp(text,"ERR")==0) oled_pitch_error++;
}
void OLED_UpdateAsync(void) {}
'''

data = no_includes(source("Bsp/Data_Proc.c"))
fixed_format = source("Bsp/FixedFormat.c")
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
    Pitch=NAN; Data_Display();
    assert(oled_pitch_cleared==1 && oled_pitch_error==1);
    puts("attitude and FIFO host checks passed"); return 0;
}
'''
generated = out / "attitude_test.c"
generated.write_text(prelude + fixed_format + variables + motor + data + fifo + test, encoding="utf-8")
exe = out / "attitude_test.exe"
subprocess.run(["D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os", "-flto", "-ffunction-sections", "-fdata-sections", "-I", str(root / "Bsp"), str(generated), "-Wl,--gc-sections", "-lm", "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)

# Actual MEKF source files are reused by both startup and adapter tests.
mekf_sources = [
    "Bsp/AttitudeTest/mekf_test.c", "Bsp/Attitude/attitude_mekf.c",
    "Bsp/Imu/f103_port.c", "Bsp/Imu/src/imu_math.c",
    "Bsp/Imu/src/imu_mpu6050.c", "Bsp/Imu/src/imu_calibration.c",
    "Bsp/Imu/src/imu_stationary.c", "Bsp/Imu/src/imu_mekf.c",
    "Bsp/Imu/src/imu_system.c",
]

# Exercise a real startup bus failure through the actual MEKF source,
# publisher, unlock gate and motor safety path in the same process.
startup_bus_stub = r'''
#include "i2c.h"
#include "Attitude/attitude_source.h"
I2C_HandleTypeDef hi2c2;
static int startup_bus_failed = 1, startup_reset_attempts;
void HAL_Delay(uint32_t ms) { tick += ms; }
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
                     uint16_t width, uint8_t *data, uint16_t len, uint32_t timeout) {
    (void)h; (void)addr; (void)reg; (void)width; (void)data; (void)len; (void)timeout;
    return startup_bus_failed ? 1 : HAL_OK;
}
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
                      uint16_t width, uint8_t *data, uint16_t len, uint32_t timeout) {
    (void)h; (void)addr; (void)width; (void)timeout;
    if (reg == 0x6b && len == 1 && data[0] == 0x80) ++startup_reset_attempts;
    return startup_bus_failed ? 1 : HAL_OK;
}
'''
startup_safety_test = r'''
static void packet(int key) { Receive_Data[0]=0; Receive_Data[2]=0; Receive_Data[5]=key; Data_Receive(); }
int main(void) {
    Data_Display(); assert(strcmp(oled_status, "IMU INIT") == 0 && oled_still);
    AttitudeSource_Init();
    assert(startup_reset_attempts == 3 && AttitudeSource_IsFaulted());
    Data_Display(); assert(strcmp(oled_status, "IMU FAULT") == 0 && oled_still && oled_restart);
    Attitude_Data_Proc();
    assert(!Attitude_IsFresh() && Lock == 1);
    packet(0); packet(8);
    assert(Lock == 1);

    /* Even forced stale output state must be cleared at the control gate. */
    Lock=0; connect_protect=0; Oil_Get=30; Oil_Set=900;
    PWM_OUT1=PWM_OUT2=PWM_OUT3=PWM_OUT4=300;
    Fly_task();
    assert(Lock == 1 && !Oil_Get && !Oil_Set);
    assert(!PWM_OUT1 && !PWM_OUT2 && !PWM_OUT3 && !PWM_OUT4);
    assert(!ccr[0] && !ccr[1] && !ccr[2] && !ccr[3]);

    startup_bus_failed=0;
    tick += 5; Attitude_Data_Proc();
    assert(startup_reset_attempts == 3 && !Attitude_IsFresh());
    packet(0); packet(8);
    assert(Lock == 1);
    puts("MEKF startup failure through motor safety checks passed");
    return 0;
}
'''
generated = out / "mekf_startup_safety_test.c"
generated.write_text("#define ATTITUDE_SOURCE_MEKF\n" + prelude + startup_bus_stub +
                     fixed_format + variables + motor + data + fifo + startup_safety_test,
                     encoding="utf-8")
exe = out / "mekf_startup_safety_test.exe"
subprocess.run([
    "D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os", "-flto",
    "-ffunction-sections", "-fdata-sections",
    "-I", str(root / "Bsp/AttitudeTest/stubs"),
    "-I", str(root / "Bsp"), "-I", str(root / "Bsp/Attitude"),
    "-I", str(root / "Bsp/Imu"), "-I", str(root / "Bsp/Imu/include"),
    str(generated), *[str(root / path) for path in mekf_sources[1:]],
    "-Wl,--gc-sections", "-lm", "-o", str(exe),
], check=True)
subprocess.run([str(exe)], check=True)

# Real adapter/core/publisher path, injecting the interrupt immediately after
# the actual Read has latched a bus fault and before the publisher resumes.
preempt_bus = r'''
#include "i2c.h"
#include "Attitude/attitude_source.h"
I2C_HandleTypeDef hi2c2;
static int bus_fail, interrupted;
void HAL_Delay(uint32_t ms) { tick += ms; }
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a, uint16_t r,
                     uint16_t w, uint8_t *d, uint16_t n, uint32_t t) {
    (void)h; (void)a; (void)w; (void)t;
    if (bus_fail) return 1;
    memset(d, 0, n);
    if (r == 0x75) d[0] = 0x68;
    if (r == 0x3a && n == 15) { d[0] = 1; d[5] = 8; }
    return HAL_OK;
}
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a, uint16_t r,
                      uint16_t w, uint8_t *d, uint16_t n, uint32_t t) {
    (void)h; (void)a; (void)r; (void)w; (void)d; (void)n; (void)t;
    return HAL_OK;
}
bool Real_AttitudeSource_Read(AttitudeOutput *out);
void Fly_task(void);
bool AttitudeSource_Read(AttitudeOutput *out) {
    bool result = Real_AttitudeSource_Read(out);
    if (bus_fail && AttitudeSource_IsFaulted()) {
        Fly_task();
        assert(Lock == 1 && !Oil_Get && !Oil_Set);
        assert(!PWM_OUT1 && !PWM_OUT2 && !PWM_OUT3 && !PWM_OUT4);
        assert(!ccr[0] && !ccr[1] && !ccr[2] && !ccr[3]);
        interrupted = 1;
    }
    return result;
}
'''
preempt_test = r'''
int main(void) {
    AttitudeSource_Init();
    for (int i = 0; i < 310; ++i) { tick += 5; Attitude_Data_Proc(); }
    assert(Attitude_IsFresh());
    Data_Display(); assert(strcmp(oled_status, "") == 0 && oled_ok); /* normal page replaces startup */
    Receive_Data[0]=0; Receive_Data[5]=0; Receive_Data[6]=0; sent_count=0;
    Lock=1; Data_Receive(); assert(sent_count==0); /* legacy joystick page */
    Receive_Data[0]=1; Data_Receive(); assert(sent_count==1 && sent_packet[0]==2 && sent_packet[3]==0xA1);
    Receive_Data[0]=0; Receive_Data[6]=0xA5; Lock=0; Data_Receive();
    assert(sent_count==2 && sent_packet[3]==0xA2); /* new joystick page */
    Receive_Data[6]=0;
    Lock=0; connect_protect=0; Oil_Get=30; Oil_Set=900;
    PWM_OUT1=PWM_OUT2=PWM_OUT3=PWM_OUT4=300;
    Motor_Control(); assert(ccr[0] && ccr[1] && ccr[2] && ccr[3]);
    bus_fail=1; tick += 5; Attitude_Data_Proc();
    assert(interrupted && !Attitude_IsFresh());
    Data_Display(); assert(strcmp(oled_status, "IMU FAULT") == 0 && oled_still && oled_restart);
    puts("real MEKF fault preemption safety checks passed");
    return 0;
}
'''
renamed_adapter = out / "preempt_attitude_mekf.c"
adapter = source("Bsp/Attitude/attitude_mekf.c")
assert adapter.count("bool AttitudeSource_Read(") == 1
renamed_adapter.write_text(adapter.replace("bool AttitudeSource_Read(", "bool Real_AttitudeSource_Read("), encoding="utf-8")
generated = out / "mekf_preemption_test.c"
generated.write_text("#define ATTITUDE_SOURCE_MEKF\n" + prelude + fixed_format + variables + motor +
                     preempt_bus + data + fifo + preempt_test, encoding="utf-8")
exe = out / "mekf_preemption_test.exe"
subprocess.run([
    "D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os", "-flto",
    "-ffunction-sections", "-fdata-sections",
    "-I", str(root / "Bsp/AttitudeTest/stubs"), "-I", str(root / "Bsp"),
    "-I", str(root / "Bsp/Attitude"), "-I", str(root / "Bsp/Imu"),
    "-I", str(root / "Bsp/Imu/include"), str(generated), str(renamed_adapter),
    *[str(root / path) for path in mekf_sources[2:]],
    "-Wl,--gc-sections", "-lm", "-o", str(exe),
], check=True)
subprocess.run([str(exe)], check=True)

# Compile the actual TIM1 callback with lightweight hardware stubs.  The
# foreground attitude slot must be 5 ms for MEKF and 4 ms for DMP, while
# Fly_task remains scheduled every 4 ms in either build.
main_source = source("Core/Src/main.c")
callback = main_source[main_source.index("void HAL_TIM_PeriodElapsedCallback("):
                       main_source.index("/* USER CODE END 4 */")]
timer_prelude = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "Variable.h"
#include "f103_port.h"
typedef struct { int Instance; } TIM_HandleTypeDef;
#define TIM1 1
static unsigned flight_calls;
void Fly_task(void) { ++flight_calls; }
void LED_Detect(void) {}
'''
timer_test = r'''
int main(void) {
    TIM_HandleTypeDef timer = { TIM1 };
    unsigned attitude_calls = 0;
#ifdef ATTITUDE_SOURCE_MEKF
    const unsigned period = 5;
#else
    const unsigned period = 4;
#endif
    for (unsigned ms = 1; ms <= 20; ++ms) {
        HAL_TIM_PeriodElapsedCallback(&timer);
        if (Attitude_Proc_time == SlowDown) {
            assert(ms % period == 0);
            ++attitude_calls;
            Attitude_Proc_time = 1;
        }
    }
    assert(attitude_calls == 20 / period && flight_calls == 5);
    puts("attitude and control timer scheduling checks passed");
    return 0;
}
'''
generated = out / "attitude_timer_test.c"
generated.write_text(timer_prelude + variables + callback + timer_test, encoding="utf-8")
for source_name in ("MEKF", "DMP"):
    exe = out / ("attitude_timer_" + source_name.lower() + ".exe")
    subprocess.run([
        "D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os",
        "-DATTITUDE_SOURCE_" + source_name,
        "-I", str(root / "Bsp"), "-I", str(root / "Bsp/Imu"),
        "-I", str(root / "Bsp/Imu/include"),
        str(generated), "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)

# Run the real F103 adapter and unmodified MEKF core with a mocked I2C bus.
# Two separate processes verify that I2C failure and a missing DATA_RDY bit
# each latch a fault without allowing later samples to revive the estimator.
exe = out / "mekf_test.exe"
subprocess.run([
    "D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os",
    "-I", str(root / "Bsp/AttitudeTest/stubs"),
    "-I", str(root / "Bsp/Attitude"),
    "-I", str(root / "Bsp/Imu"),
    "-I", str(root / "Bsp/Imu/include"),
    *[str(root / path) for path in mekf_sources],
    "-Wl,--wrap=imu_system_process_data_ready", "-lm", "-o", str(exe),
], check=True)
subprocess.run([str(exe)], check=True)
subprocess.run([str(exe), "no-ready"], check=True)
subprocess.run([str(exe), "startup-moving"], check=True)
for case in ("slow-roll", "inverted", "runtime-inverted", "direction-below", "direction-above", "stationary-tilted"):
    subprocess.run([str(exe), case], check=True)
subprocess.run([str(exe), "bias"], check=True)
subprocess.run([str(exe), "late"], check=True)
subprocess.run([str(exe), "init-retry"], check=True)
subprocess.run([str(exe), "init-reset-retry"], check=True)
subprocess.run([str(exe), "init-bus-limit"], check=True)
subprocess.run([str(exe), "init-wrong-id"], check=True)

# Exercise the real publisher and flight safety gate with a source that latches
# a failure during Read().  A later publisher invocation must not be needed to
# revoke the previous attitude.
mekf_stub = r'''
#define ATTITUDE_SOURCE_MEKF
#include "Attitude/attitude_source.h"
static bool source_faulted, source_fail_on_read, source_has_sample;
static AttitudeOutput source_output;
void Fly_task(void);
bool AttitudeSource_IsFaulted(void) { return source_faulted; }
bool AttitudeSource_Read(AttitudeOutput *out) {
    if (source_faulted) return false;
    if (source_fail_on_read) {
        source_faulted = true;
        Fly_task(); /* TIM1 preempts before publisher can clear attitude_valid. */
        assert(Lock==1 && !Oil_Get && !Oil_Set);
        assert(!PWM_OUT1 && !PWM_OUT2 && !PWM_OUT3 && !PWM_OUT4);
        assert(!ccr[0] && !ccr[1] && !ccr[2] && !ccr[3]);
        return false;
    }
    if (!source_has_sample) return false;
    *out = source_output;
    return true;
}
'''
mekf_safety_test = r'''
static void packet(int key) { Receive_Data[0]=0; Receive_Data[2]=0; Receive_Data[5]=key; Data_Receive(); }
int main(void) {
    tick=100;
    source_has_sample=true;
    source_output=(AttitudeOutput){ .pitch=1, .roll=2, .yaw=3,
        .gx=4, .gy=5, .gz=6, .sequence=1, .tick=tick };
    Attitude_Data_Proc();
    assert(Attitude_IsFresh() && Pitch==1 && gx==4);

    /* No new sample alone does not invalidate an otherwise fresh attitude. */
    tick=101; source_has_sample=false;
    Attitude_Data_Proc();
    assert(Attitude_IsFresh());

    Lock=0; connect_protect=0; Oil_Get=30; Oil_Set=900;
    PWM_OUT1=PWM_OUT2=PWM_OUT3=PWM_OUT4=300;
    PID_Pitch_Gyro_Integral=42; PID_Pitch_Gyro_err_last=42;
    tick=105; source_fail_on_read=true; mask=1;
    Attitude_Data_Proc();
    assert(mask==1 && source_faulted && !Attitude_IsFresh());
    assert(Pitch==1 && gx==4);
    mask=0; Fly_task();
    assert(Lock==1 && !Oil_Get && !Oil_Set && !PWM_OUT1 && !PWM_OUT4);
    assert(!PID_Pitch_Gyro_Integral && !PID_Pitch_Gyro_err_last);
    assert(!ccr[0] && !ccr[1] && !ccr[2] && !ccr[3]);
    packet(0); packet(8);
    assert(Lock==1);
    puts("MEKF publication and immediate fault safety checks passed");
    return 0;
}
'''
generated = out / "mekf_publication_test.c"
generated.write_text("#define ATTITUDE_SOURCE_MEKF\n" + prelude + fixed_format + variables + motor + mekf_stub + data + fifo + mekf_safety_test, encoding="utf-8")
exe = out / "mekf_publication_test.exe"
subprocess.run(["D:/dev/sdk/mingw64/bin/gcc.exe", "-std=c11", "-Os", "-flto", "-ffunction-sections", "-fdata-sections", "-I", str(root / "Bsp"), str(generated), "-Wl,--gc-sections", "-lm", "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)
