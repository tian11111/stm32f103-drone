#include "Data_Proc.h"
#include "main.h"
#include <math.h>
#include "NRF24L01.h"
#include "Variable.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "OLED.h"
#include "PowerDetection.h"
#include "Motor.h"
#include "LED.h"
#include "BlueSerial.h"

//角度转弧度，参与PID计算
#define DEG_TO_RAD 0.0174532925f

#define ATTITUDE_TIMEOUT_MS 50U
static volatile uint32_t attitude_last_success;
static volatile uint8_t attitude_valid;
/* Require release of an unlock key held across an IMU fault. */
static volatile uint8_t unlock_needs_release;

uint8_t Attitude_IsFresh(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t last;
    uint8_t valid;
    __disable_irq();
    last = attitude_last_success;
    valid = attitude_valid;
    __set_PRIMASK(primask);
    return valid && (uint32_t)(HAL_GetTick() - last) <= ATTITUDE_TIMEOUT_MS;
}


/*经过观察后得出*/
//飞机向前倾斜--pitch负数增大      角速度：  即是绕Y轴 gy负
//飞机向后倾斜--pitch正数增大                          gy正

//飞机向左倾斜--roll正数增大                 即是绕X轴 gx正
//飞机向右倾斜--roll负数增大                           gx负

//从上往下看：
//飞机逆时针--yaw正数增大                    即是绕Z轴 gz正
//飞机顺时针--yaw负数增大                              gz负

/*---PID调控数据处理---*/

#define PID_Oil_Min 360
#define PWM_MAX 2880
#define PWM_MIN 360

#define ANGLE_ERR_THRESHOLD 3.0f    // 外环积分分离阈值：误差<2°才积分（角度）
#define TARGET_GYRO_MAX 300.0f       // 最大目标角速度（°/s），空心杯推荐±50
#define TARGET_GYRO_MIN -300.0f
#define I_ANGLE_IMAX 500.0f           // 积分上限（适配角速度指令）

#define GYRO_ERR_THRESHOLD  3.0f   // 内环积分分离阈值（角速度误差<5°/s才累加）
#define TARGET_PWM_MAX 1000           // 最大目标PWM
#define TARGET_PWM_MIN -1000
#define I_GYRO_IMAX 1200.0f           // 积分上限（适配角速度指令）

/* 1 reverses the corresponding balance-remote direction. */
#define BALANCE_REMOTE_REVERSE_THROTTLE 1
#define BALANCE_REMOTE_REVERSE_PITCH    1
#define BALANCE_REMOTE_REVERSE_ROLL     1
#define BALANCE_REMOTE_REVERSE_YAW      0

#define BALANCE_REMOTE_TILT_DEADBAND     8
#define BALANCE_REMOTE_MAX_TILT_DEG      8.0f
#define BALANCE_REMOTE_YAW_DEADBAND      10
#define BALANCE_REMOTE_YAW_RATE_DPS      45.0f
#define BALANCE_REMOTE_YAW_MAX_DT_MS     200U

#define BALANCE_AXIS_TO_0_100(axis) ((uint16_t)(((int16_t)(axis) + 100) / 2))

static int16_t Balance_Clamp_Axis(int16_t axis)
{
	if(axis < -100) return -100;
	if(axis > 100) return 100;
	return axis;
}

/* Encode a small stick deadband and a gentler limit for the existing angle conversion. */
static float Balance_Tilt_To_Input(int16_t axis)
{
	const float span=4.0f*BALANCE_REMOTE_MAX_TILT_DEG;
	if(axis > BALANCE_REMOTE_TILT_DEADBAND)
		return 60.0f + (axis-BALANCE_REMOTE_TILT_DEADBAND)*span/(100-BALANCE_REMOTE_TILT_DEADBAND);
	if(axis < -BALANCE_REMOTE_TILT_DEADBAND)
		return 40.0f + (axis+BALANCE_REMOTE_TILT_DEADBAND)*span/(100-BALANCE_REMOTE_TILT_DEADBAND);
	return 50.0f;
}

/* A centered yaw stick holds the heading target instead of returning it to zero. */
static float Balance_Yaw_Target(float target, int16_t axis, uint16_t elapsed_ms)
{
	float command=0.0f;
	if(axis > BALANCE_REMOTE_YAW_DEADBAND)
		command=(float)(axis-BALANCE_REMOTE_YAW_DEADBAND)/(100-BALANCE_REMOTE_YAW_DEADBAND);
	else if(axis < -BALANCE_REMOTE_YAW_DEADBAND)
		command=(float)(axis+BALANCE_REMOTE_YAW_DEADBAND)/(100-BALANCE_REMOTE_YAW_DEADBAND);
	if(elapsed_ms > BALANCE_REMOTE_YAW_MAX_DT_MS) elapsed_ms=BALANCE_REMOTE_YAW_MAX_DT_MS;
	target += command*BALANCE_REMOTE_YAW_RATE_DPS*(float)elapsed_ms/1000.0f;
	if(target > 190.0f) return 190.0f;
	if(target < 10.0f) return 10.0f;
	return target;
}

int16_t Limit_PWM(int16_t pwm) {
    if(pwm > PWM_MAX) return PWM_MAX;
    if(pwm < PWM_MIN) return PWM_MIN;
    return pwm;
}

/*--------------------*/
/*--------俯仰--------*/
/*--------------------*/
//角度外环
float Pitch_Angle_Balance(float Angle,float Gyro) //设置的角度与角速度
{
	float PID_Angle_Out;
	PID_Pitch_Angle_err=(fly_pitch_zero+getpitch_SET)-Angle;//期望值(0点＋遥控值)-实际值
	
	if(Oil_Set > PID_Oil_Min  && fabsf(pitch_angle_balance_Ki) > (1e-6f))
  {
		// 积分分离：只有误差绝对值小于阈值，才累加积分（避免大误差饱和）
		if(fabsf(PID_Pitch_Angle_err) < ANGLE_ERR_THRESHOLD)
		{
			PID_Pitch_Angle_Integral += PID_Pitch_Angle_err;
			// 积分限幅（浮点型防溢出）
			PID_Pitch_Angle_Integral = fminf(PID_Pitch_Angle_Integral, (float)I_ANGLE_IMAX);
			PID_Pitch_Angle_Integral = fmaxf(PID_Pitch_Angle_Integral, (float)-I_ANGLE_IMAX);
		}
		//初步计算角度外环输出量（角速度）
		PID_Angle_Out=pitch_angle_balance_Kp*PID_Pitch_Angle_err+pitch_angle_balance_Ki*PID_Pitch_Angle_Integral+pitch_angle_balance_Kd*Gyro;
		// 积分抗饱和：输出的角速度超限时，反向修正积分（核心！）
		if(PID_Angle_Out > TARGET_GYRO_MAX)
		{
			PID_Pitch_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MAX) / pitch_angle_balance_Ki;
		}
		else if(PID_Angle_Out < TARGET_GYRO_MIN)
		{
			PID_Pitch_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MIN) / pitch_angle_balance_Ki;
		}	
		// 二次积分限幅
		PID_Pitch_Angle_Integral = fminf(PID_Pitch_Angle_Integral, (float)I_ANGLE_IMAX);
		PID_Pitch_Angle_Integral = fmaxf(PID_Pitch_Angle_Integral, (float)-I_ANGLE_IMAX);
		
  }
	else  PID_Pitch_Angle_Integral = 0; // 油门不足时清零积分
	
	//位置式PID公式-计算最终的输出量
	PID_Angle_Out=pitch_angle_balance_Kp*PID_Pitch_Angle_err+pitch_angle_balance_Ki*PID_Pitch_Angle_Integral+pitch_angle_balance_Kd*Gyro;
	//限幅度
	PID_Angle_Out = fminf(fmaxf(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
	
	return PID_Angle_Out;
}

//角速度内环
int16_t Pitch_Gyro_Balance(float PID_Angle_Out,float Gyro) //设置的角度与角速度
{
	float PID_PWM_Balance;
  // 计算角速度误差：目标角速度（外环输出）- 实际角速度（DMP滤波后）
  PID_Pitch_Gyro_err = PID_Angle_Out - Gyro;
  // 边界条件：油门足够 + 内环Ki有效（非零）
  if (Oil_Set > PID_Oil_Min && fabsf(pitch_gyro_balance_Ki) > (1e-6f))
  {
		// 积分分离：仅角速度误差小的时候累加积分（避免大误差饱和）
		if (fabsf(PID_Pitch_Gyro_err) < GYRO_ERR_THRESHOLD)
		{
				PID_Pitch_Gyro_Integral += PID_Pitch_Gyro_err;
				//积分限幅：双向约束，防止积分溢出
				PID_Pitch_Gyro_Integral = fminf(PID_Pitch_Gyro_Integral, I_GYRO_IMAX);
				PID_Pitch_Gyro_Integral = fmaxf(PID_Pitch_Gyro_Integral, -I_GYRO_IMAX);
		}
		//初步计算内环输出（预判，用于积分抗饱和）
		PID_PWM_Balance=pitch_gyro_balance_Kp*PID_Pitch_Gyro_err+pitch_gyro_balance_Ki*PID_Pitch_Gyro_Integral
		+pitch_gyro_balance_Kd*(PID_Pitch_Gyro_err-PID_Pitch_Gyro_err_last);

		//积分抗饱和：输出超PWM限时，反向修正积分（核心！）
		if (PID_PWM_Balance > TARGET_PWM_MAX)
		{
				PID_Pitch_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MAX) / pitch_gyro_balance_Ki;
		}
		else if (PID_PWM_Balance < TARGET_PWM_MIN)
		{
				PID_Pitch_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MIN) / pitch_gyro_balance_Ki;
		}
		
		//二次积分限幅
		PID_Pitch_Gyro_Integral = fminf(PID_Pitch_Gyro_Integral, I_GYRO_IMAX); 
    PID_Pitch_Gyro_Integral = fmaxf(PID_Pitch_Gyro_Integral, -I_GYRO_IMAX); 
		
	}
	else
	{
		//油门不足/Ki无效时，清零内环积分
		PID_Pitch_Gyro_Integral = 0.0f;
	}
	//重新计算最终输出（基于修正后的积分）
	PID_PWM_Balance=pitch_gyro_balance_Kp*PID_Pitch_Gyro_err+pitch_gyro_balance_Ki*PID_Pitch_Gyro_Integral
                  +pitch_gyro_balance_Kd*(PID_Pitch_Gyro_err-PID_Pitch_Gyro_err_last);
	// PWM限幅（内环最终输出是电机PWM，需转整型）
	PID_PWM_Balance = fminf(fmaxf(PID_PWM_Balance, (float)TARGET_PWM_MIN), (float)TARGET_PWM_MAX);
	//记录当前角速度，用于下一帧D项计算
	PID_Pitch_Gyro_err_last = PID_Pitch_Gyro_err;

	// 转整型返回（电机驱动需整型PWM）
	return (int16_t)PID_PWM_Balance;
}

/*--------------------*/
/*--------翻滚--------*/
/*--------------------*/
//角度外环
float Roll_Angle_Balance(float Angle,float Gyro) //设置的角度与角速度
{
	float PID_Angle_Out;
	PID_Roll_Angle_err=(fly_roll_zero+getroll_SET)-Angle;//期望值(0点＋遥控值)-实际值
	
	if(Oil_Set > PID_Oil_Min && fabsf(roll_angle_balance_Ki) > (1e-6f))
  {
		// 积分分离：只有误差绝对值小于阈值，才累加积分（避免大误差饱和）
		if(fabsf(PID_Roll_Angle_err) < ANGLE_ERR_THRESHOLD)
		{
			PID_Roll_Angle_Integral += PID_Roll_Angle_err;
			// 积分限幅（浮点型防溢出）
			PID_Roll_Angle_Integral = fminf(PID_Roll_Angle_Integral, (float)I_ANGLE_IMAX);
			PID_Roll_Angle_Integral = fmaxf(PID_Roll_Angle_Integral, (float)-I_ANGLE_IMAX);
		}
		//初步计算角度外环输出量（角速度）
		PID_Angle_Out=roll_angle_balance_Kp*PID_Roll_Angle_err+roll_angle_balance_Ki*PID_Roll_Angle_Integral+roll_angle_balance_Kd*Gyro;
		
		//积分抗饱和：输出的角速度超限时，反向修正积分（核心！）
		if(PID_Angle_Out > TARGET_GYRO_MAX)
		{
			PID_Roll_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MAX) / roll_angle_balance_Ki;
		}
		else if(PID_Angle_Out < TARGET_GYRO_MIN)
		{
			PID_Roll_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MIN) / roll_angle_balance_Ki;
		}	
			//二次积分限幅
		PID_Roll_Angle_Integral = fminf(PID_Roll_Angle_Integral, (float)I_ANGLE_IMAX);
		PID_Roll_Angle_Integral = fmaxf(PID_Roll_Angle_Integral, (float)-I_ANGLE_IMAX);
  }
	else  PID_Roll_Angle_Integral = 0; // 油门不足时清零积分
	
	//位置式PID公式-计算最终的输出量
	PID_Angle_Out=roll_angle_balance_Kp*PID_Roll_Angle_err+roll_angle_balance_Ki*PID_Roll_Angle_Integral+roll_angle_balance_Kd*Gyro;
	//限幅度
	PID_Angle_Out = fminf(fmaxf(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
	
	return PID_Angle_Out;
}

//角速度内环
int16_t Roll_Gyro_Balance(float PID_Angle_Out,float Gyro) //设置的角度与角速度
{
	float PID_PWM_Balance;
  // 计算角速度误差：目标角速度（外环输出）- 实际角速度（DMP滤波后）
  PID_Roll_Gyro_err = PID_Angle_Out - Gyro;
  // 边界条件：油门足够 + 内环Ki有效（非零）
  if (Oil_Set > PID_Oil_Min && fabsf(roll_gyro_balance_Ki) > (1e-6f))
  {
		// 积分分离：仅角速度误差小的时候累加积分（避免大误差饱和）
		if (fabsf(PID_Roll_Gyro_err) < GYRO_ERR_THRESHOLD)
		{
				PID_Roll_Gyro_Integral += PID_Roll_Gyro_err;
				//积分限幅：双向约束，防止积分溢出
				PID_Roll_Gyro_Integral = fminf(PID_Roll_Gyro_Integral, I_GYRO_IMAX);
				PID_Roll_Gyro_Integral = fmaxf(PID_Roll_Gyro_Integral, -I_GYRO_IMAX);
		}
		//初步计算内环输出（预判，用于积分抗饱和）
		PID_PWM_Balance=roll_gyro_balance_Kp*PID_Roll_Gyro_err+roll_gyro_balance_Ki*PID_Roll_Gyro_Integral
		+roll_gyro_balance_Kd*(PID_Roll_Gyro_err-PID_Roll_Gyro_err_last);

		//积分抗饱和：输出超PWM限时，反向修正积分（核心！）
		if (PID_PWM_Balance > TARGET_PWM_MAX)
		{
				PID_Roll_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MAX) / roll_gyro_balance_Ki;
		}
		else if (PID_PWM_Balance < TARGET_PWM_MIN)
		{
				PID_Roll_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MIN) / roll_gyro_balance_Ki;
		}
		
		//二次积分限幅
		PID_Roll_Gyro_Integral = fminf(PID_Roll_Gyro_Integral, I_GYRO_IMAX); 
    PID_Roll_Gyro_Integral = fmaxf(PID_Roll_Gyro_Integral, -I_GYRO_IMAX); 
		
	}
	else
	{
		//油门不足/Ki无效时，清零内环积分
		PID_Roll_Gyro_Integral = 0.0f;
	}
	//重新计算最终输出（基于修正后的积分）
	PID_PWM_Balance=roll_gyro_balance_Kp*PID_Roll_Gyro_err+roll_gyro_balance_Ki*PID_Roll_Gyro_Integral
                  +roll_gyro_balance_Kd*(PID_Roll_Gyro_err-PID_Roll_Gyro_err_last);
	//PWM限幅（内环最终输出是电机PWM，需转整型）
	PID_PWM_Balance = fminf(fmaxf(PID_PWM_Balance, (float)TARGET_PWM_MIN), (float)TARGET_PWM_MAX);
	//记录当前角速度，用于下一帧D项计算
	PID_Roll_Gyro_err_last = PID_Roll_Gyro_err;

	//转整型返回（电机驱动需整型PWM）
	return (int16_t)PID_PWM_Balance;
	
}

/*--------------------*/
/*--------航向--------*/
/*--------------------*/
//角度外环
float Yaw_Angle_Balance(float Angle,float Gyro) //设置的角度与角速度
{
	float PID_Angle_Out;
	PID_Yaw_Angle_err=(fly_yaw_zero+getyaw_SET)-Angle;//期望值(0点＋遥控值)-实际值
	if(PID_Yaw_Angle_err > 180.0f) PID_Yaw_Angle_err -= 360.0f;
	else if(PID_Yaw_Angle_err < -180.0f) PID_Yaw_Angle_err += 360.0f;
	
	if(Oil_Set > PID_Oil_Min && fabsf(yaw_angle_balance_Ki) > (1e-6f))
  {
		// 积分分离：只有误差绝对值小于阈值，才累加积分（避免大误差饱和）
		if(fabsf(PID_Yaw_Angle_err) < ANGLE_ERR_THRESHOLD)
		{
			PID_Yaw_Angle_Integral += PID_Yaw_Angle_err;
			// 积分限幅（浮点型防溢出）
			PID_Yaw_Angle_Integral = fminf(PID_Yaw_Angle_Integral, (float)I_ANGLE_IMAX);
			PID_Yaw_Angle_Integral = fmaxf(PID_Yaw_Angle_Integral, (float)-I_ANGLE_IMAX);
		}
		//初步计算角度外环输出量（角速度）
		PID_Angle_Out=yaw_angle_balance_Kp*PID_Yaw_Angle_err+yaw_angle_balance_Ki*PID_Yaw_Angle_Integral+yaw_angle_balance_Kd*Gyro;
		// 积分抗饱和：输出的角速度超限时，反向修正积分（核心！）
		if(PID_Angle_Out > TARGET_GYRO_MAX)
		{
			PID_Yaw_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MAX) / yaw_angle_balance_Ki;
		}
		else if(PID_Angle_Out < TARGET_GYRO_MIN)
		{
			PID_Yaw_Angle_Integral -= (PID_Angle_Out - TARGET_GYRO_MIN) / yaw_angle_balance_Ki;
		}	
		
				//二次积分限幅
		PID_Yaw_Angle_Integral = fminf(PID_Yaw_Angle_Integral, (float)I_ANGLE_IMAX);
		PID_Yaw_Angle_Integral = fmaxf(PID_Yaw_Angle_Integral, (float)-I_ANGLE_IMAX);
		
  }
	else  PID_Yaw_Angle_Integral = 0; // 油门不足时清零积分
	
	//位置式PID公式-计算最终的输出量
	PID_Angle_Out=yaw_angle_balance_Kp*PID_Yaw_Angle_err+yaw_angle_balance_Ki*PID_Yaw_Angle_Integral+yaw_angle_balance_Kd*Gyro;
	//限幅度
	PID_Angle_Out = fminf(fmaxf(PID_Angle_Out, (float)TARGET_GYRO_MIN), (float)TARGET_GYRO_MAX);
	return PID_Angle_Out;
}

//角速度内环
int16_t Yaw_Gyro_Balance(float PID_Angle_Out,float Gyro) //设置的角度与角速度
{
	float PID_PWM_Balance;
  // 计算角速度误差：目标角速度（外环输出）- 实际角速度（DMP滤波后）
  PID_Yaw_Gyro_err = PID_Angle_Out - Gyro;
  // 边界条件：油门足够 + 内环Ki有效（非零）
  if (Oil_Set > PID_Oil_Min && fabsf(yaw_gyro_balance_Ki) > (1e-6f))
  {
		// 积分分离：仅角速度误差小的时候累加积分（避免大误差饱和）
		if (fabsf(PID_Yaw_Gyro_err) < GYRO_ERR_THRESHOLD)
		{
				PID_Yaw_Gyro_Integral += PID_Yaw_Gyro_err;
				//积分限幅：双向约束，防止积分溢出
				PID_Yaw_Gyro_Integral = fminf(PID_Yaw_Gyro_Integral, I_GYRO_IMAX);
				PID_Yaw_Gyro_Integral = fmaxf(PID_Yaw_Gyro_Integral, -I_GYRO_IMAX);
		}
		//初步计算内环输出（预判，用于积分抗饱和）
		PID_PWM_Balance=yaw_gyro_balance_Kp*PID_Yaw_Gyro_err+yaw_gyro_balance_Ki*PID_Yaw_Gyro_Integral
		+yaw_gyro_balance_Kd*(PID_Yaw_Gyro_err-PID_Yaw_Gyro_err_last);

		//积分抗饱和：输出超PWM限时，反向修正积分（核心！）
		if (PID_PWM_Balance > TARGET_PWM_MAX)
		{
				PID_Yaw_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MAX) / yaw_gyro_balance_Ki;
		}
		else if (PID_PWM_Balance < TARGET_PWM_MIN)
		{
				PID_Yaw_Gyro_Integral -= (PID_PWM_Balance - TARGET_PWM_MIN) / yaw_gyro_balance_Ki;
		}
		
		//二次积分限幅
		PID_Yaw_Gyro_Integral = fminf(PID_Yaw_Gyro_Integral, I_GYRO_IMAX); 
    PID_Yaw_Gyro_Integral = fmaxf(PID_Yaw_Gyro_Integral, -I_GYRO_IMAX); 
		
	}
	else
	{
		// 油门不足/Ki无效时，清零内环积分
		PID_Yaw_Gyro_Integral = 0.0f;
	}
	//重新计算最终输出（基于修正后的积分）
	PID_PWM_Balance=yaw_gyro_balance_Kp*PID_Yaw_Gyro_err+yaw_gyro_balance_Ki*PID_Yaw_Gyro_Integral
		+yaw_gyro_balance_Kd*(PID_Yaw_Gyro_err-PID_Yaw_Gyro_err_last);
	//PWM限幅（内环最终输出是电机PWM，需转整型）
	PID_PWM_Balance = fminf(fmaxf(PID_PWM_Balance, (float)TARGET_PWM_MIN), (float)TARGET_PWM_MAX);
	// 记录当前角速度，用于下一帧D项计算
	PID_Yaw_Gyro_err_last = PID_Yaw_Gyro_err;

	//转整型返回（电机驱动需整型PWM）
	return (int16_t)PID_PWM_Balance;
	
}

void Fly_task(void)
{
    if (!Attitude_IsFresh())
    {
        unlock_needs_release = 1;
        Lock = 1;
        Oil_Get = Oil_Set = 0;
        PWM_OUT1 = PWM_OUT2 = PWM_OUT3 = PWM_OUT4 = 0;
        PID_Pitch_Angle_Integral = PID_Roll_Angle_Integral = PID_Yaw_Angle_Integral = 0;
        PID_Pitch_Gyro_Integral = PID_Roll_Gyro_Integral = PID_Yaw_Gyro_Integral = 0;
        PID_Pitch_Gyro_err_last = PID_Roll_Gyro_err_last = PID_Yaw_Gyro_err_last = 0;
        Motor_Control();
        return;
    }
	//俯仰外环输出
	 Pitch_Angle_out=Pitch_Angle_Balance(Pitch,gy);   
	 Roll_Angle_out=Roll_Angle_Balance(Roll,gx);     
	 Yaw_Angle_out=Yaw_Angle_Balance(Yaw,gz);       
	//俯仰内环输出(直接叠加作用到PWM)
	 Pitch_Balance_out=Pitch_Gyro_Balance(Pitch_Angle_out,gy);
	 Roll_Balance_out=Roll_Gyro_Balance(Roll_Angle_out,gx);
	 Yaw_Balance_out=Yaw_Gyro_Balance(Yaw_Angle_out,gz);
	
	/*组合算法*/
	if(Oil_Set>PID_Oil_Min)  //油门大于10开始PID调试
	{
		PWM_OUT1= Oil_Set+Pitch_Balance_out-Roll_Balance_out+Yaw_Balance_out;
		PWM_OUT2= Oil_Set+Pitch_Balance_out+Roll_Balance_out-Yaw_Balance_out;
		PWM_OUT3= Oil_Set-Pitch_Balance_out-Roll_Balance_out-Yaw_Balance_out;
		PWM_OUT4= Oil_Set-Pitch_Balance_out+Roll_Balance_out+Yaw_Balance_out;
		/*PWM限幅*/
		PWM_OUT1=Limit_PWM(PWM_OUT1);
		PWM_OUT2=Limit_PWM(PWM_OUT2);
	  PWM_OUT3=Limit_PWM(PWM_OUT3);
		PWM_OUT4=Limit_PWM(PWM_OUT4);
	}
	else
	{
		PWM_OUT1= Oil_Set;
		PWM_OUT2= Oil_Set;
		PWM_OUT3= Oil_Set;
		PWM_OUT4= Oil_Set;
	}
	
	Motor_Control();
}

void LED_Detect(void)
{
	//如果无线通信失败，LED为每500ms闪一次
	if(connect_protect==1 && Battery_Protect==0) LED_Time(500);
	//如果无线通信成功，LED长亮
	else if(connect_protect==0 && Battery_Protect==0)	LED_Light(1);	
	//如果电池电量不足，LED每50ms闪一次
	else if(Battery_Protect)	LED_Time(50);	
  
}


/*---数据处理---*/
void Data_Proc(void)
{
	BlueSerial_Proc();	/* online PID tuning via Bluetooth, 20 ms */
 /*---接收数据处理---*/
	//规定：getpitch的值范围为0->100，由上到下，抖动限制范围为40-60
	//规定：getroll的值范围为0->100，由右到左，抖动限制范围为40-60
	//该部分处理的目的是将其余数值线性转化为角度。角度最大为30度。
	//getpitch：      由40->0    角度getpitch_SET由0至-10
	//                由60->100  角度getpitch_SET由0至10
	//
	//getroll：       由60->100  角度getroll_SET由0至-10
	//                由40->0    角度getroll_SET由0至10
	if(getpitch>40 && getpitch<60) getpitch_SET=0;
	if(getpitch>=0 && getpitch<=40)  getpitch_SET=0.25*getpitch-10;
	if(getpitch>=60 && getpitch<=100)getpitch_SET=0.25*getpitch-15;
	if(getroll>40 && getroll<60) getroll_SET=0;
	if(getroll>=0 && getroll<=40)  getroll_SET=-0.25*getroll+10;
	if(getroll>=60 && getroll<=100)getroll_SET=-0.25*getroll+15;
	//航向
	getyaw_SET=getyaw-100;

	/*---油门限制---*/
	Oil_Set=Oil_Get*36;

	/*---机械零点以及PID调参---*/
	if(index_type!=0)
	{
		switch(index_type)
		{
			case 1://俯仰机械零点
				fly_pitch_zero=(PID_Data_Send[0]+(PID_Data_Send[1])*0.01)-100.0;	
				break;	
			case 2://翻滚机械零点
				fly_roll_zero=(PID_Data_Send[0]+(PID_Data_Send[1])*0.01)-100.0;	
				break;	
			case 3://航向机械零点重置
				fly_yaw_zero_bit=(PID_Data_Send[0]+(PID_Data_Send[1])*0.01)-100.0;		
			  if(fly_yaw_zero_bit!=fly_yaw_zero_bit_old)
				{
					/*重置航向零点*/
					fly_yaw_zero=Yaw;
					fly_yaw_zero_bit_old=fly_yaw_zero_bit;
				}
				break;	
			case 4://外环俯仰的P
		    PID_Data[0][0]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 5://外环俯仰的I
		    PID_Data[0][1]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 6://外环俯仰的D
		    PID_Data[0][2]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 7://外环翻滚的P
		    PID_Data[1][0]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 8://外环翻滚的I
		    PID_Data[1][1]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 9://外环翻滚的D
		    PID_Data[1][2]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 10://外环航向的P
		    PID_Data[2][0]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 11://外环航向的I
		    PID_Data[2][1]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 12://外环航向的D
		    PID_Data[2][2]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			//内环
			case 13://内环俯仰的P
				PID_Data[3][0]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);	
				break;	
			case 14://内环俯仰的I
				PID_Data[3][1]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);	
				break;	
			case 15://内环俯仰的D
				PID_Data[3][2]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);		
				break;	
			case 16://外环翻滚的P
		    PID_Data[4][0]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 17://外环翻滚的I
		    PID_Data[4][1]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 18://外环翻滚的D
		    PID_Data[4][2]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 19://外环航向的P
		    PID_Data[5][0]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 20://外环航向的I
		    PID_Data[5][1]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
			case 21://外环航向的D
		    PID_Data[5][2]=(PID_Data_Send[0]*0.1+(PID_Data_Send[1])*0.001);
	      break;
		}
			//设置当前的PID参数
		pitch_angle_balance_Kp=PID_Data[0][0];
		pitch_angle_balance_Ki=PID_Data[0][1];
		pitch_angle_balance_Kd=PID_Data[0][2];
		roll_angle_balance_Kp =PID_Data[1][0];
		roll_angle_balance_Ki =PID_Data[1][1];
		roll_angle_balance_Kd =PID_Data[1][2];
		yaw_angle_balance_Kp  =PID_Data[2][0];
		yaw_angle_balance_Ki  =PID_Data[2][1];
		yaw_angle_balance_Kd  =PID_Data[2][2];
		pitch_gyro_balance_Kp =PID_Data[3][0];
		pitch_gyro_balance_Ki =PID_Data[3][1];
		pitch_gyro_balance_Kd =PID_Data[3][2];
		roll_gyro_balance_Kp  =PID_Data[4][0];
		roll_gyro_balance_Ki  =PID_Data[4][1];
		roll_gyro_balance_Kd  =PID_Data[4][2];
		yaw_gyro_balance_Kp   =PID_Data[5][0];
		yaw_gyro_balance_Ki   =PID_Data[5][1];
		yaw_gyro_balance_Kd   =PID_Data[5][2];
	}
	/*---电量检测数据处理---*/
	if(Battery_average<=3.50) 	Battery_Protect=1;
	else	Battery_Protect=0;
	
}

void Attitude_Data_Proc(void)
{
	//获得姿态角度与角速度
	 float p, r, y, xrate, yrate, zrate;
     if (MPU6050_DMP_Get_Sample(&p, &r, &y, &xrate, &yrate, &zrate) == 0 &&
         isfinite(p) && isfinite(r) && isfinite(y) &&
         isfinite(xrate) && isfinite(yrate) && isfinite(zrate))
     {
         uint32_t primask = __get_PRIMASK();
         __disable_irq();
         Pitch = p; Roll = r; Yaw = y;
         gx = xrate; gy = yrate; gz = zrate;
         attitude_last_success = HAL_GetTick();
         attitude_valid = 1;
         __set_PRIMASK(primask);
     }
}

/*---数据显示---*/
void Data_Display(void)
{
	/*油门显示*/
	OLED_Printf(0,0,OLED_6X8,"o:%04d",Oil_Set);
	/*各电机的PWM显示*/
	OLED_Printf(0,8,OLED_6X8,"1:%04d",PWM_OUT1);
	OLED_Printf(0,16,OLED_6X8,"2:%04d",PWM_OUT2);
	OLED_Printf(0,24,OLED_6X8,"3:%04d",PWM_OUT3);
	OLED_Printf(0,32,OLED_6X8,"4:%04d",PWM_OUT4);
	/*当前电量*/
	OLED_Printf(0,55,OLED_6X8,"B:%03.2f",Battery_average);
  /*初始角度*/
	OLED_Printf(50,0,OLED_6X8,"pitch:%04.2f",fly_pitch_zero);
	OLED_Printf(50,8,OLED_6X8,"roll:%04.2f",fly_roll_zero);
	OLED_Printf(50,16,OLED_6X8,"yaw:%04.2f",fly_yaw_zero);
  /*PD调试*/
	OLED_Printf(46,28,OLED_6X8,"Y:%04.3f",gy);
	OLED_Printf(88,28,OLED_6X8,"P:%04.2f",Pitch);
	OLED_Printf(46,38,OLED_6X8,"X:%04.2f",gx);
	OLED_Printf(88,38,OLED_6X8,"R:%04.2f",Roll);
	OLED_Printf(46,48,OLED_6X8,"Z:%04.2f",gz);
	OLED_Printf(88,48,OLED_6X8,"Y:%04.2f",Yaw);
	OLED_UpdateAsync();
}

/*---数据接收---*/
void Data_Receive(void)
{
	int16_t lh;
	int16_t lv;
	int16_t rh;
	int16_t rv;
	uint8_t key;
	uint16_t elapsed_ms;

	if(NRF24L01_R_IRQ()==0)
	{
		if(Receive_NRF24L01_Data(Receive_Data))
		{
			/* payload[0] is reserved; [1..4] are signed axes; [5] is KEY. */
			lh=Balance_Clamp_Axis((int8_t)Receive_Data[1]);
			lv=Balance_Clamp_Axis((int8_t)Receive_Data[2]);
			rh=Balance_Clamp_Axis((int8_t)Receive_Data[3]);
			rv=Balance_Clamp_Axis((int8_t)Receive_Data[4]);
			key=Receive_Data[5];
			elapsed_ms=connect_time;
			/* Throttle: this remote encodes FORWARD push as NEGATIVE lv; push forward = more throttle, centered/pulled back = 0. */
#if BALANCE_REMOTE_REVERSE_THROTTLE
			if(lv < -BALANCE_REMOTE_TILT_DEADBAND)
			{
				Oil_Get=(uint8_t)((-lv-BALANCE_REMOTE_TILT_DEADBAND)*80/(100-BALANCE_REMOTE_TILT_DEADBAND));
			}
			else
			{
				Oil_Get=0;
			}
#else
			Oil_Get=BALANCE_AXIS_TO_0_100(lv)*80/100;
#endif
			getpitch=Balance_Tilt_To_Input(rv);
			getroll=Balance_Tilt_To_Input(rh);

#if BALANCE_REMOTE_REVERSE_PITCH
			getpitch=100-getpitch;
#endif
#if BALANCE_REMOTE_REVERSE_ROLL
			getroll=100-getroll;
#endif
#if BALANCE_REMOTE_REVERSE_YAW
			lh=-lh;
#endif

			if(key!=8) unlock_needs_release=0;
			if(key==7)
			{
				Lock=1;
				Oil_Get=0;
				Oil_Set=0;
				getyaw=100.0f;
			}
			else if((key==8) && (Oil_Get<=5))
			{
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                if (!unlock_needs_release && Attitude_IsFresh())
                {
                    /* Capture heading before an explicit, fresh-sample unlock. */
                    Oil_Set=0;
                    fly_yaw_zero=Yaw;
                    getyaw=100.0f;
                    Lock=0;
                }
                __set_PRIMASK(primask);
			}
			else if((Lock==0) && (Oil_Get>10))
			{
				getyaw=Balance_Yaw_Target(getyaw,lh,elapsed_ms);
			}

			/* This remote does not provide flight-controller PID setting data. */
			index_type=0;
			PID_Data_Send[0]=0;
			PID_Data_Send[1]=0;
			connect_value=0;
			connect_time=0;
			connect_protect=0;
			/* Telemetry downlink: reply once per received Mode==0x01 control frame (~10 Hz). */
			if(Receive_Data[0]==0x01)
			{
				uint8_t i;
				Send_Data[0]=0x02;
				Send_Data[1]=(uint8_t)(PWM_OUT1/36);		/* motor PWM 0..3600 scaled to 0..100 */
				Send_Data[2]=(uint8_t)(PWM_OUT2/36);		/* same scaling, remote shows PWM:%+05d */
				Send_Data[3]=0;
				*(float*)&Send_Data[4]=Pitch;				/* little-endian float: pitch angle */
				*(float*)&Send_Data[8]=Roll;				/* little-endian float: roll angle */
				*(float*)&Send_Data[12]=Battery_average;	/* little-endian float: battery volts */
				for(i=16;i<32;i++) Send_Data[i]=0;			/* reserved bytes */
				Send(Send_Data);								/* no retry: next frame re-sends */
			}
		}
	}
}

/*数据发送*/
void Data_Send(void)
{
	Send(Send_Data);
}

void Connect_Portect_Test(void)
{
	/*赋值*/
	if(connect_bit==0)
	{
		connect_oldvalue=connect_value; //把当前的值储存起来用于下次的比较。			
		connect_bit=1;
	}
	/*比较*/
	else
	{
		if(connect_oldvalue==connect_value)	connect_protect=1;
		else	connect_protect=0;			
		
		connect_bit=0;
	}
}
