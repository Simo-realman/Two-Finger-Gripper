#include "pid.h"

PIDController pid;
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// PID初始化
void init_pid(PIDController *pid, float kp, float ki, float kd, float limit) 
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->limit = limit;
	pid->prev_error = 0.0;
	pid->integral = 0.0;
}

// PID算法
float calculate_pid(PIDController *pid, float setpoint, float current_value) 
{
	//误差值
	float error = setpoint - current_value;
	//误差积分
	pid->integral += error;
	pid->integral = _constrain(pid->integral, -pid->limit, pid->limit);
	//误差微分
	float derivative = error - pid->prev_error;
	pid->prev_error = error;
	//输出
	float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
	out = _constrain(out, -pid->limit, pid->limit);
	
	return out;
}




