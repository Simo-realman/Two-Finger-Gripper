#include "fml_invoke_algo.h"
#include "fml_storage.h"
#include "fml_motor.h"
#include "gripper_algo.h"
#include <stdlib.h>

#define DBG_TAG "algo."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

uint8_t algo_init_flag = 0;
Trajectory g_traj_palan; //算法实例
filter_st curr_filter; //滤波

extern s_motor_state motor_state; //电机状态

//算法急停规划
void fml_algo_stop_init(void)
{
	float _pos_now = motor_state.postion; //当前关节角度
	float _speed_now = motor_state.speed;	//当前关节速度
	//g_traj_palan = gripper_emergency_stop_traj_planning((float)PLAN_CYCLE_MS/1000.0, HOME_THETA, _pos_now, _speed_now);
}

//算法运动规划-梯形加减速
void fml_algo_plan_init(uint16_t _pos_tag, uint16_t _speed, uint16_t _force)
{
	motor_state.force = 0; //清空实时力
	motor_state.force_targ = _force; //目标力
	float start_dis = gripper_kine(0, motor_state.postion); //正解起始位置
	float speed = MAX_VEL*_speed/1000.0; //速度百分比
	float postion = RM_RANGE_MAX*(_pos_tag/1000.0); //1000<->65目标位置
	//g_traj_palan = gripper_traj_planning((float)PLAN_CYCLE_MS/1000.0, HOME_THETA, speed, start_dis, postion);
}

//算法运动规划-OTG连续规划
void fml_algo_plan_otg_init(uint16_t _pos_tag, uint16_t _speed, uint16_t _force)
{
	motor_state.force = 0; //清空实时力
	motor_state.force_targ = _force; //目标力
	motor_state.speed_targ = _speed/1000.0; //速度百分比
	motor_state.postion_targ = RM_RANGE_MAX*(_pos_tag/1000.0); //1000<->65目标位置
}

//滤波器初始化
void fml_filter_init(void)
{
	//滤波器初始化
	utils_filter_init(&curr_filter);
	//滤波器设计
	utils_second_order_lowpass_filter_design((float)PLAN_CYCLE_MS/1000.0, 10.0, 0.25, &curr_filter);
}

//算法初始化
void fml_algo_init(void)
{
	algo_init_flag = 1;
	//算法初始化
	otg_init((float)PLAN_CYCLE_MS/1000.0, motor_state.postion, 0, MAX_ACC, MAX_VEL); //初始角度
	//LOG_D("algo init %d\r\n", motor_state.postion);
}

