#include <math.h>
#include <stdlib.h>
#include "gripper_algo.h"

#define DBG_TAG "ALGO"
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif
#define sign(x) ((x) >= 0 ? 1 : -1)
#define abs(x) ((x) >= 0 ? (x) : -(x))
#define RAD2DEG (180.0 / M_PI)
#define DEG2RAD (M_PI / 180.0)

/**
 * @brief 释放轨迹结构体的内存
 * 
 * @param traj 
 */
void trajectory_destroy(Trajectory *traj)
{
    free(traj->p);
    free(traj->v);
    //free(traj->a);
    traj->n = 0;

    traj->p = NULL;
    traj->v = NULL;
    //traj->a = NULL;
}

/**
 * @brief 梯形轨迹规划
 * 
 * @param dt 
 * @param p0 
 * @param p1 
 * @param v_max 
 * @param a_max 
 * @return Trajectory 
 */
Trajectory trajectory_planning_profile_trap_velocity(float dt, float p0, float p1, float v_max, float a_max)
{
    Trajectory traj;
    int traj_type = 0; // 0 for constant acceleration, 1 for constant jerk
    float dP = p1 - p0;
    float sgn = sign(dP);
    float Ta = v_max / a_max;
    float Tm = (abs(dP) - a_max * Ta * Ta) / v_max;

    float T = 0.0;
    if ( Tm > 0.0)
    {
        T = Tm + 2.0 * Ta;
        traj_type = 0;
    }
    else
    {
        Ta = sqrt(abs(dP) / abs(a_max));
        v_max = Ta * abs(a_max);

        T = 2 * Ta;
        traj_type = 1;
    }

    int point_count = (int)(T / dt);
    traj.n = point_count + 1;
    traj.p = (float*)malloc((point_count + 1) * sizeof(float));
    traj.v = (float*)malloc((point_count + 1) * sizeof(float));
    //traj.a = (float*)malloc((point_count + 1) * sizeof(float));

		//LOG_D(":%d \r\n", point_count);
		
    float   t = 0.0;
    float acc = 0.0;
    float vel = 0.0;
    float pos = 0.0;
    if ( traj_type == 0 ) // 可达最大速度
    {
        for(int i = 0; i < point_count; i++)
        {


            if ( t <= Ta )
            {
                acc = a_max * sgn;
                vel = acc * t;
                pos = p0 + 0.5 * vel * t;
            }
            else if ( t <= Tm + Ta )
            {
                acc = 0.0;
                vel = v_max * sgn;
                pos = p0 + 0.5 * a_max * Ta * Ta * sgn + vel * (t - Ta);
            }
            else
            {
                acc = -a_max * sgn;
                vel = v_max * sgn + acc * (t - Ta - Tm);
                pos = p0 + 0.5 * a_max * Ta * Ta * sgn + v_max * Tm * sgn + 0.5 * (vel + v_max * sgn) * (t - Ta - Tm);
            }

            t = (i+1) * dt;

            traj.p[i] = pos;
            traj.v[i] = vel;
            //traj.a[i] = acc;
        }

    }
    else
    {
        for (int i = 0; i < point_count; i++)
        {
            if( t <= Ta)
            {
                acc = a_max * sgn;
                vel = acc * t;
                pos = p0 + 0.5 * vel * t;
            }
            else
            {
                acc = -a_max * sgn; 
                vel = v_max * sgn + acc * (t - Ta);
                pos = p0 + 0.5 * a_max * Ta * Ta * sgn + 0.5 * (vel + v_max * sgn) * (t - Ta);
            }

            t = (i+1) * dt;

            traj.p[i] = pos;
            traj.v[i] = vel;
            //traj.a[i] = acc;
        }
    }

    traj.p[point_count] = p1;
    traj.v[point_count] = 0.0;
    //traj.a[point_count] = traj.a[point_count - 1];
    return traj;
}

/**
 * @brief 正运动学, 由关节角度计算夹爪开合距离.
 * 
 * @param home_theta     零位时, 关节角度, 单位deg.
 * @param current_theta  当前关节角度, 单位deg.
 * @return float        单位mm. 
 */
float gripper_kine(float home_theta, float current_theta)
{
    float closed_dis = CLOSED_DIS;
    float max_dis = MAX_DIS / 2.0;
    float fixed_link_a = LINK_A_LENGTH;
    float fixed_link_b = LINK_B_LENGTH;

    float theta = (home_theta - current_theta) * DEG2RAD;
    float angle_b, angle_c;
    angle_b = asin(fixed_link_b * sin(theta) / fixed_link_a);
    angle_c = M_PI - angle_b - theta;
    float dis =  sqrt(pow(fixed_link_a,2) + pow(fixed_link_b,2) - 2 * fixed_link_a * fixed_link_b * cos(angle_c)) - closed_dis;
    return dis * 2.0;
}

/**
 * @brief 逆运动学, 由夹爪开合距离计算关节角度.
 * 
 * @param home_theta     零位时, 关节角度, 单位deg.
 * @param current_dis    当前夹爪开合距离, 单位mm. 
 * @return float        单位deg. 
 */
float gripper_ikine(float home_theta,float current_dis)
{
    float closed_dis = CLOSED_DIS;
    float max_dis = MAX_DIS / 2.0;
    float fixed_link_a = LINK_A_LENGTH;
    float fixed_link_b = LINK_B_LENGTH;
    float dis = current_dis / 2.0;

    float theta =  acos((pow((closed_dis + dis),2) + pow(fixed_link_b,2) - pow(fixed_link_a, 2)) / (2 * fixed_link_b * (closed_dis + dis)));
    return (home_theta - theta * RAD2DEG);
}

/**
 * @brief 夹爪轨迹规划
 * 
 * @param dt            控制周期, 单位s.
 * @param home_theta    零位时, 关节角度, 单位deg. 
 * @param dq_max        关节速度 = MAX_VEL * 速度比例, 单位deg/s.
 * @param start_dis     起始开合距离, 单位mm. 
 * @param end_dis       终止开合距离, 单位mm. 
 * @return Trajectory 
 */
Trajectory gripper_traj_planning(float dt, float home_theta, float dq_max, float start_dis, float end_dis)
{
    if (dq_max > MAX_VEL)
    {
        dq_max = MAX_VEL;
    }

    if (end_dis > MAX_DIS)
    {
        end_dis = MAX_DIS;
    }

    if (start_dis < 0)
    {
        start_dis = 0;
    }

    float start_theta = gripper_ikine(home_theta, start_dis);
    float end_theta   = gripper_ikine(home_theta, end_dis);
    return trajectory_planning_profile_trap_velocity(dt, start_theta, end_theta, dq_max, MAX_ACC);
}