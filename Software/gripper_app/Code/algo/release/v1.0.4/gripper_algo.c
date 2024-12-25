#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "gripper_algo.h"

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif
#define sign(x) ((x) >= 0 ? 1 : -1)
#define abs(x) ((x) >= 0 ? (x) : -(x))
#define RAD2DEG (180.0 / M_PI)
#define DEG2RAD (M_PI / 180.0)

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
    Trajectory traj = {0};
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

    traj.t_sum = T;
    traj.type = traj_type;
    traj.t = 0.0;
    traj.Ta = Ta;
    traj.Tm = Tm;
    traj.aMax = a_max;
    traj.vMax = v_max;
    traj.dt = dt;
    traj.sgn = sgn;
    traj.p0 = p0;
    traj.pf = p1;

    return traj;
}

/**
 * @brief 夹爪轨迹运行
 * 
 * @param traj 
 * @param q_out           输出的关节角度, 单位deg. 
 * @param dq_out          输出的关节速度, 单位deg/s. 
 * @return int 
 *        0: 运行中
 *        1: 结束
 */
int gripper_traj_run(Trajectory* traj, float *q_out, float *dq_out)
{
    float a_max = traj->aMax;
    float v_max = traj->vMax;
    float Ta    = traj->Ta;
    float Tm    = traj->Tm;
    float sgn   = traj->sgn;
    float p0    = traj->p0;
    float p1    = traj->pf; 
    float acc   = 0.0;
    float vel   = 0.0;
    float pos   = 0.0;
    float t     = traj->t;

    traj->t += traj->dt;

    int traj_type = traj->type;
    if ( traj_type == 0 ) // 可达最大速度
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

    }
    else
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
    }

    if ( t >= traj->t_sum )
    {
        *q_out = p1;
        *dq_out = 0.0;
        return 1;
    }

    *q_out = pos;
    *dq_out = vel;
    return 0;
}

/**
 * @brief 急停轨迹规划
 * 
 * @param dt            控制周期, 单位s. 
 * @param home_theta    零位时, 关节角度, 单位deg. 
 * @param current_theta 当前关节角度(traj.p[i]), 单位deg. 
 * @param current_vel   当前关节速度(traj.v[i]), 单位deg/s. 
 * @return Trajectory    注意: 调用急停之前, 若复用上一轨迹, 需要先释放Trajectory结构体的内存.
 */
Trajectory gripper_emergency_stop_traj_planning(float dt, float home_theta, float current_theta, float current_vel)
{
    float theta_min = gripper_ikine(home_theta, 0.0);
    float theta_max = gripper_ikine(home_theta, MAX_DIS);

    Trajectory traj = {0};
    float max_acc = MAX_ACC * 1.5;
    float sgn = -(float)(sign(current_vel));

    traj.n = (int)(fabs(current_vel) / max_acc / dt) + 1;
    traj.dt = dt;
    traj.aMax = max_acc;
    traj.sgn = sgn;
    traj.curr_q = current_theta;
    traj.curr_dq = current_vel;
    traj.theta_min = theta_min;
    traj.theta_max = theta_max;

    return traj;
}

/**
 * @brief 急停轨迹执行
 * 
 * @param traj 
 * @param q_out          输出的关节角度, 单位deg. 
 * @param dq_out         输出的关节速度, 单位deg/s. 
 * @return int 
 *        0: 运行中
 *        1: 结束
 *       -1: 因限位停止
 */
int gripper_emergency_stop_traj_run(Trajectory* traj, float *q_out, float *dq_out)
{
    float max_acc = traj->aMax;
    float dt = traj->dt;

    int i = traj->count;
    traj->t = i * dt;
    
    float a = max_acc * traj->sgn;
    float v = traj->curr_dq + i * a * dt;
    float p = traj->curr_q + i * traj->curr_dq * dt + 0.5 * a * dt * dt;

    if (p > traj->theta_max)
    {
        p = traj->theta_max;

        *q_out = p;
        *dq_out = 0.0;
        return -1;

    } else if (p < traj->theta_min) {
        p = traj->theta_min;

        *q_out = p;
        *dq_out = 0.0;
        return -1;
    } else if (i >= traj->n) {
        *q_out = p;
        *dq_out = 0.0;
        return 1;
    }

    *q_out = p;
    *dq_out = v;
    traj->count++;

    return 0;
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
    current_theta = -current_theta;
    float closed_dis = CLOSED_DIS;
    float max_dis = MAX_DIS / 2.0;
    float fixed_link_a = LINK_A_LENGTH;
    float fixed_link_b = LINK_B_LENGTH;

    float theta_zero =  acos((pow((closed_dis),2) + pow(fixed_link_b,2) - pow(fixed_link_a, 2)) / (2 * fixed_link_b * (closed_dis)));

    float theta = (current_theta - home_theta) * DEG2RAD + theta_zero;
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

    float theta_zero =  acos((pow((closed_dis),2) + pow(fixed_link_b,2) - pow(fixed_link_a, 2)) / (2 * fixed_link_b * (closed_dis)));
    float theta =  acos((pow((closed_dis + dis),2) + pow(fixed_link_b,2) - pow(fixed_link_a, 2)) / (2 * fixed_link_b * (closed_dis + dis)));
    return (-(theta - theta_zero + home_theta) * RAD2DEG);
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


/**
 * @brief Initialize the filter data structure.
 *
 * @param data
 */
void utils_filter_init(filter_st *data)
{
    memset(data, 0, sizeof(filter_st));
}


/**
 * @brief second order low-pass filter design, tustin method.
 *
 * @param dT         unit: s
 * @param freq       unit: Hz    cut-off freq.
 * @param data
 */
void utils_second_order_lowpass_filter_design(float dT, float freq, float damping, filter_st *data)
{
    float wn = 2 * M_PI * freq;

    float tmp1 = dT * dT * wn * wn;
    float tmp2 = 4 * damping * dT * wn;

    float a0 = tmp1 + tmp2 + 4.0;

    data->a[0] = 1.0;
    data->a[1] = (2 * tmp1 - 8.0) / a0;
    data->a[2] = (tmp1 - tmp2 + 4.0) / a0;
    data->b[0] = tmp1 / a0;
    data->b[1] = 2 * tmp1 / a0;
    data->b[2] = data->b[0];
}

/**
 * @brief second order filter function.
 *
 * @param input
 * @param data
 * @return float
 */
float utils_second_order_filter(float input, filter_st *data)
{
    float output = data->b[0] * input + data->b[1] * data->x[0] + data->b[2] * data->x[1] - data->a[1] * data->y[0] - data->a[2] * data->y[1];

    data->y[1] = data->y[0];
    data->y[0] = output;

    data->x[1] = data->x[0];
    data->x[0] = input;

    return output;
}
