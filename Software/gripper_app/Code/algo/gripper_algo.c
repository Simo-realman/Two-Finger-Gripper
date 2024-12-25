#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "gripper_algo.h"

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif
#define sign(x) ((x) >= 0 ? 1 : -1)
#define abs(x) ((x) >= 0 ? (x) : -(x))
#define RAD2DEG (180.0 / M_PI)
#define DEG2RAD (M_PI / 180.0)

static otg data = {0}; 
static float time_1[4] = {0}, time_2[4] = {0}, time_3[4] = {0}, time_4[4] = {0}, time_5[4] = {0}; //存放不同轮廓对应的时间
static float acce_1[4] = {0}, acce_2[4] = {0}, acce_3[4] = {0}, acce_4[4] = {0}, acce_5[4] = {0}; //存放不同轮廓对应的加速度
int flag[5] = {0}; // 当轮廓i可行时,flag里面的第i个值为1,一共有5种轮廓
static float speed_max = 0;

/**
 * @brief 通过夹爪的位移和速度计算电机转动的角度和速度
 * 
*/
void calcu_motor_angle_and_vel(float *out_p,float *out_v){
    // 将得到的夹爪位移和速度转化为电机转动

    float l1 = LINK_B_LENGTH;
    float l2 = LINK_A_LENGTH;
    float dx = *out_v;
    float x = *out_p;
    float offset = CLOSED_DIS;
    float l3 = offset+x;
    float theta = acosf((pow(l1,2) + pow(offset + x,2) - pow(l2,2)) /
                  (2 * l1 * (offset + x)));  //求得夹爪位移转化为电机转角,单位是rad


    // float w = (l1 * dx * cos(theta) - offset * dx - x * dx) / (l1 * (offset + x) * sin(theta));
    float w = -(dx * (pow(l2,2) + pow(l3,2) - pow(l1,2))) / (l1 * sin(theta) * l3 * 2 * l3);
    w = w * RAD2DEG;
    theta = theta * RAD2DEG;

    theta = -1.023932*theta + 120.304306;
    w = -1.023932*w + 120.304306;
    if(theta <0) theta = 0;
    if(theta > 87) theta = 87;
    *out_p = theta;
    *out_v = w;
}

/**
 * @brief 根据当前时间计算在第几阶段,一共有0-3阶段,如果不在则返回-1,表示已经超过了规划时间
 * 
 * @param *t 时间数组,一共4个阶段
 * @param t_now 当前的时间
*/
int in_which_stage(float* t,float t_now){
    float t_sum[4] = {t[0],t[0]+t[1],t[0]+t[1]+t[2],t[0]+t[1]+t[2]+t[3]};
    if(0 <= t_now && t_now <= t_sum[0]){
        return 0;
    }else if(t_sum[0] <= t_now && t_now <= t_sum[1]){
        return 1;
    }else if(t_sum[1] <= t_now && t_now <= t_sum[2]){
        return 2;
    }else if(t_sum[2] <= t_now && t_now <= t_sum[3]+1e-6){
        return 3;
    }
    return -1;
}
/**
 * @brief 获取不同轮廓的总用时时间
 * 
 * @param use_time 将每一种轮廓对应的总时间存放在这个变量里面
*/
void calcu_use_sum_time(float *use_time){
    use_time[0] = time_1[0] + time_1[1] + time_1[2] + time_1[3];
    use_time[1] = time_2[0] + time_2[1] + time_2[2] + time_2[3];
    use_time[2] = time_3[0] + time_3[1] + time_3[2] + time_3[3];
    use_time[3] = time_4[0] + time_4[1] + time_4[2] + time_4[3];
    use_time[4] = time_5[0] + time_5[1] + time_5[2] + time_5[3];
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


/**
 * @brief OTG算法初始化
 * 
 * @param data 
 * @param dT 
 * @param v_max 
 * @param a_max
 * @param home_theta 
 */
void otg_init(float dT, float v_max, float a_max,float home_theta, float v0)
{
    data.dT = dT;
    data.am = a_max;
    data.vm = v_max;
    data.is_first_run = true;
    data.home_theta = 0;
    data.last_p = home_theta/2;
    data.last_v = v0;
    data.last_pf = home_theta/2;
    speed_max = v_max;
}


static int otg_typeI_1(otg *data, float _dp, float _v0, float vm, float am, float *t, float *a)
{
    float dp = fabsf(_dp);
    float v0 = fabsf(_v0);

    float t3 = vm / am;
    float t1 = fabsf(t3 - v0 / am) < 1e-6? 0 : t3 - v0 / am;
    float t2 = (dp - 0.5*(v0 + vm) * t1  - 0.5 * vm * t3) / vm;

    if(t1 < 0 || t2 < 0)
    {
        return 0;
    }

    t[0] = 0;
    t[1] = t1;
    t[2] = t2;
    t[3] = t3;

    a[0] = 0;
    a[1] = am*sign(_dp);
    a[2] = 0;
    a[3] = -am*sign(_dp);

    data->v[0] = 0;
    data->v[1] = v0;
    data->v[2] = vm;
    data->v[3] = vm;

    return 1;
}


static int otg_typeI_2(otg *data, float _dp, float _v0, float vm, float am, float *t, float *a)
{
    float dp = fabs(_dp);
    float v0 = fabs(_v0);

    // 联立公式,符号运算求得vs表达式
    float vs = sqrt((v0*v0 + 2*am*dp) / 2.0);
    if (vs > vm)
    {
        return 0;
    }

    float t3 = vs / am;
    float t1 = t3 - v0 / am;
    if (t1 < 0)
    {
        return 0;
    }

    t[0] = 0;
    t[1] = t1;
    t[2] = 0;
    t[3] = t3;

    a[0] = 0;
    a[1] = am*sign(_dp);
    a[2] = 0;
    a[3] = -am*sign(_dp);

    data->v[0] = 0;
    data->v[1] = v0;
    data->v[2] = vs;
    data->v[3] = vs;

    return 1;
}

static int otg_typeI_3(otg *data, float _dp, float _v0, float vm, float am, float *t, float *a)
{
    
    float dp = fabs(_dp);
    float v0 = fabs(_v0);

    float q_min = gripper_ikine(data->home_theta, 0);
    float q_max = gripper_ikine(data->home_theta, MAX_DIS);

    float d = 0;
    if(_v0 > 0 && _dp > 0){
        d = q_max - data->last_p;
    }else if(_v0 < 0 && _dp < 0){
        d = data->last_p - q_min;
    }

    float t3 = 2 * dp / v0;     //正常来说t3的时间
    float tmp = 0.5 * v0 * t3;  //正常来说位移


    
    //情况1: 如果v0方向初始速度很大,amax无法减速到边缘,增加amax
    if(tmp > d){
        t3 = 2.0 * d / v0;
        a[3] = -v0 / t3 * sign(_dp);
    }else{
        t3 = 2 * dp / v0;
        a[3] = -v0 / t3 * sign(_dp);
    }

    t[0] = 0;
    t[1] = 0;
    t[2] = 0;
    t[3] = t3;

    a[0] = 0;
    a[1] = 0;
    a[2] = 0;

    data->v[0] = 0;
    data->v[1] = 0;
    data->v[2] = 0;
    data->v[3] = v0;

    return 1;
}

static int otg_typeI_4(otg *data, float _dp, float _v0, float vm, float am, float *t, float *a)
{
    float dp = fabs(_dp);
    float v0 = fabs(_v0);

    float q_min = gripper_ikine(data->home_theta, 0);
    float q_max = gripper_ikine(data->home_theta, MAX_DIS);

    float d = 0; // 到达边界的距离
    if(_v0 > 0 && _dp < 0) {
        d = q_max - data->last_p;
    } else if (_v0 < 0 && _dp > 0) {
        d = data->last_p - q_min;
    }

    float t0 = v0 / am;
    float t1 = vm / am;
    float t3 = t1;

    float tmp = 0.5 * v0 * t0;
    if (tmp > d)
    {
        t0 = 2.0 * d / v0;
        a[0] = v0 / t0 * sign(_dp);
    } else {
        a[0] = am * sign(_dp);
    }

    float t2 = 0.5 * ((2.0*dp + v0*t0) / vm - t1 - t3);
    if(t2 < 0)
    {
        return 0;
    }

    t[0] = t0;
    t[1] = t1;
    t[2] = t2;
    t[3] = t3;

    
    a[1] = am * sign(_dp);
    a[2] = 0;
    a[3] = -am * sign(_dp);

    data->v[0] = v0;
    data->v[1] = 0;
    data->v[2] = vm;
    data->v[3] = vm;

    return 1;
}

static int otg_typeI_5(otg *data, float _dp, float _v0, float vm, float am, float *t, float *a)
{
    
    float dp = fabs(_dp);
    float v0 = fabs(_v0);

    float q_min = gripper_ikine(data->home_theta, 0);
    float q_max = gripper_ikine(data->home_theta, MAX_DIS);

    float d = 0; // 到达边界的距离
    if(_v0 > 0 && _dp < 0) {
        d = q_max - data->last_p;
    } else if (_v0 < 0 && _dp > 0) {
        d = data->last_p - q_min;
    }

    float t0 = v0 / am;
    float tmp = 0.5 * v0 * t0;
    if (tmp > d)
    {
        t0 = 2.0 * d / v0;
        a[0] = v0 / t0 * sign(_dp);
    } else {
        a[0] = am * sign(_dp);
    }

    float vs = sqrt((2*dp + t0*v0) * am / 2.0);
    if (vs > vm)
    {
        return 0;
    }

    float t1 = vs / am;
    float t3 = t1;

    t[0] = t0;
    t[1] = t1;
    t[2] = 0;
    t[3] = t3;

    a[1] = am * sign(_dp);
    a[2] = 0;
    a[3] = -am * sign(_dp);

    data->v[0] = v0;
    data->v[1] = 0;
    data->v[2] = vs;
    data->v[3] = vs;

    return 1;
}

/**
 * @brief 轨迹积分
 * 
 * @param t 
 * @param p0 
 * @param v0 
 * @param out_p 
 * @param out_v 
 */
static void integrate(float t, float p0, float v0, float a, float *out_p, float *out_v)
// static void integrate(double t, double p0, double v0, double a, double *out_p, double *out_v)
{

    out_p[0] = p0 + t * (v0 + 0.5 * a * t);
    
    out_v[0] = v0 + t * a;
    // if(fabsf( out_v[0] - data.v[data.stage]) < fabsf(data.am *data.dT))
    // {
    //     out_v[0] = data.v[data.stage];
    // }
}

void set_new_target_state(float pf)
{
    data.last_pf = pf;
}

/**
 * @brief OTG算法loop
 * 
 * @param data 
 * @param p0 
 * @param v0 
 * @param pf 
 * @param out_p 
 * @param out_v 
 * @return int 
 *        0: 运行中
 *        1: 结束
 */
int otg_update(float pf, float speed_rate, float *out_p, float *out_v)
{
    pf = pf/2;
    data.vm = speed_rate * speed_max;
    // 如果上一次的pf与当前pf不相等, 更新当前pf, 重新计算轮廓
    if(fabs(data.last_pf - pf) > 1e-6)
    {
        data.last_pf = pf;
        data.time = 0;

        // 判断dp与v0符号是否相同, 然后分别计算几种情况, 取总时间最短的一个为最终轮廓
        float _dp = data.last_pf-data.last_p;
        float _v0 = data.last_v;
        
        memset(flag,0,sizeof(flag));
        if (signbit(_v0) == signbit(_dp)) //当符号相同的时候,调用I II III轮廓
        { 
            flag[0] = otg_typeI_1(&data,_dp,_v0,data.vm,data.am,time_1,acce_1);// type I
            flag[1] = otg_typeI_2(&data,_dp,_v0,data.vm,data.am,time_2,acce_2);// type II 
            flag[2] = otg_typeI_3(&data,_dp,_v0,data.vm,data.am,time_3,acce_3);// type III            
        }else{
            flag[3] = otg_typeI_4(&data,_dp,_v0,data.vm,data.am,time_4,acce_4);// type IV  
            flag[4] = otg_typeI_5(&data,_dp,_v0,data.vm,data.am,time_5,acce_5);// type V
        }

        // 计算五种轮廓的用时
        float use_time_temp[5] = {0};
        calcu_use_sum_time(use_time_temp);

        // 计算哪个轮廓是最小用时,索引为use_index,值为use_min_time
        float use_min_time = FLT_MAX; 
        int use_index = -1;
        for (int i = 0; i < sizeof(flag)/sizeof(flag[0]); i++)
        {
            if(flag[i] == 1)
            {
                if(use_min_time > use_time_temp[i]){
                    use_min_time = use_time_temp[i];
                    use_index = i;
                }
            }
        }

        data.t_sum = use_min_time;
        data.type = use_index;

        switch(use_index)
        {
            case 0:
            {
                memcpy(data.t,time_1,sizeof(time_1));
                memcpy(data.a,acce_1,sizeof(acce_1));
                break;
            }
            case 1:
            {
                memcpy(data.t,time_2,sizeof(time_2));
                memcpy(data.a,acce_2,sizeof(acce_2));
                break;
            }
            case 2:
            {
                memcpy(data.t,time_3,sizeof(time_3));
                memcpy(data.a,acce_3,sizeof(acce_3));
                break;
            }
            case 3:
            {
                memcpy(data.t,time_4,sizeof(time_4));
                memcpy(data.a,acce_4,sizeof(acce_4));
                break;
            }
            case 4:
            {
                memcpy(data.t,time_5,sizeof(time_5));
                memcpy(data.a,acce_5,sizeof(acce_5));
                break;
            }
            default:
            {
                *out_p = data.last_p;
                *out_v = data.last_v;
                break;
            }
        }
        
    
    } 
    
    // 根据时间判断当前在哪个阶段(少了速度和位置信息就会导致规划结束以后位置和速度有一定的偏差)
    int stage = in_which_stage(data.t,data.time);
    data.stage = stage;

    //积分得到下一时刻位置和速度
    integrate(data.dT,data.last_p,data.last_v,data.a[stage],out_p,out_v);

    // 否则, 基于上一次计算的轮廓, 计算当前输出值
    double delta_time = data.dT;
    data.time += delta_time;
    
    // 当规划时间结束后跳出循环(少了速度和位置信息就会导致规划结束以后位置和速度有一定的偏差)
    int is_over = 0;

    // if(*out_p < 0) *out_p = 0;
    if(data.time > data.t_sum + 1e-6){
        *out_v = 0;
        is_over = 1;
    }
    data.last_p = *out_p;
    data.last_v = *out_v;

    // 夹爪位移转化为电机角度和角速度
    calcu_motor_angle_and_vel(out_p,out_v);
  
    if(is_over) return 1; 
    return 0;
}

