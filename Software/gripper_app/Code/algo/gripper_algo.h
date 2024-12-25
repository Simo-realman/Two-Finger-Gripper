#ifndef GRIPPER_ALGO_H
#define GRIPPER_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/*********************************************************
 * 
 * 约定:
 *       1. 关节运动方向逆时针为正, 顺时针为负
 *       2. 关节角度单位为度
 *       3. 输入开口大小为两个夹指之间的距离, 单位mm
 *       4. 二指闭合时, 行程为0, 关节角度为0
 * 
 *********************************************************/

// 最大行程, 机械结构决定, 单位mm
#define MAX_DIS 65.0

// 闭合时的最小行程, 机械结构决定, 单位mm
#define CLOSED_DIS 5.25

// 连杆长度(a:从动杆长度, b:摇杆长度), 机械结构决定, 单位mm
#define LINK_A_LENGTH 23.0
#define LINK_B_LENGTH 20.1

// 最大关节速度, 单位deg/s
#define MAX_VEL 360.0

// 最大关节加速度, 单位deg/s^2
#define MAX_ACC 2000.0

typedef struct {
    float dt;
    float t;
    float t_sum;
    float Ta;
    float Tm;
    float vMax;
    float aMax;
    float sgn;
    float p0;
    float pf;
    int type;

    int n;
    int count;
    float curr_q;
    float curr_dq;
    float theta_max;
    float theta_min;
} Trajectory;

typedef struct
{
    float a[3];     ///< den
    float b[3];     ///< num
    float x[2];
    float y[2];
}filter_st;         ///< first order OR second order filter data structure.


/**
 * @brief 正运动学, 由关节角度计算夹爪开合距离.
 * 
 * @param home_theta     零位时, 关节角度, 单位deg.
 * @param current_theta  当前关节角度, 单位deg.
 * @return float        单位mm.
 */
float gripper_kine(float home_theta, float current_theta);

/**
 * @brief 逆运动学, 由夹爪开合距离计算关节角度.
 * 
 * @param home_theta     零位时, 关节角度, 单位deg.
 * @param current_dis    当前夹爪开合距离, 单位mm. 
 * @return float        单位deg.
 */
float gripper_ikine(float home_theta, float current_dis);

/**
 * @brief 夹爪轨迹规划
 * 
 * @param dt            控制周期, 单位s.
 * @param home_theta    零位时, 关节角度, 单位deg. 
 * @param dq_max        关节速度 = MAX_VEL * 速度比例, 单位deg/s.
 * @param start_dis     起始开合距离, 单位mm. 
 * @param end_dis       终止开合距离, 单位mm. 
 * @return Trajectory   注意: 轨迹返回的是关节角度, 单位deg.
 */
Trajectory gripper_traj_planning(float dt, float home_theta, float dq_max, float start_dis, float end_dis);


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
int gripper_traj_run(Trajectory* traj, float *q_out, float *dq_out);


/**
 * @brief Initialize the filter data structure.
 * 
 * @param data 
 */
void utils_filter_init(filter_st *data);

/**
 * @brief second order low-pass filter design, tustin method.
 * 
 * @param dT         unit: s
 * @param freq       unit: Hz    cut-off freq.
 * @param damping        defult: 0.75
 * @param data 
 */
void utils_second_order_lowpass_filter_design(float dT, float freq, float damping, filter_st *data);


/**
 * @brief second order filter function.
 * 
 * @param input 
 * @param data 
 * @return float 
 */
float utils_second_order_filter(float input, filter_st *data);


typedef struct 
{
    bool is_first_run;
    float dT;
    float am;
    float vm;
    float p0;
    float v0;
    float new_dt[4];

    float p[4];
    float v[4];
    float a[4];
    float t[4];
    float last_q;
    float last_qf;
    float last_v;
    int type;
    int stage;
    int is_gripper_emergency_stop;    // 1为正在急停状态
    int is_first_calcu_emergency_stop_profile;  //1为需要计算一次轮廓
    float time;
    float t_sum;
} otg;

/////////////////////////////  软件可以调用的接口   ////////////////////////////////
/**
 * @brief                    急停算法
*/
void run_emergency();


/**
 * @brief 更新电机真实位置和程序中位置
 * @param q_motor              电机的真实位置,单位 度
*/
void update_motor_real_position(float q_motor);

/**
 * @brief OTG算法初始化
 * 
 * @param dT       		 时间间隔
 * @param q0             电机的初始位置0-86.47 
 * @param v0             电机的初始速度
 */
void otg_init(float dT,float q0, float v0,float am, float vm);

/**
 * @brief OTG算法loop
 * 
 * @param pf            夹爪的新位置，范围是0-65
 * @param speed_rate    速度比例,在0-1之间
 * @param out_p         指针，用来接受计算出来的电机位置
 * @param out_v 		指针，用来接受计算出来的电机速度
 * @return int 
 *        0: 运行中
 *        1: 结束
 */
int otg_update(float pf, float speed_rate, float *out_p, float *out_v);


#ifdef __cplusplus
}
#endif
#endif // GRIPPER_ALGO_H
