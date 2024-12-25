#ifndef GRIPPER_ALGO_H
#define GRIPPER_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

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
#define LINK_A_LENGTH 19.75
#define LINK_B_LENGTH 20.10

// 最大关节速度, 单位deg/s
#define MAX_VEL 180.0

// 最大关节加速度, 单位deg/s^2
#define MAX_ACC 1000.0

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
 * @brief 急停轨迹规划
 * 
 * @param dt            控制周期, 单位s. 
 * @param home_theta    零位时, 关节角度, 单位deg. 
 * @param current_theta 当前关节角度(traj.p[i]), 单位deg. 
 * @param current_vel   当前关节速度(traj.v[i]), 单位deg/s. 
 * @return Trajectory    注意: 调用急停之前, 若复用上一轨迹, 需要先释放Trajectory结构体的内存.
 */
Trajectory gripper_emergency_stop_traj_planning(float dt, float home_theta, float current_theta, float current_vel);

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
int gripper_emergency_stop_traj_run(Trajectory* traj, float *q_out, float *dq_out);

#ifdef __cplusplus
}
#endif
#endif // GRIPPER_ALGO_H