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
#define MAX_ACC 600.0

// 零位姿态角度
#define HOME_THETA 0


typedef struct {
    int n; //规划完一共有多少数据，乘以周期就等于时间
    float *p;
    float *v;
    //float *a;
} Trajectory;

/**
 * @brief 正运动学, 由关节角度计算夹爪开合距离.
 * 
 * @param home_theta     零位时, 关节角度, 单位deg.
 * @param current_theta  当前关节角度, 单位deg.
 * @return float        单位mm.
 */
float gripper_kine(float home_theta, float current_theta); //输入角度，返回开合距离

/**
 * @brief 逆运动学, 由夹爪开合距离计算关节角度.
 * 
 * @param home_theta     零位时, 关节角度, 单位deg.
 * @param current_dis    当前夹爪开合距离, 单位mm. 
 * @return float        单位deg.
 */
float gripper_ikine(float home_theta, float current_dis); //输入距离，返回角度

/**
 * @brief 夹爪轨迹规划
 * 
 * @param dt            控制周期, 单位s.-发送定时器的周期-如2ms
 * @param home_theta    零位时, 关节角度, 单位deg. -零位角度
 * @param dq_max        关节速度 = MAX_VEL * 速度比例, 单位deg/s. -目前先固定传180
 * @param start_dis     起始开合距离, 单位mm.  -开始的位置，通过角度逆解算出当前的位置mm
 * @param end_dis       终止开合距离, 单位mm.  -终止位置，
 * @return Trajectory   注意: 轨迹返回的是关节角度, 单位deg.
 */
Trajectory gripper_traj_planning(float dt, float home_theta, float dq_max, float start_dis, float end_dis); //

//运动开始时规划，传入这些参数，返回一个带位置的结构体数组，然后按照规划周期去发送这些角度，发送完后需要释放内存


/**
 * @brief 释放轨迹结构体的内存
 * 
 * @param traj 
 */
void trajectory_destroy(Trajectory *traj);

#ifdef __cplusplus
}
#endif
#endif // GRIPPER_ALGO_H