#include <stdio.h>
#include "gripper_algo.h"

int main()
{
    float dt = 0.001;

    float ratio = 1.0;     // 速度比例 100%
    
    float home_theta = 0;  // 零位时的关节角度

    // 逆运动学
    float current_dis = 0;
    printf("current_dis = %f\n", current_dis);
    printf("current_theta: %f\n", gripper_ikine(home_theta, current_dis));

    current_dis = 60;
    printf("current_dis = %f\n", current_dis);
    printf("current_theta: %f\n", gripper_ikine(home_theta, current_dis));

    // 正运动学
    float current_theta = gripper_ikine(home_theta, current_dis);
    printf("current_theta = %f\n", current_theta);
    printf("current_dis: %f\n", gripper_kine(home_theta, current_theta));

    // 轨迹规划
    current_dis = 0;
    float target_dis = 65;
    ratio = 1.0;
    Trajectory traj = gripper_traj_planning(dt, home_theta, MAX_VEL*ratio, current_dis, target_dis);

    printf("t_total = %f\n", traj.t_sum);

    FILE *fp = fopen("traj.txt", "w");

    float q = 0, dq = 0;
    while(1)
    {
        int ret = gripper_traj_run(&traj, &q, &dq);
        fprintf(fp, "%f, %f, %f, %f\n",traj.t, q, dq, gripper_kine(home_theta, q));
        if(ret != 0)
        {
            break;
        }
    }
    fclose(fp);

    
    // current_theta = 2.9645;
    current_theta = gripper_ikine(home_theta, 5.98466);
    float current_vel = 77.0;
    Trajectory traj_stop = gripper_emergency_stop_traj_planning(dt, home_theta, current_theta, current_vel);
    FILE *fp2 = fopen("traj_stop.txt", "w");
    while(1)
    {
        int ret = gripper_emergency_stop_traj_run(&traj_stop, &q, &dq);
        fprintf(fp2, "%f, %f, %f\n",traj_stop.t, q, dq);
        if(ret != 0)
        {
            break;
        }
    }
    fclose(fp2);



    filter_st curr_filter;

    // 滤波器初始化
    utils_filter_init(&curr_filter);
    
    // 滤波器设计
    utils_second_order_lowpass_filter_design(dt, 10.0, 0.75, &curr_filter);

    float curr_f = utils_second_order_filter(0.1, &curr_filter); 

    return 0;
}