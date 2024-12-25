#include <stdio.h>
#include "gripper_algo.h"

int sample_test()
{
    double dt = 0.01;

    double ratio = 1.0;     // 速度比例 100%
    
    double home_theta = 0;  // 零位时的关节角度

    // 逆运动学
    double current_dis = 0;
    printf("current_dis = %f\n", current_dis);
    printf("current_theta: %f\n", gripper_ikine(home_theta, current_dis));

    current_dis = 60;
    printf("current_dis = %f\n", current_dis);
    printf("current_theta: %f\n", gripper_ikine(home_theta, current_dis));

    // 正运动学
    double current_theta = gripper_ikine(home_theta, current_dis);
    printf("current_theta = %f\n", current_theta);
    printf("current_dis: %f\n", gripper_kine(home_theta, current_theta));

    // 轨迹规划
    current_dis = 0;
    double target_dis = 65;
    Trajectory traj = gripper_traj_planning(dt, home_theta, MAX_VEL*ratio, current_dis, target_dis);

    printf("t_total = %f\n", traj.n*dt);

#if record_enable
    FILE *fp = fopen("traj.txt", "w");
    for(int i = 0; i < traj.n; i++)
    {
        fprintf(fp, "%f,%f,%f,%f\n", traj.p[i], traj.v[i], traj.a[i], gripper_kine(home_theta, traj.p[i]));
    }
    fclose(fp);
#endif

    trajectory_destroy(&traj); // 每次使用完规划结果后，都要释放内存.

    return 0;
}