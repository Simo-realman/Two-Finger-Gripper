1.在使用算法时，首先要初始化，里面各个参数的意义在头文件已经写好
2.当位置为0时，不要有一个反向速度，不然会报错
3.初始化参数以后(调用otg_init(dT,v_max,a_max,p0,v0)函数以后)，在循环里面调用otg_update即可，第一个参数为更新的位置，
第二个参数为速度比例,在0-1之间，后面两个float类型指针接受返回来的电机角度和角速度，具体使用方法如下
append_float_to_file是向txt写入数据的，主要测试结果是否正确

#include <stdio.h>
#include "gripper_algo.h"

// 定义写入数据的子函数
void append_float_to_file(float data, const char *filename) {
    // 打开文件，以追加模式
    FILE *file = fopen(filename, "a");
    if (file == NULL) {
        perror("Failed to open file");
        return;
    }
    
    // 将float数据写入文件
    fprintf(file, "%f\n", data);
    
    // 关闭文件
    if (fclose(file) != 0) {
        perror("Failed to close file");
    }
}


int main()
{
    float dT = 0.002;      
    float v_max = 200;
    float a_max = 1000;
    int is_changed = 0;
    
    float p0 = 1;
    float v0 = 10;
    float pf1 = 20;
    float pf2 = 20;
    
    otg_init(dT,v_max,a_max,p0,v0);
    float out_p = 0;
    float out_v = 0;
    int flag = 1;

    const char *filename1 = "position.txt";
    const char *filename2 = "velocity.txt";

    int index = 0;
    while(flag)
    {
        int flag_temp;
        
        if(is_changed){
            if(index<=30)
            {
                flag_temp = otg_update(pf1,&out_p,&out_v);
            }else{
                flag_temp = otg_update(pf2,&out_p,&out_v);
            }         
        }else{
            flag_temp = otg_update(pf1,&out_p,&out_v);
        }
               
        if(flag_temp) flag = 0;
        // 调用子函数写入数据
        append_float_to_file(out_p, filename1);
        append_float_to_file(out_v, filename2);
        // printf("%f\n",out_p);
        index++;
    }
    
    return 0;
}