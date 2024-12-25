#ifndef __FML_PID_H
#define __FML_PID_H

#ifdef __cplusplus
extern "C" {
#endif

// PID
#define KP 0.01
#define KI 0.01
#define KD 0
#define LIMIT 2
	
#define TARGET_CURRENT 1000.0
	
typedef struct {
    float kp;
    float ki;
    float kd;
		float limit;
    float prev_error;
    float integral;
} PIDController;	
	
void init_pid(PIDController *pid, float kp, float ki, float kd, float limit);
float calculate_pid(PIDController *pid, float setpoint, float current_value);	

#ifdef __cplusplus
}
#endif

#endif /* __CTRL */

