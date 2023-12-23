#ifndef _pid_h_
#define _pid_h_
#include "zf_common_headfile.h"
#include "motor.h"

// 限幅
#define LIMITMAX(input, max) (((input) < (max)) ? (input) : (max))
#define LIMITMIN(input, min) (((input) > (min)) ? (input) : (min))
#define MINMAX(input, min, max) LIMITMIN(LIMITMAX(input, max), min)

typedef struct {
    float kp;    //P
    float ki;    //I
    float kd;    //D

//方便查看
    float out_p; //本次误差
    float out_i; //累计误差
    float out_d; //本次-上次误差
    float output; //pid输出

    float error; //本次误差
    float l_error; //上次误差
    float ll_error; //上上次误差

    float i_max; //i * ki
} pid_param_t;

#define PID_CREATE(_kp, _ki, _kd, _i_max)\
    {                                    \
        .out_p = 0,                      \
        .out_i = 0,                      \
        .out_d = 0,                      \
        .output = 0,                     \
        .error = 0,                      \
        .l_error = 0,                    \
        .ll_error = 0,                   \
        .kp = _kp,                       \
        .ki = _ki,                       \
        .kd = _kd,                       \
        .i_max = _i_max,                 \
    }

//变量声明
extern float p_error; //循迹差值
extern pid_param_t Position_Pid;
extern pid_param_t Position_Pid_Corner;
extern pid_param_t Position_Pid_Stright;
extern pid_param_t Position_Pid_kp2;
extern pid_param_t Increment_Pid_LF;
extern pid_param_t Increment_Pid_RF;
extern pid_param_t Increment_Pid_LB;
extern pid_param_t Increment_Pid_RB;

//二次kp系数
extern float kp_A;
extern float kp_B;
extern float kd_C;

// 函数声明
float Position_Pid_solve(pid_param_t *pid, float error);
float Position_Pid_solve_kp2(pid_param_t *pid, float error);
float Increment_Pid_solve(float tar_speed, float now_speed, pid_param_t *pid);

#endif