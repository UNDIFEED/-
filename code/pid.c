#include "zf_common_headfile.h"
#include "pid.h"
#include "motor.h"
#include "fliter.h"
#include "imu963.h"

//左转p_error>0，右转p_error<0
float p_error = 0.0; //寻仙误差
float angle_add = 0.0; //附加的角度抑制量

//位置环pid参数定义
//速度120参数：拐弯kp0.8 kd0.409   直线kp0.7  kd 1.109
pid_param_t Position_Pid_Corner = PID_CREATE(0.8, 0, 0, 10);  //常规转弯pid
pid_param_t Position_Pid_Stright = PID_CREATE(0.45, 0, 0, 10); //常规直线pid
pid_param_t Position_Pid_kp2 = PID_CREATE(0, 0, 0, 10);        //二次kp pid

//速度环pid参数定义
pid_param_t Increment_Pid_LF = PID_CREATE(0.23, 0.11, 0.04, 10);
pid_param_t Increment_Pid_RF = PID_CREATE(0.23, 0.11, 0.04, 10);
pid_param_t Increment_Pid_LB = PID_CREATE(0.23, 0.11, 0.04, 10);
pid_param_t Increment_Pid_RB = PID_CREATE(0.23, 0.11, 0.04, 10);

//角度环pid参数定义
pid_param_t Angle_Pid_Temp = PID_CREATE(0.8, 0, 0, 10);

// 位置式pid-二次kp
float kp_A = 0.5;
float kp_B = 1.62;
float kd_C = 0.0;
float Position_Pid_solve_kp2(pid_param_t *pid, float error)
{
    pid->error = error;

    pid->kp = kp_A * pid->error*pid->error + kp_B;
    pid->kd = kd_C * pid->kp;

    pid->out_d = pid->error - pid->l_error;
    pid->out_p = pid->error;
    pid->out_i += pid->error;

    pid->l_error = pid->error;

    if (pid->out_i != 0) pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki); //限幅
    
    if (pid->error >= 0)
    {
        pid->output = func_limit(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d - (float)imu_gyro_z / (10 + abs(p_error)) / 2.5, 8000); //临时限幅
    }
    else if (pid->error < 0)
    {
        pid->output = func_limit(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d + (float)imu_gyro_z / (10 + abs(p_error)) / 2.5, 8000); //临时限幅
    }
    return pid->output;
}

// 位置式pid
float Position_Pid_solve(pid_param_t *pid, float error)
{
    pid->error = error;

    pid->out_d = pid->error - pid->l_error;
    pid->out_p = pid->error;
    pid->out_i += pid->error;
    angle_add = func_limit((float)abs(imu_gyro_z) / (10 + fabs(p_error)) / 2, 25);

    pid->l_error = pid->error;

    if (pid->out_i != 0) pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki); //限幅
    
    if (pid->error >= 0)
    {
        pid->output = func_limit(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d - angle_add, 200); //临时限幅
    }
    else if (pid->error < 0)
    {
        pid->output = func_limit(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d + angle_add, 200); //临时限幅
    }
    return pid->output;
}

//增量式pid
float Increment_Pid_solve(float tar_speed, float now_speed, pid_param_t *pid)
{
    pid->error = tar_speed - now_speed;

    pid->out_p = pid->error - pid->l_error;
    pid->out_i = pid->error;
    pid->out_d = pid->error - 2.0f*pid->l_error + pid->ll_error;

    pid->ll_error = pid->l_error;
    pid->l_error = pid->error;

    pid->output += pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d;

    return pid->output;
}

//角度环pid
float Angle_Pid_solve(pid_param_t *pid, float error)
{
    pid->error = error;

    pid->out_d = pid->error - pid->l_error;
    pid->out_p = pid->error;
    pid->out_i += pid->error;

    pid->l_error = pid->error;

    pid->output = func_limit(pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d, 200); //临时限幅
    return pid->output;
}
