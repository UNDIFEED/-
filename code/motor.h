#ifndef _motor_h_
#define _motor_h_
#include "zf_common_headfile.h"

//电机方向以及PWM引脚
#define MOTORRF_DIR              (D0)
#define MOTORRF_PWM              (PWM2_MODULE3_CHA_D2)

#define MOTORLF_DIR              (D1 )
#define MOTORLF_PWM              (PWM2_MODULE3_CHB_D3)

#define MOTORRB_DIR              (D13 )
#define MOTORRB_PWM              (PWM1_MODULE1_CHB_D15)

#define MOTORLB_DIR              (D12 )
#define MOTORLB_PWM              (PWM1_MODULE1_CHA_D14)

//结构体定义
typedef struct 
{
    int32 encoder; //编码器计数
    float speed; //当前速度
    float tar_speed; //轮子目标速度
    float set_speed; //设定小车运行速度
    int32 duty; //占空比
} motor_param_t;

#define MOTOR_CREATE(_set_speed, _duty) \
    {                                   \
        .encoder = 0,             \
        .speed = 0,                     \
        .tar_speed = 0,                 \
        .set_speed = _set_speed,        \
        .duty = _duty,                  \
    }

//变量声明
extern motor_param_t motor_zuoqian;
extern motor_param_t motor_youqian;
extern motor_param_t motor_zuohou;
extern motor_param_t motor_youhou;

//函数声明
void Motor_Init(void);
void Motor_Give(void);
void Motor_Stop(void);
void Motor_Calculation(void);
void Motor_Control(void);

#endif