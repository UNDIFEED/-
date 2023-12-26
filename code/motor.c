#include "zf_common_headfile.h"
#include "motor.h"
#include "pid.h"
#include "control.h"
#include "ladrc.h"
#include "fliter.h"
#include "imu963.h"

//set_speed 0 - 10000
//cm/s
//参数 set_speed - p_kp - p_ki - p_kd - v_kp - v_ki - v_kd - i_max - duty
motor_param_t motor_zuoqian = MOTOR_CREATE(120, 0);//125
motor_param_t motor_youqian = MOTOR_CREATE(120, 0);
motor_param_t motor_zuohou = MOTOR_CREATE(120, 0);
motor_param_t motor_youhou = MOTOR_CREATE(120, 0);

// 控制主函数
void Motor_Control(void)
{
	if(MotorBegin_Flag) Motor_Speed_Calculation();
	else Motor_Stop();
	
	Motor_Give();
}

//位置误差解算
void Motor_Position_Calculation(void)
{
	//解算每个轮子目标速度
	//二次kp pid
	// motor_zuoqian.tar_speed = motor_zuoqian.set_speed - Position_Pid_solve_kp2(&Position_Pid_kp2, func_limit(p_error, 40)*0.117);
	// motor_youqian.tar_speed = motor_youqian.set_speed + Position_Pid_solve_kp2(&Position_Pid_kp2, func_limit(p_error, 40)*0.117);
	// motor_zuohou.tar_speed = motor_zuohou.set_speed - Position_Pid_solve_kp2(&Position_Pid_kp2, func_limit(p_error, 40)*0.117) * 0.6;
	// motor_youhou.tar_speed = motor_youhou.set_speed + Position_Pid_solve_kp2(&Position_Pid_kp2, func_limit(p_error, 40)*0.117) * 0.6;
/****************************************************************************************************************************************************/
	//普通pid
	if(p_error <= -10) //拐弯pid参数 - 右转
	{
		motor_zuoqian.tar_speed = motor_zuoqian.set_speed - Position_Pid_solve(&Position_Pid_Corner, p_error) * 1.2;
		motor_youqian.tar_speed = motor_youqian.set_speed + Position_Pid_solve(&Position_Pid_Corner, p_error);
		motor_zuohou.tar_speed = motor_zuohou.set_speed - Position_Pid_solve(&Position_Pid_Corner, p_error) * 0.5 * 1.2;
		motor_youhou.tar_speed = motor_youhou.set_speed + Position_Pid_solve(&Position_Pid_Corner, p_error) * 0.5;
	}
	if(p_error >= 10) //左转
	{
		motor_zuoqian.tar_speed = motor_zuoqian.set_speed - Position_Pid_solve(&Position_Pid_Corner, p_error);
		motor_youqian.tar_speed = motor_youqian.set_speed + Position_Pid_solve(&Position_Pid_Corner, p_error) * 1.2;
		motor_zuohou.tar_speed = motor_zuohou.set_speed - Position_Pid_solve(&Position_Pid_Corner, p_error) * 0.5;
		motor_youhou.tar_speed = motor_youhou.set_speed + Position_Pid_solve(&Position_Pid_Corner, p_error) * 0.5 * 1.2;
	}
	if(p_error > -10 && p_error < 10) //直线pid参数
	{

		motor_zuoqian.tar_speed = motor_zuoqian.set_speed - Position_Pid_solve(&Position_Pid_Stright, p_error);
		motor_youqian.tar_speed = motor_youqian.set_speed + Position_Pid_solve(&Position_Pid_Stright, p_error);
		motor_zuohou.tar_speed = motor_zuohou.set_speed - Position_Pid_solve(&Position_Pid_Stright, p_error) * 0.5;
		motor_youhou.tar_speed = motor_youhou.set_speed + Position_Pid_solve(&Position_Pid_Stright, p_error) * 0.5;
	}
}

//电机角度环解算
void Angle_Calculation(void)
{
	motor_zuoqian.tar_speed = -Angle_Pid_solve(&Angle_Pid_Temp, 0 - angle);
	motor_youqian.tar_speed = Angle_Pid_solve(&Angle_Pid_Temp, 0 - angle);
	motor_zuohou.tar_speed = -Angle_Pid_solve(&Angle_Pid_Temp, 0 - angle);
	motor_youhou.tar_speed = Angle_Pid_solve(&Angle_Pid_Temp, 0 - angle);
}

//电机速度解算
void Motor_Speed_Calculation(void)
{
	//增量式调速
	// motor_zuoqian.duty = func_limit(Increment_Pid_solve(motor_zuoqian.tar_speed, motor_zuoqian.speed, &Increment_Pid_LF), 10000);
	// motor_youqian.duty = func_limit(Increment_Pid_solve(motor_youqian.tar_speed, motor_youqian.speed, &Increment_Pid_RF), 10000);
	// motor_zuohou.duty = func_limit(Increment_Pid_solve(motor_zuohou.tar_speed, motor_zuohou.speed, &Increment_Pid_LB), 10000);
	// motor_youhou.duty = func_limit(Increment_Pid_solve(motor_youhou.tar_speed, motor_youhou.speed, &Increment_Pid_RB), 10000);

	//速度环调试
	// motor_zuoqian.duty = func_limit(Increment_Pid_solve(120, motor_zuoqian.speed, &Increment_Pid_LF), 10000);
	// motor_youqian.duty = func_limit(Increment_Pid_solve(120, motor_youqian.speed, &Increment_Pid_RF), 10000);
	// motor_zuohou.duty = func_limit(Increment_Pid_solve(120, motor_zuohou.speed, &Increment_Pid_LB), 10000);
	// motor_youhou.duty = func_limit(Increment_Pid_solve(120, motor_youhou.speed, &Increment_Pid_RB), 10000);
/****************************************************************************************************************************************************/
	//ladrc调速
	LADRC_Loop(&MOTOR_LF, motor_zuoqian.tar_speed, LowPassFilter(motor_zuoqian.speed, 0.73));
	LADRC_Loop(&MOTOR_RF, motor_youqian.tar_speed, LowPassFilter(motor_youqian.speed, 0.73));
	LADRC_Loop(&MOTOR_LB, motor_zuohou.tar_speed, LowPassFilter(motor_zuohou.speed, 0.73));
	LADRC_Loop(&MOTOR_RB, motor_youhou.tar_speed, LowPassFilter(motor_youhou.speed, 0.73));
	motor_zuoqian.duty = MOTOR_LF.u;
	motor_youqian.duty = MOTOR_RF.u;
	motor_zuohou.duty = MOTOR_LB.u;
	motor_youhou.duty = MOTOR_RB.u;
	
	//ladrc速度环调试
	// LADRC_Loop(&MOTOR_LF, motor_zuoqian.set_speed, LowPassFilter(motor_zuoqian.speed, 0.73));
	// LADRC_Loop(&MOTOR_RF, motor_youqian.set_speed, LowPassFilter(motor_youqian.speed, 0.73));
	// LADRC_Loop(&MOTOR_LB, motor_zuohou.set_speed, LowPassFilter(motor_zuohou.speed, 0.73));
	// LADRC_Loop(&MOTOR_RB, motor_youhou.set_speed, LowPassFilter(motor_youhou.speed, 0.73));
	// motor_zuoqian.duty = MOTOR_LF.u;
	// motor_youqian.duty = MOTOR_RF.u;
	// motor_zuohou.duty = MOTOR_LB.u;
	// motor_youhou.duty = MOTOR_RB.u;

/****************************************************************************************************************************************************/
	//不调速测试
	// motor_zuoqian.duty = motor_zuoqian.tar_speed;
	// motor_youqian.duty = motor_youqian.tar_speed;
	// motor_zuohou.duty = motor_zuohou.tar_speed;
	// motor_youhou.duty = motor_youhou.tar_speed;
}

// 电机设定速度函数
void Motor_Give(void)
{
	if(motor_zuoqian.duty >= 0) //左前轮
	{	
		gpio_set_level(MOTORLF_DIR, GPIO_LOW); 
		pwm_set_duty(MOTORLF_PWM, (int16)motor_zuoqian.duty);
	}
	else
	{
		gpio_set_level(MOTORLF_DIR, GPIO_HIGH);
		pwm_set_duty(MOTORLF_PWM, (int16)(-motor_zuoqian.duty));
	}

	
	if(motor_youqian.duty >= 0) //右前轮
	{	
		gpio_set_level(MOTORRF_DIR, GPIO_HIGH); 
		pwm_set_duty(MOTORRF_PWM, (int16)motor_youqian.duty);
	}
	else
	{
		gpio_set_level(MOTORRF_DIR, GPIO_LOW);
		pwm_set_duty(MOTORRF_PWM, (int16)(-motor_youqian.duty));
	}
	
    if(motor_zuohou.duty >= 0) //左后轮
	{	
		gpio_set_level(MOTORLB_DIR, GPIO_LOW); 
		pwm_set_duty(MOTORLB_PWM, (int16)motor_zuohou.duty);
	}
	else
	{
		gpio_set_level(MOTORLB_DIR, GPIO_HIGH);
		pwm_set_duty(MOTORLB_PWM, (int16)(-motor_zuohou.duty));
	}
	
	if(motor_youhou.duty >= 0) //右后轮
	{	
		gpio_set_level(MOTORRB_DIR, GPIO_HIGH); 
		pwm_set_duty(MOTORRB_PWM, (int16)motor_youhou.duty);
    }
	else
	{
		gpio_set_level(MOTORRB_DIR, GPIO_LOW);
		pwm_set_duty(MOTORRB_PWM, (int16)(-motor_youhou.duty));
	}
}

//电机停转
void Motor_Stop(void)
{
	motor_zuoqian.duty = 0;
	motor_youqian.duty = 0;
	motor_zuohou.duty = 0;
	motor_youhou.duty = 0;
}

// 电机初始化
void Motor_Init(void)
{
    gpio_init(MOTORRF_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTORRF_PWM, 17000, 0);
    gpio_init(MOTORLF_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTORLF_PWM, 17000, 0);
    gpio_init(MOTORRB_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTORRB_PWM, 17000, 0);
    gpio_init(MOTORLB_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTORLB_PWM, 17000, 0);
    
}

/*      麦轮车轮示意(车轱辘前转时)
*     线的方向表示轮胎实际速度方向
*       /       \      ↑
*      /         \     |
*                      |
*      \         /     |
*       \       /      |
*/