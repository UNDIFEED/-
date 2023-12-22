#include "zf_common_headfile.h"
#include "key.h"
#include "motor.h"
#include "pid.h"
#include "image.h"
#include "control.h"

void my_key_init(void)
{
	gpio_init(UP_KEY, GPI, 0, GPI_PULL_UP);
	gpio_init(DOWN_KEY, GPI, 0, GPI_PULL_UP);
	gpio_init(LEFT_KEY, GPI, 0, GPI_PULL_UP);
	gpio_init(RIGHT_KEY, GPI, 0, GPI_PULL_UP);
	gpio_init(MID_KEY, GPI, 0, GPI_PULL_UP);
}

void key1_control(void)
{		
	if(!gpio_get_level(UP_KEY))
	{
		system_delay_ms(20);
		while(!gpio_get_level(UP_KEY));
		system_delay_ms(20);
		//�˴�д
		//�ٶȻ�����
		// Increment_Pid_LF.kp += 0.01;
		// Increment_Pid_RF.kp += 0.01;
		// Increment_Pid_LB.kp += 0.01;
		// Increment_Pid_RB.kp += 0.01;

		//λ�û�����
		Position_Pid_Corner.kd += 0.01;

		//����kp����
//		kd_C += 0.01;
	}
	
	if(!gpio_get_level(DOWN_KEY))
	{
		system_delay_ms(20);
		while(!gpio_get_level(DOWN_KEY));
		system_delay_ms(20);
		//�˴�д
		//�ٶȻ�����
		// Increment_Pid_LF.kp -= 0.01;
		// Increment_Pid_RF.kp -= 0.01;
		// Increment_Pid_LB.kp -= 0.01;
		// Increment_Pid_RB.kp -= 0.01;

		//λ�û�����
		Position_Pid_Corner.kd -= 0.01;

		//����kp����
//		kd_C -= 0.01;
	}
	
	if(!gpio_get_level(LEFT_KEY))
	{
		system_delay_ms(20);
		while(!gpio_get_level(LEFT_KEY));
		system_delay_ms(20);
		//�˴�д
		//�ٶȻ�����
		// Increment_Pid_LF.ki -= 0.005;
		// Increment_Pid_RF.ki -= 0.005;
		// Increment_Pid_LB.ki -= 0.005;
		// Increment_Pid_RB.ki -= 0.005;

		//λ�û�����
		Position_Pid_Corner.kp -= 0.01;

		//����kp����
		// kp_A -= 0.001;

		//��ֵ����ֵ
		image_thereshold -= 1;
	}
	if(!gpio_get_level(RIGHT_KEY))
	{
		system_delay_ms(20);
		while(!gpio_get_level(RIGHT_KEY));
		system_delay_ms(20);
		//�˴�д
		//�ٶȻ�����
		// Increment_Pid_LF.ki += 0.005;
		// Increment_Pid_RF.ki += 0.005;
		// Increment_Pid_LB.ki += 0.005;
		// Increment_Pid_RB.ki += 0.005;

		//λ�û�����
		Position_Pid_Corner.kp += 0.01;

		//����kp����
		// kp_A += 0.001;

		//��ֵ����ֵ
		image_thereshold += 1;
	}
	if(!gpio_get_level(MID_KEY))
	{
		system_delay_ms(20);
		while(!gpio_get_level(MID_KEY));
		system_delay_ms(20);
		//�˴�д
		if(MotorBegin_Flag == 0) MotorBegin_Flag = 1;
		else MotorBegin_Flag = 0;
	}
		
}
