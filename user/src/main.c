#include "zf_common_headfile.h"
#include "zf_device_tft180.h"
#include "image.h"
#include "motor.h"
#include "pid.h"
#include "key.h"
#include "encoder.h"
#include "control.h"
#include "ladrc.h"
#include "imu963.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������
// �������ǿ�Դ����ֲ�ÿչ���

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    debug_init();                   // ���Զ˿ڳ�ʼ��

    tft180_init(); //tft��ʼ��
	mt9v03x_init(); //���������ͷ��ʼ��
    my_key_init(); //������ʼ��
    Motor_Init(); //�����ʼ��
    LADRC_Init_All(); //LADRC��ʼ��
    Encoder_Init(); //��������ʼ��
    imu963ra_init(); //�����ǳ�ʼ��
    imu_zero_drift(); //�ϵ�������������Ư
    // if(wireless_uart_init()) //���ߴ��ڳ�ʼ��
    // {
    //   while(1);
    // }

    pit_ms_init(PIT_CH0, 2); //pit0-2ms�ж�--�ٶȿ���
    pit_ms_init(PIT_CH1, 6); //pit1-6ms�ж�--ƫ�����
    pit_ms_init(PIT_CH2, 10);//10ms�ж�--���������١���λ��ͨ��
    pit_ms_init(PIT_CH3, 10); //pit3-10ms�ж�--����
	EnableGlobalIRQ(0);

    //��ʾ��
    // tft180_show_string(0, 55, "kp_A");
    // tft180_show_string(0, 65, "kp_B");
    // tft180_show_string(0, 75, "kd_C");
    tft180_show_string(0, 55, "p_error");
    tft180_show_string(0, 65, "kp");
    tft180_show_string(0, 75, "kd");
    // tft180_show_string(0, 65, "ki");
    // tft180_show_string(0, 75, "kp");

    // tft180_show_string(0, 85, "encoder_LF");
    // tft180_show_string(0, 95, "encoder_RF");
    // tft180_show_string(0, 105, "encoder_LB");
    // tft180_show_string(0, 115, "encoder_RB");

    tft180_show_string(90, 10, "thre");
    tft180_show_string(90, 50, "S");
    tft180_show_string(90, 60, "Y");

    //����
//    pwm_set_duty(MOTORLF_PWM, 5000);
//    pwm_set_duty(MOTORRF_PWM, 5000);
	
    while(1)
    {
        //��ʾ����
		// tft180_show_float(50, 55, kp_A, 2, 3);
		// tft180_show_float(50, 65, kp_B, 2, 3);
		// tft180_show_float(50, 75, kd_C, 2, 3);
        tft180_show_float(50, 55, p_error, 2, 3);
        tft180_show_float(50, 65, Position_Pid_Corner.kp, 2, 3);
        tft180_show_float(50, 75, Position_Pid_Corner.kd, 2, 3);
        // tft180_show_float(50, 65, Increment_Pid_LF.ki, 2, 3);
        // tft180_show_float(50, 75, Increment_Pid_LF.kp, 2, 3);

        // tft180_show_int(60, 85, motor_zuoqian.encoder, 5);
        // tft180_show_int(60, 95, motor_youqian.encoder, 5);
        // tft180_show_int(60, 105, motor_zuohou.encoder, 5);
        // tft180_show_int(60, 115, motor_youhou.encoder, 5);

        tft180_show_int(125, 10, image_thereshold, 3);
        tft180_show_int(120, 20, cornerpoint[2][2], 1);
        tft180_show_int(130, 20, cornerpoint[3][2], 1);
		tft180_show_int(120, 30, cornerpoint[0][2], 1);
        tft180_show_int(130, 30, cornerpoint[1][2], 1);
        tft180_show_int(120, 50, ShiZi_Flag, 1);
        tft180_show_int(130, 50, shizi_status, 1);
        tft180_show_int(120, 60, YuanHuan_Flag, 1);
        tft180_show_int(130, 60, yuanhuan_status, 1);

        // tft180_show_int(125, 80, data_stastics_l, 3); //90 49
        // tft180_show_int(125, 90, data_stastics_r, 3); //50 75

        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag=0;
            // Turn_Bin(200);
            // tft180_displayimage03x(mt9v03x_image[0],157,100);
            image_process();
        }
    }
}


