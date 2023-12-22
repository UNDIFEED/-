#include "zf_common_headfile.h"
#include "zf_device_tft180.h"
#include "image.h"
#include "motor.h"
#include "pid.h"
#include "key.h"
#include "encoder.h"

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
    Encoder_Init(); //��������ʼ��
    // if(wireless_uart_init()) //���ߴ��ڳ�ʼ��
    // {
    //   while(1);
    // }

    pit_ms_init(PIT_CH0, 2); //pit0-2ms�ж�--����
    pit_ms_init(PIT_CH3, 10); //pit3-10ms�ж�--����
    pit_ms_init(PIT_CH2, 10);//10ms�ж�--���������١���λ��ͨ��
	EnableGlobalIRQ(0);

    //��ʾ��
    // tft180_show_string(0, 55, "kp_A");
    // tft180_show_string(0, 65, "kp_B");
    // tft180_show_string(0, 75, "kd_C");
    tft180_show_string(0, 55, "p_error");
    tft180_show_string(0, 65, "kp");
    tft180_show_string(0, 75, "kd");

    // tft180_show_string(0, 85, "encoder_LF");
    // tft180_show_string(0, 95, "encoder_RF");
    // tft180_show_string(0, 105, "encoder_LB");
    // tft180_show_string(0, 115, "encoder_RB");

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

        // tft180_show_int(60, 85, motor_zuoqian.encoder, 5);
        // tft180_show_int(60, 95, motor_youqian.encoder, 5);
        // tft180_show_int(60, 105, motor_zuohou.encoder, 5);
        // tft180_show_int(60, 115, motor_youhou.encoder, 5);

        tft180_show_int(120, 0, image_thereshold, 3);
		tft180_show_int(120, 20, cornerpoint[0][2], 1);
        tft180_show_int(120, 30, cornerpoint[1][2], 1);

        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag=0;
            // Turn_Bin(200);
            // tft180_displayimage03x(mt9v03x_image[0],157,100);
            image_process();
        }
        Mode_Switch();
    }
}


