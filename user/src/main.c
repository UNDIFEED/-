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

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
// 本例程是开源库移植用空工程

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    tft180_init(); //tft初始化
	mt9v03x_init(); //总钻风摄像头初始化
    my_key_init(); //按键初始化
    Motor_Init(); //电机初始化
    LADRC_Init_All(); //LADRC初始化
    Encoder_Init(); //编码器初始化
    imu963ra_init(); //陀螺仪初始化
    imu_zero_drift(); //上电消除陀螺仪零漂
    // if(wireless_uart_init()) //无线串口初始化
    // {
    //   while(1);
    // }

    pit_ms_init(PIT_CH0, 2); //pit0-2ms中断--速度控制
    pit_ms_init(PIT_CH1, 6); //pit1-6ms中断--偏差计算
    pit_ms_init(PIT_CH2, 10);//10ms中断--编码器测速、上位机通信
    pit_ms_init(PIT_CH3, 10); //pit3-10ms中断--按键
	EnableGlobalIRQ(0);

    //显示屏
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

    //测试
//    pwm_set_duty(MOTORLF_PWM, 5000);
//    pwm_set_duty(MOTORRF_PWM, 5000);
	
    while(1)
    {
        //显示参数
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


