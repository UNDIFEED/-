#include "zf_common_headfile.h"
#include "image.h"

uint8 MotorBegin_Flag = 0; //启动标志位

void Mode_Switch(void)
{
    //异常停车
    if(data_stastics_l <= 5)
    {
        MotorBegin_Flag = 0;
    }
}