#include "zf_common_headfile.h"
#include "image.h"
#include "math.h"
#include "imu963.h"

uint8 MotorBegin_Flag = 0; //启动标志位
uint8 shizi_status = 0; //十字状态
uint8 ShiZi_Flag = 0; //十字标志位
uint8 yuanhuan_status = 0; //圆环状态
uint8 YuanHuan_Flag = 0; //圆环标志位

void Mode_Switch(void)
{
    uint8 i;
    uint8 shizi_count1, shizi_count2 = 0;
    //检测十字
    if (shizi_status == 0 && YuanHuan_Flag == 0) //未检测到十字
    {
        if (cornerpoint[0][2] == 1 && cornerpoint[1][2] == 1 && cornerpoint[2][2] == 1 && cornerpoint[3][2] == 0 ||
            cornerpoint[0][2] == 1 && cornerpoint[1][2] == 1 && cornerpoint[2][2] == 0 && cornerpoint[3][2] == 1)
        {
            shizi_status = 1;
            ShiZi_Flag = 1;
        }
    }
    if (shizi_status == 1) //检测到开始进十字
    {
        AngleGet_Flag = 1; //角度开始积分
        if (cornerpoint[0][2] == 0 && cornerpoint[1][2] == 0 && cornerpoint[2][2] == 0 && cornerpoint[3][2] == 0)
        {
            for (i = CUT_H; i <= image_h - 3; i++)
            {
                if (l_border[i] <= 4) shizi_count1++; //左丢线行数
                if (r_border[i] >= 184) shizi_count2++; //右丢线行数
            }
            if ((shizi_count1 + shizi_count2) <= 20) //总丢线行数少
            {
                shizi_status = 2;
                shizi_count1 = 0;
                shizi_count2 = 0;
            }
        }
    }
    if (shizi_status == 2) //在十字内
    {
        if (fabs(angle) >= 270)
        {
            shizi_status = 3; 
        }
    }
    if (shizi_status == 3) //开始出十字
    {
        if (cornerpoint[0][2] == 0 && cornerpoint[1][2] == 0 && cornerpoint[2][2] == 0 && cornerpoint[3][2] == 0)
        {
            for (i = CUT_H; i <= image_h - 3; i++)
            {
                if (l_border[i] <= 4) shizi_count1++; //左丢线行数
                if (r_border[i] >= 184) shizi_count2++; //右丢线行数
            }
            if ((shizi_count1 + shizi_count2) <= 20) //总丢线行数少
            {
                shizi_status = 0; //出十字le
                ShiZi_Flag = 0;
                AngleGet_Flag = 0;
                shizi_count1 = 0;
                shizi_count2 = 0;
            }
        }
    }

    uint8 yuanhuan_count = 0; //圆环不丢边行计数
    uint8 yuanhuan_count2 = 0; //圆环单调行计数
    uint8 yuanhuan_count3 = 0; //圆环出环左边线丢线计数
    line_equation front_l = LINE_CREATE(); //左边线前方线段
    line_equation behind_l = LINE_CREATE(); //左边线后方线段
    line_equation front_r = LINE_CREATE(); //右边线前方线段
    line_equation behind_r = LINE_CREATE(); //右边线后方线段
    line_equation front_mid = LINE_CREATE(); //中线后方线段
    line_equation behind_mid = LINE_CREATE(); //中边线后方线段
    //检测圆环
    if (yuanhuan_status == 0 && ShiZi_Flag == 0) //未检测到圆环状态
    {
        if (cornerpoint[0][2] == 1 && cornerpoint[1][2] == 0 && cornerpoint[0][1] >= USED_LINE_END - 10) //左圆环-拐点超过循迹最后一行
        {
            for (i = cornerpoint[0][1] + 3; i > cornerpoint[0][1] - 12; i--) //在拐点所在往前数3行起前15行不丢线
            {
                if (r_border[i] < 186) yuanhuan_count++;//没丢边
            }
            for (i = CUT_H; i <= image_h - 3; i++)
            {
                if(r_border[i] <= r_border[i+1]) yuanhuan_count2++;//右边线单调
            }
            if (yuanhuan_count >= 13 && yuanhuan_count2 >= 82 &&
                (data_stastics_l + data_stastics_r) >= 300 && data_stastics_l - data_stastics_r >= 120) //最后一个条件防止弯道误判
            {
                YuanHuan_Flag = 1; //左圆环
                yuanhuan_status = 1;
            }
        }
        else if (cornerpoint[0][2] == 0 && cornerpoint[1][2] == 1 && cornerpoint[1][1] >= USED_LINE_END - 10) //右圆环
        {
            for (i = cornerpoint[1][1] + 3; i > cornerpoint[1][1] - 12; i--) //在拐点所在往前数3行起前15行不丢线
            {
                if (l_border[i] > 2) yuanhuan_count++;//没丢边
            }
            for (i = CUT_H; i <= image_h - 3; i++)
            {
                if(l_border[i] >= l_border[i+1]) yuanhuan_count2++;//左边线单调
            }
            if (yuanhuan_count >= 13 && yuanhuan_count2 >= 82 &&
                (data_stastics_l + data_stastics_r) >= 300 && data_stastics_r - data_stastics_l >= 120)
            {
                YuanHuan_Flag = 2; //右圆环
                yuanhuan_status = 1;
            }
        }
        yuanhuan_count2 = 0;
    }
    if (YuanHuan_Flag == 1) //左圆环
    {
        AngleGet_Flag = 1; //角度开始积分
        if (yuanhuan_status == 1) //开始进圆环直道
        {   
            //左上角点到位开始进圆环
            if (cornerpoint[2][2] == 1 && cornerpoint[3][2] == 0 && cornerpoint[2][1] >= USED_LINE_END - 10) yuanhuan_status = 2; 
        }
        if (yuanhuan_status == 2) //正在进入圆环弯道内
        {   
            //左上角点消失
            if (cornerpoint[2][2] == 0) yuanhuan_status = 3;
        }
        if (yuanhuan_status == 3) //正在环岛弯道内
        {   
            //检测到右下角点
            if (cornerpoint[1][2] == 1)
            {   //计算右下角点上面和下面斜率，判断尖尖
                k_and_b(&front_mid, (l_border[cornerpoint[1][1] - 3] + r_border[cornerpoint[1][1] - 3]) / 2, cornerpoint[1][1] - 3, (l_border[cornerpoint[1][1]] + r_border[cornerpoint[1][1]]) / 2, cornerpoint[1][1]);
                k_and_b(&behind_mid, (l_border[cornerpoint[1][1]] + r_border[cornerpoint[1][1]]) / 2, cornerpoint[1][1], (l_border[cornerpoint[1][1] + 3] + r_border[cornerpoint[1][1] + 3]) / 2, cornerpoint[1][1] + 3);
                if (front_mid.k * behind_mid.k < 0) yuanhuan_status = 4;
            }
        }
        if (yuanhuan_status == 4) //开始出环岛
        {
            //角度差不多转了一圈
            if (fabs(angle) >= 340) yuanhuan_status = 5;
        }
        if (yuanhuan_status == 5) //正在补直线出环岛
        {
            //左边线单调
            for (i = CUT_H; i<= image_h - 3; i++)
            {
                if (l_border[i] >= l_border[i+1]) yuanhuan_count2++;
            }
            if (yuanhuan_count2 >= 80)
            {
                YuanHuan_Flag = 0;
                AngleGet_Flag = 0;
                yuanhuan_status = 0;
            }
        }
    }
    if (YuanHuan_Flag == 2) //右圆环
    {
        AngleGet_Flag = 1; //角度开始积分
        if (yuanhuan_status == 1) //开始进圆环直道
        {   
            //右上角点到位开始进圆环
            if (cornerpoint[2][2] == 0 && cornerpoint[3][2] == 1 && cornerpoint[3][1] >= USED_LINE_END - 10) yuanhuan_status = 2; 
        }
        if (yuanhuan_status == 2) //正在进入圆环弯道内
        {
            //右上角点消失
            if (cornerpoint[3][2] == 0) yuanhuan_status = 3;
        }
        if (yuanhuan_status == 3) //正在环岛弯道内
        {   
            //检测到左下角点
            if (cornerpoint[0][2] == 1)
            {   //计算右下角点上面和下面斜率，判断尖尖
                k_and_b(&front_mid, (l_border[cornerpoint[1][1] - 3] + r_border[cornerpoint[1][1] - 3]) / 2, cornerpoint[1][1] - 3, (l_border[cornerpoint[1][1]] + r_border[cornerpoint[1][1]]) / 2, cornerpoint[1][1]);
                k_and_b(&behind_mid, (l_border[cornerpoint[1][1]] + r_border[cornerpoint[1][1]]) / 2, cornerpoint[1][1], (l_border[cornerpoint[1][1] + 3] + r_border[cornerpoint[1][1] + 3]) / 2, cornerpoint[1][1] + 3);
                if (front_mid.k * behind_mid.k < 0) yuanhuan_status = 4;
                // tft180_show_int(110, 80, front_mid.k, 3); //90 49
                // tft180_show_int(110, 90, behind_mid.k, 3); //50 75
            }
        }
        if (yuanhuan_status == 4) //开始出环岛
        {
            //角度差不多转了一圈
            if (fabs(angle) >= 340) yuanhuan_status = 5;
        }
        if (yuanhuan_status == 5) //正在补直线出环岛
        {
            //右边线单调
            for (i = CUT_H; i<= image_h - 3; i++)
            {
                if (r_border[i] >= r_border[i+1]) yuanhuan_count2++;
            }
            if (yuanhuan_count2 >= 80)
            {
                YuanHuan_Flag = 0;
                AngleGet_Flag = 0;
                yuanhuan_status = 0;
            }
        }
    }
    line_equation l_line = LINE_CREATE(); 
    line_equation r_line = LINE_CREATE(); 
    calculate_s_i(hightest, image_h - 3, l_border, &l_line);
    calculate_s_i(hightest, image_h - 3, r_border, &r_line);
    // tft180_show_int(110, 80, l_border[USED_LINE_BEGIN], 3); //90 49
    // tft180_show_int(110, 90, r_border[USED_LINE_BEGIN], 3); //50 75

    //异常停车
    if (data_stastics_l <= 5 || (bin_image[118][94] == 0 && bin_image[117][94] == 0 && bin_image[116][94] == 0))
    {
        MotorBegin_Flag = 0;
    }
}