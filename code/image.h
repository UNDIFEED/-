#ifndef _image_h_
#define _image_h_
#include "zf_common_headfile.h"

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色

//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值	

extern uint8 image_thereshold;//二值化阈值

extern uint16 data_stastics_l, data_stastics_r;

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组

extern uint8 image_0[MT9V03X_H][MT9V03X_W];
extern uint8 image_1[MT9V03X_H][MT9V03X_W];

extern uint8 l_border[image_h];//左线数组
extern uint8 r_border[image_h];//右线数组

extern uint16 cornerpoint[2][3];//两个角点 -- 左下，右下 {x,y,flag}

#define TFTSHOW_W           (80) //TFT屏幕显示的宽高 157  100
#define TFTSHOW_H           (50)
#define USED_LINE_BEGIN     (90) //用来循迹的首行
#define USED_LINE_END       (50) //用来循迹的尾行

void Turn_Bin(uint8 threshold);
void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了


#endif

