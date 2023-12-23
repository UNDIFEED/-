#include "image.h"
#include "pid.h"
#include "swj.h"
#include "control.h"

//声明函数
void Cal_P_Error(uint8 *_l_border, uint8 *_r_border, uint16 begin_line, uint16 end_line, float center);
void Find_CornerPoint(uint8 *_l_border, uint8 *_r_border);
void Corner_Addline(uint8 *_l_border, uint8 *_r_border);
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);
void Circle_Addline(uint8 _flag, uint8 _status);

uint8 image_0[MT9V03X_H][MT9V03X_W];
//显示线段
void my_tft180_draw_line (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
	tft180_draw_line((uint16)((float)x_start/image_h*TFTSHOW_H),
					 (uint16)((float)y_start/image_h*TFTSHOW_H),
					 (uint16)((float)x_end/image_h*TFTSHOW_H),
					 (uint16)((float)y_end/image_h*TFTSHOW_H), color);
}
//二值化
void Turn_Bin(uint8 threshold)
{
    unsigned short i = 0, j = 0;
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < MT9V03X_W; j++)
		{                
			if (mt9v03x_image[i][j] > (threshold)) //数值越大，显示的内容越多，较浅的图像也能显示出来
				image_0[i][j] = 255;
			else
				image_0[i][j] = 0;
		}
	}
}
	
//填充最上边图像不使用
void Fill_UpPic(uint8 (*bin_image)[image_w])
{
	unsigned short i = 0, j = 0;
	for (i = 0; i < CUT_H; i++)
	{
		for (j = 0; j < MT9V03X_W; j++)
		{                
			bin_image[i][j] = 0;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}

/*
函数名称：void k_and_b(line_equation *temp_line, int16 startX, int16 startY, int16 endX, int16 endY)
功能说明：求直线的k和b
参数说明：
temp_line：直线结构体
startX   ：起始点x
startY   ：起始点y
endX     ：结束点X
endY     ：结束点Y
函数返回：无
修改时间：2023年12月21日
备    注：y = kx + b
example：  k_and_b(line, 1, 1, 9, 9)
 */
void k_and_b(line_equation *temp_line, int16 startX, int16 startY, int16 endX, int16 endY)
{
	temp_line->k = (float)((float)endY - (float)startY) / (float)((float)endX - (float)startX);
	temp_line->b = (float)startY - (float)startX * temp_line->k;
}

/** 
* @brief 最小二乘法
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &line);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, line_equation *temp_line)
{
	uint16 i, num = 0;
	uint16 xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)(xsum / num);
		y_average = (float)(ysum / num);

	}

	/*计算斜率*/
	temp_line->k = Slope_Calculate(start, end, border);//斜率
	temp_line->b = y_average - (temp_line->k)*x_average;//截距
}

/*变量声明*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold = 156;//图像分割阈值
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num		1	//1就是不压缩，2就是压缩一倍	
	uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
			line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};
	
	uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//图像数组
void turn_to_bin(void)
{
  uint8 i,j;
  image_thereshold = otsuThreshold(original_image[0], image_w, image_h); //动态阈值
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
  //消除暗角
	// for(i = 0;i<image_h - 5;i++)
	// {
	// 	for(j = 0;j<image_w;j++)
	// 	{
	// 		if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
	// 		else bin_image[i][j] = black_pixel;
	// 	}
	// }
	// for(i = image_h - 5;i<image_h;i++)
	// {
	// 	for(j = 0;j<image_w;j++)
	// 	{
	// 		if(original_image[i][j]>image_thereshold-10)bin_image[i][j] = white_pixel;
	// 		else bin_image[i][j] = black_pixel;
	// 	}
	// }
}


/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag				：最多需要循环的次数
(*image)[image_w]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
					   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//中心坐标点
	uint8 index_r = 0;//索引下标
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{

		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++) // 8个邻域
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0]; //暂存找到的边界点
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i); //记录生长方向
			}

			if (index_l) //找到了边界点
			{
				//更新边界坐标点
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1]) //把找到的赛道最前面的边界点更新给边界坐标点
					{
						center_point_l[0] = temp_l[j][0]; //x
						center_point_l[1] = temp_l[j][1]; //y
					}
				}
			}

		}
		if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
			&& points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
			||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
				&& points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			//printf("\n左右相遇退出\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			// printf("\n如果右边比左边高了，右边等待左边\n");	
			continue;//如果右边比左边高了，右边等待左边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];//x
						center_point_r[1] = temp_r[j][1];//y
					}
				}

			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}

/*
函数名称：void search_l_r2(uint8(*image)[image_w])
功能说明：中线往左右两边找边界
参数说明：
(*image)[image_w]：需要进行找点的图像数组，必须是二值图,填入数组名称即可
				   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	search_l_r2(image);
 */
void search_l_r2(uint8(*image)[image_w])
{
	uint16 i, j; //行、列
	uint16 now_centerline; //当前中线

	now_centerline = image_w / 2;
	for(i = image_h - 2; i >= 2;i--) //从底行开始中线往两边扫
	{
		if (image[i][now_centerline] == 0) now_centerline = image_w / 2;//检测到中线扫描点为黑,重新从中间开始扫
		for(j = now_centerline; j >= 2; j--) //扫左边线
		{
			if (image[i][j] == 255 && image[i][j-1] == 0) //发生白黑跳变
			{
				points_l[data_stastics_l][0] = j;
				points_l[data_stastics_l][1] = i;
				data_stastics_l++;
				break;
			}
		}
		for(j = now_centerline; j <= image_w-2; j++) //扫右边线
		{
			if (image[i][j] == 255 && image[i][j+1] == 0) //发生白黑跳变
			{
				points_r[data_stastics_r][0] = j;
				points_r[data_stastics_r][1] = i;
				data_stastics_r++;
				break;
			}
		}
		now_centerline = (points_l[data_stastics_l - 1][0] + points_r[data_stastics_r - 1][0]) / 2; //更新中线
	}
}

/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L	：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[image_h];//左线数组
uint8 r_border[image_h];//右线数组
uint8 center_line[image_h];//中线数组
void get_left(uint16 total_L)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;

	for (i = 0;i<image_h;i++)
	{
		l_border[i] = border_min;
	}
	h = image_h - 2;
	//左边
	for (j = 0; j < total_L; j++)
	{
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0]+1;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0) 
		{
			break;//到最后一行退出
		}
	}
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h - 2;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < image_h - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			//统计八个方向的像素值
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;//黑

			}

		}
	}

}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

	uint8 i = 0;
	for (i = 0; i < image_h; i++) // 左右画两格宽一条横线
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][image_w - 1] = 0;
		image[i][image_w - 2] = 0;

	}
	for (i = 0; i < image_w; i++) // 上面画两格宽一横线
	{
		image[0][i] = 0;
		image[1][i] = 0;
	}
}

/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
	uint16 i;
	hightest = 0;//最高行，tip：这里的最高指的是y值的最小
	/*这是离线调试用的*/
	Get_image(mt9v03x_image);
	turn_to_bin();
	Fill_UpPic(bin_image);
	/*提取赛道边界*/
	image_filter(bin_image);//滤波
	image_draw_rectan(bin_image);//预处理
	//清零
	data_stastics_l = 0;
	data_stastics_r = 0;
	if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
	{
		//爬线
		search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
		// if (ShiZi_Flag == 0)search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
		// if (ShiZi_Flag == 1)search_l_r2(bin_image);
		// 从爬取的边界线内提取边线 ，这个才是最终有用的边线
		get_left(data_stastics_l);
		get_right(data_stastics_r);
		//处理函数放这里，不要放到if外面去了
		Find_CornerPoint(l_border, r_border);
		// Corner_Addline(l_border, r_border); //十字补线
		// cross_fill(bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
		Circle_Addline(YuanHuan_Flag, yuanhuan_status); //圆环补线
		Cal_P_Error(l_border, r_border, USED_LINE_BEGIN, USED_LINE_END, 93.5); //计算循迹误差
	}


	//显示图像
	// sendimg_binary(mt9v03x_image, image_w, image_h, 180); //图传调试
	tft180_displayimage03x(bin_image[0], TFTSHOW_W, TFTSHOW_H);// TFT屏幕调试
	tft180_draw_line(0, (uint16)((float)USED_LINE_BEGIN/image_h*TFTSHOW_H), TFTSHOW_W, (uint16)((float)USED_LINE_BEGIN/image_h*TFTSHOW_H), uesr_BLUE);
	tft180_draw_line(0, (uint16)((float)USED_LINE_END/image_h*TFTSHOW_H), TFTSHOW_W, (uint16)((float)USED_LINE_END/image_h*TFTSHOW_H), uesr_BLUE);

	uint8 x_x, y_y;
	//根据最终循环次数画出边界点
	// for (i = 0; i < data_stastics_l; i++)
	// {
	// 	x_x = (uint8)(((float)(points_l[i][0]+2) / image_w) * TFTSHOW_W);
	// 	y_y = (uint8)(((float)points_l[i][1] / image_h) * TFTSHOW_H);
	// 	tft180_draw_point(x_x, y_y, uesr_BLUE); //tft屏幕调试
	// }

	// for (i = 0; i < data_stastics_r; i++)
	// {
	// 	x_x = (uint8)(((float)(points_r[i][0]+2) / image_w) * TFTSHOW_W);
	// 	y_y = (uint8)(((float)points_r[i][1] / image_h) * TFTSHOW_H);
	// 	tft180_draw_point(x_x, y_y, uesr_RED); //tft屏幕调试
	// }

	for (i = hightest; i < image_h-1; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
		//求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
		//当然也有多组边线的找法，但是个人感觉很繁琐，不建议
		y_y = (uint8)(((float)i / image_h) * TFTSHOW_H);
		tft180_draw_point((uint8)(((float)center_line[i] / image_w) * TFTSHOW_W), y_y, uesr_GREEN);//显示起点 显示中线	
		tft180_draw_point((uint8)(((float)l_border[i] / image_w) * TFTSHOW_W), y_y, uesr_GREEN);//显示起点 显示左边线
		tft180_draw_point((uint8)(((float)r_border[i] / image_w) * TFTSHOW_W), y_y, uesr_GREEN);//显示起点 显示右边线 -- tft调试
	}

	//显示角点
	for(i = 0; i <= 3; i++)
	{
		if(cornerpoint[i][2] == 1)
		{
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W), (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H), uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W)-1, (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H)+1, uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W)-1, (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H)-1, uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W)+1, (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H)-1, uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W)+1, (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H)+1, uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W)-1, (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H), uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W), (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H)-1, uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W)+1, (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H), uesr_RED);
			tft180_draw_point((uint8)(((float)cornerpoint[i][0] / image_w) * TFTSHOW_W), (uint8)(((float)cornerpoint[i][1] / image_h) * TFTSHOW_H)+1, uesr_RED);
		}
	}
	printf("%d, %d\n", data_stastics_l, data_stastics_r);
	

}

/**********************************以下处理赛道****************************************/
/*
函数名称：void Cal_P_Error(uint8 *_l_border, uint8 *_r_border)
功能说明：获取循迹一行及其后两行差值均值
参数说明：
*_l_border：左边线点
*_r_border：右边线点
begin_line:开始行
end_line:结束行
center：中间点x坐标
函数返回：无
修改时间：2022年9月25日
备    注：路线在中线左边p_error>0，路线在中线右边p_error<0
example：Cal_P_Error(l_border, r_border, 95, 94);
 */
uint16_t weight_array[41] = {1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
                        	 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
							 3, 3, 3, 3, 4, 4, 5, 5, 5, 5};
void Cal_P_Error(uint8 *_l_border, uint8 *_r_border, uint16 begin_line, uint16 end_line, float center)
{
	uint16 i = 0;
	uint16 j = 0;
	uint16 sum_j = 0;
	uint16 line = begin_line - end_line + 1; //搜线的总行数
	float sum_error = 0.0;

	for(i = 0, j = line;j > 0;j--, i++)
	{
		sum_error += (center - ((_l_border[begin_line - i] + _r_border[begin_line - i]) * 0.5)) * weight_array[j];
		sum_j += weight_array[j];
	}
	p_error = sum_error / sum_j;
}

/*
函数名称：void Find_CornerPoint(uint8 *_l_border, uint8 *_r_border)
功能说明：寻找角点
参数说明：
*_l_border：左边线点
*_r_border：右边线点
函数返回：无
修改时间：2022年9月25日
备    注：无
example：Find_CornerPoint(l_border, r_border);
 */
uint16 cornerpoint[4][3]; //两个角点 -- 左下，右下 {x,y,flag}

void Find_CornerPoint(uint8 *_l_border, uint8 *_r_border)
{
	uint16 i, j;
	uint8 diuxian_count = 0; //丢线数
	memset(cornerpoint, 0, sizeof(cornerpoint)); //数组清零

	for(i=CUT_H+4; i<=image_h-4; i++) //从循迹起始行到结束行搜索十字角点
	{   //此行前两行均相差不大，后一行相差巨大判断为角点
		if (_l_border[0] != border_max && _l_border[1] != border_max && _l_border[2] != border_max) //判断左下角点
		{
			if (abs(_l_border[i] - _l_border[i+1]) < 3 && abs(_l_border[i] - _l_border[i+2]) < 3 &&
			    _l_border[i] - _l_border[i-2] >= 6 &&
				_l_border[i] >= 8 &&  _l_border[i+1] >= 8 &&  _l_border[i+2] >= 8) //拐点及前两行不在丢线边缘
			{
				cornerpoint[0][0] = _l_border[i];
				cornerpoint[0][1] = i;
				cornerpoint[0][2] = 1;
			}
		}
		if (_r_border[0] != border_min && _r_border[1] != border_min && _r_border[2] != border_min) //判断右下角点
		{
			if (abs(_r_border[i] - _r_border[i+1]) < 3 && abs(_r_border[i] - _r_border[i+2]) < 3 &&
			    _r_border[i] - _r_border[i-2] <= -6 &&
				_r_border[i] <= 180 &&  _r_border[i+1] <= 180 &&  _r_border[i+2] <= 180) //拐点及前两行不在丢线边缘
			{
				cornerpoint[1][0] = _r_border[i];
				cornerpoint[1][1] = i;
				cornerpoint[1][2] = 1;
			}
		}
		if (abs(_l_border[i] - _l_border[i-1]) < 3 && abs(_l_border[i] - _l_border[i-2]) < 3 &&
			_l_border[i] - _l_border[i+2] >= 6 &&
			_l_border[i] >= 8 && _l_border[i-1] >= 8 && _l_border[i-2] >= 8) //判断左上角点
		{
			for (j = i; j >= CUT_H; j--)
			{
				if (_l_border[i] == border_min) diuxian_count++;//丢线
			}
			if (diuxian_count < 3) //丢线超过3行
			{
				cornerpoint[2][0] = _l_border[i];
				cornerpoint[2][1] = i;
				cornerpoint[2][2] = 1;
			}
			diuxian_count = 0;
		}
		if (abs(_r_border[i] - _r_border[i-1]) < 3 && abs(_r_border[i] - _r_border[i-2]) < 3 &&
			_r_border[i] - _r_border[i+2] <= -6 &&
			_r_border[i] <= 180 && _r_border[i-1] <= 180 && _r_border[i-2] <= 180) //判断右上角点
		{
			for (j = i; j >= CUT_H; j--)
			{
				if (_r_border[i] == border_max) diuxian_count++;//丢线
			}
			if (diuxian_count < 3) //丢线超过3行
			{
				cornerpoint[3][0] = _r_border[i];
				cornerpoint[3][1] = i;
				cornerpoint[3][2] = 1;
			}
			diuxian_count = 0;
		}
	}
}

/*
函数名称：void ImageAddingLine(uint8 *border, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
功能说明：图像补线
参数说明：
imageSide  ：边线
startX     : 起始点 列数
startY     : 起始点 行数
endX       : 结束点 列数
endY       : 结束点 行数
函数返回：无
修改时间：2022年9月25日
备    注：endY 一定要大于 startY
example：ImageAddingLine(l_border, 0, 0, 8, 8);
 */
 void ImageAddingLine(uint8 *border, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
 {
    int i = 0;

	// 直线 y = kx + b
	line_equation temp_addline = LINE_CREATE();
	k_and_b(&temp_addline, startX, startY, endX, endY);

	for(i = startY; i < endY; i++)
	{
		border[i] = (uint8_t)((float)(i - temp_addline.b) / temp_addline.k);
	}
 }

//十字补线
uint16 record_cornerpoint[4][3]; //  {x,y,flag}
void Corner_Addline(uint8 *_l_border, uint8 *_r_border)
{
	if (shizi_status == 1)
	{
		ImageAddingLine(_l_border, 77, 50, 53, 90);
		ImageAddingLine(_r_border, 113, 50, 137, 90);
	}
	if (shizi_status == 3)
	{
		ImageAddingLine(_l_border, 77, 50, 53, 90);
		ImageAddingLine(_r_border, 113, 50, 137, 90);
	}
}

/** 
* @brief 十字补线函数
* @param uint8(*image)[image_w]		输入二值图像
* @param uint8 *l_border			输入左边界首地址
* @param uint8 *r_border			输入右边界首地址
* @param uint16 total_num_l			输入左边循环总次数
* @param uint16 total_num_r			输入右边循环总次数
* @param uint16 *dir_l				输入左边生长方向首地址
* @param uint16 *dir_r				输入右边生长方向首地址
* @param uint16(*points_l)[2]		输入左边轮廓首地址
* @param uint16(*points_r)[2]		输入右边轮廓首地址
*  @see CTest		cross_fill(image,l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
										 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
	uint8 i;
	uint8 break_num_l = 0;
	uint8 break_num_r = 0;
	uint8 start, end;
	line_equation l_line_temp = LINE_CREATE();
	//出十字
	for (i = 1; i < total_num_l; i++)
	{
		if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
		{
			break_num_l = points_l[i][1];//传递y坐标
			printf("brea_knum-L:%d\n", break_num_l);
			printf("I:%d\n", i);
			printf("十字标志位：1\n");
			break;
		}
	}
	for (i = 1; i < total_num_r; i++)
	{
		if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
		{
			break_num_r = points_r[i][1];//传递y坐标
			printf("brea_knum-R:%d\n", break_num_r);
			printf("I:%d\n", i);
			printf("十字标志位：1\n");
			break;
		}
	}
	if (break_num_l&&break_num_r&&image[image_h - 1][4] && image[image_h - 1][image_w - 4])//两边生长方向都符合条件
	{
		//计算斜率
		start = break_num_l - 15;
		start = limit_a_b(start, 0, image_h);
		end = break_num_l - 5;
		calculate_s_i(start, end, l_border, &l_line_temp);
		//printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
		for (i = break_num_l - 5; i < image_h - 1; i++)
		{
			l_border[i] = l_line_temp.k * (i)+l_line_temp.b;//y = kx+b
			l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
			// l_border[i] = (uint8_t)((float)(i - l_line_temp.b) / l_line_temp.k);//y = kx+b
			// l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
		}

		//计算斜率
		start = break_num_r - 15;//起点
		start = limit_a_b(start, 0, image_h);//限幅
		end = break_num_r - 5;//终点
		calculate_s_i(start, end, r_border, &l_line_temp);
		//printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
		for (i = break_num_r - 5; i < image_h - 1; i++)
		{
			r_border[i] = l_line_temp.k * (i)+l_line_temp.b;
			r_border[i] = limit_a_b(r_border[i], border_min, border_max);
			// r_border[i] = (uint8_t)((float)(i - l_line_temp.b) / l_line_temp.k);
			// r_border[i] = limit_a_b(r_border[i], border_min, border_max);
		}


	}

}


/*
函数名称：void Circle_Addline(uint8 _status, uint16 l_sum, uint16 r_sum)
功能说明：圆环补线
参数说明：
_flag	   ：圆环左右 ：1：左圆环 2：右圆环
_status    ：圆环状态 ：1：近直道 2：进弯道 3：弯道 4：出弯道 5：远直道
函数返回：无
修改时间：2022年9月25日
备    注：无
example：Circle_Addline(YuanHuan_Flag, yuanhuan_status)
 */
void Circle_Addline(uint8 _flag, uint8 _status)
{
	uint8 i;
	if (_flag == 1) //左圆环
	{
		switch (_status)
		{
		case 1: //圆环直线补线
			ImageAddingLine(l_border, 63, 50, 53, 90);
			my_tft180_draw_line(63, 50, 53, 90, uesr_RED);
			break;
		case 2: //进弯道，补线往左边打角
			ImageAddingLine(r_border, 73, 48, 130, 90);
			my_tft180_draw_line(73, 48, 130, 90, uesr_RED);
			break;  
		case 3: //在弯道，正常循迹不做处理
			break;
		case 4: //出弯道，补线往左边打角
			ImageAddingLine(r_border, 48, 50, 130, 90);
			my_tft180_draw_line(48, 50, 130, 90, uesr_RED);
			break;
		case 5: //已经出弯道，补直线直走
			ImageAddingLine(l_border, 75, 50, 49, 90);
			my_tft180_draw_line(75, 50, 49, 90, uesr_RED);
			break;
		default:
			break;
		}
	}
	if (_flag == 2) //右圆环
	{
		switch (_status)
		{
		case 1: //圆环直线补线
			ImageAddingLine(r_border, 117, 50, 137, 90);
			my_tft180_draw_line(117, 50, 137, 90, uesr_RED);
			break;
		case 2: //进弯道，补线往右边打角
			ImageAddingLine(l_border, 115, 48, 58, 90);
			my_tft180_draw_line(115, 48, 58, 90, uesr_RED);
			break;
		case 3: //在弯道，正常循迹不做处理
			break;
		case 4: //出弯道，补线往右边打角
			ImageAddingLine(l_border, 140, 50, 58, 90);
			my_tft180_draw_line(140, 50, 58, 90, uesr_RED);
			break;
		case 5: //已经出弯道，补直线直走
			ImageAddingLine(r_border, 113, 50, 139, 90);
			my_tft180_draw_line(113, 50, 139, 90, uesr_RED);
			break;
		default:
			break;
		}
	}
}
/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/


