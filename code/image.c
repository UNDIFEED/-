#include "image.h"
#include "pid.h"
#include "swj.h"
#include "control.h"

//��������
void Cal_P_Error(uint8 *_l_border, uint8 *_r_border, uint16 begin_line, uint16 end_line, float center);
void Find_CornerPoint(uint8 *_l_border, uint8 *_r_border);
void Corner_Addline(uint8 *_l_border, uint8 *_r_border);
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);
void Circle_Addline(uint8 _flag, uint8 _status);

uint8 image_0[MT9V03X_H][MT9V03X_W];
//��ʾ�߶�
void my_tft180_draw_line (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
	tft180_draw_line((uint16)((float)x_start/image_h*TFTSHOW_H),
					 (uint16)((float)y_start/image_h*TFTSHOW_H),
					 (uint16)((float)x_end/image_h*TFTSHOW_H),
					 (uint16)((float)y_end/image_h*TFTSHOW_H), color);
}
//��ֵ��
void Turn_Bin(uint8 threshold)
{
    unsigned short i = 0, j = 0;
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < MT9V03X_W; j++)
		{                
			if (mt9v03x_image[i][j] > (threshold)) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
				image_0[i][j] = 255;
			else
				image_0[i][j] = 0;
		}
	}
}
	
//������ϱ�ͼ��ʹ��
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
//  ���:������ͼ����

//------------------------------------------------------------------------------------------------------------------
#include "image.h"

/*
�������ƣ�int my_abs(int value)
����˵���������ֵ
����˵����
�������أ�����ֵ
�޸�ʱ�䣺2022��9��8��
��    ע��
example��  my_abs( x)��
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
�������ƣ�int16 limit(int16 x, int16 y)
����˵������x,y�е���Сֵ
����˵����
�������أ�������ֵ�е���Сֵ
�޸�ʱ�䣺2022��9��8��
��    ע��
example��  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}

/*
�������ƣ�void k_and_b(line_equation *temp_line, int16 startX, int16 startY, int16 endX, int16 endY)
����˵������ֱ�ߵ�k��b
����˵����
temp_line��ֱ�߽ṹ��
startX   ����ʼ��x
startY   ����ʼ��y
endX     ��������X
endY     ��������Y
�������أ���
�޸�ʱ�䣺2023��12��21��
��    ע��y = kx + b
example��  k_and_b(line, 1, 1, 9, 9)
 */
void k_and_b(line_equation *temp_line, int16 startX, int16 startY, int16 endX, int16 endY)
{
	temp_line->k = (float)((float)endY - (float)startY) / (float)((float)endX - (float)startX);
	temp_line->b = (float)startY - (float)startX * temp_line->k;
}

/** 
* @brief ��С���˷�
* @param uint8 begin				�������
* @param uint8 end					�����յ�
* @param uint8 *border				������Ҫ����б�ʵı߽��׵�ַ
*  @see CTest		Slope_Calculate(start, end, border);//б��
* @return ����˵��
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
	if ((end - begin)*x2sum - xsum * xsum) //�жϳ����Ƿ�Ϊ��
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
* @brief ����б�ʽؾ�
* @param uint8 start				�������
* @param uint8 end					�����յ�
* @param uint8 *border				������Ҫ����б�ʵı߽�
* @param float *slope_rate			����б�ʵ�ַ
* @param float *intercept			����ؾ��ַ
*  @see CTest		calculate_s_i(start, end, r_border, &line);
* @return ����˵��
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

	//�������ƽ����
	if (num)
	{
		x_average = (float)(xsum / num);
		y_average = (float)(ysum / num);

	}

	/*����б��*/
	temp_line->k = Slope_Calculate(start, end, border);//б��
	temp_line->b = y_average - (temp_line->k)*x_average;//�ؾ�
}

/*��������*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold = 156;//ͼ��ָ���ֵ
//------------------------------------------------------------------------------------------------------------------
//  @brief      ���һ���Ҷ�ͼ��
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num		1	//1���ǲ�ѹ����2����ѹ��һ��	
	uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//����Ĳ�����д�������ͷ�ɼ�����ͼ��
			line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     ��̬��ֵ
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
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // ��䷽��;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height��ΪY =Image_Height���Ա���� �ж�ֵ��
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //ͳ��ÿ���Ҷ�ֵ�ĸ�����Ϣ
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
    {
        return MaxValue;          // ͼ����ֻ��һ����ɫ
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // ͼ����ֻ�ж�����ɫ
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  ��������
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //ǰ�����ص���
          PixelFore = Amount - PixelBack;         //�������ص���
          OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
          OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
          PixelIntegralBack += HistGram[Y] * Y;  //ǰ���Ҷ�ֵ
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
          MicroBack = (double)PixelIntegralBack / PixelBack;//ǰ���ҶȰٷֱ�
          MicroFore = (double)PixelIntegralFore / PixelFore;//�����ҶȰٷֱ�
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//����������䷽��g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      ͼ���ֵ���������õ��Ǵ�򷨶�ֵ����
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//ͼ������
void turn_to_bin(void)
{
  uint8 i,j;
  image_thereshold = otsuThreshold(original_image[0], image_w, image_h); //��̬��ֵ
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
  //��������
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
�������ƣ�void get_start_point(uint8 start_row)
����˵����Ѱ�������߽�ı߽����Ϊ������ѭ������ʼ��
����˵����������������
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example��  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//�������x��yֵ
uint8 start_point_r[2] = { 0 };//�ұ�����x��yֵ
uint8 get_start_point(uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//����
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//���м�����ߣ��������
	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
		{
			//printf("�ҵ�������image[%d][%d]\n", start_row,i);
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
			//printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("δ�ҵ����\n");
		return 0;
	} 
}

/*
�������ƣ�void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

����˵������������ʽ��ʼ���ұߵ�ĺ�������������е�࣬���õ�ʱ��Ҫ©�ˣ������������һ�������ꡣ
����˵����
break_flag				�������Ҫѭ���Ĵ���
(*image)[image_w]		����Ҫ�����ҵ��ͼ�����飬�����Ƕ�ֵͼ,�����������Ƽ���
					   �ر�ע�⣬��Ҫ�ú궨��������Ϊ����������������ݿ����޷����ݹ���
*l_stastic				��ͳ��������ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
*r_stastic				��ͳ���ұ����ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
l_start_x				�������������
l_start_y				��������������
r_start_x				���ұ���������
r_start_y				���ұ����������
hightest				��ѭ���������õ�����߸߶�
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example��
	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	image_h*3	//�����ҵ�������Ա��������˵300�����ܷ��£�������Щ�������ȷʵ�Ѷ����ඨ����һ��

 //��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//����
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//����
uint16 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint16 dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���
uint8 hightest = 0;//��ߵ�
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//��߱���
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//ͳ�����
	//����˸�����
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//�����˳ʱ��

	//�ұ߱���
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//���������
	uint8 index_r = 0;//�����±�
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//ͳ���ұ�
	//����˸�����
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//�������ʱ��

	l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
	r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������

	//��һ�θ��������  ���ҵ������ֵ������
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//��������ѭ��
	while (break_flag--)
	{

		//���
		for (i = 0; i < 8; i++)//����8F����
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//�����������䵽�Ѿ��ҵ��ĵ���
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//������һ

		//�ұ�
		for (i = 0; i < 8; i++)//����8F����
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//�����������䵽�Ѿ��ҵ��ĵ���
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//�����㣬��ʹ��
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//�����㣬��ʹ��
			temp_l[i][1] = 0;//�����㣬��ʹ��
		}

		//����ж�
		for (i = 0; i < 8; i++) // 8������
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0]; //�ݴ��ҵ��ı߽��
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i); //��¼��������
			}

			if (index_l) //�ҵ��˱߽��
			{
				//���±߽������
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1]) //���ҵ���������ǰ��ı߽����¸��߽������
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
			//printf("���ν���ͬһ���㣬�˳�\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			//printf("\n���������˳�\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
			//printf("\n��y=%d���˳�\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			// printf("\n����ұ߱���߸��ˣ��ұߵȴ����\n");	
			continue;//����ұ߱���߸��ˣ��ұߵȴ����
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
		{
			//printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//������һ

		index_r = 0;//�����㣬��ʹ��
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//�����㣬��ʹ��
			temp_r[i][1] = 0;//�����㣬��ʹ��
		}

		//�ұ��ж�
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//������һ
				dir_r[r_data_statics - 1] = (i);//��¼��������
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//���������
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


	//ȡ��ѭ������
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}

/*
�������ƣ�void search_l_r2(uint8(*image)[image_w])
����˵�������������������ұ߽�
����˵����
(*image)[image_w]����Ҫ�����ҵ��ͼ�����飬�����Ƕ�ֵͼ,�����������Ƽ���
				   �ر�ע�⣬��Ҫ�ú궨��������Ϊ����������������ݿ����޷����ݹ���
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example��
	search_l_r2(image);
 */
void search_l_r2(uint8(*image)[image_w])
{
	uint16 i, j; //�С���
	uint16 now_centerline; //��ǰ����

	now_centerline = image_w / 2;
	for(i = image_h - 2; i >= 2;i--) //�ӵ��п�ʼ����������ɨ
	{
		if (image[i][now_centerline] == 0) now_centerline = image_w / 2;//��⵽����ɨ���Ϊ��,���´��м俪ʼɨ
		for(j = now_centerline; j >= 2; j--) //ɨ�����
		{
			if (image[i][j] == 255 && image[i][j-1] == 0) //�����׺�����
			{
				points_l[data_stastics_l][0] = j;
				points_l[data_stastics_l][1] = i;
				data_stastics_l++;
				break;
			}
		}
		for(j = now_centerline; j <= image_w-2; j++) //ɨ�ұ���
		{
			if (image[i][j] == 255 && image[i][j+1] == 0) //�����׺�����
			{
				points_r[data_stastics_r][0] = j;
				points_r[data_stastics_r][1] = i;
				data_stastics_r++;
				break;
			}
		}
		now_centerline = (points_l[data_stastics_l - 1][0] + points_r[data_stastics_r - 1][0]) / 2; //��������
	}
}

/*
�������ƣ�void get_left(uint16 total_L)
����˵�����Ӱ�����߽�����ȡ��Ҫ�ı���
����˵����
total_L	���ҵ��ĵ������
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example�� get_left(data_stastics_l );
 */
uint8 l_border[image_h];//��������
uint8 r_border[image_h];//��������
uint8 center_line[image_h];//��������
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
	//���
	for (j = 0; j < total_L; j++)
	{
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0]+1;
		}
		else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
		h--;
		if (h == 0) 
		{
			break;//�����һ���˳�
		}
	}
}
/*
�������ƣ�void get_right(uint16 total_R)
����˵�����Ӱ�����߽�����ȡ��Ҫ�ı���
����˵����
total_R  ���ҵ��ĵ������
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��
example��get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_border[i] = border_max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
	}
	h = image_h - 2;
	//�ұ�
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
		}
		else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
		h--;
		if (h == 0)break;//�����һ���˳�
	}
}

//�������ͺ͸�ʴ����ֵ����
#define threshold_max	255*5//�˲����ɸ����Լ����������
#define threshold_min	255*2//�˲����ɸ����Լ����������
void image_filter(uint8(*bin_image)[image_w])//��̬ѧ�˲�������˵�������ͺ͸�ʴ��˼��
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < image_h - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			//ͳ�ư˸����������ֵ
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;//��  ���Ը�ɺ궨�壬�������

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;//��

			}

		}
	}

}

/*
�������ƣ�void image_draw_rectan(uint8(*image)[image_w])
����˵������ͼ��һ���ڿ�
����˵����uint8(*image)[image_w]	ͼ���׵�ַ
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example�� image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

	uint8 i = 0;
	for (i = 0; i < image_h; i++) // ���һ������һ������
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][image_w - 1] = 0;
		image[i][image_w - 2] = 0;

	}
	for (i = 0; i < image_w; i++) // ���滭�����һ����
	{
		image[0][i] = 0;
		image[1][i] = 0;
	}
}

/*
�������ƣ�void image_process(void)
����˵�������մ�����
����˵������
�������أ���
�޸�ʱ�䣺2022��9��8��
��    ע��
example�� image_process();
 */
void image_process(void)
{
	uint16 i;
	hightest = 0;//����У�tip����������ָ����yֵ����С
	/*�������ߵ����õ�*/
	Get_image(mt9v03x_image);
	turn_to_bin();
	Fill_UpPic(bin_image);
	/*��ȡ�����߽�*/
	image_filter(bin_image);//�˲�
	image_draw_rectan(bin_image);//Ԥ����
	//����
	data_stastics_l = 0;
	data_stastics_r = 0;
	if (get_start_point(image_h - 2))//�ҵ�����ˣ���ִ�а�����û�ҵ���һֱ��
	{
		//����
		search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
		// if (ShiZi_Flag == 0)search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
		// if (ShiZi_Flag == 1)search_l_r2(bin_image);
		// ����ȡ�ı߽�������ȡ���� ����������������õı���
		get_left(data_stastics_l);
		get_right(data_stastics_r);
		//�������������Ҫ�ŵ�if����ȥ��
		Find_CornerPoint(l_border, r_border);
		// Corner_Addline(l_border, r_border); //ʮ�ֲ���
		// cross_fill(bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
		Circle_Addline(YuanHuan_Flag, yuanhuan_status); //Բ������
		Cal_P_Error(l_border, r_border, USED_LINE_BEGIN, USED_LINE_END, 93.5); //����ѭ�����
	}


	//��ʾͼ��
	// sendimg_binary(mt9v03x_image, image_w, image_h, 180); //ͼ������
	tft180_displayimage03x(bin_image[0], TFTSHOW_W, TFTSHOW_H);// TFT��Ļ����
	tft180_draw_line(0, (uint16)((float)USED_LINE_BEGIN/image_h*TFTSHOW_H), TFTSHOW_W, (uint16)((float)USED_LINE_BEGIN/image_h*TFTSHOW_H), uesr_BLUE);
	tft180_draw_line(0, (uint16)((float)USED_LINE_END/image_h*TFTSHOW_H), TFTSHOW_W, (uint16)((float)USED_LINE_END/image_h*TFTSHOW_H), uesr_BLUE);

	uint8 x_x, y_y;
	//��������ѭ�����������߽��
	// for (i = 0; i < data_stastics_l; i++)
	// {
	// 	x_x = (uint8)(((float)(points_l[i][0]+2) / image_w) * TFTSHOW_W);
	// 	y_y = (uint8)(((float)points_l[i][1] / image_h) * TFTSHOW_H);
	// 	tft180_draw_point(x_x, y_y, uesr_BLUE); //tft��Ļ����
	// }

	// for (i = 0; i < data_stastics_r; i++)
	// {
	// 	x_x = (uint8)(((float)(points_r[i][0]+2) / image_w) * TFTSHOW_W);
	// 	y_y = (uint8)(((float)points_r[i][1] / image_h) * TFTSHOW_H);
	// 	tft180_draw_point(x_x, y_y, uesr_RED); //tft��Ļ����
	// }

	for (i = hightest; i < image_h-1; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//������
		//�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
		//��ȻҲ�ж�����ߵ��ҷ������Ǹ��˸о��ܷ�����������
		y_y = (uint8)(((float)i / image_h) * TFTSHOW_H);
		tft180_draw_point((uint8)(((float)center_line[i] / image_w) * TFTSHOW_W), y_y, uesr_GREEN);//��ʾ��� ��ʾ����	
		tft180_draw_point((uint8)(((float)l_border[i] / image_w) * TFTSHOW_W), y_y, uesr_GREEN);//��ʾ��� ��ʾ�����
		tft180_draw_point((uint8)(((float)r_border[i] / image_w) * TFTSHOW_W), y_y, uesr_GREEN);//��ʾ��� ��ʾ�ұ��� -- tft����
	}

	//��ʾ�ǵ�
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

/**********************************���´�������****************************************/
/*
�������ƣ�void Cal_P_Error(uint8 *_l_border, uint8 *_r_border)
����˵������ȡѭ��һ�м�������в�ֵ��ֵ
����˵����
*_l_border������ߵ�
*_r_border���ұ��ߵ�
begin_line:��ʼ��
end_line:������
center���м��x����
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��·�����������p_error>0��·���������ұ�p_error<0
example��Cal_P_Error(l_border, r_border, 95, 94);
 */
uint16_t weight_array[41] = {1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
                        	 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
							 3, 3, 3, 3, 4, 4, 5, 5, 5, 5};
void Cal_P_Error(uint8 *_l_border, uint8 *_r_border, uint16 begin_line, uint16 end_line, float center)
{
	uint16 i = 0;
	uint16 j = 0;
	uint16 sum_j = 0;
	uint16 line = begin_line - end_line + 1; //���ߵ�������
	float sum_error = 0.0;

	for(i = 0, j = line;j > 0;j--, i++)
	{
		sum_error += (center - ((_l_border[begin_line - i] + _r_border[begin_line - i]) * 0.5)) * weight_array[j];
		sum_j += weight_array[j];
	}
	p_error = sum_error / sum_j;
}

/*
�������ƣ�void Find_CornerPoint(uint8 *_l_border, uint8 *_r_border)
����˵����Ѱ�ҽǵ�
����˵����
*_l_border������ߵ�
*_r_border���ұ��ߵ�
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע����
example��Find_CornerPoint(l_border, r_border);
 */
uint16 cornerpoint[4][3]; //�����ǵ� -- ���£����� {x,y,flag}

void Find_CornerPoint(uint8 *_l_border, uint8 *_r_border)
{
	uint16 i, j;
	uint8 diuxian_count = 0; //������
	memset(cornerpoint, 0, sizeof(cornerpoint)); //��������

	for(i=CUT_H+4; i<=image_h-4; i++) //��ѭ����ʼ�е�����������ʮ�ֽǵ�
	{   //����ǰ���о����󣬺�һ�����޴��ж�Ϊ�ǵ�
		if (_l_border[0] != border_max && _l_border[1] != border_max && _l_border[2] != border_max) //�ж����½ǵ�
		{
			if (abs(_l_border[i] - _l_border[i+1]) < 3 && abs(_l_border[i] - _l_border[i+2]) < 3 &&
			    _l_border[i] - _l_border[i-2] >= 6 &&
				_l_border[i] >= 8 &&  _l_border[i+1] >= 8 &&  _l_border[i+2] >= 8) //�յ㼰ǰ���в��ڶ��߱�Ե
			{
				cornerpoint[0][0] = _l_border[i];
				cornerpoint[0][1] = i;
				cornerpoint[0][2] = 1;
			}
		}
		if (_r_border[0] != border_min && _r_border[1] != border_min && _r_border[2] != border_min) //�ж����½ǵ�
		{
			if (abs(_r_border[i] - _r_border[i+1]) < 3 && abs(_r_border[i] - _r_border[i+2]) < 3 &&
			    _r_border[i] - _r_border[i-2] <= -6 &&
				_r_border[i] <= 180 &&  _r_border[i+1] <= 180 &&  _r_border[i+2] <= 180) //�յ㼰ǰ���в��ڶ��߱�Ե
			{
				cornerpoint[1][0] = _r_border[i];
				cornerpoint[1][1] = i;
				cornerpoint[1][2] = 1;
			}
		}
		if (abs(_l_border[i] - _l_border[i-1]) < 3 && abs(_l_border[i] - _l_border[i-2]) < 3 &&
			_l_border[i] - _l_border[i+2] >= 6 &&
			_l_border[i] >= 8 && _l_border[i-1] >= 8 && _l_border[i-2] >= 8) //�ж����Ͻǵ�
		{
			for (j = i; j >= CUT_H; j--)
			{
				if (_l_border[i] == border_min) diuxian_count++;//����
			}
			if (diuxian_count < 3) //���߳���3��
			{
				cornerpoint[2][0] = _l_border[i];
				cornerpoint[2][1] = i;
				cornerpoint[2][2] = 1;
			}
			diuxian_count = 0;
		}
		if (abs(_r_border[i] - _r_border[i-1]) < 3 && abs(_r_border[i] - _r_border[i-2]) < 3 &&
			_r_border[i] - _r_border[i+2] <= -6 &&
			_r_border[i] <= 180 && _r_border[i-1] <= 180 && _r_border[i-2] <= 180) //�ж����Ͻǵ�
		{
			for (j = i; j >= CUT_H; j--)
			{
				if (_r_border[i] == border_max) diuxian_count++;//����
			}
			if (diuxian_count < 3) //���߳���3��
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
�������ƣ�void ImageAddingLine(uint8 *border, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
����˵����ͼ����
����˵����
imageSide  ������
startX     : ��ʼ�� ����
startY     : ��ʼ�� ����
endX       : ������ ����
endY       : ������ ����
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע��endY һ��Ҫ���� startY
example��ImageAddingLine(l_border, 0, 0, 8, 8);
 */
 void ImageAddingLine(uint8 *border, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
 {
    int i = 0;

	// ֱ�� y = kx + b
	line_equation temp_addline = LINE_CREATE();
	k_and_b(&temp_addline, startX, startY, endX, endY);

	for(i = startY; i < endY; i++)
	{
		border[i] = (uint8_t)((float)(i - temp_addline.b) / temp_addline.k);
	}
 }

//ʮ�ֲ���
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
* @brief ʮ�ֲ��ߺ���
* @param uint8(*image)[image_w]		�����ֵͼ��
* @param uint8 *l_border			������߽��׵�ַ
* @param uint8 *r_border			�����ұ߽��׵�ַ
* @param uint16 total_num_l			�������ѭ���ܴ���
* @param uint16 total_num_r			�����ұ�ѭ���ܴ���
* @param uint16 *dir_l				����������������׵�ַ
* @param uint16 *dir_r				�����ұ����������׵�ַ
* @param uint16(*points_l)[2]		������������׵�ַ
* @param uint16(*points_r)[2]		�����ұ������׵�ַ
*  @see CTest		cross_fill(image,l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);
* @return ����˵��
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
	//��ʮ��
	for (i = 1; i < total_num_l; i++)
	{
		if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
		{
			break_num_l = points_l[i][1];//����y����
			printf("brea_knum-L:%d\n", break_num_l);
			printf("I:%d\n", i);
			printf("ʮ�ֱ�־λ��1\n");
			break;
		}
	}
	for (i = 1; i < total_num_r; i++)
	{
		if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
		{
			break_num_r = points_r[i][1];//����y����
			printf("brea_knum-R:%d\n", break_num_r);
			printf("I:%d\n", i);
			printf("ʮ�ֱ�־λ��1\n");
			break;
		}
	}
	if (break_num_l&&break_num_r&&image[image_h - 1][4] && image[image_h - 1][image_w - 4])//�����������򶼷�������
	{
		//����б��
		start = break_num_l - 15;
		start = limit_a_b(start, 0, image_h);
		end = break_num_l - 5;
		calculate_s_i(start, end, l_border, &l_line_temp);
		//printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
		for (i = break_num_l - 5; i < image_h - 1; i++)
		{
			l_border[i] = l_line_temp.k * (i)+l_line_temp.b;//y = kx+b
			l_border[i] = limit_a_b(l_border[i], border_min, border_max);//�޷�
			// l_border[i] = (uint8_t)((float)(i - l_line_temp.b) / l_line_temp.k);//y = kx+b
			// l_border[i] = limit_a_b(l_border[i], border_min, border_max);//�޷�
		}

		//����б��
		start = break_num_r - 15;//���
		start = limit_a_b(start, 0, image_h);//�޷�
		end = break_num_r - 5;//�յ�
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
�������ƣ�void Circle_Addline(uint8 _status, uint16 l_sum, uint16 r_sum)
����˵����Բ������
����˵����
_flag	   ��Բ������ ��1����Բ�� 2����Բ��
_status    ��Բ��״̬ ��1����ֱ�� 2������� 3����� 4������� 5��Զֱ��
�������أ���
�޸�ʱ�䣺2022��9��25��
��    ע����
example��Circle_Addline(YuanHuan_Flag, yuanhuan_status)
 */
void Circle_Addline(uint8 _flag, uint8 _status)
{
	uint8 i;
	if (_flag == 1) //��Բ��
	{
		switch (_status)
		{
		case 1: //Բ��ֱ�߲���
			ImageAddingLine(l_border, 63, 50, 53, 90);
			my_tft180_draw_line(63, 50, 53, 90, uesr_RED);
			break;
		case 2: //���������������ߴ��
			ImageAddingLine(r_border, 73, 48, 130, 90);
			my_tft180_draw_line(73, 48, 130, 90, uesr_RED);
			break;  
		case 3: //�����������ѭ����������
			break;
		case 4: //���������������ߴ��
			ImageAddingLine(r_border, 48, 50, 130, 90);
			my_tft180_draw_line(48, 50, 130, 90, uesr_RED);
			break;
		case 5: //�Ѿ����������ֱ��ֱ��
			ImageAddingLine(l_border, 75, 50, 49, 90);
			my_tft180_draw_line(75, 50, 49, 90, uesr_RED);
			break;
		default:
			break;
		}
	}
	if (_flag == 2) //��Բ��
	{
		switch (_status)
		{
		case 1: //Բ��ֱ�߲���
			ImageAddingLine(r_border, 117, 50, 137, 90);
			my_tft180_draw_line(117, 50, 137, 90, uesr_RED);
			break;
		case 2: //��������������ұߴ��
			ImageAddingLine(l_border, 115, 48, 58, 90);
			my_tft180_draw_line(115, 48, 58, 90, uesr_RED);
			break;
		case 3: //�����������ѭ����������
			break;
		case 4: //��������������ұߴ��
			ImageAddingLine(l_border, 140, 50, 58, 90);
			my_tft180_draw_line(140, 50, 58, 90, uesr_RED);
			break;
		case 5: //�Ѿ����������ֱ��ֱ��
			ImageAddingLine(r_border, 113, 50, 139, 90);
			my_tft180_draw_line(113, 50, 139, 90, uesr_RED);
			break;
		default:
			break;
		}
	}
}
/*

��������㣨0.0��***************����>*************xֵ���
************************************************************
************************************************************
************************************************************
************************************************************
******************��������һ��ͼ��*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
yֵ���*******************************************(188.120)

*/


