#ifndef SWJ_H_
#define SWJ_H_

#include "zf_common_headfile.h"

//颜色 color定义 [图传边界或特殊标记点用]
#define swj_BLACK 0
#define swj_WHITE 1
#define swj_RED 2
#define swj_GREEN 3
#define swj_BLUE 4
#define swj_PURPLE 5
#define swj_YELLOW 6
#define swj_CYAN 7
#define swj_ORANGE 8

//标志位type
#define swj_point_type1 1 //小十字 3x3
#define swj_point_type2 2 //中十字 5x5
#define swj_point_type3 3 //大十字 7x7
#define swj_point_type4 4 //小  X  3x3
#define swj_point_type5 5 //中  X  5x5
#define swj_point_type6 6 //大  X  7x7
#define swj_point_type7 7 //全屏十字
#define swj_point_type8 8 //横向一条直线 y起作用 x不起作用
#define swj_point_type9 9 //众向一条直线 x起作用 y不起作用


/*[  传图像    ]*/
//灰度
void sendimg(uint8* image, uint8 width, uint8 height);
//带校验的灰度
void sendimg_A( uint8* image, uint8 width, uint8 height);
//尺寸压缩灰度
void sendimg_zoom(uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height);
//二值化图传 (特色 串口能实现实时图传 比灰度快40倍)
void sendimg_binary( uint8* image, uint8 width, uint8 height,uint8 otu);
//尺寸压缩二值化图传
void sendimg_binary_zoom( uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height,uint8 otu);
//带校验的压缩图传(*推荐)
void sendimg_binary_CHK(uint8* image, uint8 width, uint8 height,uint8 otu,uint8 chk);
//传jpeg彩色图像
void sendimg_JPEG(uint8* image,int lon);

/*[  传边界    ]*/
//清屏 图传边界前要清屏 不然线条会叠加
void sendline_clear( uint8 color,uint8 width, uint8 height);
//特殊类型的边界图传 例如int数组储存边界信息
void sendline_type(uint8 color,uint8 *buff, uint32 len,uint8 size_lon);
//传边界 uint8数组类型的 每一位对应一行的边界点
void sendline( uint8 color,uint8 *buff, uint32 len);
//无顺序边界 八邻域这种用
void sendline2( uint8 color,uint8 *linex,uint8 *liney, uint32 len);
//传左右边线 推荐用上面的
void sendline_xy( uint8 *line_zx,uint8 *line_yx, uint32 len);
//绘制特殊标记点 列如拐点
void sendpoint(uint8 color,uint8 x,uint8 y, uint8 type);

/*[  传边界+图像    ]*/
//说明: 后期用传边界+图像    传图像与传边界 会把图像分开，左边是图像 右边是边界 可以点融合 边界会融合图像上
//优点是可以看原始二值化图像 和 进行代码仿真
//但是因为解包时序稳定等 当速度很快的时候 会发现边界与图像对应不起来
//后期速度快了 传边界+图像 边界会先缓存在缓冲区 图传后会把边界直接显示图像上 不用清屏(图像会覆盖)

//边界 用法和上面一致
void sendimgAndLine( uint8 color,uint8 *buff, uint32 len);
void sendimgAndLine_type(uint8 color,uint8 *buff, uint32 len,uint8 size_lon);
void sendimgAndLine_point(uint8 color,uint8 x,uint8 y, uint8 type);
//图像
void sendimg_BinaryAndLine(uint8* image, uint8 width, uint8 height,uint8 otu);




/*[  数据标签    ]*/
//*数据标签  录制回放 找bug，闭环，看标志位 很重要
//name通道 dat数据 会显示在右侧标签栏 左键示波器 右键仪表盘绑定 颜色标定
void put_int32(uint8 name, int dat);
void put_float(uint8 name, float dat);


//可能有些选手会搭配使用vofa+ 匿名等优秀的上位机
//这个功能就是数据转发给其余软件 实现共同协助使用的功能
//停止上位机解析数据 开启数据转发*
//*需要在上位机 图传新 设置 数据中转 连接其余软件
void swj_stop();
//开启上位机解析数据 关闭数据转发
void swj_start();

//内部调用
uint16 swj_CRC(uint8 *Buf,uint8 CRC_sta, uint8 CRC_CNT);
//int32 ByteToInt(int8 *ndsk);
//float ByteToFloat(unsigned char* byteArry);
#define sendline_uint8(color,buff,len) sendline_type(color,buff,len,1)
#define sendline_int(color,buff,len) sendline_type(color,buff,len,4)
#define sendline_uint16(color,buff,len) sendline_type(color,buff,len,2)

#endif /* SWJ_H_ */
