#ifndef SWJ_H_
#define SWJ_H_

#include "zf_common_headfile.h"

//��ɫ color���� [ͼ���߽�������ǵ���]
#define swj_BLACK 0
#define swj_WHITE 1
#define swj_RED 2
#define swj_GREEN 3
#define swj_BLUE 4
#define swj_PURPLE 5
#define swj_YELLOW 6
#define swj_CYAN 7
#define swj_ORANGE 8

//��־λtype
#define swj_point_type1 1 //Сʮ�� 3x3
#define swj_point_type2 2 //��ʮ�� 5x5
#define swj_point_type3 3 //��ʮ�� 7x7
#define swj_point_type4 4 //С  X  3x3
#define swj_point_type5 5 //��  X  5x5
#define swj_point_type6 6 //��  X  7x7
#define swj_point_type7 7 //ȫ��ʮ��
#define swj_point_type8 8 //����һ��ֱ�� y������ x��������
#define swj_point_type9 9 //����һ��ֱ�� x������ y��������


/*[  ��ͼ��    ]*/
//�Ҷ�
void sendimg(uint8* image, uint8 width, uint8 height);
//��У��ĻҶ�
void sendimg_A( uint8* image, uint8 width, uint8 height);
//�ߴ�ѹ���Ҷ�
void sendimg_zoom(uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height);
//��ֵ��ͼ�� (��ɫ ������ʵ��ʵʱͼ�� �ȻҶȿ�40��)
void sendimg_binary( uint8* image, uint8 width, uint8 height,uint8 otu);
//�ߴ�ѹ����ֵ��ͼ��
void sendimg_binary_zoom( uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height,uint8 otu);
//��У���ѹ��ͼ��(*�Ƽ�)
void sendimg_binary_CHK(uint8* image, uint8 width, uint8 height,uint8 otu,uint8 chk);
//��jpeg��ɫͼ��
void sendimg_JPEG(uint8* image,int lon);

/*[  ���߽�    ]*/
//���� ͼ���߽�ǰҪ���� ��Ȼ���������
void sendline_clear( uint8 color,uint8 width, uint8 height);
//�������͵ı߽�ͼ�� ����int���鴢��߽���Ϣ
void sendline_type(uint8 color,uint8 *buff, uint32 len,uint8 size_lon);
//���߽� uint8�������͵� ÿһλ��Ӧһ�еı߽��
void sendline( uint8 color,uint8 *buff, uint32 len);
//��˳��߽� ������������
void sendline2( uint8 color,uint8 *linex,uint8 *liney, uint32 len);
//�����ұ��� �Ƽ��������
void sendline_xy( uint8 *line_zx,uint8 *line_yx, uint32 len);
//���������ǵ� ����յ�
void sendpoint(uint8 color,uint8 x,uint8 y, uint8 type);

/*[  ���߽�+ͼ��    ]*/
//˵��: �����ô��߽�+ͼ��    ��ͼ���봫�߽� ���ͼ��ֿ��������ͼ�� �ұ��Ǳ߽� ���Ե��ں� �߽���ں�ͼ����
//�ŵ��ǿ��Կ�ԭʼ��ֵ��ͼ�� �� ���д������
//������Ϊ���ʱ���ȶ��� ���ٶȺܿ��ʱ�� �ᷢ�ֱ߽���ͼ���Ӧ������
//�����ٶȿ��� ���߽�+ͼ�� �߽���Ȼ����ڻ����� ͼ�����ѱ߽�ֱ����ʾͼ���� ��������(ͼ��Ḳ��)

//�߽� �÷�������һ��
void sendimgAndLine( uint8 color,uint8 *buff, uint32 len);
void sendimgAndLine_type(uint8 color,uint8 *buff, uint32 len,uint8 size_lon);
void sendimgAndLine_point(uint8 color,uint8 x,uint8 y, uint8 type);
//ͼ��
void sendimg_BinaryAndLine(uint8* image, uint8 width, uint8 height,uint8 otu);




/*[  ���ݱ�ǩ    ]*/
//*���ݱ�ǩ  ¼�ƻط� ��bug���ջ�������־λ ����Ҫ
//nameͨ�� dat���� ����ʾ���Ҳ��ǩ�� ���ʾ���� �Ҽ��Ǳ��̰� ��ɫ�궨
void put_int32(uint8 name, int dat);
void put_float(uint8 name, float dat);


//������Щѡ�ֻ����ʹ��vofa+ �������������λ��
//������ܾ�������ת����������� ʵ�ֹ�ͬЭ��ʹ�õĹ���
//ֹͣ��λ���������� ��������ת��*
//*��Ҫ����λ�� ͼ���� ���� ������ת �����������
void swj_stop();
//������λ���������� �ر�����ת��
void swj_start();

//�ڲ�����
uint16 swj_CRC(uint8 *Buf,uint8 CRC_sta, uint8 CRC_CNT);
//int32 ByteToInt(int8 *ndsk);
//float ByteToFloat(unsigned char* byteArry);
#define sendline_uint8(color,buff,len) sendline_type(color,buff,len,1)
#define sendline_int(color,buff,len) sendline_type(color,buff,len,4)
#define sendline_uint16(color,buff,len) sendline_type(color,buff,len,2)

#endif /* SWJ_H_ */
