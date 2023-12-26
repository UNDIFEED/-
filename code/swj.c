/*
 * GL-Shw���ܳ���λ��  qqȺ:297809802 qq1849057843
 *
 *Ŀ¼��
 *|||||||||||||||||||������ͼ��Э��ֻ��ѡ��һ�֡�
 * һ���Ҷ�ͼ��
 *  ����ͨ�Ҷ�ͼ��
 *  �ڿ����ŻҶ�ͼ��
 *  ��ѹ���Ҷ�ͼ��
 * ������ֵ��ͼ����*��Э�����40�����ҵ�ѹ���ʺ�У�� ��[ǿ���Ƽ�]
 *  �ٶ�ֵ��ͼ��
 *  �ڴ�У���ֵ��ͼ��
 *  ���޸�ͼ���С�Ķ�ֵ��ͼ��
 *
 * ||||||||||||||||||������Э�������ͼ��Э�鲢��ʹ�á�
 * ����ͼ���߽磨Ѱ�ߵı߽���Ϣ���Ƽ������ֵͼ����ʵʱ�鿴������Ѱ�߲��ߣ�[�Ƽ�]
 * �ġ����ݱ�ǩ������ʾ����������ɫ�궨�����ݼ��ӣ��Ǳ��̣�[ǿ���Ƽ�]
 *                          ����ʱ�䣺2022��12��9��
 */
#include "swj.h"

//��Ҫ�޸ĵĵط�
#define  sw_write_byte(dat)  uart_write_byte(UART_1, dat)//���ڷ����ֽ�
#define  sw_write_buffer(dat,len)  uart_write_buffer(UART_1, dat,len)//���ߴ��ڷ�������򴮿ڷ�������

/*
 * �޸�˵��:
 * sw_write_byte ȥ�Ҵ��ڿ��еķ����ֽں���
 * sw_write_buffer ȥ�� *���ߴ��� �� �еķ������麯��
 *
 * ���ʹ�õ���ɵĿ� �� ��ɵ�����ģ�� sw_write_buffer���ע��һ��
 * */




//--------------��ͨ�Ҷ�ͼ��-------------------//
//*imageͼ���ַ widthͼ��� heightͼ���
//����sendimg(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H);
//���ݰ���С:6+width * height(ͼ��һ֡���ֽ���)
void sendimg(uint8* image, uint8 width, uint8 height)
{
    uint8 dat[6] = { 0x21, 0x7A, width, height, 0x21, 0x7A };
    sw_write_buffer(dat, 6);
    sw_write_buffer(image, width * height);
}
//--------------�����ŻҶ�ͼ��-------------------//
//����ʧ���ݵ�����£���Э�������¶�λ����ʵ��һ���̶ȿ���������
//*imageͼ���ַ widthͼ��� heightͼ���
//����sendimg(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H);
//���ݰ���С:6+��width+3�� * height(ͼ��һ֡���ֽ���)
void sendimg_A( uint8* image, uint8 width, uint8 height)
{

    sw_write_byte(0x21); sw_write_byte(0x7A);
    sw_write_byte(width);sw_write_byte(height);
    sw_write_byte((width+height)/2);sw_write_byte(0x7A);

    uint8 line=0,col=0;
    for(line=0;line<width;line++)
        {
        sw_write_byte(21);
        sw_write_byte(line);
        sw_write_byte(133);
           for(col=0;col<height;col++)
           {
               sw_write_byte(*(image+line*height+col));

           }

        }
}
//--------------ѹ���Ҷ�ͼ��-------------------//
//����ѹ��ͼ�� ���� 120x180��ͼ��̫�� �����ٶ���  ����Է��� 60x90��ͼ��������ٶ�
//*imageͼ���ַ widthͼ��� heightͼ���dis_widthѹ�����ͼ��� dis_heightѹ�����ͼ���
//����sendimg(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H,MT9V03X_DVP_W/2,MT9V03X_DVP_H/2);
//���ݰ���С:6+dis_width * dis_height(ͼ��һ֡���ֽ���)
void sendimg_zoom(uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height)
{
    uint8 dat[6] = { 0x21, 0x7A, dis_width, dis_height, 0x21, 0x7A };
    sw_write_buffer(dat, 6);
    uint8 i,j;
    for(j=0;j<dis_height;j++)
    {
    for(i=0;i<dis_width;i++)
    {
       sw_write_byte(*(image+(j*height/dis_height)*width+i*width/dis_width));//��ȡ���ص�

    }
    }
}

//--------------��ֵ��ͼ��-------------------//
//imageͼ���ַ widthͼ��� heightͼ��� otu��ֵ����ֵ
//����sendimg_binary(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H,100);
void sendimg_binary( uint8* image, uint8 width, uint8 height,uint8 otu)
{

    uint8 dat[6]={0x7A,0x21,width,height,0x7A,0x21};
    sw_write_buffer(dat,6);
    int databool=255;uint8 lon=1;int data=255;
    uint8 line=0,col=0;

    for(line=0;line<width;line++)
    {
        for(col=0;col<height;col++)
        {
            if(*(image+line*height+col)>otu)data=255;
            else data=0;
            if(data==databool)
            {lon++;}else{sw_write_byte(lon);lon=1;}
            if(lon==190){sw_write_byte(lon-1);sw_write_byte(0);lon=1;}
            databool=data;
        }
    }
}
//ѹ����ֵͼ��
//uint16 dis_width, uint16 dis_height Ҫѹ��ͼ���С
void sendimg_binary_zoom( uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height,uint8 otu)
{

    uint8 dat[6]={0x7A,0x21,dis_width,dis_height,0x7A,0x21};
    sw_write_buffer(dat,6);
    int databool=255;uint8 lon=1;int data=255;
    uint8 i,j;
    for(j=0;j<dis_height;j++)
    {
    for(i=0;i<dis_width;i++)
    {
    if(*(image+(j*height/dis_height)*width+i*width/dis_width)>otu)//��ȡ���ص�
    data=255;
    else data=0;
    if(data==databool)
    {lon++;}
    else{sw_write_byte(lon);lon=1;}
    if(lon==190){sw_write_byte(lon-1);sw_write_byte(0);lon=1;}
    databool=data;
    }
    }

}
//����У��Ķ�ֵͼ��
//chkֵԽ�� ������Խǿ ֵ0-55
//�����ʵ��ʹ��������е���
void sendimg_binary_CHK(uint8* image, uint8 width, uint8 height,uint8 otu,uint8 chk)
{
    chk=chk>0?chk:0;
    chk=chk<56?chk:55;
    uint8 dat[7]={0x7A,0x21,width,height,0x7A,0x21,200+chk};
      sw_write_buffer(dat,7);
      int databool=255;uint8 lon=0;int data=255;
      uint8 line=0,col=0;
      int imglon=0;
      int imgdatlo=width*height/chk;
      uint8 CHK=0;
      for(line=0;line<width;line++)
          {
             for(col=0;col<height;col++)
             {imglon++;

                if(*(image+line*height+col)>otu)data=255;
                else data=0;
                if(data==databool)
                {lon++;}
                else{sw_write_byte(lon);lon=1;}

                if(imglon==imgdatlo)
                {CHK++;sw_write_byte(lon);data=255; databool=255;sw_write_byte(200+CHK);lon=0;imglon=0;}
                if(lon==190){sw_write_byte(lon);sw_write_byte(0);lon=0;}
               databool=data;
             }
          }
}
//--------------��ֵ��ͼ�����߽�-------------------//
//!ע��:�ȷ��߽� �� ��־�� ��ͼ����ͼ��
//----------------------���ͻ�����------------------------------
#define Line_SIZE (300) // �߽绺������С ��̫�� ���Լ�������� (�߽糤��+5)*�߽����+7*��־�����
static uint8 Line_Buffer[Line_SIZE] = {0};
int Line_lon=0;
//�߽� *�߽����Ͷ�����Ҫ��uint8
void sendimgAndLine( uint8 color,uint8 *buff, uint32 len)
{
    Line_Buffer[Line_lon]=(0x21);Line_lon++;
    Line_Buffer[Line_lon]=(color);Line_lon++;
    Line_Buffer[Line_lon]=(len);Line_lon++;
    Line_Buffer[Line_lon]=(255);Line_lon++;
    Line_Buffer[Line_lon]=(255);Line_lon++;
    for(int i=0;i<len;i++){ Line_Buffer[Line_lon]=*(buff+i);Line_lon++;}
}
/*���Ʊ���   color������ɫ  uint8_t *buff ���͵ı��������ַ  len���͵ı��߳���
 * �����ű߽��������int���� size_lon��4(int ռ4�ֽ�)
 *�����ű߽��������uint6���� size_lon��2(uint6 ռ2�ֽ�)
 * �����ű߽��������uint8���� size_lon��1(uint8 ռ1�ֽ�) ��ͬ��sendimgAndLine( uint8 color,uint8 *buff, uint32 len)
 */
void sendimgAndLine_type(uint8 color,uint8 *buff, uint32 len,uint8 size_lon)
{
    Line_Buffer[Line_lon]=(0x21);Line_lon++;
    Line_Buffer[Line_lon]=(color);Line_lon++;
    Line_Buffer[Line_lon]=(len);Line_lon++;
    Line_Buffer[Line_lon]=(255);Line_lon++;
    Line_Buffer[Line_lon]=(255);Line_lon++;
    for(int i=0;i<len*size_lon;size_lon){ Line_Buffer[Line_lon]=*(buff+i);Line_lon++;}
}
//#define swj_point_type1 1 //Сʮ�� 3x3
//#define swj_point_type2 2 //��ʮ�� 5x5
//#define swj_point_type3 3 //��ʮ�� 7x7
//#define swj_point_type4 4 //С  X  3x3
//#define swj_point_type5 5 //��  X  5x5
//#define swj_point_type6 6 //��  X  7x7
//#define swj_point_type7 7 //ȫ��ʮ��
//#define swj_point_type8 8 //����һ��ֱ�� y������ x��������
//#define swj_point_type9 9 //����һ��ֱ�� x������ y��������
//��־�㣨����յ� ���ⲹ�ߵĵ㣩
//���� �㣨10,11��sendpoint(swj_RED,10,11,swj_point_type1);//��10��11�������ֺ�ɫСʮ��
//��ɫ ����x ����y �����ͣ�������ĺ궨�壩
void sendimgAndLine_point(uint8 color,uint8 x,uint8 y, uint8 type)
{
    Line_Buffer[Line_lon++]=(0x22);
    Line_Buffer[Line_lon++]=(color);
    Line_Buffer[Line_lon++]=(type);
    Line_Buffer[Line_lon++]=(254);
    Line_Buffer[Line_lon++]=(x);
    Line_Buffer[Line_lon++]=(y);
    Line_Buffer[Line_lon++]=(255);
}
//ͼ��
void sendimg_BinaryAndLine(uint8* image, uint8 width, uint8 height,uint8 otu)
{uint8 chk=50;
uint8 dat2[6]={0x77,0x22,0x77,0x22,Line_lon/255%255,Line_lon%255};
sw_write_buffer(dat2,6);
uint8 dat[7]={0x7A,0x21,width,height,0x7A,0x21,200+chk};
        sw_write_buffer(dat,7);
        int databool=255;uint8 lon=0;int data=255;
        uint8 line=0,col=0;
        int imglon=0;
        int imgdatlo=width*height/chk;
        uint8 CHK=0;
        for(line=0;line<width;line++)
            {
               for(col=0;col<height;col++)
               {imglon++;

                  if(*(image+line*height+col)>otu)data=255;
                  else data=0;
                  if(data==databool)
                  {lon++;}
                  else{sw_write_byte(lon);lon=1;}

                  if(imglon==imgdatlo)
                  {CHK++;sw_write_byte(lon);data=255; databool=255;sw_write_byte(200+CHK);lon=0;imglon=0;}
                  if(lon==190){sw_write_byte(lon);sw_write_byte(0);lon=0;}
                 databool=data;
               }
            }
        uint8 dat3[6]={0x34,0xFC,0x34,0xFC,0x34,0xFC};
        sw_write_buffer(dat3,6);
        sw_write_buffer(Line_Buffer,Line_lon);Line_lon=0;
}
//��ɫͼ�� imageͼ���ַ lon���ݳ�
void sendimg_JPEG(uint8* image,int lon)
{
    uint8_t dat[7] = { 0x21,0xFE, 0x7A, lon/255, lon%255, 0x21, 0x7A };
    sw_write_buffer(dat,7);sw_write_buffer(image,lon);
}
//--------------���ݱ�ǩ������ʾ����������ɫ�궨�����ݼ��ӣ��Ǳ��̣�-------------------//
//һ������ռһ����ַ,��ֱ�ӳ�����ͼ��ҳ������� ��һ�¿���ʾ�����Ҽ�����������ɫ�궨 �趨��
//��ֵ����ɫ�궨���� ���ټ���������û�д�����Ҳ���԰󶨱��̣��ٶȸ�ֱ�ۡ�¼��ʱ��ͬ��¼��
//�����ں�ϻ�ӣ�С��ʲô״̬һ�۱�֪
//name���ݱ�ʶ(ͨ������ַ)[0-255]  dat:����
//��:int a=0;put_int32(0,a);

//����У�������ʾ��
void put_int32(uint8 name, int dat)
{
    uint8 datc[10] = { 197, name,1,0,0,0,0,0,0,198};
    datc[3] = (uint8)(dat & 0xFF);
    datc[4] = (uint8)((dat & 0xFF00) >> 8);
    datc[5] = (uint8)((dat & 0xFF0000) >> 16);
    datc[6] = (uint8)((dat >> 24) & 0xFF);
    uint8 crc[6] = { name, 1,datc[3],datc[4],datc[5],datc[6]};
    uint16 CRC16 =  swj_CRC(crc,0,6);
    datc[7] = (uint8)(CRC16&0xff);
    datc[8] = (uint8)(CRC16>>8);
    sw_write_buffer(datc,10);
}

void put_float(uint8 name, float dat)
{
    uint8 datc[10] = { 197, name,2,0,0,0,0,0,0,198};
    char farray[4] = {0};

    *(float*)farray=dat;
    unsigned char *p = (unsigned char*)&dat + 3;
    datc[3] = *(p-3);
    datc[4] = *(p-2);
    datc[5] = *(p-1);
    datc[6] = *p;

    uint8 crc[6] = { name, 2,datc[3],datc[4],datc[5],datc[6]};
    uint16 CRC16 =  swj_CRC(crc,0,6);
    datc[7] = (uint8)(CRC16&0xff);
    datc[8] = (uint8)(CRC16>>8);
    sw_write_buffer(datc,10);
}


//--------------����-------------------//
/*
 * ���Խ�Ѱ�ߵõ������ұ��ߣ�������߷��͵���λ���鿴
 * ���磺
 * ���� ͼ��img[H][W];�õ����ұ��ߴ����zx[H] yx[H] �����������Ϊw[H]
 * sendline_clear(swj_BLACK,W,H);//���� ������ɫ
 * sendline(swj_WHITE,zx,H);//���������
 * sendline(swj_WHITE,yx,H);//�����ұ���
 * sendline(swj_WHITE,wx,H);//��������
 * ���磺
 * sendline_clear(swj_BLACK,W,H);//���� ������ɫ
 * sendline_xy(zx,yx,H)//�������ұ���
 * sendline(swj_WHITE,wx,H);//��������
 *
 * ������������Ч��һ�µģ��������������λ���϶�Ӧʹ��������ԭ����
 * ע�⣺
 * ��ÿ������һ֡��ͼ����� ��Ҫ����sendline_clear��������
 * ���������sendline_xy������ʹ����λ��������ԭ����ʱ�����ٵ���sendline һ������sendline_xy���� ��ֹ������
 * */
//#define swj_BLACK 0
//#define swj_WHITE 1
//#define swj_RED 2
//#define swj_GREEN 3
//#define swj_BLUE 4
//#define swj_PURPLE 5
//#define swj_YELLOW 6
//#define swj_CYAN 7
//#define swj_ORANGE 8

//�������   color������ı�����ɫ  uint16 width uint16 height ͼ��Ĵ�С
void sendline_clear(uint8 color,uint8 width, uint8 height)
{
    sw_write_byte(0x21); sw_write_byte(0x7A);
    sw_write_byte(width);sw_write_byte(height);
    sw_write_byte(color);sw_write_byte(0x21);
}

/*���Ʊ���   color������ɫ  uint8_t *buff ���͵ı��������ַ  len���͵ı��߳���
 * �����ű߽��������int���� size_lon��4(int ռ4�ֽ�)
 *�����ű߽��������uint6���� size_lon��2(uint6 ռ2�ֽ�)
 * �����ű߽��������uint8���� size_lon��1(uint8 ռ1�ֽ�) ��ͬ��sendline( uint8 color,uint8 *buff, uint32 len)
 */
 void sendline_type(uint8 color,uint8 *buff, uint32 len,uint8 size_lon)
 {
    sw_write_byte(0x21);
    sw_write_byte(color);
    sw_write_byte(len);
    sw_write_byte(255);
    sw_write_byte(255);
 for(int i=0;i<len*size_lon;i+=size_lon)
 {
     sw_write_byte(*(buff+i));
 }

}
/*Ĭ��ÿ��һ����*/
//���Ʊ���   color������ɫ  uint8_t *buff ���͵ı��������ַ  len���͵ı��߳��� *�߽����Ͷ�����Ҫ��uint8
void sendline( uint8 color,uint8 *buff, uint32 len)
{
    sw_write_byte(0x21);
    sw_write_byte(color);
    sw_write_byte(len);
    sw_write_byte(255);
    sw_write_byte(255);
    sw_write_buffer(buff,len);
}
/*˵��:
 * ������������ a(x1,y1)b(x2,y2)c(x3,y3)
 * �� uint8 x[3]={x1,x2,x3};uint8 y[3]={y1,y2,y3};
 *  sendline2(swj_WHITE,x,y,3);
 *  sendline����ֻ�ܰ�˳��ÿ��һ�㷢�ͱ߽��
 *  sendline2��������ڰ������ ÿ�в��̶������ı߽�
  *           Ҳ�����ڷ�������� ����յ� �������
 *
 * */
//��˳����Ʊ���  color������ɫ linex��Ӧ���x���꼯�� liney��Ӧ���y���꼯��  len���͵ı��߳���
void sendline2( uint8 color,uint8 *linex,uint8 *liney, uint32 len)
{
    sw_write_byte(0x21);
    sw_write_byte(color);
    sw_write_byte(len);
    sw_write_byte(254);
    sw_write_byte(255);
    sw_write_buffer(linex,len);
    sw_write_buffer(liney,len);
}
//ͼ�������߽�  uint8_t *zx:��߽�   uint8_t *yx:�ұ߽�, uint32_t len���͵ı��߳���
void sendline_xy( uint8 *line_zx,uint8 *line_yx, uint32 len)
{
    sw_write_byte(0x21);
    sw_write_byte(9);
    sw_write_byte(len);
    sw_write_byte(255);
    sw_write_byte(255);
    sw_write_buffer(line_zx,len);
    sw_write_buffer(line_yx,len);
}
//#define swj_point_type1 1 //Сʮ�� 3x3
//#define swj_point_type2 2 //��ʮ�� 5x5
//#define swj_point_type3 3 //��ʮ�� 7x7
//#define swj_point_type4 4 //С  X  3x3
//#define swj_point_type5 5 //��  X  5x5
//#define swj_point_type6 6 //��  X  7x7
//#define swj_point_type7 7 //ȫ��ʮ��
//#define swj_point_type8 8 //����һ��ֱ�� y������ x��������
//#define swj_point_type9 9 //����һ��ֱ�� x������ y��������
//��־�㣨����յ� ���ⲹ�ߵĵ㣩
//���� �㣨10,11��sendpoint(swj_RED,10,11,swj_point_type1);//��10��11�������ֺ�ɫСʮ��
//��ɫ ����x ����y �����ͣ�������ĺ궨�壩
void sendpoint(uint8 color,uint8 x,uint8 y, uint8 type)
{
    sw_write_byte(0x22);
    sw_write_byte(color);
    sw_write_byte(type);
    sw_write_byte(254);
    sw_write_byte(x);
    sw_write_byte(y);
    sw_write_byte(255);
}

//��ͼ��ʹ�ccd����ͬʱ����
void send_CCD(uint16 *dat)
{
   sw_write_byte(0x00);
   sw_write_byte(0xff);
   sw_write_byte(0x01);
   sw_write_byte(0x00);
    for(int i=0; i<128; i++)
    { uint8 *p;p=&dat[i];
    sw_write_byte(*(p++));
    sw_write_byte(*(p));
    }
}







void swj_stop()
{
  uint8 buff[8]={0xFB,0xFE,0x00,0x20,0xFA,0x20,0xFA,0x20};
  sw_write_buffer(buff,8);
}

void swj_start()
{
  uint8 buff[8]={0xFB,0xFE,0x01,0xEC,0xFA,0x20,0xFA,0x20};
  sw_write_buffer(buff,8);
}










//�ڲ�����
uint16 swj_CRC(uint8 *Buf,uint8 CRC_sta, uint8 CRC_CNT)
{
    uint16 CRC_Temp;
    uint8 i,j;
    CRC_Temp = 0xffff;

    for (i=CRC_sta;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
