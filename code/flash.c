#include "zf_common_headfile.h"

//����flash���һ������������������ͻ
//�洢������ô����һ��������ʼʹ��
#define FLASH_SECTOR_126        (126)

#define FLASH_SAVE_NUM 4	 								//�洢��������--��������
#define FLASH_SECTOR_PAGE0   (0)          //ҳ��Ϊ0-7
#define FLASH_SECTOR_PAGE1   (1)
#define FLASH_SECTOR_PAGE2   (2)
#define FLASH_SECTOR_PAGE3   (3)
#define FLASH_SECTOR_PAGE4   (4)
#define FLASH_SECTOR_PAGE5   (5)
#define FLASH_SECTOR_PAGE6   (6)
#define FLASH_SECTOR_PAGE7   (7)

// д������
uint8  write_data_mode = 1;  //�����޸�ģʽ
uint16 write_data2 = 2;
uint32 write_data3 = 3;
float  write_data4 = 4.4;

// ��ȡ����
uint8  read_data_mode;
uint16 read_data2;
uint32 read_data3;
float  read_data4;

// �洢��������
uint32 write_buf[FLASH_SAVE_NUM];
uint32 read_buf[FLASH_SAVE_NUM];


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������(ͬʱʵ��ͬ������)
//  @param      sector_num			�������
//  @param      page_num				����ҳ�����
//  @param      save_num				�洢��������---��ʱ����Ϊ4��
//  @return     void
//  @example    save_data(FLASH_SECTOR_126, FLASH_SECTOR_PAGE0, FLASH_SAVE_NUM);
//  @note       �����ı�write_dataֵ�Ⱥ����д˺���
//-------------------------------------------------------------------------------------------------------------------
void save_data(int sector_num,int page_num,int save_num)
{
	uint8 status;
	if(flash_check(page_num,page_num))//У�鵱ǰ ��������ҳ�Ƿ������ݣ�����������������������
    {
        status = flash_erase_page(page_num,page_num);//������������������Ѿ�������������������֮������ٴ�д���µ�����
        if(status)  while(1);//����ʧ��
    }
		
		write_buf[0] = write_data_mode;
		write_buf[1] = write_data2;
		write_buf[2] = write_data3;
		write_buf[3] = *(uint32 *)&write_data4;  //�洢����ʱ������ȡ������ַȻ����uint32 *�����ʱ�����ȡ���ݡ�
		
		status = flash_write_page(page_num, page_num, write_buf, save_num);//д������
    if(status)  while(1);//д��ʧ��
		

		/*------------------------------------α���루ͬ�����Σ�----------------------------------------
			ģʽ = write_data_mode;
			����2 = write_data2;
			����3 = write_data3;
		  ����4 = write_data4
		*/
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��flash��ȡ����(����һ�δ洢��������)
//  @param      sector_num			�������
//  @param      page_num				����ҳ�����
//  @param      save_num				�洢��������---��ʱ����Ϊ4��
//  @example    read_data(FLASH_SECTOR_126, FLASH_SECTOR_PAGE0, FLASH_SAVE_NUM);
//  @return     void
//  @note       
//-------------------------------------------------------------------------------------------------------------------
void read_data(int sector_num,int page_num,int save_num)
{
		flash_read_page(sector_num,page_num,read_buf,save_num);
		read_data_mode = read_buf[0];
		read_data2 = read_buf[1];
		read_data3 = read_buf[2];
		read_data4 = read_buf[3];

		/*------------------------------------α���루���������----------------------------------------
			ģʽ = read_data_mode;
			����2 = read_data2;
			����3 = read_data3;
		  ����4 = read_data4
		*/
}

// ��ջ�������flash_buffer_clear()