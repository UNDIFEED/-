#include "zf_common_headfile.h"

//定义flash最后一个扇区，避免与程序冲突
//存储参数最好从最后一个扇区开始使用
#define FLASH_SECTOR_126        (126)

#define FLASH_SAVE_NUM 4	 								//存储参数个数--可以增加
#define FLASH_SECTOR_PAGE0   (0)          //页数为0-7
#define FLASH_SECTOR_PAGE1   (1)
#define FLASH_SECTOR_PAGE2   (2)
#define FLASH_SECTOR_PAGE3   (3)
#define FLASH_SECTOR_PAGE4   (4)
#define FLASH_SECTOR_PAGE5   (5)
#define FLASH_SECTOR_PAGE6   (6)
#define FLASH_SECTOR_PAGE7   (7)

// 写入数据
uint8  write_data_mode = 1;  //用于修改模式
uint16 write_data2 = 2;
uint32 write_data3 = 3;
float  write_data4 = 4.4;

// 读取数据
uint8  read_data_mode;
uint16 read_data2;
uint32 read_data3;
float  read_data4;

// 存储数据数组
uint32 write_buf[FLASH_SAVE_NUM];
uint32 read_buf[FLASH_SAVE_NUM];


//-------------------------------------------------------------------------------------------------------------------
//  @brief      储存数据(同时实现同步调参)
//  @param      sector_num			扇区编号
//  @param      page_num				扇区页数编号
//  @param      save_num				存储变量数量---暂时定义为4个
//  @return     void
//  @example    save_data(FLASH_SECTOR_126, FLASH_SECTOR_PAGE0, FLASH_SAVE_NUM);
//  @note       按键改变write_data值等后运行此函数
//-------------------------------------------------------------------------------------------------------------------
void save_data(int sector_num,int page_num,int save_num)
{
	uint8 status;
	if(flash_check(page_num,page_num))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
    {
        status = flash_erase_page(page_num,page_num);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
        if(status)  while(1);//擦除失败
    }
		
		write_buf[0] = write_data_mode;
		write_buf[1] = write_data2;
		write_buf[2] = write_data3;
		write_buf[3] = *(uint32 *)&write_data4;  //存储浮点时，首先取变量地址然后以uint32 *来访问变量获取数据。
		
		status = flash_write_page(page_num, page_num, write_buf, save_num);//写入扇区
    if(status)  while(1);//写入失败
		

		/*------------------------------------伪代码（同步调参）----------------------------------------
			模式 = write_data_mode;
			变量2 = write_data2;
			变量3 = write_data3;
		  变量4 = write_data4
		*/
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      从flash读取数据(把上一次存储参数部署)
//  @param      sector_num			扇区编号
//  @param      page_num				扇区页数编号
//  @param      save_num				存储变量数量---暂时定义为4个
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

		/*------------------------------------伪代码（部署参数）----------------------------------------
			模式 = read_data_mode;
			变量2 = read_data2;
			变量3 = read_data3;
		  变量4 = read_data4
		*/
}

// 清空缓冲区是flash_buffer_clear()