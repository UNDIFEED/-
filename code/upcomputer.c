#include "zf_common_headfile.h"
#include "upcomputer.h"

//VOFA JustFloat
void Upcomputer_Send(float data1, float data2, float data3, float data4)
{
    float data[4];
    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};

    data[0] = data1;
    data[1] = data2;
    data[2] = data3;
    data[3] = data4;
    //发送数据
    wireless_uart_send_buffer((uint8 *)data, sizeof(float)*4);
    wireless_uart_send_buffer(tail, 4);
}