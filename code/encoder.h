#ifndef _encoder_h_
#define _encoder_h_
#include "zf_common_headfile.h"

#define M_TRANSFER_CM 100  //转换系数
#define IMPULSE_DIS 0.0000820225 //一个脉冲距离-单位m

void Encoder_Init(void);
void Encoder_Get(void);

#endif
