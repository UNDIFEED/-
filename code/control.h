#ifndef __CONTROL_H
#define __CONTROL_H
#include "zf_common_headfile.h"

extern uint8 MotorBegin_Flag; //启动标志位
extern uint8 shizi_status; //十字状态
extern uint8 ShiZi_Flag; //十字标志位
extern uint8 yuanhuan_status; //圆环标志位
extern uint8 YuanHuan_Flag; //圆环标志位

void Mode_Switch(void);

#endif
