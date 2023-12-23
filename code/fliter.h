#ifndef _MYFLITER_H
#define _MYFLITER_H
#include "zf_common_headfile.h"

uint16 LowPassFilter(float AdcSample, float a);
int16 Average_Filter(float Sample_Value);

#endif
