#include "zf_common_headfile.h"
#include "fliter.h"

#define FO_LOW_PASS_FILTER_SENSITIVE_a                   0.73f                        //一阶低通滤波系数取值范围为(0,1)。值越小越稳定，越大越灵敏，二者难兼顾。
#define FO_LOW_PASS_FILTER_STEADY_a                      0.15f

//一阶低通滤波
//输入Adc采样值，输出低通滤波值
//优点：调节精细，稳定度和灵敏度偏向分明。缺点：带有浮点运算
uint16 FilterResult, FilterResultL;
uint16 LowPassFilter(float Sample_Value, float a)                                                                                                       \
{                                      
    FilterResult = (uint16)(a * Sample_Value + (1 - a) * FilterResultL);                                         \
    FilterResultL = FilterResult;   
    return FilterResult;                                                                           
}

//平均值滤波
int16 past_samples[10] = {0};
int16 Average_Filter(float Sample_Value)
{
    uint8 i;
    int16 sum_value = 0;
    for (i = 1; i <= 9; i++)
    {
        past_samples[i] = past_samples[i-1];
    }
    past_samples[0] = Sample_Value;

    for (i = 0; i <= 9; i++)
    {
        sum_value += past_samples[i];
    }

    return (int16)(sum_value / 10);
}
