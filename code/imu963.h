#ifndef _MYIMU963_H
#define _MYIMU963_H
#include "zf_common_headfile.h"

extern int16 imu_gyro_z;
extern int8 zero_drift_min;
extern int8 zero_drift_max;
extern int8 zero_drift_average;
extern float angle;

void imu_zero_drift(void);
void imu_get_gyro(void);

#endif
